#!/usr/bin/env python3
"""Unit and lifecycle contract tests: staged, non-driving Tesla IC port."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import ast
import math
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "migration/tesla_v0112/ic_model_snapshot.py"
LEGACY = ROOT / "migration/tesla_opendbc_legacy/opendbc/car/tesla/interface.py"
CARD = ROOT / "openpilot/selfdrive/car/card.py"
MANAGER = ROOT / "openpilot/system/manager/process_config.py"

spec = importlib.util.spec_from_file_location("tesla_ic_offline", MODULE)
ic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ic)


def model(probs=(0.1, 0.5, 0.8, 0.2), xs=(0.0, 1.0, 2.0), ys=(0.0, 0.1, 0.2)):
  return SimpleNamespace(laneLineProbs=probs, position=SimpleNamespace(x=xs, y=ys))


class TeslaICPortContract(unittest.TestCase):
  def test_extracts_legacy_fields(self):
    sample = ic.extract_ic_model_data(model())
    self.assertEqual(sample, {
      'lane_probs': (0.1, 0.5, 0.8, 0.2),
      'position_x': (0.0, 1.0, 2.0),
      'position_y': (0.0, 0.1, 0.2),
    })

  def test_matches_previous_ic_field_selection(self):
    text = LEGACY.read_text()
    for field in ("laneLineProbs", "position.x", "position.y", "ic_model_data"):
      self.assertIn(field, text)

  def test_no_message_retains_last_good(self):
    last = ic.extract_ic_model_data(model())
    self.assertIs(ic.snapshot_from_message(None, last), last)
    self.assertEqual(ic.snapshot_from_message(SimpleNamespace(modelV2=model()), None), last)

  def test_rejects_nonfinite_and_invalid_lanes(self):
    good = ic.extract_ic_model_data(model())
    for sample in (
      model(probs=(math.nan, 0.5, 0.8, 0.2)),
      model(probs=(math.inf, 0.5, 0.8, 0.2)),
      model(probs=(-0.1, 0.5, 0.8, 0.2)),
      model(probs=(1.1, 0.5, 0.8, 0.2)),
      model(xs=(math.nan, 1.0, 2.0)),
      model(xs=(0.0, 1.0), ys=(0.0,)),
      model(probs=(0.1, 0.8, 0.2)),
      model(xs=(0.0,), ys=(0.0,)),
      model(xs=range(65), ys=range(65)),
      SimpleNamespace(laneLineProbs=[0.1]),
    ):
      with self.subTest(sample=str(sample)[:40]):
        self.assertIs(ic.extract_ic_model_data(sample, good), good)

  def test_never_mutates_source_data(self):
    probs = [0.1, 0.5, 0.8, 0.2]
    x = [0.0, 1.0]
    y = [0.0, 2.0]
    source = model(probs, x, y)
    snapshot = ic.extract_ic_model_data(source)
    x[0] = 99
    probs[0] = 1
    self.assertEqual(snapshot['position_x'][0], 0.0)
    self.assertEqual(snapshot['lane_probs'][0], 0.1)

  def test_no_runtime_or_car_control_dependencies(self):
    tree = ast.parse(MODULE.read_text())
    modules = [alias.name for node in ast.walk(tree)
               if isinstance(node, (ast.Import, ast.ImportFrom))
               for alias in (node.names if isinstance(node, ast.Import) else
                             [SimpleNamespace(name=node.module or "")])]
    self.assertEqual(set(modules), {'__future__', 'math', 'typing'})
    self.assertNotIn("tesla_v0112", CARD.read_text())
    self.assertNotIn("tesla_v0112", MANAGER.read_text())

  def test_native_card_does_not_automatically_call_legacy_post_update(self):
    native = CARD.read_text()
    self.assertIn("self.CI.update(can_list)", native)
    self.assertIn("self.CI.apply(CC, now_nanos)", native)
    self.assertNotIn("self.CI.post_update", native)


if __name__ == "__main__":
  unittest.main(verbosity=2)
