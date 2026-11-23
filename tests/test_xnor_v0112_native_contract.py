#!/usr/bin/env python3
"""No-dependency contract tests for staging the actual comma v0.11.2 tree.

Run: python3 tests/test_xnor_v0112_native_contract.py -v

These verify static migration boundaries, native ModelState signature,
preservation of Tesla legacy source snapshots and deliberate non-activation.
They DO NOT qualify real models, CAN/panda safety or driving.
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import ast
import unittest

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = ROOT / "openpilot/jetlink_v0112_native.py"
MODELD = ROOT / "openpilot/selfdrive/modeld/modeld.py"
MANAGER = ROOT / "openpilot/system/manager/process_config.py"
SNAPSHOT = ROOT / "migration/tesla_opendbc_legacy/opendbc"

spec = importlib.util.spec_from_file_location("jetlink_v0112_native_unconnected", ADAPTER)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fake_native_model():
  class NativeSmall:
    def __init__(self):
      self.chestnut = False
      self.vision_input_names = ("img", "big_img")
      self.state_pairs = {"prev_feat": "next_prev_feat"}
      self.input_queues = {"prev_feat": object()}
      self.packed_input = bytearray(12)
      self.prev_desire = [0] * 8

    def run(self, bufs, transforms, inputs, after_enqueue=None):
      return {}

  return NativeSmall()


class NativeContractTest(unittest.TestCase):
  def test_is_actual_v0112_source(self):
    self.assertIn('COMMA_VERSION "0.11.2"', (ROOT/"openpilot/common/version.h").read_text())

  def test_modeld_remains_unmodified_for_authority(self):
    src = MODELD.read_text()
    self.assertIn("def main(demo=False)", src)
    self.assertIn("def get_action_from_model(", src)
    self.assertNotIn("jetlink_v0112_native", src)
    self.assertNotIn("from openpilot import jetlink_authority", src)
    self.assertNotIn("attach_offroad_probe", src)

  def test_manager_does_not_enable_jetlink(self):
    self.assertNotIn("jetlink_v0112_native", MANAGER.read_text())

  def test_native_state_contract_accepted(self):
    self.assertIsNone(module.validate_native_small_model(fake_native_model()))

  def test_old_0111_run_contract_rejected(self):
    small = fake_native_model()
    small.run = lambda bufs, transforms, inputs, prepare_only=False: {}
    with self.assertRaisesRegex(ValueError, "after_enqueue"):
      module.validate_native_small_model(small)

  def test_missing_native_recurrent_state_rejected(self):
    small = fake_native_model()
    small.input_queues = {}
    with self.assertRaisesRegex(ValueError, "recurrent"):
      module.validate_native_small_model(small)

  def test_legacy_gpu_state_rejected(self):
    small = fake_native_model()
    del small.state_pairs
    with self.assertRaisesRegex(ValueError, "state_pairs"):
      module.validate_native_small_model(small)

  def test_legacy_tesla_models_preserved(self):
    values = (SNAPSHOT/"car/tesla/values.py").read_text()
    for platform in ("TESLA_MODEL_S_HW1", "TESLA_MODEL_S_HW2", "TESLA_MODEL_S_HW3",
                     "TESLA_MODEL_X_HW1", "TESLA_MODEL_X_HW2"):
      self.assertIn(platform, values)

  def test_legacy_safety_and_test_sources_preserved(self):
    for file in ("safety/modes/tesla_legacy.h",
                 "safety/tests/test_tesla_hw1.py",
                 "safety/tests/test_tesla_hw23.py",
                 "car/docs_definitions.py",
                 "dbc/tesla_can.dbc"):
      with self.subTest(path=file):
        self.assertTrue((SNAPSHOT/file).is_file())

  def test_legacy_jetlink_sources_present(self):
    for file in ("jetlink/openpilot/joining.py", "jetlink/openpilot/model_state.py",
                 "jetlink/openpilot/stock.py", "openpilot/jetlink_native_authority.py"):
      self.assertTrue((ROOT/file).is_file())

  def test_no_connected_authority_in_parking_probe(self):
    src = ADAPTER.read_text()
    self.assertIn("in_control=lambda: True", src)
    self.assertNotIn("in_control=stock.in_control", src)


if __name__ == "__main__":
  unittest.main(verbosity=2)
