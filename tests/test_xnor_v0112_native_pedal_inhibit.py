#!/usr/bin/env python3
"""Runtime contract for restored Tesla Hybrid brake/regen and V234 auto-resume."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / "openpilot/selfdrive/controls/controlsd.py"


class TestHybridPedalAndAutoResume(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.source = CONTROL.read_text()
    cls.tree = ast.parse(cls.source)
    cls.controls = next(n for n in cls.tree.body if isinstance(n, ast.ClassDef) and n.name == "Controls")
    cls.init = next(n for n in cls.controls.body if isinstance(n, ast.FunctionDef) and n.name == "__init__")
    cls.state_control = next(n for n in cls.controls.body if isinstance(n, ast.FunctionDef) and n.name == "state_control")

  def test_runtime_owns_a_separate_brake_longitudinal_latch(self):
    init = ast.unparse(self.init)
    self.assertIn("self._xnor_hybrid_brake_long_cancelled = False", init)
    self.assertIn("self._xnor_hybrid_brake_last_press_ms = 0", init)
    self.assertIn("self.params.get_bool", init)
    self.assertIn("TinklaAutoResumeACC", init)
    self.assertIn("ExperimentalMode", init)

  def test_brake_and_regen_fail_closed_before_rearm(self):
    body = ast.unparse(self.state_control)
    self.assertIn("CS.brakePressed", body)
    self.assertIn("CS.regenBraking", body)
    self.assertIn("self._xnor_hybrid_brake_long_cancelled = True", body)
    self.assertIn("not CS.brakePressed", body)
    self.assertIn("not CS.regenBraking", body)
    self.assertIn("not self._xnor_hybrid_brake_long_cancelled", body)

  def test_v234_auto_resume_has_all_original_gates(self):
    body = ast.unparse(self.state_control)
    for required in (
      "self._xnor_auto_resume_acc",
      "self._xnor_experimental_mode",
      "not CS.gasPressed",
      "accFaulted",
      ">= 1000",
      "resumeCruise",
      "XNOR_V234_AUTORESUME",
    ):
      self.assertIn(required, body)

  def test_longitudinal_rearm_does_not_change_lateral_gate(self):
    body = ast.unparse(self.state_control)
    lat_assignment = next(
      n for n in ast.walk(self.state_control)
      if isinstance(n, ast.Assign) and any(isinstance(t, ast.Attribute) and t.attr == "latActive" for t in n.targets)
    )
    lat = ast.unparse(lat_assignment)
    self.assertNotIn("_xnor_hybrid_brake", lat)
    self.assertNotIn("TinklaAutoResumeACC", lat)
    self.assertIn("CC.latActive", body)


if __name__ == "__main__":
  unittest.main(verbosity=2)
