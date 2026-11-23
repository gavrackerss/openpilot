#!/usr/bin/env python3
"""Static, pinned-source Tesla port blockers (not a Panda safety test)."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "tesla_preflight", ROOT / "tools/xnor_v0112_tesla_preflight.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


class NativeTeslaPreflightContract(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.report = tool.report()

  def test_native_HW1_HW2_HW3_definitions_are_present(self):
    self.assertEqual(len(self.report["supported_platforms_present_in_both"]), 5)
    self.assertTrue(all(self.report["supported_platforms_present_in_both"].values()))

  def test_custom_safety_params_not_silently_accepted(self):
    self.assertEqual(self.report["custom_safety_bits"]["OP_STALK_ENABLE"]["legacy"], 64)
    self.assertEqual(self.report["custom_safety_bits"]["IGNORE_STOCK_AEB"]["legacy"], 128)
    self.assertNotEqual(self.report["custom_safety_bits"]["OP_STALK_ENABLE"]["native"], 64)
    self.assertNotEqual(self.report["custom_safety_bits"]["IGNORE_STOCK_AEB"]["native"], 128)

  def test_car_post_update_lifecycle_mismatch_is_tracked(self):
    self.assertTrue(self.report["legacy_post_update_removed"])

  def test_lateral_limits_api_needs_rebase(self):
    self.assertTrue(self.report["angle_limits_VM_migration_needed"])

  def test_not_wired_or_qualified(self):
    self.assertFalse(self.report["ready_to_merge_or_drive"])
    blockers = self.report["blockers"]
    for k in ("custom_safety_flags_not_in_native_python_and_C",
              "legacy_post_update_not_called_by_native_car_lifecycle",
              "custom_Tesla_CAN_actuator_controller_not_rebased"):
      self.assertIn(k, blockers)


if __name__ == "__main__":
  unittest.main(verbosity=2)
