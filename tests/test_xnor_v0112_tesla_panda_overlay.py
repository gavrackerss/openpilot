#!/usr/bin/env python3
"""Pinned Tesla Panda/opendbc static tests (no safety firmware execution)."""
import ast
import importlib.util
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(name, file):
  spec = importlib.util.spec_from_file_location(name, ROOT / file)
  obj = importlib.util.module_from_spec(spec)
  sys.modules[name] = obj
  spec.loader.exec_module(obj)
  return obj


audit = load("v0112_tesla_safety_audit", "tools/xnor_v0112_tesla_panda_preflight.py")
di = load("v0112_tesla_di_state", "migration/tesla_v0112/di_state_layout.py")


class TeslaNativePandaContract(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.result = audit.audit()

  def test_pinned_native_files_verified(self):
    self.assertEqual(self.result["verified_native_blob_count"], 4)
    self.assertFalse(self.result["ready_to_flash_panda"])
    self.assertFalse(self.result["ready_to_enable_tesla_longitudinal"])

  def test_safe_native_panda_aeb_behavior_is_not_replaced(self):
    source = audit.read(audit.OVERLAY / "safety/modes/tesla_legacy.h")
    self.assertIn("if (aeb_event != 0)", source)
    self.assertIn("if (tesla_legacy_stock_aeb)", source)
    self.assertNotIn("TESLA_LEGACY_FLAG_IGNORE_STOCK_AEB", source)
    self.assertNotIn("0x659", source)
    self.assertEqual(source, audit.read(audit.PINNED / "safety/modes/tesla_legacy.h"))

  def test_hw2_external_panda_and_legacy_flags_are_unmerged(self):
    self.assertEqual(self.result["legacy_unsupported_safety_flags"],
                     {"OP_STALK_ENABLE": 64, "IGNORE_STOCK_AEB": 128})
    self.assertEqual(self.result["tesla_platform_flags"]["FLAG_EXTERNAL_PANDA"], 4)
    self.assertEqual(self.result["tesla_platform_flags"]["FLAG_HW2"], 16)
    self.assertNotIn("OP_STALK_ENABLE", self.result["tesla_platform_flags"])
    self.assertNotIn("IGNORE_STOCK_AEB", self.result["tesla_platform_flags"])
    self.assertIn("Tesla HW2 dual-Panda", " ".join(self.result["blocked_on"]))

  def test_native_DBC_only_adds_lane_telemetry(self):
    overlay = audit.read(audit.OVERLAY / "dbc/tesla_can.dbc")
    pinned = audit.read(audit.PINNED / "dbc/tesla_can.dbc")
    insertion_start = overlay.index("BO_ 937 DAS_telemetry:")
    insertion_end = overlay.index("\n\n", insertion_start)
    self.assertEqual(overlay[:insertion_start] + overlay[insertion_end + 2:], pinned)
    self.assertEqual(audit.message_names(overlay)[937], "DAS_telemetry")
    self.assertEqual(len(audit.message_names(overlay)), len(audit.message_names(pinned)) + 1)

  def test_cruise_signal_layout_requires_vehicle_replay(self):
    self.assertIn("32|9@1+", self.result["native_DI_cruiseSet"])
    self.assertIn("48|8@1+", self.result["legacy_DI_cruiseSet"])
    self.assertIn("48|8@1+", self.result["native_DI_digitalSpeed"])
    self.assertIn("32|8@1+", self.result["legacy_DI_digitalSpeed"])
    # Both can be plausible; interpreting the payload alone cannot pick one.
    frame = bytes((0, 0, 0, 0, 80, 0, 60, 0))
    observation = di.inspect_di_state(frame)
    self.assertEqual(observation.native_cruise_set_units, 40.0)
    self.assertEqual(observation.native_digital_speed_units, 60.0)
    self.assertEqual(observation.installed_cruise_set_units, 60.0)
    self.assertEqual(observation.installed_digital_speed_units, 80.0)
    self.assertFalse(observation.layout_qualified)

  def test_status2_conflicting_bits_are_not_merged(self):
    blocker = " ".join(self.result["blocked_on"])
    self.assertIn("DAS_status2", blocker)
    self.assertIn("DAS_lssState", blocker)
    dbc = audit.read(audit.OVERLAY / "dbc/tesla_can.dbc")
    self.assertIn(" SG_ DAS_lssState : 31|3@1+", dbc)
    self.assertNotIn(" SG_ DAS_relaxCruiseLimits : 31|1@1+", dbc)

  def test_stalk_enums_are_passive_and_native_VM_limits_retained(self):
    overlay = audit.read(audit.OVERLAY / "car/tesla/values.py")
    self.assertIn("class CruiseButtons:", overlay)
    self.assertIn("CURVE_ASSIST_GAIN_V", overlay)
    self.assertIn("AngleSteeringLimitsVM", overlay)
    self.assertNotIn("  OP_STALK_ENABLE =", overlay)
    self.assertNotIn("  IGNORE_STOCK_AEB =", overlay)
    compile(overlay, str(audit.OVERLAY / "car/tesla/values.py"), "exec")

  def test_unconnected_to_native_car_and_panda(self):
    active = (ROOT / "openpilot/selfdrive/car/card.py").read_text()
    self.assertNotIn("tesla_v0112", active)
    f = ROOT / "migration/tesla_v0112/di_state_layout.py"
    imports = {a.name for n in ast.walk(ast.parse(f.read_text()))
               if isinstance(n, ast.Import) for a in n.names}
    self.assertNotIn("socket", imports)
    for invalid in (b"", bytes(7), bytes(9), bytearray(8)):
      with self.assertRaises(ValueError):
        di.inspect_di_state(invalid)


if __name__ == "__main__":
  unittest.main(verbosity=2)
