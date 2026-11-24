#!/usr/bin/env python3
"""Static integration guard for the live XNOR 0.11.2 active-parity paths."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class ActiveParityContract(unittest.TestCase):
  def test_v229_priming_and_v241_autoresume_are_in_active_opendbc(self):
    ctrl = (ROOT / "opendbc_repo/opendbc/car/tesla/carcontroller.py").read_text()
    safety = (ROOT / "opendbc_repo/opendbc/safety/modes/tesla_legacy.h").read_text()

    self.assertIn("_V229_PRIME_ACCEL = (-1.4, 1.8)", ctrl)
    self.assertIn("_V229_PRIME_JERK = (-0.46, 0.476)", ctrl)
    self.assertIn('mode = "priming"', ctrl)
    self.assertIn("def _v241_auto_resume_stalk", ctrl)
    self.assertIn("BTN_UP1", ctrl)
    self.assertIn("def _v279_manual_target_mph", ctrl)

    self.assertIn("const bool priming =", safety)
    self.assertIn("!long_allowed && !neutral && !priming", safety)
    for decoder in (
      "TESLA_LEGACY_V229_PRIME_ACCEL_RAW_MIN",
      "TESLA_LEGACY_V229_PRIME_JERK_MIN_RAW",
      "tesla_legacy_das_raw_set_speed",
      "tesla_legacy_das_raw_jerk_min",
      "tesla_legacy_das_raw_jerk_max",
    ):
      self.assertIn(decoder, safety)
    self.assertIn("TESLA_LEGACY_V229_UNOWNED_DI_CANCEL_US", safety)
    self.assertIn("original_das_control_aeb_event == 1", safety)

  def test_v234_runtime_rearm_is_connected(self):
    controls = (ROOT / "openpilot/selfdrive/controls/controlsd.py").read_text()
    for marker in (
      "_xnor_hybrid_brake_long_cancelled",
      "_xnor_hybrid_brake_last_press_ms",
      "TinklaAutoResumeACC",
      "ExperimentalMode",
      "XNOR_V234_AUTORESUME",
      ">= 1000",
    ):
      self.assertIn(marker, controls)

  def test_v279_live_ui_writes_only_the_manual_param(self):
    ui = (ROOT / "openpilot/selfdrive/ui/onroad/manual_speed_limit.py").read_text()
    hud = (ROOT / "openpilot/selfdrive/ui/onroad/hud_renderer.py").read_text()
    params = (ROOT / "openpilot/common/params_keys.h").read_text()

    self.assertIn('PARAM = "XNORManualSpeedLimitMph"', ui)
    self.assertIn("CONFIRM_SECONDS = 3.0", ui)
    self.assertIn("self._params.put(PARAM, int(value))", ui)
    self.assertNotIn("sendcan", ui)
    self.assertNotIn("CANPacker", ui)
    self.assertIn("ManualSpeedLimitBar", hud)
    self.assertIn("self._manual_speed_limit_bar.render", hud)
    self.assertIn('"XNORManualSpeedLimitMph"', params)


if __name__ == "__main__":
  unittest.main(verbosity=2)
