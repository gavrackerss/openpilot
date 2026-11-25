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

    mici_hud = (ROOT / "openpilot/selfdrive/ui/mici/onroad/hud_renderer.py").read_text()
    mici_road = (ROOT / "openpilot/selfdrive/ui/mici/onroad/augmented_road_view.py").read_text()
    self.assertIn("ManualSpeedLimitBar", mici_hud)
    self.assertIn("self._manual_speed_limit_bar.render", mici_hud)
    self.assertIn("manual_speed_limit_contains", mici_hud)
    self.assertIn("manual_speed_limit_contains(mouse_pos)", mici_road)
    self.assertIn('"XNORManualSpeedLimitMph"', params)


  def test_v229_prime_is_preengagement_and_v231_sync_matches_0111(self):
    ctrl = (ROOT / "opendbc_repo/opendbc/car/tesla/carcontroller.py").read_text()

    prime_start = ctrl.index("def _v229_stopgo_long(")
    prime_end = ctrl.index("def update(", prime_start)
    prime = ctrl[prime_start:prime_end]
    self.assertIn("op_enabled = bool(CC.enabled)", prime)
    self.assertIn('mode = "priming"', prime)
    self.assertIn("preengage_prime=", prime)
    # 0.11.1 parity: priming is the final disengaged branch and does not require OP engagement.
    self.assertIn('else:\n      mode = "priming"', prime)

    sync_start = ctrl.index("if (not manual_active) and bool(self._v229_owner_now)")
    sync_end = ctrl.index("elif bool(self._v229_owner_now):", sync_start)
    sync = ctrl[sync_start:sync_end]
    # 0.11.1 V231 was bidirectional. Correct the target source rather than masking UP pulses.
    self.assertIn("BTN_UP2", sync)
    self.assertIn("BTN_UP1", sync)
    self.assertIn("BTN_DOWN2", sync)
    self.assertIn("BTN_DOWN1", sync)
    self.assertNotIn("blocked_upward_auto", sync)

    manual_start = ctrl.index("def _v279_manual_cruise_sync")
    manual_end = ctrl.index("def _speed_limit_sync", manual_start)
    manual = ctrl[manual_start:manual_end]
    self.assertIn("BTN_UP2", manual)
    self.assertIn("BTN_UP1", manual)
    self.assertIn("BTN_DOWN2", manual)
    self.assertIn("BTN_DOWN1", manual)

  def test_0111_hybrid_vcruise_and_legacy_di_contracts_are_restored(self):
    cruise = (ROOT / "openpilot/selfdrive/car/cruise.py").read_text()
    state = (ROOT / "opendbc_repo/opendbc/car/tesla/carstate.py").read_text()
    dbc = (ROOT / "opendbc_repo/opendbc/dbc/tesla_can.dbc").read_text()
    interface = (ROOT / "opendbc_repo/opendbc/car/tesla/interface.py").read_text()

    for marker in (
      "self.xnor_hybrid_op_longitudinal = bool(",
      'CP.brand == "tesla"',
      "CP.openpilotLongitudinalControl",
      "CP.pcmCruise",
      "if not self.CP.pcmCruise or self.xnor_hybrid_op_longitudinal:",
      "if self.CP.pcmCruise and not self.xnor_hybrid_op_longitudinal:",
    ):
      self.assertIn(marker, cruise)

    for marker in (
      'di_state.get("XNOR_DI_cruiseSetLegacy"',
      'di_state.get("XNOR_DI_digitalSpeedLegacy"',
      "_cruise_set_scale",
      "abs((2.0 * a) - b)",
      "abs((2.0 * a) - v_u)",
    ):
      self.assertIn(marker, state)

    self.assertIn('SG_ XNOR_DI_digitalSpeedLegacy : 32|8@1+ (1,0)', dbc)
    self.assertIn('SG_ XNOR_DI_cruiseSetLegacy : 48|8@1+ (1,0)', dbc)

    self.assertIn('p.get_bool("TinklaIgnoreStockAeb")', interface)
    self.assertIn("ret.deprecated.vEgoStopping = 0.1", interface)
    self.assertIn("ret.deprecated.vEgoStarting = 0.1", interface)
    self.assertIn("ret.deprecated.stoppingDecelRate = 0.3", interface)

  def test_0111_dec_release_and_stopping_behavior_are_restored(self):
    planner = (ROOT / "openpilot/selfdrive/controls/lib/longitudinal_planner.py").read_text()
    longcontrol = (ROOT / "openpilot/selfdrive/controls/lib/longcontrol.py").read_text()

    for marker in (
      "class XnorDynamicExperimentalControl:",
      "_DEC_ENTER_URGENCY = 0.24",
      "_DEC_EXIT_URGENCY = 0.12",
      "model_should_stop",
      "cruise_recovery",
      "XNOR_V230_DEC",
      "_RELEASE_BOOST_FLOOR_MS2 = 0.45",
      "XNOR_V232_RELEASE",
      "def _xnor_mpc_should_stop(",
      'getattr(deprecated, "vEgoStopping"',
      "v_target_1sec",
      "_xnor_v220_diag",
    ):
      self.assertIn(marker, planner)

    self.assertIn("_xnor_stopping_decel_rate", longcontrol)
    self.assertIn('getattr(deprecated, "stoppingDecelRate"', longcontrol)
    self.assertIn("output_accel -= self._xnor_stopping_decel_rate * DT_CTRL", longcontrol)


if __name__ == "__main__":
  unittest.main(verbosity=2)
