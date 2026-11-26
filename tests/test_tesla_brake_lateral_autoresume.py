#!/usr/bin/env python3
"""Regression guards for Tesla brake/lateral and Auto Resume parity."""
from pathlib import Path
import unittest

from openpilot.selfdrive.selfdrived.hybrid_alerts import tesla_op_long_brake_preserves_lateral

ROOT = Path(__file__).resolve().parents[1]


class TeslaBrakeLateralAutoResume(unittest.TestCase):
  def test_brake_preserves_lateral_for_both_tesla_op_long_modes(self):
    # pcmCruise/Hybrid is deliberately not part of this policy: both paths use
    # the same OP-long brake/lateral separation.
    self.assertTrue(tesla_op_long_brake_preserves_lateral("tesla", True))
    self.assertFalse(tesla_op_long_brake_preserves_lateral("tesla", False))
    self.assertFalse(tesla_op_long_brake_preserves_lateral("toyota", True))

  def test_brake_and_pcm_drop_do_not_turn_tesla_into_full_disengagement(self):
    selfdrive = (ROOT / "openpilot/selfdrive/selfdrived/selfdrived.py").read_text()
    self.assertIn("tesla_op_long_brake_preserves_lateral(", selfdrive)
    self.assertIn("brake_or_regen and not tesla_brake_long_only", selfdrive)

    car_events = (ROOT / "openpilot/selfdrive/car/car_events.py").read_text()
    start = car_events.index("# Tesla OP-long separates lateral engagement")
    block = car_events[start:start + 620]
    self.assertIn("self.CP.brand == 'tesla'", block)
    self.assertIn("self.CP.openpilotLongitudinalControl", block)
    self.assertIn("if not tesla_op_long:", block)
    self.assertIn("events.add(EventName.pcmDisable)", block)

  def test_controlsd_latch_covers_hybrid_and_normal_op_long(self):
    source = (ROOT / "openpilot/selfdrive/controls/controlsd.py").read_text()
    start = source.index("tesla_op_long_brake_latch =")
    block = source[start:start + 320]
    self.assertIn("self.CP.brand == 'tesla'", block)
    self.assertIn("self.CP.openpilotLongitudinalControl", block)
    self.assertNotIn("pcmCruise", block)

  def test_autoresume_is_owned_by_tinkla_toggle_not_experimental_mode(self):
    source = (ROOT / "openpilot/selfdrive/controls/controlsd.py").read_text()
    start = source.index("auto_resume_ready = bool(")
    block = source[start:start + 620]
    self.assertIn("self._xnor_auto_resume_acc", block)
    self.assertNotIn("_xnor_experimental_mode", block)
    self.assertIn("(now_ms - self._xnor_tesla_brake_last_press_ms) >= 1000", block)

  def test_hybrid_native_tacc_rearm_uses_same_autoresume_toggle(self):
    source = (ROOT / "opendbc_repo/opendbc/car/tesla/carcontroller.py").read_text()
    start = source.index("def _v241_auto_resume_stalk(")
    block = source[start:start + 5200]
    self.assertIn("self._cached_auto_resume_acc", block)
    self.assertNotIn("_cached_experimental_mode", block)

  def test_physical_main_resume_accepts_legacy_release_edge(self):
    source = (ROOT / "openpilot/selfdrive/controls/controlsd.py").read_text()
    start = source.index("physical_resume = any(")
    block = source[start:start + 260]
    self.assertIn("ButtonEvent.Type.resumeCruise", block)
    self.assertNotIn("be.pressed and", block)


if __name__ == "__main__":
  unittest.main(verbosity=2)
