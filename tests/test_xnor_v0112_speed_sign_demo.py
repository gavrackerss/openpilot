#!/usr/bin/env python3
"""Unit regression for the isolated manual sign interaction."""
import importlib.util
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("speed_demo_state", ROOT / "openpilot/selfdrive/ui/onroad/manual_speed_limit_demo_state.py")
state = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = state
spec.loader.exec_module(state)


class SpeedSignDemoTests(unittest.TestCase):
  def test_two_taps_and_immediate_cancel(self):
    s = state.SpeedSignDemo()
    self.assertFalse(s.tap(50, 10, True))
    self.assertEqual((s.pending, s.selected), (50, 0))
    self.assertTrue(s.tap(50, 12, True))
    self.assertEqual((s.pending, s.selected), (0, 50))
    self.assertTrue(s.tap(0, 13, True))
    self.assertEqual((s.pending, s.selected), (0, 0))

  def test_timeout_and_different_selection(self):
    s = state.SpeedSignDemo()
    s.tap(40, 10, True)
    self.assertFalse(s.tap(40, 13, True))
    self.assertFalse(s.tap(60, 14, True))
    self.assertEqual(s.selected, 0)
    self.assertTrue(s.tap(60, 15, True))

  def test_losing_offroad_confirmation_resets_demo(self):
    s = state.SpeedSignDemo()
    s.tap(70, 10, True)
    self.assertTrue(s.tap(70, 11, True))
    s.tick(12, False)
    self.assertEqual((s.selected, s.pending), (0, 0))
    self.assertFalse(s.tap(40, 13, False))

  def test_offset_display(self):
    self.assertEqual(state.preview_target_mph(50, 5, False, False), 55)
    self.assertAlmostEqual(state.preview_target_mph(60, 10, True, True), 66)
    self.assertAlmostEqual(state.preview_target_mph(40, 10, False, True), 46.21371)
    for args in ((30, 0, False, False), (70, 20, False, False), (40, float("nan"), False, False)):
      self.assertIsNone(state.preview_target_mph(*args))

  def test_widget_not_attached_to_live_hud(self):
    demo = (ROOT / "openpilot/selfdrive/ui/onroad/manual_speed_limit_demo.py").read_text()
    hud = (ROOT / "openpilot/selfdrive/ui/onroad/hud_renderer.py").read_text()
    self.assertNotIn("XNORManualSpeedLimitMph", demo)
    self.assertNotIn(".put(", demo)
    self.assertNotIn("SpeedSignDemoRow", hud)


if __name__ == "__main__":
  unittest.main(verbosity=2)
