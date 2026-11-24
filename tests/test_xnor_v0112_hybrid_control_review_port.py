#!/usr/bin/env python3
"""Offline integration tests for the combined Hybrid review port."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from migration.tesla_v0112.hybrid_control_review_port import ReviewFrame, ReviewState, observe_frame


class HybridReviewPortTest(unittest.TestCase):
  def test_brake_regen_and_resume_sequence(self):
    s = ReviewState()
    results = []
    for frame in (
        ReviewFrame(100),
        ReviewFrame(110, brake=True),
        ReviewFrame(500, brake=True),
        ReviewFrame(900, regen=True),
        ReviewFrame(1899),
        ReviewFrame(1900),
    ):
      out = observe_frame(s, frame)
      s = out.state
      results.append(out)
      self.assertFalse(out.runtime_activated)
      self.assertFalse(out.vehicle_qualified)
      self.assertTrue(out.lateral_eligibility_observed)
    self.assertEqual([x.reference_latch.would_allow_long for x in results],
                     [True, False, False, False, False, True])
    self.assertEqual(results[1].native_pedal_events, ("pedalPressed",))
    self.assertEqual(results[1].legacy_reference_pedal_events, ("gasPressedOverride",))
    self.assertEqual(results[5].reference_latch.transition, "auto_resume")

  def test_manual_resume_and_disabled_auto_resume(self):
    a = observe_frame(ReviewState(), ReviewFrame(100, brake=True))
    b = observe_frame(a.state, ReviewFrame(1200, auto_resume_preference=False))
    self.assertTrue(b.state.latch.cancelled_by_brake)
    c = observe_frame(b.state, ReviewFrame(1210, physical_main_res=True,
                                           auto_resume_preference=False, experimental=False))
    self.assertEqual(c.reference_latch.transition, "manual_resume")
    self.assertFalse(c.state.latch.cancelled_by_brake)

  def test_no_resume_while_regen_held(self):
    a = observe_frame(ReviewState(), ReviewFrame(100, brake=True))
    b = observe_frame(a.state, ReviewFrame(2000, regen=True, physical_main_res=True))
    self.assertTrue(b.state.latch.cancelled_by_brake)
    self.assertFalse(b.reference_latch.would_allow_long)

  def test_fault_and_driver_override_are_visible_independently(self):
    for changes, reason in (
        (dict(stock_aeb=True), "stock_aeb"),
        (dict(can_valid=False), "invalid_can"),
        (dict(can_timeout=True), "can_timeout"),
        (dict(drive_gear=False), "not_drive"),
        (dict(acc_fault=True), "acc_fault"),
        (dict(gas=True), "gas_pressed"),
        (dict(external_long_override=True), "external_long_override"),
    ):
      with self.subTest(reason=reason):
        o = observe_frame(ReviewState(), replace(ReviewFrame(100), **changes))
        self.assertIn(reason, o.independent_owner_blockers)
        self.assertFalse(o.runtime_activated)
        self.assertFalse(o.vehicle_qualified)

  def test_disengage_clears_latch_without_rewriting_native_state(self):
    a = observe_frame(ReviewState(), ReviewFrame(100, brake=True))
    b = observe_frame(a.state, ReviewFrame(200, enabled=False, lateral_eligible=False))
    self.assertFalse(b.state.latch.cancelled_by_brake)
    self.assertFalse(b.native_enabled_observed)
    self.assertFalse(b.lateral_eligibility_observed)
    self.assertFalse(b.reference_latch.would_allow_long)

  def test_non_tesla_brake_retains_native_event(self):
    a = observe_frame(ReviewState(), ReviewFrame(100, brand="other", brake=True))
    self.assertEqual(a.native_pedal_events, a.legacy_reference_pedal_events)
    self.assertFalse(a.reference_latch.would_allow_long)

  def test_monotonic_clock(self):
    a = observe_frame(ReviewState(), ReviewFrame(100))
    with self.assertRaises(ValueError):
      observe_frame(a.state, ReviewFrame(100))

  def test_no_runtime_integration(self):
    for path in ("openpilot/selfdrive/controls/controlsd.py",
                 "openpilot/selfdrive/selfdrived/selfdrived.py"):
      self.assertNotIn("hybrid_control_review_port", (ROOT / path).read_text())


if __name__ == "__main__":
  unittest.main(verbosity=2)
