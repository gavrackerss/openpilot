#!/usr/bin/env python3
"""Offroad/unconnected parity checks for staged comma v0.11.2 policy helpers."""
import math
import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.lib.xnor_v0112_candidates import (
  get_cruise_accel_0112, select_longitudinal_candidates_0112,
)


class V0112CruiseCandidateTest(unittest.TestCase):
  def setUp(self):
    self.cp = SimpleNamespace(steerRatio=15.0, wheelbase=2.96)

  def cruise(self, **kw):
    return get_cruise_accel_0112(
      e2e=kw.get("e2e", False),
      v_cruise=kw.get("v_cruise", 25.0),
      v_ego=kw.get("v_ego", 15.0),
      a_cruise_prev=kw.get("a_cruise_prev", 0.0),
      angle_steers=kw.get("angle_steers", 0.0),
      CP=self.cp, dt=0.05, accel_coast=-0.3,
      allow_throttle=kw.get("allow_throttle", True),
      accel_min=-3.5, accel_max=2.0,
    )

  def test_jerk_limited_positive_acceleration(self):
    # v=15m/s => j=1.04m/s^3, dt=0.05 => 0.052m/s^2
    v = self.cruise()
    self.assertGreater(v, 0.0)
    self.assertLess(v, 0.08)

  def test_cruise_target_below_speed_limits_request(self):
    self.assertLess(self.cruise(v_cruise=10.0), 0.0)

  def test_disallow_throttle_does_not_increase_acceleration(self):
    # compare under steady enough a_cruise_prev that coast limiting is active
    with_throttle = self.cruise(v_ego=5.0, a_cruise_prev=0.9, allow_throttle=True)
    without_throttle = self.cruise(v_ego=5.0, a_cruise_prev=0.9, allow_throttle=False)
    self.assertLessEqual(without_throttle, with_throttle)

  def test_turn_limit_non_e2e(self):
    straight = self.cruise(v_ego=25, a_cruise_prev=1.0)
    turning = self.cruise(v_ego=25, a_cruise_prev=1.0, angle_steers=30.0)
    self.assertLessEqual(turning, straight)

  def test_model_braking_wins_when_experimental(self):
    a, source, stop = select_longitudinal_candidates_0112(
      0.2, "lead", False, 0.5, "cruise", False, -1.0, "e2e", True, True)
    self.assertEqual((a, source, stop), (-1.0, "e2e", True))

  def test_model_excluded_outside_experimental(self):
    a, source, stop = select_longitudinal_candidates_0112(
      0.2, "lead", False, 0.5, "cruise", False, -1.0, "e2e", True, False)
    self.assertEqual((a, source, stop), (0.2, "lead", False))

  def test_stop_is_or_of_candidates_even_when_other_source_wins(self):
    a, source, stop = select_longitudinal_candidates_0112(
      -0.2, "lead", True, -0.4, "cruise", False, 0.1, "e2e", False, True)
    self.assertEqual((a, source, stop), (-0.4, "cruise", True))


if __name__ == "__main__":
  unittest.main(verbosity=2)
