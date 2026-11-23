#!/usr/bin/env python3
"""Offline parity tests for v0.11.2 short-action-horizon interpolation.

The XNOR helper retains its two-value (acceleration, should_stop) API.
"""
import unittest
from types import SimpleNamespace

import numpy as np
from openpilot.selfdrive.controls.lib.drive_helpers import (
  MIN_STABLE_DELAY, get_accel_from_plan, get_curvature_from_plan, should_stop, radar_lead_present,
)


class V0112StableDelayTest(unittest.TestCase):
  def setUp(self):
    self.t = np.array([0, 0.05, 0.10, 0.20, 0.30, 0.50, 1.0, 2.0])
    # Deliberate discontinuity within first 0.3 seconds. The v0.11.2
    # interpolation must use the stable 0.3-second horizon for action_t=0.1.
    self.speed = np.array([10, 10, 10, 10, 9, 9, 9, 9], dtype=float)
    self.accel = np.zeros_like(self.t)

  def test_stable_delay_matches_upstream_0112(self):
    self.assertAlmostEqual(MIN_STABLE_DELAY, 0.3)
    a, should_stop_now = get_accel_from_plan(self.speed, self.accel, self.t, action_t=0.1)
    self.assertAlmostEqual(a, 2.0*(9.0-10.0)/0.3, places=5)
    self.assertFalse(should_stop_now)

  def test_long_horizon_preserves_interpolation(self):
    a, _ = get_accel_from_plan(self.speed, self.accel, self.t, action_t=0.5)
    self.assertAlmostEqual(a, -4.0, places=5)

  def test_legacy_stop_flag_contract_still_exists(self):
    speed = np.zeros_like(self.speed)
    a, stop = get_accel_from_plan(speed, self.accel, self.t, action_t=0.1,
                                  vEgoStopping=0.05)
    self.assertTrue(stop)
    self.assertAlmostEqual(a, 0.0)

  def test_curvature_horizon_stable(self):
    yaw = np.array([0, 0, 0, 0, 0.06, 0.06, 0.06, 0.06], dtype=float)
    yaw_rates = np.zeros_like(yaw)
    val = get_curvature_from_plan(yaw, yaw_rates, self.t, 10.0, 0.1)
    # stable psi at .1 = (.1/.3)*.06=.02; curvature = 2*.02/(10*.1)
    self.assertAlmostEqual(val, 0.04, places=5)

  def test_upstream_low_speed_stop_predicate(self):
    self.assertTrue(should_stop(0.2, -0.1))
    self.assertFalse(should_stop(5.0, -2.0))
    self.assertFalse(should_stop(0.2, 0.3))


  def test_legacy_radar_lead_status(self):
    self.assertTrue(radar_lead_present(SimpleNamespace(status=True)))
    self.assertFalse(radar_lead_present(SimpleNamespace(status=False)))

  def test_v0112_radar_lead_present(self):
    self.assertTrue(radar_lead_present(SimpleNamespace(present=True)))
    self.assertFalse(radar_lead_present(SimpleNamespace(present=False)))

  def test_missing_and_dict_radar_leads(self):
    self.assertFalse(radar_lead_present(None))
    self.assertTrue(radar_lead_present({"present": True}))
    self.assertTrue(radar_lead_present({"status": True}))


if __name__ == "__main__":
  unittest.main(verbosity=2)
