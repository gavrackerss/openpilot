#!/usr/bin/env python3
"""Regression coverage for Tesla Hybrid steering alert classification."""
import unittest

from openpilot.selfdrive.selfdrived.hybrid_alerts import hybrid_lane_change_steer_handover


class HybridLaneChangeSteeringAlerts(unittest.TestCase):
  def test_native_lkas_owns_lane_change(self):
    self.assertTrue(hybrid_lane_change_steer_handover(
      "tesla", True, True, False, True,
    ))

  def test_waiting_for_epas_handover(self):
    self.assertTrue(hybrid_lane_change_steer_handover(
      "tesla", True, False, False, True,
    ))

  def test_normal_saturation_checks_resume_after_handover(self):
    self.assertFalse(hybrid_lane_change_steer_handover(
      "tesla", True, False, True, True,
    ))

  def test_no_suppression_outside_hybrid_lane_change(self):
    self.assertFalse(hybrid_lane_change_steer_handover(
      "tesla", True, True, False, False,
    ))
    self.assertFalse(hybrid_lane_change_steer_handover(
      "tesla", False, True, False, True,
    ))
    self.assertFalse(hybrid_lane_change_steer_handover(
      "toyota", True, True, False, True,
    ))


if __name__ == "__main__":
  unittest.main(verbosity=2)
