#!/usr/bin/env python3
"""Regression coverage for Tesla Hybrid steering alert classification."""
from types import SimpleNamespace
import unittest

from openpilot.cereal import log
from openpilot.selfdrive.selfdrived.helpers import hybrid_lane_change_steer_handover


class HybridLaneChangeSteeringAlerts(unittest.TestCase):
  @staticmethod
  def cp(*, hybrid=True):
    return SimpleNamespace(brand="tesla", pcmCruise=hybrid)

  @staticmethod
  def cs(*, stock_lkas=False, handover_ready=False):
    return SimpleNamespace(stockLkas=stock_lkas, teslaAlcHandoverReady=handover_ready)

  @staticmethod
  def model(state):
    return SimpleNamespace(meta=SimpleNamespace(laneChangeState=state))

  def test_native_lkas_owns_lane_change(self):
    self.assertTrue(hybrid_lane_change_steer_handover(
      self.cp(), self.cs(stock_lkas=True, handover_ready=False),
      self.model(log.LaneChangeState.preLaneChange),
    ))

  def test_waiting_for_epas_handover(self):
    self.assertTrue(hybrid_lane_change_steer_handover(
      self.cp(), self.cs(stock_lkas=False, handover_ready=False),
      self.model(log.LaneChangeState.laneChangeStarting),
    ))

  def test_normal_saturation_checks_resume_after_handover(self):
    self.assertFalse(hybrid_lane_change_steer_handover(
      self.cp(), self.cs(stock_lkas=False, handover_ready=True),
      self.model(log.LaneChangeState.laneChangeStarting),
    ))

  def test_no_suppression_outside_hybrid_lane_change(self):
    self.assertFalse(hybrid_lane_change_steer_handover(
      self.cp(), self.cs(stock_lkas=True, handover_ready=False),
      self.model(log.LaneChangeState.off),
    ))
    self.assertFalse(hybrid_lane_change_steer_handover(
      self.cp(hybrid=False), self.cs(stock_lkas=True, handover_ready=False),
      self.model(log.LaneChangeState.preLaneChange),
    ))


if __name__ == "__main__":
  unittest.main(verbosity=2)
