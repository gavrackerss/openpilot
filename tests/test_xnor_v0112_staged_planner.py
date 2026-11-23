#!/usr/bin/env python3
"""Parked/offline tests for the quarantined v0.11.2 three-candidate planner.

This intentionally does NOT exercise real Tesla or LS2 actuation.
It patches the MPC factory so no native solver or car is controlled.
"""
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from cereal import log

from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.v0112_migration import longitudinal_planner as staged
from openpilot.selfdrive.controls.lib.v0112_migration.long_mpc import T_IDXS


class DeterministicMpc:
  def __init__(self, dt):
    self.v_solution = np.full(len(T_IDXS), 15.0)
    self.a_solution = np.zeros(len(T_IDXS))
    self.j_solution = np.zeros(len(T_IDXS)-1)
    self.crash_cnt = 0
    self.solve_time = 0.0
    self.source = log.LongitudinalPlan.LongitudinalPlanSource.lead0
    self.update_kwargs = None

  def set_weights(self, *args, **kwargs):
    pass

  def set_cur_state(self, v, a):
    pass

  def update(self, radarstate, **kwargs):
    self.update_kwargs = kwargs


def make_inputs(exp=True, model_a=-1.0, model_stop=False, lead_present=True):
  sm = {
    "carControl": SimpleNamespace(orientationNED=[0.0, 0.0, 0.0]),
    "carState": SimpleNamespace(vEgo=15.0, aEgo=0.0, vCruise=90.0, standstill=False,
                                 steeringAngleDeg=0.0, gasPressed=False, followDistanceS=3),
    "controlsState": SimpleNamespace(longControlState=LongCtrlState.pid, forceDecel=False),
    "selfdriveState": SimpleNamespace(enabled=True, experimentalMode=exp,
                                      personality=log.LongitudinalPersonality.standard),
    "liveParameters": SimpleNamespace(angleOffsetDeg=0.0),
    "modelV2": SimpleNamespace(
      action=SimpleNamespace(desiredAcceleration=model_a, shouldStop=model_stop),
      meta=SimpleNamespace(disengagePredictions=SimpleNamespace(gasPressProbs=[0.9,0.9]))),
    "radarState": SimpleNamespace(
      leadOne=SimpleNamespace(status=lead_present), leadTwo=SimpleNamespace(status=False)),
  }
  return sm


class StagedPlannerTest(unittest.TestCase):
  def setUp(self):
    self.patch_mpc = patch.object(staged, "LongitudinalMpc", DeterministicMpc)
    self.patch_mpc.start()
    self.addCleanup(self.patch_mpc.stop)
    self.cp = SimpleNamespace(openpilotLongitudinalControl=True, longitudinalActuatorDelay=0.2,
                              steerRatio=15.0, wheelbase=2.96, vEgoStopping=0.05, brand="tesla")

  def test_big_model_deceleration_wins_in_experimental_mode(self):
    p = staged.LongitudinalPlanner(self.cp, init_v=15.0)
    p.update(make_inputs(exp=True, model_a=-1.2))
    self.assertLess(p.output_a_target, -1.0)
    self.assertEqual(p.mpc.source, log.LongitudinalPlan.LongitudinalPlanSource.e2e)
    self.assertEqual(p.mpc.update_kwargs["carstate"].followDistanceS, 3)

  def test_big_model_not_selected_in_non_experimental_mode(self):
    p = staged.LongitudinalPlanner(self.cp, init_v=15.0)
    p.update(make_inputs(exp=False, model_a=-1.2))
    self.assertGreater(p.output_a_target, -0.2)
    self.assertNotEqual(p.mpc.source, log.LongitudinalPlan.LongitudinalPlanSource.e2e)

  def test_stop_flag_propagates_in_experimental_mode(self):
    p = staged.LongitudinalPlanner(self.cp, init_v=15.0)
    p.update(make_inputs(exp=True, model_a=-1.2, model_stop=True))
    self.assertTrue(p.output_should_stop)

  def test_e2e_does_not_own_normal_cruise(self):
    p = staged.LongitudinalPlanner(self.cp, init_v=15.0)
    p.update(make_inputs(exp=False, model_a=-1.0))
    self.assertNotEqual(p.mpc.source, log.LongitudinalPlan.LongitudinalPlanSource.e2e)


if __name__ == "__main__":
  unittest.main(verbosity=2)
