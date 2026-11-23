#!/usr/bin/env python3
"""Parked/offline tests for staged, NOT LIVE, comma v0.11.2 lead-only MPC.

The Acados native solver must already exist to import the module, but is
never instantiated: a recorder replaces it. This validates obstacle
selection and Tesla follow-distance behaviour, NOT numerical convergence.
"""
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.controls.lib.v0112_migration.long_mpc import (
  COST_DIM, LongitudinalMpc, LongitudinalPlanSource,
  N, PARAM_DIM, T_IDXS, get_t_follow_from_carstate,
)


class RecordingSolver:
  def __init__(self):
    self.set_calls = []

  def set(self, *args):
    self.set_calls.append(args)


def make_lead(status, distance=35.0, speed=9.0, accel=0.0):
  return SimpleNamespace(
    status=status, dRel=distance, vLead=speed,
    aLeadK=accel, aLeadTau=1.5, modelProb=0.9)


def initialized_mpc():
  mpc = object.__new__(LongitudinalMpc)  # No native solver allocation
  mpc.x0 = np.array([0.0, 18.0, 0.0])
  mpc.a_prev = np.zeros(N+1)
  mpc.params = np.zeros((N+1, PARAM_DIM))
  mpc.yref = np.zeros((N+1, COST_DIM))
  mpc.x_sol = np.zeros((N+1, 3))
  mpc.solver = RecordingSolver()
  mpc.run = lambda: None
  mpc.crash_cnt = 0
  mpc.source = LongitudinalPlanSource.cruise
  mpc.status = False
  return mpc


class LeadOnlyMPCTest(unittest.TestCase):
  def test_tesla_gap_setting_preserved(self):
    self.assertAlmostEqual(get_t_follow_from_carstate(SimpleNamespace(followDistanceS=3)), 1.3)
    self.assertAlmostEqual(get_t_follow_from_carstate(SimpleNamespace(followDistanceS=0)), 0.7)

  def test_lead_only_obstacles_and_source(self):
    mpc = initialized_mpc()
    radar = SimpleNamespace(
      leadOne=make_lead(True, 22.0, 8.0),
      leadTwo=make_lead(False))
    mpc.update(radar, carstate=SimpleNamespace(followDistanceS=3))
    self.assertTrue(mpc.status)
    self.assertEqual(mpc.source, LongitudinalPlanSource.lead0)
    self.assertAlmostEqual(mpc.params[0,4], 1.3)
    expected0 = mpc.process_lead(radar.leadOne)
    expected1 = mpc.process_lead(radar.leadTwo)
    from openpilot.selfdrive.controls.lib.v0112_migration.long_mpc import get_stopped_equivalence_factor
    obstacle0 = expected0[:,0] + get_stopped_equivalence_factor(expected0[:,1])
    obstacle1 = expected1[:,0] + get_stopped_equivalence_factor(expected1[:,1])
    np.testing.assert_allclose(mpc.params[:,2], np.minimum(obstacle0, obstacle1))
    self.assertEqual(mpc.params[:,2].shape, (N+1,))

  def test_low_confidence_legacy_status_is_not_assumed_present(self):
    mpc = initialized_mpc()
    radar = SimpleNamespace(leadOne=make_lead(False),leadTwo=make_lead(False))
    mpc.update(radar)
    self.assertFalse(mpc.status)

  def test_new_present_schema_works(self):
    mpc = initialized_mpc()
    radar = SimpleNamespace(
      leadOne=SimpleNamespace(present=True, dRel=26.0, vLead=10.0,
                              aLeadK=0.0, aLeadTau=1.5, modelProb=0.9),
      leadTwo=SimpleNamespace(present=False))
    mpc.update(radar)
    self.assertTrue(mpc.status)

  def test_no_cruise_obstacle_code_in_staged_mpc(self):
    from openpilot.selfdrive.controls.lib.v0112_migration import long_mpc
    src = Path(long_mpc.__file__).read_text()
    self.assertNotIn("cruise_obstacle", src)
    self.assertIn("x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])", src)


if __name__ == "__main__":
  unittest.main(verbosity=2)
