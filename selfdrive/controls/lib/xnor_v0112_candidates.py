"""Port of comma openpilot v0.11.2 cruise and arbitration calculations.

NOT CONNECTED TO LIVE PLANNING. Staged for the full migration of lead-only MPC
and the three-candidate planner. No direct vehicle commands are produced.
See V0112_MIGRATION_STATUS.md.

0.11.2 upstream source:
  openpilot/selfdrive/controls/lib/longitudinal_planner.py @ v0.11.2

This adaptation receives the acceleration bounds explicitly to stay independent
of whether the Tesla branch uses legacy or moved opendbc constants.
"""
import math

import numpy as np


A_CRUISE_MAX_VALS = (1.6, 1.2, 0.8, 0.6)
A_CRUISE_MAX_BP = (0.0, 10.0, 25.0, 40.0)
J_CRUISE_VALS = (1.6, 1.2, 0.8, 0.6)
A_CRUISE_MIN = -0.5
_A_TOTAL_MAX_V = (1.7, 3.2)
_A_TOTAL_MAX_BP = (20.0, 40.0)
MIN_ALLOW_THROTTLE_SPEED = 2.5


def get_cruise_accel_0112(
    e2e: bool, v_cruise: float, v_ego: float, a_cruise_prev: float,
    angle_steers: float, CP, dt: float, accel_coast: float, allow_throttle: bool,
    accel_min: float, accel_max: float,
) -> float:
  """Official 0.11.2 cruise candidate; bounds supplied by caller."""
  max_accel = accel_max if e2e else float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))

  if not e2e:
    a_total_max = float(np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V))
    a_y = v_ego ** 2 * angle_steers * math.pi / 180.0 / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.0))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_min)
      coast_limit = float(np.interp(v_ego,
                                  [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2.0],
                                  [max_accel, clipped_accel_coast]))
      max_accel = min(max_accel, coast_limit)

  target_accel = float(np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel))
  j_cruise = float(np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS))
  target_accel = float(np.clip(target_accel,
                               a_cruise_prev - j_cruise * dt,
                               a_cruise_prev + j_cruise * dt))
  return target_accel


def select_longitudinal_candidates_0112(
    mpc_accel, mpc_source, mpc_should_stop,
    cruise_accel, cruise_source, cruise_should_stop,
    e2e_accel, e2e_source, e2e_should_stop,
    experimental_mode: bool,
):
  """Minimum requested acceleration and OR'd stop intents, as in 0.11.2.

  WARNING: requires a lead-only MPC to reproduce upstream semantics.
  The legacy XNOR MPC includes cruise, so this function must not be wired
  into live planning before the matching MPC migration and replay validation.
  """
  candidates = [
    (float(mpc_accel), mpc_source, bool(mpc_should_stop)),
    (float(cruise_accel), cruise_source, bool(cruise_should_stop)),
  ]
  if experimental_mode:
    candidates.append((float(e2e_accel), e2e_source, bool(e2e_should_stop)))
  chosen_accel, chosen_source, _ = min(candidates, key=lambda c: c[0])
  return chosen_accel, chosen_source, any(stop for _, _, stop in candidates)
