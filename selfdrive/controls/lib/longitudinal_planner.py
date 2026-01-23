#!/usr/bin/env python3
import math
import numpy as np

from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params
from cereal import log
import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from openpilot.common.swaglog import cloudlog

# Physics and MPC Constants
LON_MPC_STEP = 0.2
A_CRUISE_MIN = -4.0
A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]

# Vision curve detection thresholds (more conservative / less jumpy)
CURVE_PEAK_THRESHOLD = 0.0030
AREA_THRESHOLD = 0.020

# Curve speed target lateral accel (higher = slows less)
VISION_CURVE_TARGET_LAT_A = 2.6

def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """Prevents acceleration when lateral Gs are high"""
  a_total_max = interp(v_ego, [20., 40.], [1.5, 2.8])
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
  return [a_target[0], min(a_target[1], a_x_allowed)]

def _rate_limited_filter(f: FirstOrderFilter, new_x: float, rc_up: float, rc_down: float) -> float:
  # smaller rc => faster response
  if new_x < f.x:
    f.update_alpha(rc_down)
  else:
    f.update_alpha(rc_up)
  return f.update(new_x)

def _min_positive(*vals: float) -> float:
  v = [x for x in vals if x is not None and x > 0.1]
  return min(v) if len(v) else 0.0

class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0):
    self.CP = CP
    self.mpc = LongitudinalMpc()
    self.fcw = False

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 1.0, DT_MDL)
    self.v_model_error = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

    self.params = Params()
    self.param_read_counter = 0
    self.personality = log.LongitudinalPersonality.standard
    self.read_param()

    # Curve / mapd smoothing
    self.v_turn_filter = FirstOrderFilter(init_v, 1.0, DT_MDL)
    self.curve_detected = False

    self.mapd_active = False
    self.v_mapd_filter = FirstOrderFilter(init_v, 2.0, DT_MDL)

  def read_param(self):
    try:
      p = self.params.get('LongitudinalPersonality')
      self.personality = int(p) if p is not None else log.LongitudinalPersonality.standard
    except (ValueError, TypeError):
      self.personality = log.LongitudinalPersonality.standard

  def _vision_curve_speed(self, model_msg, model_error, v_ego) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]:
    if not (len(model_msg.position.x) == 33 and len(model_msg.velocity.x) == 33):
      z = np.zeros(len(T_IDXS_MPC))
      return z, z, z, z, False

    x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
    v_raw = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
    a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
    j = np.zeros(len(T_IDXS_MPC))

    # If mapd is active, don’t allow vision to suddenly demand huge drops
    v_floor = (v_ego - 0.5) if self.mapd_active else (v_ego - 1.5)
    v = np.maximum(v_raw, v_floor) - model_error

    # Curvature estimate
    raw_curv = np.abs(np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z)) / np.clip(v, 0.3, 100.0)
    curve_area = float(np.sum(raw_curv[:25]) * 0.2)
    max_curv_ahead = float(np.max(raw_curv[2:20]))

    detected = (curve_area > AREA_THRESHOLD) and (max_curv_ahead > CURVE_PEAK_THRESHOLD)
    if detected:
      # Target speed from lateral accel limit
      max_v_curve = math.sqrt(max(VISION_CURVE_TARGET_LAT_A / (max_curv_ahead + 1e-6), 0.0))

      # Smooth: react faster to decreasing targets, slower to increasing
      max_v_curve = _rate_limited_filter(self.v_turn_filter, max_v_curve, rc_up=2.5, rc_down=0.35)

      v = np.minimum(v, max_v_curve)

    return x, v, a, j, detected

  def update(self, sm):
    if self.param_read_counter % 50 == 0:
      self.read_param()
    self.param_read_counter += 1

    self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    v_cruise = sm['controlsState'].vCruise * CV.KPH_TO_MS

    # --- mapd integration (primary cruise clamp) ---
    self.mapd_active = False
    try:
      if sm.valid.get('mapdOut', False):
        mo = sm['mapdOut']
        v_mapd = _min_positive(mo.suggestedSpeed, mo.speedLimitSuggestedSpeed)
        if v_mapd > 0.1:
          v_mapd_f = _rate_limited_filter(self.v_mapd_filter, v_mapd, rc_up=4.0, rc_down=0.6)
          v_cruise = min(v_cruise, v_mapd_f)
          self.mapd_active = True
    except Exception:
      # don’t let mapd plumbing break planner
      self.mapd_active = False
    # ---------------------------------------------

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
    accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    # Vision curve as a fallback / secondary limiter
    x, v_model, a_model, j_model, curve = self._vision_curve_speed(sm['modelV2'], self.v_model_error, v_ego)
    self.curve_detected = curve

    self.mpc.set_weights(prev_accel_constraint, personality=self.personality)

    # Don’t force negative max accel; just prevent throttle during curve handling
    if self.curve_detected:
      max_accel = min(accel_limits_turns[1], 0.20)
      self.mpc.set_accel_limits(accel_limits_turns[0], max_accel)
    else:
      self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])

    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    # Final target: cruise (already mapd-clamped) plus model fallback
    v_target = np.maximum(v_model, v_cruise - 0.1)

    self.mpc.update(sm['carState'], sm['radarState'], v_cruise, x, v_target, a_model, j_model, personality=self.personality)

    # Solution interpolation
    self.v_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    a_prev = self.a_desired
    self.a_desired = float(np.interp(DT_MDL, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + DT_MDL * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    lp = plan_send.longitudinalPlan
    lp.modelMonoTime = sm.logMonoTime['modelV2']
    lp.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    lp.speeds = self.v_desired_trajectory.tolist()
    lp.accels = self.a_desired_trajectory.tolist()
    lp.jerks = self.j_desired_trajectory.tolist()

    lp.hasLead = sm['radarState'].leadOne.status
    lp.longitudinalPlanSource = self.mpc.source
    lp.fcw = self.fcw
    lp.solverExecutionTime = self.mpc.solve_time
    lp.personality = self.personality

    pm.send('longitudinalPlan', plan_send)
