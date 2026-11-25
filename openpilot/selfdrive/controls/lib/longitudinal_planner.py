#!/usr/bin/env python3
import math
import os
import numpy as np

import openpilot.cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan, should_stop
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
J_CRUISE_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MIN = -1.2
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# XNOR 0.11.1 V230 Dynamic Experimental Control compatibility.
_DEC_SLOW_DOWN_BP_KPH = [0., 10., 20., 30., 40., 50., 55., 60.]
_DEC_SLOW_DOWN_DIST_M = [32., 46., 64., 86., 108., 130., 145., 165.]
_DEC_ENTER_URGENCY = 0.24
_DEC_EXIT_URGENCY = 0.12
_DEC_CLEAR_FRAMES = 10
_DEC_STANDSTILL_FRAMES = 3

# XNOR 0.11.1 V232 visual-stop release assist.
_RELEASE_STOP_MIN_FRAMES = 5
_RELEASE_BOOST_FRAMES = 16
_RELEASE_BOOST_FLOOR_MS2 = 0.45
_RELEASE_MAX_EGO_MS = 2.5
_RELEASE_MIN_MODEL_3S_MS = 1.0
_RELEASE_MIN_MODEL_GAIN_MS = 0.5
_RELEASE_MIN_CRUISE_MARGIN_MS = 1.0


class XnorDynamicExperimentalControl:
  """0.11.1 XNOR DEC behavior adapted to the 0.11.2 planner candidates."""

  def __init__(self, CP):
    self.CP = CP
    self.mode = "acc"
    self.reason = "init"
    self.urgency = 0.0
    self.endpoint_x = float("inf")
    self.expected_distance = 0.0
    self.trajectory_valid = False
    self.standstill_count = 0
    self.clear_count = 0

  def enabled(self):
    forced = os.path.exists("/data/xnor_enable_dec")
    disabled = os.path.exists("/data/xnor_disable_dec")
    return (forced or getattr(self.CP, "brand", "") == "tesla") and not disabled

  @staticmethod
  def _model_slowdown_urgency(model_msg, v_ego_kph):
    endpoint_x = float("inf")
    expected_distance = float(np.interp(v_ego_kph, _DEC_SLOW_DOWN_BP_KPH, _DEC_SLOW_DOWN_DIST_M))
    trajectory_valid = len(model_msg.position.x) == ModelConstants.IDX_N
    if not trajectory_valid or expected_distance <= 0.1:
      return 0.0, endpoint_x, expected_distance, trajectory_valid

    endpoint_x = float(model_msg.position.x[-1])
    urgency = 0.0
    if endpoint_x < expected_distance:
      shortage_ratio = max(0.0, (expected_distance - endpoint_x) / expected_distance)
      urgency = min(1.0, shortage_ratio * 2.0)
      if endpoint_x < expected_distance * 0.3:
        urgency = min(1.0, urgency * 2.0)
      if v_ego_kph > 25.0:
        urgency = min(1.0, urgency * (1.0 + (v_ego_kph - 25.0) / 80.0))
    return urgency, endpoint_x, expected_distance, trajectory_valid

  def update(self, sm, fcw, e2e_should_stop):
    experimental = bool(sm["selfdriveState"].experimentalMode)
    active = experimental and self.enabled()

    if sm["carState"].standstill:
      self.standstill_count = min(20, self.standstill_count + 1)
    else:
      self.standstill_count = max(0, self.standstill_count - 1)

    raw_urgency, self.endpoint_x, self.expected_distance, self.trajectory_valid = self._model_slowdown_urgency(
      sm["modelV2"], float(sm["carState"].vEgo) * 3.6,
    )
    alpha = 0.45 if raw_urgency > self.urgency else 0.18
    self.urgency += alpha * (raw_urgency - self.urgency)

    if not active:
      self.mode = "blended" if experimental else "acc"
      self.reason = "dec_disabled" if experimental else "experimental_off"
      self.clear_count = 0
      return self.mode

    if bool(fcw):
      requested_mode, reason = "blended", "fcw"
    elif bool(e2e_should_stop):
      requested_mode, reason = "blended", "model_should_stop"
    elif self.standstill_count > _DEC_STANDSTILL_FRAMES:
      requested_mode, reason = "blended", "standstill"
    elif self.urgency >= _DEC_ENTER_URGENCY:
      requested_mode, reason = "blended", "model_slowdown"
    elif self.mode == "blended" and self.urgency > _DEC_EXIT_URGENCY:
      requested_mode, reason = "blended", "slowdown_hysteresis"
    else:
      requested_mode, reason = "acc", "cruise_recovery"

    if requested_mode == "acc" and self.mode == "blended":
      self.clear_count += 1
      if self.clear_count < _DEC_CLEAR_FRAMES:
        requested_mode, reason = "blended", "clear_hysteresis"
    else:
      self.clear_count = 0

    old_mode = self.mode
    self.mode = requested_mode
    self.reason = reason
    if self.mode != old_mode:
      lead = bool(sm["radarState"].leadOne.present)
      cloudlog.info(
        f"[XNOR_V230_DEC] transition={old_mode}->{self.mode} reason={self.reason} "
        + f"urgency={self.urgency:.3f} endpoint={self.endpoint_x:.1f} expected={self.expected_distance:.1f} "
        + f"ego_kph={float(sm['carState'].vEgo) * 3.6:.1f} lead={int(lead)}"
      )
    return self.mode


# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def get_cruise_accel(e2e, v_cruise, v_ego, a_cruise_prev, angle_steers, CP, dt, accel_coast, allow_throttle):
  max_accel = ACCEL_MAX if e2e else get_max_accel(v_ego)

  if not e2e:
    a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
    a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
    a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
    max_accel = min(max_accel, a_x_allowed)
    if not allow_throttle:
      clipped_accel_coast = max(accel_coast, ACCEL_MIN)
      coast_limit = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [max_accel, clipped_accel_coast])
      max_accel = min(max_accel, coast_limit)

  target_accel = np.clip(v_cruise - v_ego, A_CRUISE_MIN, max_accel)
  if not e2e:
    j_cruise = np.interp(v_ego, A_CRUISE_MAX_BP, J_CRUISE_VALS)
    target_accel = float(np.clip(target_accel, a_cruise_prev - j_cruise * dt, a_cruise_prev + j_cruise * dt))

  return target_accel


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.a_cruise = 0.0
    self.output_a_target = 0.0
    self.output_should_stop = False

    # Restore the 0.11.1 XNOR planner behavior that the 0.11.2 migration dropped.
    self._xnor_dec = XnorDynamicExperimentalControl(CP)
    self._xnor_dec_diag_frames = 0
    self._xnor_release_prev_e2e_stop = False
    self._xnor_release_stop_frames = 0
    self._xnor_release_boost_frames = 0
    self._xnor_release_diag_frames = 0
    self._xnor_v220_diag = (
      getattr(CP, "brand", "") == "tesla"
      and os.path.exists("/data/xnor_enable_long_owner_shadow_bench")
      and not os.path.exists("/data/xnor_disable_long_owner_shadow_bench")
    )
    self._xnor_v220_diag_frames = 0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
        len(model_msg.velocity.x) == ModelConstants.IDX_N and
        len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
    return x, v, a

  def _xnor_mpc_should_stop(self, action_t, v_ego, a_target):
    if getattr(self.CP, "brand", "") != "tesla" or not self.CP.openpilotLongitudinalControl:
      return should_stop(v_ego, a_target)

    deprecated = getattr(self.CP, "deprecated", None)
    threshold = float(getattr(deprecated, "vEgoStopping", 0.0) or 0.1) if deprecated is not None else 0.1
    if len(self.v_desired_trajectory) != len(CONTROL_N_T_IDX):
      return False
    v_target = float(np.interp(action_t, CONTROL_N_T_IDX, self.v_desired_trajectory))
    v_target_1sec = float(np.interp(action_t + 1.0, CONTROL_N_T_IDX, self.v_desired_trajectory))
    return bool(v_target < threshold and v_target_1sec < threshold)

  def update(self, sm):
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    force_slow_decel = bool(sm['controlsState'].forceDecel)
    if force_slow_decel:
      v_cruise = 0.0

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET
    reset_state = reset_state or not v_cruise_initialized

    throttle_probs = sm['modelV2'].meta.disengagePredictions.gasPressProbs
    throttle_prob = throttle_probs[1] if len(throttle_probs) > 1 else 1.0
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['vehicleParameters'].angleOffsetDeg

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, ACCEL_MIN, ACCEL_MAX)

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Save starting point for next iteration
    a_prev = self.a_desired

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                              action_t=action_t)
    output_should_stop_mpc = self._xnor_mpc_should_stop(action_t, v_ego, output_a_target_mpc)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    experimental_mode = bool(sm['selfdriveState'].experimentalMode)
    dec_mode = self._xnor_dec.update(sm, self.fcw, output_should_stop_e2e) if experimental_mode else "acc"

    self.a_cruise = get_cruise_accel(experimental_mode, v_cruise, v_ego,
                                     self.a_cruise, steer_angle_without_offset, self.CP, self.dt,
                                     accel_coast, self.allow_throttle)
    cruise_should_stop = should_stop(v_ego, self.a_cruise)

    # 0.11.1 DEC contract: in ACC mode, E2E must not veto normal cruise/lead
    # recovery. In blended mode (or when DEC is disabled), E2E participates.
    candidates = [(output_a_target_mpc, self.mpc.source, output_should_stop_mpc),
                  (self.a_cruise, LongitudinalPlanSource.cruise, cruise_should_stop)]
    if experimental_mode and (not self._xnor_dec.enabled() or dec_mode == "blended"):
      candidates.append((output_a_target_e2e, LongitudinalPlanSource.e2e, output_should_stop_e2e))

    output_a_target, self.mpc.source, _ = min(candidates, key=lambda c: c[0])
    self.output_should_stop = any(stop for _, _, stop in candidates)

    # V232 release assist from 0.11.1, bounded by BOTH 0.11.2 base candidates.
    model_x, model_v, _ = self.parse_model(sm['modelV2'])
    model_3s_v = float(np.interp(3.0, T_IDXS_MPC, model_v))
    prev_e2e_stop = bool(self._xnor_release_prev_e2e_stop)
    stop_dwell_frames = int(self._xnor_release_stop_frames)
    release_edge = bool(prev_e2e_stop and not bool(output_should_stop_e2e)
                        and stop_dwell_frames >= int(_RELEASE_STOP_MIN_FRAMES))

    if bool(output_should_stop_e2e):
      self._xnor_release_stop_frames = min(10000, stop_dwell_frames + 1)
    else:
      self._xnor_release_stop_frames = 0
    self._xnor_release_prev_e2e_stop = bool(output_should_stop_e2e)

    brake_pressed = bool(getattr(sm['carState'], 'brakePressed', False))
    gas_pressed = bool(getattr(sm['carState'], 'gasPressed', False))
    lead_present = bool(sm['radarState'].leadOne.present)
    release_forward = bool(model_3s_v >= max(float(_RELEASE_MIN_MODEL_3S_MS),
                                             float(v_ego) + float(_RELEASE_MIN_MODEL_GAIN_MS)))
    release_room = bool(float(v_cruise) >= float(v_ego) + float(_RELEASE_MIN_CRUISE_MARGIN_MS))

    if (release_edge and experimental_mode and self._xnor_dec.enabled()
        and float(v_ego) <= float(_RELEASE_MAX_EGO_MS)
        and not lead_present and not bool(self.fcw)
        and not brake_pressed and not gas_pressed and not force_slow_decel
        and not bool(output_should_stop_mpc) and release_forward and release_room):
      self._xnor_release_boost_frames = int(_RELEASE_BOOST_FRAMES)
      cloudlog.info(
        f"[XNOR_V232_RELEASE] edge=1 action=arm dwell={stop_dwell_frames} "
        + f"ego={v_ego:.2f} model3s_v={model_3s_v:.2f} cruise={v_cruise:.2f}"
      )

    release_cancel = bool(
      bool(output_should_stop_e2e) or bool(output_should_stop_mpc) or lead_present or bool(self.fcw)
      or brake_pressed or gas_pressed or force_slow_decel
      or float(v_ego) > float(_RELEASE_MAX_EGO_MS) or not release_room
    )
    if self._xnor_release_boost_frames > 0:
      if release_cancel:
        self._xnor_release_boost_frames = 0
      else:
        # Exact 0.11.1 V232 behavior: the short release floor is bounded by
        # MPC's already-computed positive request. The surrounding release gates
        # still require no lead/FCW/pedal/stop and forward model/cruise room.
        mpc_positive = max(0.0, float(output_a_target_mpc))
        release_floor = min(float(_RELEASE_BOOST_FLOOR_MS2), mpc_positive)
        if release_floor > float(output_a_target):
          output_a_target = float(release_floor)
        self._xnor_release_boost_frames -= 1

    self.output_a_target = np.clip(output_a_target, ACCEL_MIN, ACCEL_MAX)

    if experimental_mode and self._xnor_dec.enabled():
      self._xnor_dec_diag_frames += 1
      if self._xnor_dec_diag_frames % 20 == 0:
        cloudlog.info(
          f"[XNOR_V230_DEC] mode={self._xnor_dec.mode} reason={self._xnor_dec.reason} "
          + f"lead={int(lead_present)} ego={v_ego:.2f} cruiseCap={v_cruise:.2f} "
          + f"urgency={self._xnor_dec.urgency:.3f} endpoint={self._xnor_dec.endpoint_x:.1f} "
          + f"expected={self._xnor_dec.expected_distance:.1f} "
          + f"e2e_a={float(output_a_target_e2e):.3f} mpc_a={float(output_a_target_mpc):.3f} "
          + f"cruise_a={float(self.a_cruise):.3f} final_a={float(self.output_a_target):.3f}"
        )

    if self._xnor_v220_diag:
      self._xnor_v220_diag_frames += 1
      if self._xnor_v220_diag_frames % 20 == 0:
        preview_x = float(np.interp(3.0, T_IDXS_MPC, model_x))
        cloudlog.info(
          f"[XNOR_V220_PLANNER_SHADOW] experimental={int(experimental_mode)} "
          + f"lead={int(lead_present)} ego={v_ego:.2f} cruiseCap={v_cruise:.2f} "
          + f"model3s_x={preview_x:.2f} model3s_v={model_3s_v:.2f} "
          + f"e2e_a={float(output_a_target_e2e):.3f} mpc_a={float(output_a_target_mpc):.3f} "
          + f"final_a={float(self.output_a_target):.3f} "
          + f"e2e_stop={int(bool(output_should_stop_e2e))} mpc_stop={int(bool(output_should_stop_mpc))} "
          + f"final_stop={int(bool(self.output_should_stop))}"
        )

    self.a_desired = float(self.output_a_target)
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.output_a_target + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks()

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.present
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
