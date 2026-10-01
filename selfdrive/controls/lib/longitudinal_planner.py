#!/usr/bin/env python3
import math
import os
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog

A_CRUISE_MAX_VALS = [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# Lookup table for turns
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


# XNOR V230 DEC: compact port of sunnypilot Dynamic Experimental Control.
# Keep the existing XNOR MPC/E2E planners; only decide when the E2E candidate is allowed
# to participate in the final acceleration arbitration.
_DEC_SLOW_DOWN_BP_KPH = [0., 10., 20., 30., 40., 50., 55., 60.]
_DEC_SLOW_DOWN_DIST_M = [32., 46., 64., 86., 108., 130., 145., 165.]
_DEC_ENTER_URGENCY = 0.24
_DEC_EXIT_URGENCY = 0.12
_DEC_CLEAR_FRAMES = 10
_DEC_STANDSTILL_FRAMES = 3

# XNOR V232: gentle visual-stop release assist. This never exceeds the existing
# MPC acceleration request; it only prevents the lingering blended/E2E candidate
# from holding the first ~0.8 s of a verified no-lead release near zero.
_RELEASE_STOP_MIN_FRAMES = 5
_RELEASE_BOOST_FRAMES = 16
_RELEASE_BOOST_FLOOR_MS2 = 0.45
_RELEASE_MAX_EGO_MS = 2.5
_RELEASE_MIN_MODEL_3S_MS = 1.0
_RELEASE_MIN_MODEL_GAIN_MS = 0.5
_RELEASE_MIN_CRUISE_MARGIN_MS = 1.0


class XnorDynamicExperimentalControl:
  """Small compatibility port of sunnypilot DEC for the XNOR 0.11.x planner.

  ACC mode: the existing MPC owns acceleration/following/cruise recovery.
  Blended mode: the existing XNOR Experimental arbitration is retained, so the
  lower of E2E and MPC wins and E2E can stop for model-predicted controls.

  No Tesla sender, panda, cruise state, limits, AEB or stop/go ownership code is
  changed here.
  """

  def __init__(self, CP):
    self.CP = CP
    self.mode = "acc"
    self.reason = "init"
    self.urgency = 0.0
    self.endpoint_x = float('inf')
    self.expected_distance = 0.0
    self.trajectory_valid = False
    self.standstill_count = 0
    self.clear_count = 0
    self.frame = 0
    self._last_logged_mode = None

  def enabled(self):
    # Default-on for Tesla Experimental mode. A file switch provides an immediate
    # rollback to the previous min(E2E, MPC) behaviour without touching the sender.
    forced = os.path.exists('/data/xnor_enable_dec')
    disabled = os.path.exists('/data/xnor_disable_dec')
    return (forced or getattr(self.CP, 'brand', '') == 'tesla') and not disabled

  @staticmethod
  def _model_slowdown_urgency(model_msg, v_ego_kph):
    endpoint_x = float('inf')
    expected_distance = float(np.interp(v_ego_kph, _DEC_SLOW_DOWN_BP_KPH, _DEC_SLOW_DOWN_DIST_M))
    trajectory_valid = len(model_msg.position.x) == ModelConstants.IDX_N
    if not trajectory_valid or expected_distance <= 0.1:
      return 0.0, endpoint_x, expected_distance, trajectory_valid

    endpoint_x = float(model_msg.position.x[-1])
    urgency = 0.0
    if endpoint_x < expected_distance:
      shortage_ratio = max(0.0, (expected_distance - endpoint_x) / expected_distance)
      urgency = min(1.0, shortage_ratio * 2.0)
      if endpoint_x < (expected_distance * 0.3):
        urgency = min(1.0, urgency * 2.0)
      if v_ego_kph > 25.0:
        urgency = min(1.0, urgency * (1.0 + (v_ego_kph - 25.0) / 80.0))
    return urgency, endpoint_x, expected_distance, trajectory_valid

  def update(self, sm, fcw, e2e_should_stop):
    self.frame += 1
    experimental = bool(sm['selfdriveState'].experimentalMode)
    active = experimental and self.enabled()

    if sm['carState'].standstill:
      self.standstill_count = min(20, self.standstill_count + 1)
    else:
      self.standstill_count = max(0, self.standstill_count - 1)

    raw_urgency, self.endpoint_x, self.expected_distance, self.trajectory_valid = self._model_slowdown_urgency(
      sm['modelV2'], float(sm['carState'].vEgo) * 3.6)
    # Similar intent to sunnypilot's filtered slowdown signal, but use a compact
    # asymmetric low-pass: stops enter quickly, clear more slowly.
    alpha = 0.45 if raw_urgency > self.urgency else 0.18
    self.urgency += alpha * (raw_urgency - self.urgency)

    if not active:
      self.mode = "blended" if experimental else "acc"
      self.reason = "dec_disabled" if experimental else "experimental_off"
      self.clear_count = 0
      return self.mode

    # Explicit model stop and FCW are immediate E2E/blended conditions.
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
      cloudlog.info(f'[XNOR_V230_DEC] transition={old_mode}->{self.mode} reason={self.reason} '
                    f'urgency={self.urgency:.3f} endpoint={self.endpoint_x:.1f} expected={self.expected_distance:.1f} '
                    f'ego_kph={float(sm["carState"].vEgo) * 3.6:.1f} lead={int(bool(sm["radarState"].leadOne.status))}')
    return self.mode

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False
    # V220: observational comparison of model acceleration vs MPC vs final.
    # Does not alter selection, stop/go, limits or the generated MPC solver.
    self._xnor_v220_diag = (getattr(CP, 'brand', '') == 'tesla'
                            and os.path.exists('/data/xnor_enable_long_owner_shadow_bench')
                            and not os.path.exists('/data/xnor_disable_long_owner_shadow_bench'))
    self._xnor_v220_diag_frames = 0
    # V230: DEC only changes planner arbitration. Tesla stop/go sender and panda are untouched.
    self._xnor_dec = XnorDynamicExperimentalControl(CP)
    self._xnor_dec_diag_frames = 0
    # V232 visual-stop release assist state.
    self._xnor_release_prev_e2e_stop = False
    self._xnor_release_stop_frames = 0
    self._xnor_release_boost_frames = 0
    self._xnor_release_diag_frames = 0

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
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality, carstate=sm['carState'])

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    experimental_mode = bool(sm['selfdriveState'].experimentalMode)
    dec_mode = self._xnor_dec.update(sm, self.fcw, output_should_stop_e2e) if experimental_mode else "acc"

    if experimental_mode and (not self._xnor_dec.enabled() or dec_mode == "blended"):
      # Original XNOR Experimental behaviour: the most conservative candidate wins.
      output_a_target = min(output_a_target_e2e, output_a_target_mpc)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc
      if output_a_target < output_a_target_mpc:
        self.mpc.source = LongitudinalPlanSource.e2e
    else:
      # DEC ACC mode: do not let a near-zero E2E acceleration veto normal cruise
      # recovery or lead following. MPC still obeys vCruise, leads, acceleration
      # limits and XNOR's existing curve/speed-limit target.
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc

    # V232: a sustained visual stop can clear one model cycle before DEC's blended
    # hysteresis has returned to ACC. On that falling edge, allow a small launch
    # floor for a short window, but NEVER above MPC's already-computed request.
    # The assist is deliberately restricted to a no-lead, brake-off, low-speed
    # release with a forward-expanding model trajectory. Any renewed stop, lead,
    # FCW or driver pedal input cancels it immediately.
    model_x, model_v, _, _, _ = self.parse_model(sm['modelV2'])
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
    lead_present = bool(sm['radarState'].leadOne.status)
    release_forward = bool(model_3s_v >= max(float(_RELEASE_MIN_MODEL_3S_MS),
                                             float(v_ego) + float(_RELEASE_MIN_MODEL_GAIN_MS)))
    release_room = bool(float(v_cruise) >= float(v_ego) + float(_RELEASE_MIN_CRUISE_MARGIN_MS))

    if (release_edge and experimental_mode and self._xnor_dec.enabled()
        and float(v_ego) <= float(_RELEASE_MAX_EGO_MS)
        and not lead_present and not bool(self.fcw)
        and not brake_pressed and not gas_pressed and not bool(force_slow_decel)
        and not bool(output_should_stop_mpc) and release_forward and release_room):
      self._xnor_release_boost_frames = int(_RELEASE_BOOST_FRAMES)
      cloudlog.info(f'[XNOR_V232_RELEASE] edge=1 action=arm dwell={stop_dwell_frames} '
                    f'ego={v_ego:.2f} model3s_v={model_3s_v:.2f} cruise={v_cruise:.2f} '
                    f'e2e_a={float(output_a_target_e2e):.3f} mpc_a={float(output_a_target_mpc):.3f}')

    release_cancel = bool(
      bool(output_should_stop_e2e) or bool(output_should_stop_mpc) or lead_present or bool(self.fcw)
      or brake_pressed or gas_pressed or bool(force_slow_decel)
      or float(v_ego) > float(_RELEASE_MAX_EGO_MS) or not release_room
    )
    if self._xnor_release_boost_frames > 0:
      if release_cancel:
        cloudlog.info(f'[XNOR_V232_RELEASE] action=cancel remaining={self._xnor_release_boost_frames} '
                      f'e2e_stop={int(bool(output_should_stop_e2e))} mpc_stop={int(bool(output_should_stop_mpc))} '
                      f'lead={int(lead_present)} brake={int(brake_pressed)} gas={int(gas_pressed)} fcw={int(bool(self.fcw))}')
        self._xnor_release_boost_frames = 0
      else:
        mpc_positive = max(0.0, float(output_a_target_mpc))
        release_floor = min(float(_RELEASE_BOOST_FLOOR_MS2), mpc_positive)
        if release_floor > float(output_a_target):
          output_a_target = float(release_floor)
          self._xnor_release_diag_frames += 1
          if self._xnor_release_diag_frames == 1 or self._xnor_release_diag_frames % 5 == 0:
            cloudlog.info(f'[XNOR_V232_RELEASE] action=boost remaining={self._xnor_release_boost_frames} '
                          f'ego={v_ego:.2f} model3s_v={model_3s_v:.2f} '
                          f'e2e_a={float(output_a_target_e2e):.3f} mpc_a={float(output_a_target_mpc):.3f} '
                          f'boosted_a={float(output_a_target):.3f}')
        self._xnor_release_boost_frames -= 1
    else:
      self._xnor_release_diag_frames = 0

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

    if experimental_mode and self._xnor_dec.enabled():
      self._xnor_dec_diag_frames += 1
      if self._xnor_dec_diag_frames % 20 == 0:
        cloudlog.info(f'[XNOR_V230_DEC] mode={self._xnor_dec.mode} reason={self._xnor_dec.reason} '
                      f'lead={int(bool(sm["radarState"].leadOne.status))} ego={v_ego:.2f} cruiseCap={v_cruise:.2f} '
                      f'urgency={self._xnor_dec.urgency:.3f} endpoint={self._xnor_dec.endpoint_x:.1f} '
                      f'expected={self._xnor_dec.expected_distance:.1f} '
                      f'e2e_a={float(output_a_target_e2e):.3f} mpc_a={float(output_a_target_mpc):.3f} '
                      f'final_a={float(self.output_a_target):.3f} e2e_stop={int(bool(output_should_stop_e2e))} '
                      f'mpc_stop={int(bool(output_should_stop_mpc))} final_stop={int(bool(self.output_should_stop))}')

    if self._xnor_v220_diag:
      self._xnor_v220_diag_frames += 1
      if self._xnor_v220_diag_frames % 20 == 0:
        model_x, model_v, _, _, _ = self.parse_model(sm['modelV2'])
        preview_x = float(np.interp(3.0, T_IDXS_MPC, model_x))
        preview_v = float(np.interp(3.0, T_IDXS_MPC, model_v))
        cloudlog.info(f'[XNOR_V220_PLANNER_SHADOW] experimental={int(bool(sm["selfdriveState"].experimentalMode))} '
                      f'lead={int(bool(sm["radarState"].leadOne.status))} '
                      f'ego={v_ego:.2f} cruiseCap={v_cruise:.2f} '
                      f'model3s_x={preview_x:.2f} model3s_v={preview_v:.2f} '
                      f'e2e_a={float(output_a_target_e2e):.3f} mpc_a={float(output_a_target_mpc):.3f} '
                      f'final_a={float(self.output_a_target):.3f} '
                      f'e2e_stop={int(bool(output_should_stop_e2e))} mpc_stop={int(bool(output_should_stop_mpc))} '
                      f'final_stop={int(bool(self.output_should_stop))} '
                      'mode=observer_only recognition=UNPROVEN')

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
