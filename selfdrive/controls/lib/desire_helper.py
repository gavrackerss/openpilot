from __future__ import annotations

import os
import time

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.realtime import DT_MDL


LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection


LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.0

_TINKLA_PARAM_POLL_S = 1.0


DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self) -> None:
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none

    # Unity ALC parity (tap indicator -> auto start after delay)
    self._params = Params()
    self._tinkla_enable_alc = False
    self._tinkla_alc_delay_s = 2.0
    self._last_param_poll_t = 0.0
    self._pre_lane_change_start_t: float | None = None
    self._v209_native_alc_owns_lane_changes = False
    self._v211_hybrid_native_ap = False

  def _poll_tinkla_params(self) -> None:
    now = time.monotonic()
    if now - self._last_param_poll_t < _TINKLA_PARAM_POLL_S:
      return

    self._last_param_poll_t = now
    try:
      self._tinkla_enable_alc = self._params.get_bool("TinklaEnableALC")
    except UnknownKeyName:
      self._tinkla_enable_alc = False

    try:
      raw = self._params.get("TinklaAlcDelay")
      if raw is None:
        raw = self._params.get("TinklaALCDelay")
      if isinstance(raw, (bytes, bytearray)):
        raw = raw.decode("utf-8", errors="ignore")
      self._tinkla_alc_delay_s = float(raw) if raw not in (None, "") else 2.0
    except (UnknownKeyName, ValueError, TypeError):
      self._tinkla_alc_delay_s = 2.0

    # clamp for safety
    self._tinkla_alc_delay_s = max(0.0, min(self._tinkla_alc_delay_s, 10.0))

    # V211: select the actual lateral owner before permitting an independent OP
    # lane-change trajectory. This does not make native state 4 into availability.
    try:
      self._v211_hybrid_native_ap = (self._params.get_bool('TinklaHybridNativeAP') and
                                     not self._params.get_bool('TinklaAutopilotDisabled'))
    except (UnknownKeyName, OSError):
      self._v211_hybrid_native_ap = False

    # Opt-in native ALC uses the genuine Tesla trajectory; don't simultaneously
    # ask the OP model for an independent lane change. Default OFF and no impact
    # on standalone OP or on V208's existing lane-change path.
    try:
      self._v209_native_alc_owns_lane_changes = (
        self._params.get_bool('TinklaHybridNativeAP') and
        not self._params.get_bool('TinklaAutopilotDisabled') and
        os.path.exists('/data/xnor_enable_native_alc_bridge'))
    except (UnknownKeyName, OSError):
      self._v209_native_alc_owns_lane_changes = False

  @staticmethod
  def _v211_op_lane_change_allowed(hybrid_native_ap: bool, native_lkas: bool) -> bool:
    # An active native type-1 0x488 is a live lane-keeping controller even when
    # Tesla ALC is state 4/unavailable. The independent OP trajectory is valid
    # only on the existing idle native carrier / OP-only steering path.
    return not (bool(hybrid_native_ap) and bool(native_lkas))

  def update(self, carstate, lateral_active: bool, lane_change_prob: float) -> None:
    self._poll_tinkla_params()

    v_ego = carstate.vEgo
    physical_one_blinker = carstate.leftBlinker != carstate.rightBlinker
    native_lkas = bool(getattr(carstate, 'stockLkas', False))
    op_lane_change_allowed = self._v211_op_lane_change_allowed(self._v211_hybrid_native_ap, native_lkas)
    one_blinker = bool(physical_one_blinker and op_lane_change_allowed)
    if not op_lane_change_allowed and self.lane_change_state != LaneChangeState.off:
      # Genuine native steering took ownership mid-request: cancel OP's model
      # desire instead of generating a second physical lane-change trajectory.
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self._pre_lane_change_start_t = None
      self.lane_change_timer = 0.0
      self.lane_change_ll_prob = 1.0
    # V210: do not suppress OP's independent lane-change planner merely because
    # the optional native indicator bridge is configured. Tesla can report ALC
    # unavailable (state 4) on ordinary multi-lane roads. Only genuine native
    # ALC-in-progress (9/10) may take steering ownership downstream; the panda
    # tracking guard remains in force otherwise. No full-detent/availability spoof.
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self._pre_lane_change_start_t = None
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self._pre_lane_change_start_t = time.monotonic()

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and                          ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        auto_start = False
        if self._tinkla_enable_alc and self._pre_lane_change_start_t is not None:
          auto_start = (time.monotonic() - self._pre_lane_change_start_t) >= self._tinkla_alc_delay_s

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
          self._pre_lane_change_start_t = None
        elif (torque_applied or auto_start) and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self._pre_lane_change_start_t = None

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
            self._pre_lane_change_start_t = time.monotonic()
          else:
            self.lane_change_state = LaneChangeState.off
            self._pre_lane_change_start_t = None

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    # A held/latching indicator is not a fresh OP request after native LKAS
    # releases. Require physical off -> on before another independent change.
    self.prev_one_blinker = physical_one_blinker
    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeState.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
