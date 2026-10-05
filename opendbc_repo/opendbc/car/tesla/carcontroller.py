# /data/openpilot/opendbc/car/tesla/carcontroller.py
"""Tesla CarController (xnor C3)

Stable steering + Unity-parity virtual stalk for cruise speed-limit matching.

Native TACC note: this full-file replacement intentionally contains no below-floor
DAS_setSpeed/climb test code. Low-speed TACC authority is now handled in
LONG_module by native-TACC set-speed passthrough.

What this file does (only two things):
  1) publishes internal 0x659 (fake DAS) on bus 0 and bus 4 for panda safety (existing xnor behavior)
  2) when enabled + Tesla cruise is engaged, nudges Tesla cruise SET speed toward map speed limit
     by emitting STW_ACTN_RQ (cruise stalk up/down), using TeslaCAN.create_action_request() (CRC+counter).

It does *not* change steering behavior or ALC behavior.

V229 (XNOR_V229_UNITY_STOPGO_OWNER): Unity-style stop-and-go. On HW2 legacy cars in Hybrid or
Autopilot-Disabled+EnableACC mode, openpilot is the single DAS_control author on both buses
(main 0x2B9 / external 0x2BF). Userspace sends TEMPLATES; panda substitutes them onto the genuine
AP frames. While disengaged a gentle Unity "priming" frame (ACC_ON, 0 kph) is kept on the bus so a
stalk pull arms the DI from 0 mph without a lead. Opt out: touch /data/xnor_disable_op_stopgo.
"""

from __future__ import annotations

import math
import os
import numpy as np
import time

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.modules.LONG_module import LongController

from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_std_steer_angle_limits

from opendbc.car.tesla.teslacan import TeslaCAN
try:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANLegacy as TeslaCANLegacy
except ImportError:
  from opendbc.car.tesla.teslacan_legacy import TeslaCANRaven as TeslaCANLegacy

from opendbc.car.tesla.values import CarControllerParams, CANBUS, LEGACY_CARS, CAR

try:
  from opendbc.car.tesla.teslacan import create_fake_das_msg as create_fake_das
except ImportError:
  from opendbc.car.tesla.teslacan import create_fake_das_message as create_fake_das


# SpdCtrlLvr_Stat (STW_ACTN_RQ)
BTN_IDLE = 0
BTN_CANCEL = 1
BTN_MAIN = 2
BTN_UP2 = 4
BTN_DOWN2 = 8
BTN_UP1 = 16
BTN_DOWN1 = 32

# --- Low-speed DI-arming bisection toggles (bench only) ---------------------------------------
# Arming is TWO independent mechanisms; we cannot yet prove which (if either) disrupts the EPAS,
# so each is switchable here. Edit these two lines and restart to bisect on the bench:
#   Test A (baseline, no arming): both False   -> steering should be rock-solid
#   Test B (0x2B9 only):          CHASSIS True,  STALK False
#   Test C (stalk only, bounded): CHASSIS False, STALK True
#   Test D (both, the target):    both True
# Module-level constants on purpose: no param registration / no UI / no rebuild of the params lib.
_ARM_ENABLE_CHASSIS = False  # IMPORTANT: no unsolicited chassis 0x2B9; it inhibits EPAS on this vehicle
_ARM_ENABLE_STALK = False    # OFF (isolate GTW MITM test)
# Unity-style stop/go: author a chassis 0x2B9 TEMPLATE only. Panda captures it, blocks direct TX,
# and overlays it onto the genuine AP 0x2B9 as that frame crosses bus2->bus0, preserving stock
# cadence/counter. This is intentionally different from _ARM_ENABLE_CHASSIS (unsolicited direct TX).
_UNITY_2B9_OVERLAY_TEMPLATE = True

# --- GTW_carConfig autopilot=2 native TX on bus 2 (config-unlock experiment, bench only) --------
# Transmit a full GTW_carConfig (0x398) frame with GTW_autopilot=2 NATIVELY on the AP module's
# own segment (bus 2 = CANBUS.autopilot_party), at ~1Hz (matching the real GTW cadence). Prior
# runs only edited the FORWARDED copy (bus130/134); this puts autopilot=2 on the bus the AP module
# actually reads. Fixed payload verified stable on this car: 6987474215320020 (byte7 0x00->0x20,
# all other config bytes identical, no checksum/counter on 0x398). Set False to disable.
# WATCH: does AutopilotStatus(0x399) climb UNAVAILABLE(1) -> AVAILABLE(2)? If yes, config is read
# live and AP/sub-18 is reachable. If it stays UNAVAILABLE with ap=2 native on bus2 => NVRAM-gated.
_GTW_TX_AUTOPILOT2_BUS2 = True
_GTW_CARCONFIG_AP2_PAYLOAD = bytes.fromhex("6987474215320020")

ROADWORKS_CAP_FILE = "/data/xnor_roadworks_speed_cap_kph.txt"
ROADWORKS_PRESET_FILE = "/data/xnor_roadworks_speed_cap_preset_kph.txt"
ROADWORKS_DEFAULT_KPH = 50.0 * CV.MPH_TO_KPH

# --- V229: Unity-style stop-and-go longitudinal owner (tesla_legacy.h XNOR_V229_UNITY_STOPGO_OWNER)
# Drive-latched. Default ON for HW2 Hybrid / Autopilot-Disabled+EnableACC; opt out with this file.
_V229_STOPGO_DISABLE_FILE = "/data/xnor_disable_op_stopgo"
_V229_STOPGO_CARS = (CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_X_HW2)
_V229_ACCEL_TO_SPEED_S = 3.0                # Unity ACCEL_TO_SPEED_MULTIPLIER: setSpeed = vEgo + 3 * accel
_V229_SET_SPEED_MAX_KPH = 200.0             # Unity clip
_V229_ACTIVE_JERK = (-8.0, 8.0)             # Unity JERK_LIMIT_MIN/MAX
_V229_PRIME_ACCEL = (-1.4, 1.8)             # Unity: "send this values so we can enable at 0 km/h"
_V229_PRIME_JERK = (-0.46, 0.476)
_V229_ACC_STATE_CANCEL_GENERIC = 0
_V229_ACC_STATE_ON = 4
_V229_ACC_STATE_CANCEL_SILENT = 13
# DI engaged by the stalk before OP enables (same stalk edge). Only after this grace, or if OP was
# enabled during this DI engagement and has since disengaged, do we cancel an unowned DI cruise.
_V229_UNOWNED_DI_GRACE_FRAMES = 50          # 0.5 s at 100 Hz (panda backstop cancels at 1.5 s)
_V229_DI_ENGAGED_STATES = ("ENABLED", "STANDSTILL", "OVERRIDE")


def _v226_powertrain_tx_allowed(hybrid_native_ap: bool, exclusive_hybrid_selected: bool,
                                owner_request: bool) -> bool:
  """Only a qualified Hybrid owner may author powertrain DAS_control.

  When exclusive Hybrid is selected, the real AP is the *sole* standby sender.
  Sending OP ACC_ON while disengaged competes with native ACC_OFF and can poison
  the next native cruise MAIN transition. Non-Hybrid and legacy unselected modes
  retain their previous sender lifecycle; the panda remains the final safety gate.
  """
  return not (hybrid_native_ap and exclusive_hybrid_selected) or bool(owner_request)


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, VM=None):
    try:
      super().__init__(dbc_names, CP, VM)
    except TypeError:
      super().__init__(dbc_names, CP)

    self.CP = CP
    self.frame = 0

    self.params = Params()
    self._long_module = LongController()
    self._cached_autopilot_disabled = False
    # Hybrid mode is latched at CarController startup. UI changes apply next drive/restart so
    # CarParams, panda safety ownership, and longitudinal authority cannot change mid-drive.
    self._cached_hybrid_native_ap = bool(self.params.get_bool("TinklaHybridNativeAP")) and not bool(self.params.get_bool("TinklaAutopilotDisabled"))
    # V221: the HIL request is latched at startup and cannot follow a touch change mid-drive.
    # It has no vehicle effect with stock V221 firmware: firmware cutover is compile-disabled.
    self._v221_hil_selected = (os.path.exists('/data/xnor_enable_v221_hil_owner') and
                               not os.path.exists('/data/xnor_disable_v221_hil_owner'))
    self._cached_autosteer_247_test = bool(self.params.get_bool("TinklaAutosteer247Test"))
    # V229: Unity-style stop-and-go owner, latched for the drive like Hybrid.
    _v229_ap_disabled_init = bool(self.params.get_bool("TinklaAutopilotDisabled"))
    _v229_enable_acc_init = bool(self.params.get_bool("TinklaEnableACC"))
    self._v229_stopgo_selected = bool(
      CP.carFingerprint in _V229_STOPGO_CARS
      and CP.openpilotLongitudinalControl
      and (self._cached_hybrid_native_ap or (_v229_ap_disabled_init and _v229_enable_acc_init))
      and not os.path.exists(_V229_STOPGO_DISABLE_FILE)
    )
    if self._v229_stopgo_selected:
      # Exactly one longitudinal ownership mechanism per drive: V229 replaces the V221 HIL owner.
      self._v221_hil_selected = False
    self._v229_owner_now = False
    self._v229_owner_last_sent = None
    self._v229_di_engaged_since_frame = -1
    self._v229_op_enabled_during_di = False
    self._v229_mode = "init"
    self._v229_last_diag_frame = -100000
    # V241: Unity-style brake auto-resume must re-arm the real Tesla cruise state,
    # not merely clear controlsd's longitudinal brake latch. Track one bounded
    # RES_ACCEL sequence per real brake cycle while OP remains engaged.
    self._v241_autoresume_brake_seen = False
    self._v241_autoresume_attempts = 0
    self._v241_autoresume_last_attempt_frame = -100000
    self._v241_autoresume_failed_reported = False
    self._cached_auto_resume_acc = bool(self.params.get_bool("TinklaAutoResumeACC"))
    self._cached_experimental_mode = bool(self.params.get_bool("ExperimentalMode"))
    # V231: Unity-style native cruise SET synchronisation while V229 owns
    # DAS_control. This is deliberately independent of acceleration authority.
    self._v231_setsync_last_button_ms = 0
    self._v231_setsync_last_target_ms = None
    cloudlog.info(f'[XNOR_V229_STOPGO] selected={int(self._v229_stopgo_selected)} '
                  f'hybrid={int(self._cached_hybrid_native_ap)} ap_disabled={int(_v229_ap_disabled_init)} '
                  f'enable_acc={int(_v229_enable_acc_init)} '
                  f'opt_out_file={int(os.path.exists(_V229_STOPGO_DISABLE_FILE))}')
    self._cached_pedal_enabled = False
    self._cached_adjust_acc_with_speed_limit = False
    self._cached_speed_limit_offset_uom = 0.0
    self._cached_speed_limit_use_relative = False
    self._params_last_read_frame = -100000

    self._op659_prev_btn = 0
    self.apply_angle_last = 0.0
    self._lat_active_prev = False
    self._steer_warmup_until_frame = -1
    # V186 Hybrid EPAS recovery: arm only after genuine native Autosteer was active and
    # subsequently drops while OP remains engaged. This avoids the V174/V175 regression where
    # OP sent APS_eacMonitor during normal Hybrid pre-engagement and disturbed Tesla's native
    # Autosteer handshake.
    self._hybrid_native_lkas_prev = False
    # V219: distinguish a genuinely native-Autosteer-supervised session from
    # the original OP-only co-op case. Hybrid configuration alone is NOT native LKAS.
    self._hybrid_native_lkas_seen_this_engagement = False
    self._hybrid_eac_recovery = False
    # V199 co-op re-arm is independent of native-LKAS state. A physical hands-on takeover can
    # leave the real EPAS in EAC_AVAILABLE/HANDS_ON before native 0x488 falls. Hold a controller
    # latch long enough for panda's state-driven neutral/allow sequence to observe real recovery.
    self._hybrid_coop_rearm = False
    self._hybrid_coop_healthy_frames = 0
    self._hybrid_coop_last_physical_hands_frame = -1
    self._hybrid_coop_failed = False
    # V214: V207's brake cancellation remains persistent even during a gas override.
    # This mirrors the controlsd latch; only its accepted CC.longActive rearm clears it.
    # Never use a pedal override as permission to rearm a brake-cancelled session.
    self._hybrid_setspeed_brake_cancelled = False
    self._hybrid_drop_frame = -1
    self._hybrid_drop_warmup_until_frame = -1
    self._hybrid_alc_phase = "IDLE"
    self._hybrid_alc_last_diag_frame = -100000
    self._v210_native_alc_prev_active = False
    self._v210_native_alc_handback_until_frame = -1
    self._v211_alc_epas_abort = False
    self._v211_alc_route = 'IDLE'
    self._v211_alc_last_notice_frame = -100000
    # V212: observe actual native release and sustained REAL EPAS acceptance before
    # granting an OP ALC trajectory. This does not request or spoof native release.
    self._v212_epas_good_frames = 0
    self._v212_native_idle_good_frames = 0
    self._v212_handover_state = 'IDLE'
    self._v212_handover_since_frame = -1
    self._v212_pending_direction = 0
    self._v212_prev_physical_turn = 0
    # V213: separate opt-in off-highway ALC *configuration*, latched for the drive.
    # This modifies only the AP-facing genuine 0x3E8 configuration copy; no road-class spoof.
    self._v213_offhighway_alc_enable = bool(self._cached_hybrid_native_ap and
                                             os.path.exists("/data/xnor_enable_offhighway_alc_bench"))
    # V216: native AP ACC-from-zero capability REQUEST, independently opt-in and
    # latched at controller init. This is not proof of native ECU acceptance.
    # Disable-file override allows a rapid rollback without editing any CAN code.
    self._v216_acc_from_zero_enable = bool(
      self._cached_hybrid_native_ap
      and os.path.exists("/data/xnor_enable_acc_from_zero_bench")
      and not os.path.exists("/data/xnor_disable_acc_from_zero_bench")
    )
    # V217: separate AP Always On configuration REQUEST, never a virtual engage.
    # Latched at controller init and requires explicit enable without disable override.
    self._v217_autopilot_always_on_enable = bool(
      self._cached_hybrid_native_ap
      and os.path.exists("/data/xnor_enable_autopilot_always_on_bench")
      and not os.path.exists("/data/xnor_disable_autopilot_always_on_bench")
    )
    # V209 experimental AP-facing indicator hold. OFF unless enabled on the bench.
    # The DBC gives direction but not half/full detent; only native AP can accept ALC.
    self._native_alc_hold_direction = 0
    self._native_alc_hold_since_frame = -1
    self._native_alc_tap_previous = 0
    self._native_alc_progress_seen = False
    self._speed_sync_last_frame = -100000
    # Unity-parity pacing for automated cruise stalk presses
    self._human_cruise_action_time_ms = 0
    self._automated_cruise_action_time_ms = 0
    self._prev_cruise_buttons = BTN_IDLE


    self._stw_seed = None
    self._stw_seed_bus = int(CANBUS.party)
    self._stw_last_send_frame = -100000
    self._stw_release_frame = -1
    self._stw_release_bus = int(CANBUS.party)
    # Low-speed DI-arming stalk pulse: strictly bounded, one-shot-per-engagement burst.
    self._arm_pulse_last_frame = -1000   # last frame a pulse was emitted (spacing)
    self._arm_pulse_count = 0            # pulses emitted in the current engagement cycle
    self._arm_engage_frame = -1000       # frame long_active last went true (window start)
    self._arm_long_active_prev = False   # edge detector for long_active
    self._stw_sequence = []  # list[(frame:int, btn:int)]
    self._op_enabled_prev = False
    self._xnor_diag_last_log_ms = 0
    self._body_controls_prev_turn = 0
    self._virtual_turn_prev = 0
    self._virtual_turn_last_send_frame = -100000
    self._hud_prev_enabled = False
    self._telemetry_prev_active = False

    self._roadworks_main_pulls_ms: list[int] = []
    self._roadworks_toggle_latch_until_ms = 0

    if CP.carFingerprint in LEGACY_CARS:
      if CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.packers = {
        CANBUS.party: CANPacker(dbc_names[Bus.party]),
        CANBUS.powertrain: CANPacker(dbc_names[Bus.pt]),
      }
      self.tesla_can = TeslaCANLegacy(self.packers)

      # STW_ACTN_RQ needs CRC/counter; legacy helper doesn't implement it.
      self._action_can_by_bus = {int(bus): TeslaCAN(pkr) for bus, pkr in self.packers.items()}
      self._body_controls_can = self._action_can_by_bus[int(CANBUS.party)]
    else:
      self.packer = CANPacker(dbc_names[Bus.party])
      self.tesla_can = TeslaCAN(CP, self.packer)
      self._action_can_by_bus = {int(CANBUS.party): self.tesla_can}
      self._body_controls_can = self.tesla_can

  def _refresh_cached_params(self) -> None:
    if (self.frame - self._params_last_read_frame) < 50:
      return
    self._params_last_read_frame = int(self.frame)

    self._cached_autopilot_disabled = bool(self.params.get_bool("TinklaAutopilotDisabled"))
    # XNOR native-ACC (sub-17 TACC): when enabled, openpilot authors its own longitudinal
    # via DAS_control even in the autopilot_disabled ("Unity"/lateral-only) mode, with no
    # stock-cruise dependency and no 17.1 mph engage floor. Cached here so the longitudinal
    # gate below can keep long_active true in that mode.
    self._cached_enable_acc = bool(self.params.get_bool("TinklaEnableACC"))
    self._cached_auto_resume_acc = bool(self.params.get_bool("TinklaAutoResumeACC"))
    self._cached_experimental_mode = bool(self.params.get_bool("ExperimentalMode"))
    self._cached_autosteer_247_test = bool(self.params.get_bool("TinklaAutosteer247Test"))
    self._cached_pedal_enabled = bool(
      self.params.get_bool("TinklaPedalEnabled") or
      self.params.get_bool("PedalEnabled")
    )
    self._cached_adjust_acc_with_speed_limit = bool(self.params.get_bool("TinklaAdjustAccWithSpeedLimit"))
    self._cached_speed_limit_use_relative = bool(self.params.get_bool("TinklaSpeedLimitUseRelative"))
    try:
      self._cached_speed_limit_offset_uom = float(self.params.get("TinklaSpeedLimitOffset", encoding="utf-8") or "0")
    except Exception:
      self._cached_speed_limit_offset_uom = 0.0


  @staticmethod
  def _now_ms() -> int:
    return int(time.monotonic_ns() // 1_000_000)


  def _read_roadworks_file_float(self, path: str) -> float | None:
    try:
      with open(path, "r", encoding="utf-8") as f:
        raw = str(f.read()).strip()
    except OSError:
      return None

    if not raw:
      return None

    try:
      value = float(raw)
    except Exception:
      return None

    return float(value) if np.isfinite(value) and float(value) > 0.1 else None

  def _write_roadworks_file_float(self, path: str, value: float) -> None:
    tmp_path = f"{path}.tmp"
    with open(tmp_path, "w", encoding="utf-8") as f:
      f.write(f"{float(value):.3f}")
    os.replace(tmp_path, path)

  def _clear_roadworks_file(self, path: str) -> None:
    try:
      os.remove(path)
    except OSError:
      pass

  def _roadworks_cap_current_kph(self) -> float | None:
    return self._read_roadworks_file_float(ROADWORKS_CAP_FILE)

  def _roadworks_cap_preset_kph(self) -> float:
    preset = self._read_roadworks_file_float(ROADWORKS_PRESET_FILE)
    if preset is not None:
      return float(preset)
    return float(ROADWORKS_DEFAULT_KPH)

  def _toggle_roadworks_cap(self) -> None:
    current_kph = self._roadworks_cap_current_kph()
    if current_kph is None:
      preset_kph = self._roadworks_cap_preset_kph()
      self._write_roadworks_file_float(ROADWORKS_CAP_FILE, float(preset_kph))
      cloudlog.info(f"[XNOR_RW_CAP] enabled cap_kph={preset_kph:.3f}")
    else:
      self._clear_roadworks_file(ROADWORKS_CAP_FILE)
      cloudlog.info(f"[XNOR_RW_CAP] cleared previous_cap_kph={current_kph:.3f}")

  def _maybe_handle_roadworks_triple_pull(self, CS) -> None:
    now_ms = int(self._now_ms())
    if int(now_ms) < int(self._roadworks_toggle_latch_until_ms):
      return

    btn = int(getattr(CS, "cruise_buttons", BTN_IDLE) or BTN_IDLE)
    prev_btn = int(getattr(self, "_prev_cruise_buttons", BTN_IDLE) or BTN_IDLE)
    main_edge = (btn == BTN_MAIN) and (prev_btn != BTN_MAIN)
    if not main_edge:
      return

    recent = [int(ts) for ts in self._roadworks_main_pulls_ms if (int(now_ms) - int(ts)) <= 1800]
    recent.append(int(now_ms))
    self._roadworks_main_pulls_ms = recent[-3:]

    if len(self._roadworks_main_pulls_ms) >= 3:
      span_ms = int(self._roadworks_main_pulls_ms[-1]) - int(self._roadworks_main_pulls_ms[-3])
      if span_ms <= 1800:
        self._toggle_roadworks_cap()
        self._roadworks_main_pulls_ms = []
        self._roadworks_toggle_latch_until_ms = int(now_ms) + 1800

  def _track_human_cruise_actions(self, CS) -> None:
    btn = int(getattr(CS, 'cruise_buttons', BTN_IDLE) or BTN_IDLE)
    prev_btn = int(getattr(self, '_prev_cruise_buttons', BTN_IDLE) or BTN_IDLE)
    self._maybe_handle_roadworks_triple_pull(CS)
    # Unity: throttle automation on any *physical* button other than MAIN/IDLE.
    # V231: do not mistake our own short SET/RES echo for a human action, otherwise
    # the first automatic detent suppresses the remaining speed-limit sync pulses.
    now_ms = int(self._now_ms())
    virtual_btn = int(getattr(CS, '_xnor_last_virtual_btn', BTN_IDLE) or BTN_IDLE)
    virtual_ms = int(getattr(CS, '_xnor_last_virtual_ms', 0) or 0)
    virtual_echo = btn == virtual_btn and 0 <= now_ms - virtual_ms <= 250
    if (not virtual_echo and btn not in (BTN_MAIN, BTN_IDLE)) and (btn != prev_btn):
      self._human_cruise_action_time_ms = now_ms
    self._prev_cruise_buttons = btn

  @staticmethod
  def _v211_lane_change_route(*, native_alc_active: bool, native_lkas: bool,
                              op_request: bool, epas_healthy: bool,
                              coop_rearm: bool, coop_failed: bool) -> str:
    # Route OP's lane change only after genuine native steering has released.
    # A Tesla state-4/unavailable report alone does NOT release native 0x488.
    if native_alc_active and native_lkas and epas_healthy:
      return 'NATIVE_ALC'
    if op_request:
      if native_lkas:
        return 'NATIVE_LKAS_BLOCK'
      if coop_failed or coop_rearm or not epas_healthy:
        return 'WAIT_EPAS'
      return 'OP_IDLE_CARRIER'
    return 'IDLE'

  @staticmethod
  def _v212_handover_ready(*, native_lkas: bool, epas_healthy: bool,
                            epas_good_frames: int, native_idle_good_frames: int,
                            coop_rearm: bool, coop_failed: bool,
                            steer_inhibit: bool) -> bool:
    """Authorize only the ALREADY-EXISTING idle-carrier OP path.

    A type-1 native steering request cannot be converted to OP authority by
    Tesla's ALC-unavailable status. This predicate deliberately has no control
    output, no native-release command, and no change to panda's tracking guard.
    """
    return bool(not native_lkas and epas_healthy and
                int(epas_good_frames) >= 25 and int(native_idle_good_frames) >= 5 and
                not coop_rearm and not coop_failed and not steer_inhibit)

  def _native_alc_virtual_hold(self, CC, CS) -> int:
    """Request only a bounded AP-facing indicator HOLD; never authorize ALC/EPAS.

    Native state 4 (exiting-highway unavailable) is intentionally NOT overridden.
    The input is the genuine tap detected by BLNK, not our own virtual stalk.
    The feature is OFF unless the bench-only enable file exists. Firmware's
    fwd_msg overlays the genuine 0x45 AP-facing frame; synthetic TX alone cannot
    be assumed to reach the AP ECU. Always obey direct physical stalk changes.
    """
    enabled = (self._cached_hybrid_native_ap and not self._cached_autopilot_disabled
               and os.path.exists('/data/xnor_enable_native_alc_bridge'))
    tap = int(getattr(CS, 'tap_direction', 0) or 0)
    physical = int(getattr(CS, 'turnSignalStalkState', 0) or 0)
    state = int(getattr(CS, '_native_alc_state', 31))
    valid = bool(getattr(CS, '_native_alc_valid', False))
    out = getattr(CS, 'out', None)
    left_blind = bool(getattr(out, 'leftBlindspot', False))
    right_blind = bool(getattr(out, 'rightBlindspot', False))
    healthy = (bool(getattr(out, 'cruiseState', None) and out.cruiseState.enabled)
               and bool(getattr(out, 'stockLkas', False))
               and bool(getattr(CC, 'latActive', False))
               and int(getattr(CS, 'eac_status_raw', -1)) == 2
               and int(getattr(CS, 'eac_error_code_raw', -1)) == 0
               and not bool(getattr(out, 'brakePressed', False))
               and not bool(getattr(out, 'steerFaultTemporary', False))
               and not bool(getattr(out, 'steerFaultPermanent', False)))
    prior = self._native_alc_hold_direction
    direction_available = (valid and ((state in (6, 8) and tap == 1) or
                                      (state in (7, 8) and tap == 2)))
    fresh_tap_edge = tap in (1, 2) and tap != self._native_alc_tap_previous
    self._native_alc_tap_previous = tap
    if not enabled or not healthy or physical not in (0, prior) or (prior == 1 and left_blind) or (prior == 2 and right_blind):
      self._native_alc_hold_direction = 0
    elif not prior and fresh_tap_edge and physical == 0 and direction_available and not (left_blind if tap == 1 else right_blind):
      self._native_alc_hold_direction = tap
      self._native_alc_hold_since_frame = int(self.frame)
      self._native_alc_progress_seen = False
      cloudlog.info(f'[XNOR_V209_ALC] tap={tap} native={state} AP_fwd_request=ARMED')
    elif prior:
      if state == (9 if prior == 1 else 10):
        self._native_alc_progress_seen = True
      # Keep the genuine stalk request only while native AP is still accepting it.
      # 9/10 indicate native in progress; 28 is a hands-on request, NOT permission
      # to spoof hands or continue without driver acknowledgement.
      valid_state = state in ((6, 8, 9, 28) if prior == 1 else (7, 8, 10, 28))
      opposite_input = physical in (1, 2) and physical != prior
      if (not valid or not valid_state or opposite_input or
          (self._native_alc_progress_seen and state != (9 if prior == 1 else 10)) or
          int(self.frame) - self._native_alc_hold_since_frame > 700 or
          (state == 28 and int(self.frame) - self._native_alc_hold_since_frame > 200)):
        self._native_alc_hold_direction = 0
    if prior != self._native_alc_hold_direction:
      cloudlog.info(f'[XNOR_V209_ALC] bridge={self._native_alc_hold_direction} native={state} prior={prior}')
    return int(self._native_alc_hold_direction)

  def _emit_internal_0x659(self, CS, can_sends, *, native_alc_turn: int = 0, v221_hil_request: bool = False,
                           v229_stopgo_owner: bool = False) -> None:
    stalk_btn = int(getattr(CS, "cruise_buttons", 0) or 0)
    prev_btn = int(self._op659_prev_btn)

    main_edge = (stalk_btn == BTN_MAIN) and (prev_btn != BTN_MAIN)
    cancel_edge = (stalk_btn == BTN_CANCEL) and (prev_btn != BTN_CANCEL)

    self._op659_prev_btn = stalk_btn

    native_alc_mode = bool(self._cached_hybrid_native_ap and not self._cached_autopilot_disabled and
                           os.path.exists('/data/xnor_enable_native_alc_bridge'))
    offhighway_alc = bool(self._v213_offhighway_alc_enable)
    acc_from_zero = bool(self._v216_acc_from_zero_enable)
    autopilot_always_on = bool(self._v217_autopilot_always_on_enable)
    alc_signal = ((4 if native_alc_mode else 0) | (8 if offhighway_alc else 0) |
                  (16 if acc_from_zero else 0) | (32 if autopilot_always_on else 0) |
                  (64 if v221_hil_request else 0) | int(native_alc_turn))
    v229_changed = self._v229_owner_last_sent is None or bool(v229_stopgo_owner) != bool(self._v229_owner_last_sent)
    if (self.frame % 10 == 0) or main_edge or cancel_edge or v229_changed or \
       alc_signal != int(getattr(self, '_native_alc_last_sent', 0)):
      self._native_alc_last_sent = int(alc_signal)
      self._v229_owner_last_sent = bool(v229_stopgo_owner)
      buses = {int(CANBUS.party)}
      if self.CP.carFingerprint in LEGACY_CARS:
        buses.add(int(CANBUS.powertrain))
      for bus in sorted(buses):
        can_sends.append(create_fake_das(
          self._cached_pedal_enabled,
          self._cached_autopilot_disabled,
          bus=bus,
          stalk_main=main_edge,
          stalk_cancel=cancel_edge,
          hybrid_native_ap=(self._cached_hybrid_native_ap and not self._cached_autopilot_disabled),
          autosteer_247_test=self._cached_autosteer_247_test,
          native_alc_turn=int(native_alc_turn) if int(bus) == int(CANBUS.party) else 0,
          native_alc_mode=bool(native_alc_mode and int(bus) == int(CANBUS.party)),
          offhighway_alc_enable=bool(offhighway_alc and int(bus) == int(CANBUS.party)),
          acc_from_zero_enable=bool(acc_from_zero and int(bus) == int(CANBUS.party)),
          autopilot_always_on_enable=bool(autopilot_always_on and int(bus) == int(CANBUS.party)),
          v221_hil_owner_request=bool(v221_hil_request),
          v229_stopgo_owner=bool(v229_stopgo_owner),
        ))

  def _speed_limit_target_ms(self, CS) -> float:
    # Prefer CarState's helper (uses DAS fused if present + supports relative offset)
    try:
      return float(CS._calc_speed_limit_target_ms(str(getattr(CS, "speed_units", "MPH"))))
    except Exception:
      pass

    limit_ms = float(getattr(CS, "speed_limit_ms_das", 0.0) or getattr(CS, "speed_limit_ms", 0.0) or 0.0)
    if limit_ms <= 0.0:
      return 0.0

    off = float(self._cached_speed_limit_offset_uom)
    if self._cached_speed_limit_use_relative:
      return max(0.0, limit_ms * (1.0 + off / 100.0))

    uom = str(getattr(CS, "speed_units", "MPH"))
    return max(0.0, limit_ms + (off * (CV.KPH_TO_MS if uom == "KPH" else CV.MPH_TO_MS)))

  def _stw_bus(self, CS) -> int:
    try:
      b = int(getattr(CS, "stw_actn_bus", CANBUS.party))
      # Never transmit on CANBUS.radar (bus 1); some harnesses have no ACK on that bus.
      return int(CANBUS.party) if b == int(CANBUS.radar) else int(b)
    except Exception:
      return int(CANBUS.party)

  def _action_can_for_bus(self, bus: int):
    return (
      self._action_can_by_bus.get(int(bus)) or
      self._action_can_by_bus.get(int(CANBUS.party)) or
      next(iter(self._action_can_by_bus.values()))
    )
  def _send_stw(self, CS, can_sends, btn: int, *, bus: int | None = None, turn_signal_stalk_state: int | None = None) -> bool:
    msg = getattr(CS, "msg_stw_actn_req", None)
    if msg is None:
      return False

    b = int(bus if bus is not None else self._stw_bus(CS))
    seed = dict(msg)  # Unity parity: seed from latest observed frame every send

    can_sends.append(
      self._action_can_for_bus(b).create_stalk_request(
        int(b),
        seed,
        cruise_button=int(btn),
        turn_signal_stalk_state=(None if turn_signal_stalk_state is None else int(turn_signal_stalk_state)),
      )
    )
    # Mark last virtual stalk press so CarState can ignore it for adaptive double-pull detection.
    try:
      now_ms = int(self._now_ms())
      if int(btn) != int(BTN_IDLE):
        CS._xnor_last_virtual_btn = int(btn)
        CS._xnor_last_virtual_ms = now_ms
      if turn_signal_stalk_state is not None:
        CS._xnor_last_virtual_turn = int(turn_signal_stalk_state)
        CS._xnor_last_virtual_turn_ms = now_ms
    except Exception:
      pass
    self._stw_seed_bus = int(b)
    self._stw_last_send_frame = int(self.frame)
    return True

  def _queue_stalk_pulse(self, CS, can_sends, btn: int) -> bool:
    # Unity-like pulse: press now, release next frame.
    if int(self._stw_release_frame) > int(self.frame):
      return False

    if not self._send_stw(CS, can_sends, btn):
      return False

    self._stw_release_frame = int(self.frame) + 1
    self._stw_release_bus = int(self._stw_seed_bus)
    return True

  def _v241_auto_resume_stalk(self, CC, CS, can_sends, *, native_lkas: bool) -> bool:
    """Re-arm native Tesla TACC after V234 clears the brake-only LONG latch.

    Unity's Auto Resume ACC path sends RES_ACCEL once cruise is back in STANDBY.
    V234 only restored CC.longActive, which leaves DI/TACC in STANDBY after a real
    brake press. Keep the V229 DAS_control owner unchanged and emit a bounded
    Unity-style RES pulse sequence only for a brake cycle observed while OP stayed
    engaged. Returns True on a frame where a RES pulse was queued so SET-sync does
    not compete for the same stalk frame.
    """
    if not (self._v229_stopgo_selected and self._cached_hybrid_native_ap):
      self._v241_autoresume_brake_seen = False
      self._v241_autoresume_attempts = 0
      self._v241_autoresume_failed_reported = False
      return False

    cs_out = getattr(CS, "out", None)
    enabled = bool(getattr(CC, "enabled", False))
    long_active = bool(getattr(CC, "longActive", False))
    brake = bool(getattr(cs_out, "brakePressed", False)) if cs_out is not None else False
    regen = bool(getattr(cs_out, "regenBraking", False)) if cs_out is not None else False
    gas = bool(getattr(cs_out, "gasPressed", False)) if cs_out is not None else False
    stock_state = str(getattr(CS, "stock_cruise_state", "") or "").upper()

    if not enabled:
      self._v241_autoresume_brake_seen = False
      self._v241_autoresume_attempts = 0
      self._v241_autoresume_failed_reported = False
      return False

    if brake:
      self._v241_autoresume_brake_seen = True
      self._v241_autoresume_attempts = 0
      self._v241_autoresume_last_attempt_frame = -100000
      self._v241_autoresume_failed_reported = False
      return False

    if stock_state in _V229_DI_ENGAGED_STATES:
      if self._v241_autoresume_brake_seen and self._v241_autoresume_attempts > 0:
        cloudlog.info(f'[XNOR_V241_AUTORESUME_STALK] action=di_rearmed state={stock_state} attempts={self._v241_autoresume_attempts}')
      self._v241_autoresume_brake_seen = False
      self._v241_autoresume_attempts = 0
      self._v241_autoresume_failed_reported = False
      return False

    if not self._v241_autoresume_brake_seen:
      return False

    # controlsd V234 owns the one-second brake/regen guard. Its longActive rising
    # state is the permission to attempt the native re-arm; do not duplicate that timer here.
    eligible = bool(
      self._cached_auto_resume_acc
      and self._cached_experimental_mode
      and long_active
      and stock_state == "STANDBY"
      and not brake and not regen and not gas
      and not native_lkas
      and not bool(getattr(CS, "human_control", False))
      and int(getattr(CS, "cruise_buttons", BTN_IDLE) or BTN_IDLE) == BTN_IDLE
      and int(getattr(self, "_virtual_turn_prev", 0) or 0) == 0
    )
    if not eligible:
      return False

    # Bounded Unity-style retry: one press/release pulse every 0.5 s, maximum three.
    # A successfully re-armed DI clears this latch above on the next observed state.
    if self._v241_autoresume_attempts >= 3:
      if (not self._v241_autoresume_failed_reported and
          int(self.frame) - int(self._v241_autoresume_last_attempt_frame) >= 50):
        cloudlog.warning('[XNOR_V241_AUTORESUME_STALK] action=failed state=STANDBY attempts=3')
        self._v241_autoresume_failed_reported = True
        self._v241_autoresume_brake_seen = False
      return False

    if int(self.frame) - int(self._v241_autoresume_last_attempt_frame) < 50:
      return False

    if self._queue_stalk_pulse(CS, can_sends, BTN_UP1):
      self._v241_autoresume_attempts += 1
      self._v241_autoresume_last_attempt_frame = int(self.frame)
      self._automated_cruise_action_time_ms = int(self._now_ms())
      cloudlog.info(f'[XNOR_V241_AUTORESUME_STALK] action=res_accel attempt={self._v241_autoresume_attempts} state={stock_state}')
      return True

    return False


  def _process_stalk_actions(self, CS, can_sends) -> None:
    hold_turn = 0
    if self.CP.carFingerprint in LEGACY_CARS:
      # patch141 fixed non-ALC taps by tightening the owned blinker lifecycle in CarState.
      # For the physical HW2 hold, use the planner-owned ALC direction directly rather than
      # the derived engaged/done booleans, which can clear too early on this base.
      if int(getattr(CS, "turnSignalStalkState", 0) or 0) == 0:
        turn = int(getattr(CS, "alca_direction", 0) or 0)
        if turn in (1, 2):
          hold_turn = turn

    # Release pending cruise pulse, preserving any active virtual turn hold.
    if int(self._stw_release_frame) == int(self.frame):
      self._send_stw(
        CS,
        can_sends,
        BTN_IDLE,
        bus=int(self._stw_release_bus),
        turn_signal_stalk_state=(hold_turn if hold_turn in (1, 2) else None),
      )
      self._stw_release_frame = -1

    # Legacy HW2 virtual ALC turn hold must mimic a physically-held Tesla stalk, not the
    # lamp's flash period. Unity simply reads the real STW_ACTN_RQ at ~10 Hz and lets the
    # BCM own the normal on/off indicator cadence. Refresh our synthetic TurnIndLvr_Stat at
    # that same ~10 Hz cadence; re-triggering only every 0.5 s can visibly slow the lamps.
    # Send one explicit release when virtual ownership ends.
    if self.CP.carFingerprint in LEGACY_CARS:
      prev_hold_turn = int(getattr(self, "_virtual_turn_prev", 0) or 0)
      last_send_frame = int(getattr(self, "_virtual_turn_last_send_frame", -100000) or -100000)

      send_turn = None
      if hold_turn in (1, 2):
        # Native STW_ACTN_RQ is ~10 Hz on this HW2 wiring. Holding the lever state at
        # the message cadence lets Tesla's BCM generate the standard lamp sequence.
        if hold_turn != prev_hold_turn or (int(self.frame) - last_send_frame) >= 10:
          send_turn = int(hold_turn)
      elif prev_hold_turn in (1, 2):
        # Release immediately when the owned lane change ends.
        send_turn = 0

      if send_turn is not None:
        self._send_stw(
          CS,
          can_sends,
          BTN_IDLE,
          bus=int(self._stw_bus(CS)),
          turn_signal_stalk_state=int(send_turn),
        )
        self._virtual_turn_last_send_frame = int(self.frame)

      self._virtual_turn_prev = int(hold_turn)



  def _ic_lane_model_state(self, CS) -> tuple[float, bool, bool, float, float, float, float, float, int, int]:
    """Unity modelV2 -> legacy IC virtual/fused lane mapping."""
    lane_width_m = 4.0
    lane_range_m = 50.0
    left_visible = False
    right_visible = False
    left_quality = 0
    right_quality = 0
    c0 = c1 = c2 = c3 = 0.0

    model = getattr(CS, "ic_model_data", None)
    if not isinstance(model, dict):
      return lane_width_m, left_visible, right_visible, lane_range_m, c0, c1, c2, c3, left_quality, right_quality

    try:
      probs = tuple(float(v) for v in model.get("lane_probs", ()))
      if len(probs) >= 4:
        # Unity HUD_module: inner lines determine visibility; outer lines provide quality/ALC availability.
        left_visible = probs[1] > 0.45
        right_visible = probs[2] > 0.45
        left_quality = 1 if probs[0] > 0.25 else 0
        right_quality = 1 if probs[3] > 0.25 else 0

      x = np.asarray(model.get("position_x", ()), dtype=float)
      y = np.asarray(model.get("position_y", ()), dtype=float)
      n = min(int(x.size), int(y.size))
      if n >= 4:
        x = x[:n]
        y = y[:n]
        valid = np.isfinite(x) & np.isfinite(y) & (x >= 0.0) & (x <= 100.0)
        x_fit = x[valid]
        y_fit = y[valid]
        if x_fit.size >= 4:
          coefs = np.polyfit(x_fit, y_fit, 3)
          # Unity IC_LANE_SCALE = 0.5, so the rendered path coefficients are scaled 2x.
          # Unity also intentionally suppresses C1 for the IC representation.
          scale = 2.0
          c0 = float(np.clip(coefs[3], -3.5, 3.5))
          c1 = 0.0
          c2 = float(np.clip(coefs[1] * scale * scale, -0.0025, 0.0025))
          c3 = float(np.clip(coefs[0] * scale * scale * scale, -0.00003, 0.00003))
    except (TypeError, ValueError, np.linalg.LinAlgError):
      pass

    return lane_width_m, left_visible, right_visible, lane_range_m, c0, c1, c2, c3, left_quality, right_quality

  def _hud_alca_state(self, CS) -> int:
    # Native AP1/MCU1 blue-lane captures do not require the Unity 6/7/8 lane-availability
    # states. Keep the normal no-ALC state unless an actual OP lane change is in progress.
    turn = int(getattr(CS, "alca_direction", 0) or 0)
    if bool(getattr(CS, "alca_pre_engage", False) or getattr(CS, "alca_engaged", False)) and turn in (1, 2):
      return 8 + turn
    return 1

  def _hud_speed_limit_uom(self, CS) -> float:
    limit_ms = float(getattr(CS, "speed_limit_ms", 0.0) or 0.0)
    units = str(getattr(CS, "speed_units", "MPH"))
    return max(0.0, limit_ms * (CV.MS_TO_KPH if units == "KPH" else CV.MS_TO_MPH))

  def _process_hud_status(self, CC, CS, can_sends, human_control: bool) -> None:
    """Publish full Unity-style HUD status for panda-side stock-frame overlay.

    The safety hook consumes these 0x399/0x389 frames and blocks direct TX. Their
    payload is merged onto the stock AP frames in the panda forward path, keeping
    stock timing/counters while clearing the false PMM/AEB HUD state.
    """
    if self.CP.carFingerprint not in LEGACY_CARS:
      return
    if not (hasattr(self.tesla_can, "create_das_status") and hasattr(self.tesla_can, "create_das_status2")):
      return

    op_enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    prev_enabled = bool(getattr(self, "_hud_prev_enabled", False))
    stock_ap_enabled = bool(getattr(CS, "autopilot_enabled", False))
    ap_disabled = bool(getattr(CS, "autopilot_disabled", False) or getattr(self, "_cached_autopilot_disabled", False))

    if stock_ap_enabled:
      self._hud_prev_enabled = op_enabled
      return

    should_send = op_enabled or prev_enabled or ap_disabled
    edge_send = prev_enabled and not op_enabled
    if not should_send:
      self._hud_prev_enabled = op_enabled
      return
    if not edge_send and (self.frame % 10 != 0):
      self._hud_prev_enabled = op_enabled
      return

    cs_out = getattr(CS, "out", None)
    cruise_state = getattr(cs_out, "cruiseState", None) if cs_out is not None else None

    speed_limit = float(self._hud_speed_limit_uom(CS))
    cruise_speed = speed_limit
    try:
      cruise_speed = max(0.0, float(getattr(cruise_state, "speed", 0.0) or 0.0) * CV.MS_TO_MPH)
    except (TypeError, ValueError):
      cruise_speed = speed_limit

    op_status = 3 if op_enabled else 2
    hands_on_state = 3 if bool(human_control) and op_enabled else 2
    alca_state = int(self._hud_alca_state(CS)) if op_enabled else 1
    blind_left = bool(getattr(cs_out, "leftBlindspot", False)) if cs_out is not None else False
    blind_right = bool(getattr(cs_out, "rightBlindspot", False)) if cs_out is not None else False
    fleet_state = int(getattr(CS, "fleet_speed_state", 0) or 0)

    counter = 1
    can_sends.append(self.tesla_can.create_das_status(
      counter,
      op_status,
      False,
      0,
      hands_on_state,
      alca_state,
      blind_left,
      blind_right,
      speed_limit,
      fleet_state,
    ))
    can_sends.append(self.tesla_can.create_das_status2(counter, cruise_speed, False))

    self._hud_prev_enabled = op_enabled


  def _telemetry_alca_state(self, CS) -> int:
    turn = int(getattr(CS, "alca_direction", 0) or 0)
    if bool(getattr(CS, "alca_pre_engage", False) or getattr(CS, "alca_engaged", False)) and turn in (1, 2):
      return turn
    return 0

  def _process_lane_telemetry(self, CC, CS, can_sends) -> None:
    """Native-parity IC lane overlay while OP is enabled.

    Dashcam/native-Autosteer captures on this car show blue lanes with DAS_autopilotState
    transitioning 2->3, no DAS_telemetry (0x3A9), and DAS_lanes (0x239) switching both
    LineUsage fields to FUSED with both Fork fields cleared. Submit only a bus-0 overlay
    payload; panda merges it onto the genuine AP 0x239 so stock timing and the rolling
    counter remain authoritative.
    """
    op_enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    stock_ap_enabled = bool(getattr(CS, "autopilot_enabled", False))
    lane_active = bool(op_enabled and not stock_ap_enabled)

    if not lane_active or (self.frame % 10 != 0):
      self._telemetry_prev_active = lane_active
      return

    (lane_width_m, left_visible, right_visible, lane_range_m,
     c0, c1, c2, c3, left_quality, right_quality) = self._ic_lane_model_state(CS)

    alca_active = bool(getattr(CS, "alca_engaged", False))
    lane_left_visible = bool(left_visible or alca_active)
    lane_right_visible = bool(right_visible or alca_active)

    # Counter is only a non-zero placeholder for the capture guard. Panda replaces the
    # high-nibble counter with the genuine AP 0x239 counter on the forwarded frame.
    can_sends.append(self._body_controls_can.create_lane_message(
      lane_width_m,
      lane_left_visible,
      lane_right_visible,
      lane_range_m,
      c0,
      c1,
      c2,
      c3,
      left_quality,
      right_quality,
      int(CANBUS.party),
      1,
    ))

    self._telemetry_prev_active = True

  def _hud_hands_on(self, CS) -> int:
    return 0



  def _lane_positioned_target_angle(self, desired_angle_deg: float, current_angle_deg: float, v_ego: float) -> float:
    desired_angle_deg = float(desired_angle_deg)
    current_angle_deg = float(current_angle_deg)
    v_ego = float(v_ego)

    desired_mag = abs(desired_angle_deg)
    if (desired_mag < 1.5) or (v_ego < 4.0):
      return desired_angle_deg

    assist_gain = float(np.interp(
      desired_mag,
      CarControllerParams.CURVE_ASSIST_ANGLE_BP,
      CarControllerParams.CURVE_ASSIST_GAIN_V,
    ))
    assist_extra = float(np.interp(
      desired_mag,
      CarControllerParams.CURVE_ASSIST_ANGLE_BP,
      CarControllerParams.CURVE_ASSIST_EXTRA_DEG_V,
    ))
    assist_speed_gain = float(np.interp(
      v_ego,
      CarControllerParams.CURVE_ASSIST_SPEED_BP,
      CarControllerParams.CURVE_ASSIST_SPEED_GAIN_V,
    ))
    max_delta = float(np.interp(
      v_ego,
      CarControllerParams.CURVE_ASSIST_MAX_DELTA_BP,
      CarControllerParams.CURVE_ASSIST_MAX_DELTA_V,
    ))

    assisted_angle = (desired_angle_deg * assist_gain) + (np.sign(desired_angle_deg) * assist_extra * assist_speed_gain)

    # Keep the lane-positioning assist as a modest bias around the planner request.
    return float(np.clip(
      assisted_angle,
      desired_angle_deg - max_delta,
      desired_angle_deg + max_delta,
    ))

  def _body_controls_turn(self, CS) -> int:
    if not bool(getattr(CS, "enableALC", False)):
      return 0

    cs_out = getattr(CS, "out", None)
    if cs_out is not None:
      left = bool(getattr(cs_out, "leftBlinker", False))
      right = bool(getattr(cs_out, "rightBlinker", False))
      if left != right:
        return 1 if left else 2

    turn = int(getattr(CS, "alca_direction", 0) or 0)
    return turn if turn in (1, 2) else 0

  def _process_body_controls(self, CS, can_sends) -> None:
    if self.CP.carFingerprint in LEGACY_CARS:
      self._body_controls_prev_turn = 0
      return

    turn = int(self._body_controls_turn(CS))
    prev_turn = int(getattr(self, "_body_controls_prev_turn", 0) or 0)

    # Unity HUD parity: body-controls indicator requests are refreshed continuously at 20 Hz
    # while a lane-change direction is owned, then sent once more with turn=0 to clear.
    if (self.frame % 5 == 0) and (turn in (1, 2) or prev_turn in (1, 2)):
      can_sends.append(
        self._body_controls_can.create_body_controls_message(
          turn,
          1 if bool(getattr(CS, "needs_hazard", False)) else 0,
          int(CANBUS.party),
          1,
        )
      )

    self._body_controls_prev_turn = turn

  def _diag_log(self, msg: str) -> None:
    now_ms = int(self._now_ms())
    if (now_ms - int(self._xnor_diag_last_log_ms)) < 1000:
      return
    self._xnor_diag_last_log_ms = int(now_ms)
    cloudlog.info(msg)

  def _speed_limit_sync(self, CC, CS, can_sends) -> None:
    enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))
    # V218 direct remains advisory-only. V229 is different: its Unity-style
    # DAS_control owner also needs the native Tesla SET value synchronised to the
    # stable posted speed target. Temporary lead/curve/E2E slowdowns remain direct
    # accel/decel only and never rewrite DI_cruiseSet.
    direct_policy = bool(getattr(CS, '_xnor_experimental_direct_active', False))
    if direct_policy or bool(self._v229_owner_now):
      policy_enabled = bool(getattr(CC, 'longActive', False)) and bool(enabled)
      cs_out = getattr(CS, 'out', None)
      brake = bool(getattr(cs_out, 'brakePressed', False))
      regen = bool(getattr(cs_out, 'regenBraking', False))
      if brake or regen:
        policy_enabled = False
      decision = self._long_module.update(CS, enabled=policy_enabled,
                                          frame=int(self.frame), now_ms=int(self._now_ms()))

      setsync_btn = None
      setsync_reason = 'inactive'
      if bool(self._v229_owner_now) and bool(getattr(CC, 'enabled', False)) and not brake and not regen:
        now_ms = int(self._now_ms())
        target_ms = getattr(self._long_module, 'v231_setsync_target_ms', None)
        target_time_ms = int(getattr(self._long_module, 'v231_setsync_mono_ms', 0) or 0)
        target_src = str(getattr(self._long_module, 'v231_setsync_source', 'none') or 'none')
        stock_state = str(getattr(CS, 'stock_cruise_state', '') or '').upper()
        current_set_ms = float(getattr(CS, 'stock_cruise_set_speed_ms', 0.0) or 0.0)
        speed_units = str(getattr(CS, 'speed_units', 'MPH') or 'MPH').upper()
        cruise_buttons = int(getattr(CS, 'cruise_buttons', BTN_IDLE) or BTN_IDLE)
        hold_turn = int(getattr(self, '_virtual_turn_prev', 0) or 0)
        human_quiet = (now_ms - int(self._human_cruise_action_time_ms)) >= 900
        cooldown_ok = (now_ms - int(self._v231_setsync_last_button_ms)) >= 400
        target_fresh = target_ms is not None and 0 <= now_ms - target_time_ms <= 700
        state_ok = stock_state in ('ENABLED', 'OVERRIDE')
        pulse_free = int(self._stw_release_frame) < 0 and hold_turn not in (1, 2)
        if target_fresh and state_ok and pulse_free and human_quiet and cooldown_ok and cruise_buttons == BTN_IDLE:
          target_ms = float(target_ms)
          if math.isfinite(target_ms) and target_ms > 0.1:
            ms_to_u = CV.MS_TO_MPH if speed_units == 'MPH' else CV.MS_TO_KPH
            target_u = max(0.0, target_ms * ms_to_u)
            current_u = max(0.0, current_set_ms * ms_to_u)
            tol_u = 0.7 if speed_units == 'MPH' else 1.0
            full_step_u = 5.0
            offset_u = target_u - current_u
            if offset_u >= (full_step_u - tol_u):
              setsync_btn = int(BTN_UP2)
            elif offset_u >= tol_u:
              setsync_btn = int(BTN_UP1)
            elif offset_u <= -(full_step_u - tol_u):
              setsync_btn = int(BTN_DOWN2)
            elif offset_u <= -tol_u:
              setsync_btn = int(BTN_DOWN1)
            else:
              setsync_reason = f'no-op src={target_src} tgt={target_u:.1f} cur={current_u:.1f}'
            if setsync_btn is not None:
              if self._queue_stalk_pulse(CS, can_sends, int(setsync_btn)):
                self._v231_setsync_last_button_ms = now_ms
                self._v231_setsync_last_target_ms = target_ms
                self._automated_cruise_action_time_ms = now_ms
                setsync_reason = f'pulse src={target_src} tgt={target_u:.1f} cur={current_u:.1f} btn={int(setsync_btn)}'
                cloudlog.info(f'[XNOR_V231_SETSYNC] {setsync_reason}')
              else:
                setsync_reason = 'queue_blocked'
        elif bool(self._v229_owner_now):
          setsync_reason = (f'gate fresh={int(bool(target_fresh))} state={stock_state or "UNKNOWN"} '
                            f'pulse_free={int(bool(pulse_free))} human_quiet={int(bool(human_quiet))} '
                            f'cooldown={int(bool(cooldown_ok))} physical_btn={cruise_buttons}')

      if self.frame % 100 == 0:
        cloudlog.info(f'[XNOR_V218_DIRECT] v229={int(bool(self._v229_owner_now))} ccEnabled={int(bool(getattr(CC,"enabled",False)))} latActive={int(bool(getattr(CC,"latActive",False)))} longActive={int(bool(getattr(CC,"longActive",False)))} '
                      f'policy={getattr(self._long_module,"op_target_ms",None)} '
                      f'policy_src={getattr(self._long_module,"op_target_source","none")} '
                      f'setsync={getattr(self._long_module,"v231_setsync_target_ms",None)} '
                      f'setsync_src={getattr(self._long_module,"v231_setsync_source","none")} '
                      f'native_tacc={str(getattr(CS,"stock_cruise_state","unknown"))} '
                      f'planner_stop={int(bool(getattr(self._long_module,"_lp_should_stop",False)))} '
                      f'planner_a={float(getattr(self._long_module,"_lp_a_target",0.0)):.2f} '
                      f'ego={float(getattr(cs_out,"vEgo",0.0)):.2f} '
                      f'accel_cmd={float(getattr(CC.actuators,"accel",0.0)):.2f} '
                      f'virtual_stalk={int(setsync_btn or 0)} setsync_reason={setsync_reason} decision={decision.log}')
      return
    hybrid_original_long = bool(
      self._cached_hybrid_native_ap and not self._cached_autopilot_disabled
      and self.CP.openpilotLongitudinalControl
    )
    # V214: cruise SET synchronization is NOT acceleration authority. An accelerator
    # override makes CC.longActive false, but the original LONG/ACC road-target
    # synchronizer must continue through Tesla's normal OVERRIDE cruise state.
    # The V207 physical-brake latch still wins: no virtual SET/RES while braking,
    # regen or after brake cancellation until controlsd accepts a fresh MAIN/RES.
    # Explicit non-pedal longitudinal overrides must not be treated as gas requests.
    if hybrid_original_long:
      cs_out = getattr(CS, "out", None)
      brake = bool(getattr(cs_out, "brakePressed", False))
      regen = bool(getattr(cs_out, "regenBraking", False))
      gas = bool(getattr(cs_out, "gasPressed", False))
      long_active = bool(getattr(CC, "longActive", False))
      stock_state = str(getattr(CS, "stock_cruise_state", "") or "").upper()
      if not enabled:
        self._hybrid_setspeed_brake_cancelled = False
      elif brake:
        self._hybrid_setspeed_brake_cancelled = True
      elif long_active and not regen:
        # controlsd owns the physical-button rearm decision. Do not independently
        # parse synthetic/echoed virtual stalk commands in this controller.
        self._hybrid_setspeed_brake_cancelled = False
      pedal_only_override = gas and stock_state == "OVERRIDE" and not self._hybrid_setspeed_brake_cancelled
      enabled = (enabled and not brake and not regen and not self._hybrid_setspeed_brake_cancelled
                 and (long_active or pedal_only_override))
      if not enabled:
        self._long_module.update(CS, enabled=False, frame=int(self.frame), now_ms=int(self._now_ms()))
      elif pedal_only_override and not long_active and int(self.frame) % 100 == 0:
        self._diag_log("[XNOR_V214_LONG] accelerator_override: SET sync remains enabled; acceleration authority stays with driver")
    if (not enabled) or (not (self._cached_autopilot_disabled or hybrid_original_long)):
      self._diag_log(
        f"[XNOR_CC_DIAG] gate=pre enabled={int(enabled)} "
        f"latActive={int(bool(getattr(CC, 'latActive', False)))} "
        f"cc_enabled={int(bool(getattr(CC, 'enabled', False)))} "
        f"ap_disabled={int(bool(self._cached_autopilot_disabled))} "
        f"hybrid_original_long={int(hybrid_original_long)}"
      )
      return

    # Don't overlap with explicit sequences, a pending pulse release, or legacy virtual turn hold.
    if self.CP.carFingerprint in LEGACY_CARS:
      hold_turn = int(getattr(self, "_virtual_turn_prev", 0) or 0)
      if hold_turn in (1, 2):
        self._diag_log(f"[XNOR_CC_DIAG] gate=turn_hold turn={hold_turn}")
        return

    if (int(self._stw_release_frame) >= 0):
      self._diag_log(f"[XNOR_CC_DIAG] gate=pending_release release_frame={int(self._stw_release_frame)} frame={int(self.frame)}")
      return

    decision = self._long_module.update(CS, enabled=enabled, frame=int(self.frame), now_ms=int(self._now_ms()))
    if decision.button is None:
      self._diag_log(f"[XNOR_CC_DIAG] gate=no_decision detail={decision.log or 'none'}")
      return

    # One Unity-style pulse; release is handled next frame by _queue_stalk_pulse().
    if self._queue_stalk_pulse(CS, can_sends, int(decision.button)):
      self._automated_cruise_action_time_ms = int(self._now_ms())

  # --- V229: Unity-style stop-and-go owner -------------------------------------------------------
  def _v229_compute_owner(self, CS) -> bool:
    """True while OP should be the single DAS_control author (panda enforces the same rules)."""
    if not self._v229_stopgo_selected:
      return False
    autopilot_disabled = bool(self._cached_autopilot_disabled)
    hybrid = bool(self._cached_hybrid_native_ap) and not autopilot_disabled
    ap_disabled_acc = autopilot_disabled and (bool(getattr(self, "_cached_enable_acc", False)) or
                                              bool(getattr(CS, "enableACC", False)))
    if not (hybrid or ap_disabled_acc):
      return False
    cs_out = getattr(CS, "out", None)
    # Unity parity: genuine native Autosteer owns TACC. Hand DAS_control back to the AP.
    if hybrid and bool(getattr(cs_out, "stockLkas", False)):
      return False
    return True

  def _v229_stopgo_long(self, CC, CS, can_sends) -> None:
    """Emit Unity-parity DAS_control TEMPLATES at 50 Hz for both pandas.

    Modes (first match wins):
      not_drive : ACC_CANCEL_GENERIC(0), 0 kph, priming envelope          (Unity parity)
      cancel    : ACC_CANCEL_SILENT(13), neutral 0/0 -> DI cruise engaged but OP is not
      active    : ACC_ON(4), setSpeed = vEgo + 3*a, split accel limits     (Unity parity)
      neutral   : ACC_ON(4), setSpeed = vEgo, 0/0 (OP engaged, long overridden: gas/brake latch)
      priming   : ACC_ON(4), 0 kph, accel -1.4..+1.8, jerk -0.46..+0.476   (Unity parity)
    """
    cs_out = CS.out
    stock_state = str(getattr(CS, "stock_cruise_state", "") or "").upper()
    di_engaged = stock_state in _V229_DI_ENGAGED_STATES
    op_enabled = bool(CC.enabled)

    if di_engaged:
      if self._v229_di_engaged_since_frame < 0:
        self._v229_di_engaged_since_frame = int(self.frame)
        self._v229_op_enabled_during_di = False
      if op_enabled:
        self._v229_op_enabled_during_di = True
    else:
      self._v229_di_engaged_since_frame = -1
      self._v229_op_enabled_during_di = False

    if (not self._v229_owner_now) or (self.frame % 2 != 0):
      if not self._v229_owner_now:
        self._v229_mode = "native"
      return

    v_ego = max(float(cs_out.vEgo), 0.0)
    v_ego_kph = v_ego * CV.MS_TO_KPH
    in_drive = cs_out.gearShifter == structs.CarState.GearShifter.drive
    di_engaged_frames = (int(self.frame) - int(self._v229_di_engaged_since_frame)) if di_engaged else 0
    unowned_di = di_engaged and (not op_enabled) and (
      self._v229_op_enabled_during_di or di_engaged_frames >= _V229_UNOWNED_DI_GRACE_FRAMES)

    accel = 0.0
    if not in_drive:
      mode = "not_drive"
      acc_state, set_kph = _V229_ACC_STATE_CANCEL_GENERIC, 0.0
      accel_min, accel_max = _V229_PRIME_ACCEL
      jerk_min, jerk_max = _V229_PRIME_JERK
    elif unowned_di:
      mode = "cancel"
      acc_state, set_kph = _V229_ACC_STATE_CANCEL_SILENT, v_ego_kph
      accel_min, accel_max = 0.0, 0.0
      jerk_min, jerk_max = _V229_ACTIVE_JERK
    elif bool(CC.longActive):
      mode = "active"
      accel = float(np.clip(float(CC.actuators.accel), CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      acc_state = _V229_ACC_STATE_ON
      set_kph = min(max(v_ego + accel * _V229_ACCEL_TO_SPEED_S, 0.0) * CV.MS_TO_KPH, _V229_SET_SPEED_MAX_KPH)
      accel_min, accel_max = (accel, 0.0) if accel < 0.0 else (0.0, accel)
      jerk_min, jerk_max = _V229_ACTIVE_JERK
    elif op_enabled:
      mode = "neutral"
      acc_state, set_kph = _V229_ACC_STATE_ON, v_ego_kph
      accel_min, accel_max = 0.0, 0.0
      jerk_min, jerk_max = _V229_ACTIVE_JERK
    else:
      mode = "priming"
      acc_state, set_kph = _V229_ACC_STATE_ON, 0.0
      accel_min, accel_max = _V229_PRIME_ACCEL
      jerk_min, jerk_max = _V229_PRIME_JERK

    counter = (int(self.frame) // 2) % 8
    for powertrain in (False, True):
      can_sends.append(self.tesla_can.create_das_control_template(
        powertrain=powertrain, acc_state=acc_state, set_speed_kph=set_kph,
        accel_min=accel_min, accel_max=accel_max, jerk_min=jerk_min, jerk_max=jerk_max,
        counter=counter,
      ))

    if mode != self._v229_mode or (int(self.frame) - int(self._v229_last_diag_frame)) >= 100:
      self._v229_last_diag_frame = int(self.frame)
      stuck = (mode == "active") and (not di_engaged) and (v_ego < 0.5)
      cloudlog.info(f'[XNOR_V229_STOPGO] mode={mode} prev={self._v229_mode} di={stock_state or "UNKNOWN"} '
                    f'ccEnabled={int(op_enabled)} longActive={int(bool(CC.longActive))} '
                    f'accel={accel:.2f} set_kph={set_kph:.1f} ego={v_ego:.2f} '
                    f'di_frames={di_engaged_frames} di_not_armed_while_active={int(stuck)}')
    self._v229_mode = mode

  def update(self, CC, CS, now_nanos):


    actuators = CC.actuators
    can_sends = []

    self._track_human_cruise_actions(CS)


    self._refresh_cached_params()
    self._v229_owner_now = self._v229_compute_owner(CS)
    native_alc_turn = self._native_alc_virtual_hold(CC, CS)
    # V223: direct longitudinal owner request follows OP's independent stalk latch,
    # not native TACC. Both pandas independently require fresh real pedal/brake/AP
    # messages and enforce safety; no request alone grants longitudinal authority.
    v221_hil_request = bool(
      self._v221_hil_selected and self._cached_hybrid_native_ap and
      getattr(CS, '_xnor_experimental_direct_active', False) and
      CC.longActive and not CC.cruiseControl.cancel and
      not bool(getattr(CS.out, 'brakePressed', False)) and
      not bool(getattr(CS.out, 'gasPressed', False))
    )
    self._emit_internal_0x659(CS, can_sends, native_alc_turn=native_alc_turn,
                              v221_hil_request=v221_hil_request,
                              v229_stopgo_owner=self._v229_owner_now)
    if self._v221_hil_selected and self.frame % 100 == 0:
      cloudlog.info(f'[XNOR_V221_HIL] request={int(v221_hil_request)} '
                    f'longActive={int(bool(CC.longActive))} '
                    f'nativeTacc={getattr(CS,"stock_cruise_state","UNKNOWN")} '
                    'firmware_required=V224_AEB_ACTIVE_ONLY_OWNER v226_tx_lifecycle=OWNER_ONLY image_on_device=UNVERIFIED acceptance=UNPROVEN')
    # Request diagnostics only. Actual acceptance is evidenced by real native
    # cruise-state / DI_cruiseSet changes and AP-facing frame capture, not this log.
    if self._v216_acc_from_zero_enable and (self.frame % 500 == 0):
      cloudlog.info(
        f'[XNOR_V216_ACC_ZERO] request=1 stock={getattr(CS, "stock_cruise_state", "UNKNOWN")} '
        f'set_ms={float(getattr(CS, "stock_cruise_set_speed_ms", 0.0) or 0.0):.2f} '
        f'ego_ms={float(getattr(getattr(CS, "out", None), "vEgo", 0.0) or 0.0):.2f}'
      )

    if self._v217_autopilot_always_on_enable and (self.frame % 500 == 0):
      cloudlog.info(
        f'[XNOR_V217_AP_ALWAYS_ON] request=1 stock={getattr(CS, "stock_cruise_state", "UNKNOWN")} '
        f'ego_ms={float(getattr(getattr(CS, "out", None), "vEgo", 0.0) or 0.0):.2f}'
      )

    # Config-unlock experiment: transmit GTW_carConfig(0x398) with autopilot=2 natively on bus 2
    # (the AP module's own segment), ~1Hz. Fixed valid payload; no checksum/counter on this frame.
    if _GTW_TX_AUTOPILOT2_BUS2 and (not self._cached_hybrid_native_ap) and (self.frame % 100 == 0):
      can_sends.append((0x398, bytes(_GTW_CARCONFIG_AP2_PAYLOAD), int(CANBUS.autopilot_party)))

    autopilot_disabled = bool(self._cached_autopilot_disabled)
    hybrid_native_ap = bool(self._cached_hybrid_native_ap) and not autopilot_disabled

    # Always define before use
    cs_out = getattr(CS, "out", None)
    out_steer_pressed = bool(getattr(cs_out, "steeringPressed", False)) if cs_out is not None else False
    human_control = bool(getattr(CS, "human_control", False) or out_steer_pressed)
    steer_inhibit = bool(
      (bool(getattr(cs_out, "steerFaultTemporary", False)) if cs_out is not None else False) or
      (bool(getattr(cs_out, "steerFaultPermanent", False)) if cs_out is not None else False) or
      (bool(getattr(cs_out, "steeringDisengage", False)) if cs_out is not None else False)
    )

    op_enabled = bool(getattr(CC, "enabled", False) or getattr(CC, "latActive", False))

    # Native EPAS co-op recovery is only for a session that actually observed
    # genuine native type-1 0x488. The original OP-only steering did not enter
    # this native neutral/probe/timeout lifecycle for a routine driver takeover.
    native_ap_lateral_active = bool(getattr(cs_out, "stockLkas", False)) if cs_out is not None else False
    if not op_enabled:
      self._hybrid_native_lkas_seen_this_engagement = False
    elif hybrid_native_ap and native_ap_lateral_active:
      self._hybrid_native_lkas_seen_this_engagement = True
    native_supervised_coop = bool(hybrid_native_ap and self._hybrid_native_lkas_seen_this_engagement)

    # XNOR_V199_HYBRID_COOP_STATE_DRIVEN_REARM:
    # Arm from the real EPAS hands/error/status tuple, not HSO's 1.5 s presentation timer. Panda
    # owns the actual neutral-carrier gate and clears it after five real 0x370 frames; userspace
    # waits 25 control frames (~250 ms) before stopping 0x27D so it cannot clear first.
    eac_status_raw = int(getattr(CS, "eac_status_raw", -1))
    eac_error_raw = int(getattr(CS, "eac_error_code_raw", -1))
    hands_on_level = int(getattr(CS, "hands_on_level", 0))
    physical_coop_takeover = (
      native_supervised_coop and hands_on_level >= 3 and eac_error_raw == 3 and eac_status_raw in (2, 4, 6)
    )
    if native_supervised_coop and op_enabled:
      if physical_coop_takeover:
        self._hybrid_coop_rearm = True
        self._hybrid_coop_healthy_frames = 0

      if self._hybrid_coop_rearm:
        allowed_coop_state = eac_error_raw in (0, 3) and eac_status_raw in (1, 2, 4, 6)
        real_epas_healthy = eac_error_raw == 0 and eac_status_raw == 2 and hands_on_level <= 1
        if not allowed_coop_state:
          # Never conceal or work around a non-hands EPAS error.
          self._hybrid_coop_rearm = False
          self._hybrid_coop_healthy_frames = 0
        elif real_epas_healthy:
          self._hybrid_coop_healthy_frames += 1
          # V214: 25 *100Hz control cycles cleared this before panda's independent
          # physical 0x370 probe had remained ACTIVE long enough. Require a longer
          # stable window than panda's 25 genuine 0x370 acknowledgements (~0.5s).
          if self._hybrid_coop_healthy_frames >= 75:
            self._hybrid_coop_rearm = False
            self._hybrid_coop_healthy_frames = 0
        else:
          self._hybrid_coop_healthy_frames = 0
    else:
      self._hybrid_coop_rearm = False
      self._hybrid_coop_healthy_frames = 0

    # V203: a physical co-op release or genuine native LKAS drop that never
    # reacquires real EPAS authority must not leave the driver with green/blue
    # steering indications but no actuator. The CarState fault is published on
    # the next card cycle and triggers the normal OP fault/disengagement path.
    if native_supervised_coop and op_enabled:
      if self._hybrid_coop_rearm and hands_on_level >= 2:
        self._hybrid_coop_last_physical_hands_frame = int(self.frame)
      real_epas_healthy = eac_status_raw == 2 and eac_error_raw == 0 and hands_on_level <= 1
      if (self._hybrid_coop_rearm and not real_epas_healthy and
          0 <= self._hybrid_coop_last_physical_hands_frame < int(self.frame) - 450 and hands_on_level <= 1):
        self._hybrid_coop_failed = True
      if (self._hybrid_drop_frame >= 0 and
          int(self.frame) - self._hybrid_drop_frame >= (450 if self._hybrid_coop_rearm else 200) and
          not real_epas_healthy and hands_on_level <= 1):
        self._hybrid_coop_failed = True
      if self._hybrid_coop_failed:
        if not getattr(self, '_hybrid_coop_failure_reported', False):
          cloudlog.error(
            f'[XNOR_COOP_FAULT] real EPAS did not acknowledge rearm: '
            f'status={eac_status_raw} error={eac_error_raw} hands={hands_on_level}; '
            'steering inhibited; driver must take over, safely re-engage after fault clears'
          )
          self._hybrid_coop_failure_reported = True
        self._hybrid_coop_rearm = False
        self._hybrid_eac_recovery = False
    else:
      self._hybrid_coop_failed = False
      self._hybrid_coop_failure_reported = False
      self._hybrid_coop_last_physical_hands_frame = -1
      self._hybrid_drop_frame = -1
      self._hybrid_drop_warmup_until_frame = -1
    CS._xnor_hybrid_epas_failed = bool(self._hybrid_coop_failed)
    CS._xnor_hybrid_epas_rearm = bool(self._hybrid_coop_rearm)
    steer_inhibit = bool(steer_inhibit or self._hybrid_coop_failed)

    if op_enabled and (not bool(self._op_enabled_prev)):
      # Avoid a first-command step when engaging with wheel turned (EPS inhibit prevention).
      try:
        self.apply_angle_last = float(getattr(cs_out, "steeringAngleDeg", 0.0) if cs_out is not None else 0.0)
      except Exception:
        pass
      # Hybrid grace window: keep native pre-engagement 0x488/0x27D untouched briefly after the
      # stalk engages OP. This leaves enough time for a second stalk pull to engage native Tesla
      # Autosteer. If native LKAS does not become active, OP direct lateral fallback starts after
      # ~1 second. Without this grace, the first pull could make OP seize EPAS before Tesla sees
      # the second pull, defeating the native-Autosteer path we restored in V171.
      if hybrid_native_ap:
        self._hybrid_direct_fallback_after_frame = int(self.frame) + 100
    if not op_enabled:
      self._hybrid_direct_fallback_after_frame = -1
    self._op_enabled_prev = bool(op_enabled)

    if not hybrid_native_ap:
      self._process_stalk_actions(CS, can_sends)
      self._process_body_controls(CS, can_sends)
      self._process_hud_status(CC, CS, can_sends, human_control)
      self._process_lane_telemetry(CC, CS, can_sends)
      self._speed_limit_sync(CC, CS, can_sends)
    else:
      # V219: restore the original (3) OP ALC comfort-tap hold when actual
      # native Autosteer is OFF. ALC indicator ownership is independent of LONG.
      # When native LKAS is ON, the existing AP-facing native tap bridge remains
      # the sole virtual indicator owner. Never author competing synthetic stalk.
      raw_native_lkas = bool(getattr(cs_out, 'stockLkas', False)) if cs_out is not None else False
      autoresume_stalk_sent = False
      if op_enabled and not raw_native_lkas:
        self._process_stalk_actions(CS, can_sends)
        autoresume_stalk_sent = self._v241_auto_resume_stalk(CC, CS, can_sends, native_lkas=raw_native_lkas)
      else:
        # Still observe/reset the brake-cycle latch even when native Tesla LKAS owns
        # the stalk lifecycle; never emit a competing synthetic RES in that state.
        self._v241_auto_resume_stalk(CC, CS, can_sends, native_lkas=raw_native_lkas)
        # On OP disengagement (native still idle), release our own virtual
        # hold explicitly. On native takeover do not send a competing release:
        # native Tesla now owns the AP-side stalk/indicator lifecycle.
        if not raw_native_lkas and int(self._virtual_turn_prev) in (1, 2):
          self._send_stw(CS, can_sends, BTN_IDLE,
                         bus=int(self._stw_bus(CS)), turn_signal_stalk_state=0)
        self._virtual_turn_prev = 0
        if int(self._stw_release_frame) == int(self.frame):
          self._send_stw(CS, can_sends, BTN_IDLE, bus=int(self._stw_release_bus))
          self._stw_release_frame = -1
      if not autoresume_stalk_sent:
        self._speed_limit_sync(CC, CS, can_sends)

    # Normal xnor: OP owns lateral directly only in Autopilot Disabled mode.
    # V180 Hybrid uses the genuine AP 0x488 stream as a carrier whether Tesla Autosteer is idle
    # (controlType=0) or active. OP never becomes a second physical sender: userspace emits a
    # validated TEMPLATE and panda substitutes it onto the next genuine AP frame, preserving the
    # AP cadence/counter. This lets Hybrid run OP lateral with native TACC after the first stalk
    # pull without requiring native Tesla Autosteer to be engaged (and therefore avoids relying on
    # Tesla's native hands-on supervision lifecycle).
    hybrid_carrier_overlay = bool(hybrid_native_ap)
    native_ap_lateral_active = bool(getattr(cs_out, "stockLkas", False)) if cs_out is not None else False

    # V211: genuine native 0x488 ownership and OP's independent lane-change
    # request are mutually exclusive. Tesla state 4 does not grant OP actuator
    # authority; OP can run on the genuine idle carrier only after native LKAS
    # releases and real EPAS acknowledges steering.
    if hybrid_native_ap:
      direction = int(getattr(CS, "alca_direction", 0) or 0)
      native_state = int(getattr(CS, "_native_alc_state", 31))
      native_valid = bool(getattr(CS, "_native_alc_valid", False))
      native_match = (native_valid and native_ap_lateral_active and
                      eac_status_raw == 2 and eac_error_raw == 0 and
                      native_state == (9 if direction == 1 else 10 if direction == 2 else -1))
      if bool(getattr(CS, "alca_engaged", False)):
        alc_phase = "NATIVE_MATCH" if native_match else "OP_INDEPENDENT"
      elif bool(getattr(CS, "alca_pre_engage", False)):
        alc_phase = "PREPARE_OP"
      else:
        alc_phase = "IDLE"
      if alc_phase != self._hybrid_alc_phase or (
          alc_phase != "IDLE" and int(self.frame) - self._hybrid_alc_last_diag_frame >= 100):
        native_angle = float(getattr(CS, "native_steer_angle_deg", 0.0))
        op_angle = float(actuators.steeringAngleDeg)
        cloudlog.info(
          f"[XNOR_HYBRID_ALC] phase={alc_phase} op_dir={direction} "
          f"native_state={native_state} valid={int(native_valid)} "
          f"native_lkas={int(native_ap_lateral_active)} "
          f"native_angle={native_angle:.1f} op_angle={op_angle:.1f} "
          f"tracking_delta={abs(op_angle-native_angle):.1f} "
          f"guard_risk={int(native_ap_lateral_active and abs(op_angle-native_angle)>10.0)} "
          f"eac={int(eac_status_raw)} error={int(eac_error_raw)}"
        )
        self._hybrid_alc_last_diag_frame = int(self.frame)
      self._hybrid_alc_phase = alc_phase
      # State 4 on an active native type-1 carrier is a native availability
      # restriction, not OP steering authority. Log a tap even if the OP model
      # correctly remains idle; users must not see a fake completed lane change.
      if (int(getattr(CS, 'tap_direction', 0) or 0) in (1, 2) and native_ap_lateral_active and
          native_valid and native_state == 4 and
          int(self.frame) - int(getattr(self, '_v211_native_unavailable_log_frame', -100000)) >= 100):
        cloudlog.warning('[XNOR_V211_ALC] native_state=4 native_type=1 tap_seen=1; '
                         'OP independent trajectory unavailable until genuine native LKAS release')
        self._v211_native_unavailable_log_frame = int(self.frame)
      if alc_phase == 'OP_INDEPENDENT' and native_ap_lateral_active and native_valid and native_state == 4:
        if int(self.frame) - int(getattr(self, '_v210_alc_conflict_log_frame', -100000)) >= 100:
          cloudlog.warning('[XNOR_V210_ALC_LIMIT] native_unavailable=4 OP_plan_active=1 '
                           'physical_steering_still_subject_to_native_10deg_guard; no_forced_takeover')
          self._v210_alc_conflict_log_frame = int(self.frame)

    # XNOR_V186_HYBRID_EAC_RECOVERY:
    # The V185 logs prove that after a physical co-op takeover Tesla can drop native Autosteer
    # (raw 0x488 type 1 -> 0) while OP stays enabled and keeps overlaying type-1 steering onto the
    # genuine idle 0x488 carrier. EPAS, however, falls EAC_ACTIVE -> EAC_AVAILABLE and therefore
    # ignores those otherwise-valid OP steering commands. Arm OP's normal APS_eacMonitor/0x27D
    # handshake ONLY on a genuine native-LKAS falling edge while OP remains enabled. Keep it
    # active until native Autosteer returns or OP disengages.
    if hybrid_native_ap:
      if native_ap_lateral_active:
        self._hybrid_eac_recovery = False
      elif bool(self._hybrid_native_lkas_prev) and bool(op_enabled):
        self._hybrid_eac_recovery = True
        self._hybrid_drop_frame = int(self.frame)
        # First fallback commands start at measured wheel angle, not the
        # potentially distant planner angle. Existing angle-rate limits apply.
        self._hybrid_drop_warmup_until_frame = int(self.frame) + 25
        self.apply_angle_last = float(getattr(cs_out, "steeringAngleDeg", 0.0))
        cloudlog.warning(
          f"[XNOR_HYBRID_FALLBACK] native_drop op={int(op_enabled)} "
          f"real_eac={eac_status_raw} error={eac_error_raw} hands={hands_on_level} "
          f"path={'OP_CARRIER_ACTIVE' if (eac_status_raw == 2 and eac_error_raw == 0 and hands_on_level <= 1) else 'EPAS_REARM_PENDING'}"
        )
      if not op_enabled or self._hybrid_coop_failed:
        self._hybrid_eac_recovery = False
      if native_ap_lateral_active and eac_status_raw == 2 and eac_error_raw == 0:
        self._hybrid_drop_frame = -1
      self._hybrid_native_lkas_prev = bool(native_ap_lateral_active)
    else:
      self._hybrid_eac_recovery = False
      self._hybrid_native_lkas_prev = False

    # Hybrid carrier mode does not add a second Tesla hands-on veto. openpilot's own state machine
    # remains responsible for CC.latActive/driver override. Preserve the old human-control
    # suppression for normal/direct OP operation.
    # V202: restore original HSO's finite hands-on steering override for Hybrid, too. During
    # the original numb window there must be no active OP steering template; genuine Tesla
    # carrier and V199 safety-side neutral/re-arm continue unchanged.
    human_control_blocks_lateral = bool(human_control)
    lat_active = (
      bool(CC.latActive) and
      (autopilot_disabled or hybrid_carrier_overlay) and
      (not CS.out.cruiseState.standstill) and
      (not human_control_blocks_lateral) and
      (not steer_inhibit)
    )

    # Steering warm-up: for a short window after lateral becomes active, command current wheel angle.
    # This prevents an initial command step (EPS inhibit) when engaging with the wheel turned.
    if lat_active and (not bool(self._lat_active_prev)):
      self._steer_warmup_until_frame = int(self.frame) + 2  # shorter warmup so turn-in starts sooner
    self._lat_active_prev = bool(lat_active)

    # V210: a genuine native ALC (including one started by a physical FULL stalk)
    # owns its trajectory regardless of the optional tap bridge's setting. When
    # it finishes, briefly retain genuine native lane centering while OP's
    # model settles; don't immediately overlay a stale mid-change OP angle.
    native_alc_owns_steer = bool(
      hybrid_native_ap and bool(getattr(CS, '_native_alc_valid', False))
      and int(getattr(CS, '_native_alc_state', 31)) in (9, 10)
      and native_ap_lateral_active and eac_status_raw == 2 and eac_error_raw == 0
    )
    op_alc_requested = bool(getattr(CS, 'alca_pre_engage', False) or
                            getattr(CS, 'alca_engaged', False))
    real_epas_healthy = bool(eac_status_raw == 2 and eac_error_raw == 0 and hands_on_level <= 1)
    # V212 handover qualification runs at the 100 Hz controller cadence and
    # observes ACTUAL EPAS state, not the AP-presented/scrubbed 0x370. A one-frame
    # ACTIVE reading immediately after native LKAS falls is not enough.
    if hybrid_native_ap and op_enabled and real_epas_healthy and not steer_inhibit:
      self._v212_epas_good_frames = min(25, self._v212_epas_good_frames + 1)
      if not native_ap_lateral_active:
        self._v212_native_idle_good_frames = min(5, self._v212_native_idle_good_frames + 1)
      else:
        self._v212_native_idle_good_frames = 0
    else:
      self._v212_epas_good_frames = 0
      self._v212_native_idle_good_frames = 0

    v212_ready = self._v212_handover_ready(
      native_lkas=native_ap_lateral_active, epas_healthy=real_epas_healthy,
      epas_good_frames=self._v212_epas_good_frames,
      native_idle_good_frames=self._v212_native_idle_good_frames,
      coop_rearm=bool(self._hybrid_coop_rearm), coop_failed=bool(self._hybrid_coop_failed),
      steer_inhibit=steer_inhibit,
    )

    # Passive observation of a GENUINE physical turn request. The tap can be
    # transient, so observe both the tap decoder and the raw stalk direction.
    # A stale/held indicator never becomes an automatic OP trajectory on release.
    turn = int(getattr(CS, 'turnSignalStalkState', 0) or 0)
    tap = int(getattr(CS, 'tap_direction', 0) or 0)
    physical_turn = turn if turn in (1, 2) else (tap if tap in (1, 2) else 0)
    physical_edge = physical_turn in (1, 2) and physical_turn != self._v212_prev_physical_turn
    self._v212_prev_physical_turn = physical_turn
    previous_handover_state = self._v212_handover_state
    if not hybrid_native_ap or not op_enabled or steer_inhibit or self._hybrid_coop_failed:
      self._v212_handover_state = 'IDLE'
      self._v212_pending_direction = 0
      self._v212_handover_since_frame = -1
    elif native_alc_owns_steer:
      self._v212_handover_state = 'NATIVE_ALC'
      self._v212_pending_direction = 0
      self._v212_handover_since_frame = -1
    elif physical_edge:
      self._v212_pending_direction = physical_turn
      self._v212_handover_since_frame = int(self.frame)
      self._v212_handover_state = 'WAIT_NATIVE_RELEASE' if native_ap_lateral_active else 'VERIFY_EPAS'
    elif self._v212_pending_direction in (1, 2):
      if native_ap_lateral_active:
        if int(self.frame) - self._v212_handover_since_frame >= 300:
          self._v212_handover_state = 'BLOCKED_NATIVE_ACTIVE'
        else:
          self._v212_handover_state = 'WAIT_NATIVE_RELEASE'
      elif not v212_ready:
        self._v212_handover_state = 'VERIFY_EPAS'
      else:
        # READY describes availability for a NEW explicit input only. It does
        # not change desire_helper's off->on edge rule or queue a manoeuvre.
        self._v212_handover_state = 'READY_FRESH_REQUEST'
      if int(self.frame) - self._v212_handover_since_frame >= 500:
        self._v212_handover_state = 'EXPIRED_FRESH_REQUEST_REQUIRED'
        self._v212_pending_direction = 0
    elif op_alc_requested and v212_ready:
      self._v212_handover_state = 'OP_IDLE_CARRIER'
    else:
      self._v212_handover_state = 'IDLE'

    if self._v212_handover_state != previous_handover_state:
      cloudlog.warning(
        f'[XNOR_V212_ALC_HANDOVER] state={self._v212_handover_state} '
        f'native_type={int(native_ap_lateral_active)} '
        f'native_alc={int(getattr(CS, "_native_alc_state", 31))} '
        f'physical_dir={physical_turn} pending_dir={self._v212_pending_direction} '
        f'real_epas={eac_status_raw} error={eac_error_raw} hands={hands_on_level} '
        f'epas_good={self._v212_epas_good_frames} idle_good={self._v212_native_idle_good_frames} '
        f'coop_rearm={int(self._hybrid_coop_rearm)} coop_failed={int(self._hybrid_coop_failed)} '
        f'op_plan={int(op_alc_requested)} ready={int(v212_ready)}'
      )

    if self._v210_native_alc_prev_active and not native_alc_owns_steer:
      self._v210_native_alc_handback_until_frame = int(self.frame) + 100
      self.apply_angle_last = float(CS.out.steeringAngleDeg)
      cloudlog.info('[XNOR_V211_ALC] native_complete; retain native until OP model settles')
    self._v210_native_alc_prev_active = bool(native_alc_owns_steer)
    native_alc_handback = bool(
      hybrid_native_ap and native_ap_lateral_active and real_epas_healthy and
      int(self._v210_native_alc_handback_until_frame) >= 0 and
      (int(self.frame) < int(self._v210_native_alc_handback_until_frame) or op_alc_requested)
    )
    if not native_ap_lateral_active or not real_epas_healthy:
      self._v210_native_alc_handback_until_frame = -1
      native_alc_handback = False

    alc_route = (self._v211_lane_change_route(
      native_alc_active=native_alc_owns_steer, native_lkas=native_ap_lateral_active,
      op_request=op_alc_requested, epas_healthy=(real_epas_healthy and (native_ap_lateral_active or v212_ready)),
      coop_rearm=bool(self._hybrid_coop_rearm), coop_failed=bool(self._hybrid_coop_failed))
      if hybrid_native_ap else 'IDLE')
    if op_alc_requested and alc_route == 'WAIT_EPAS':
      # An in-flight OP trajectory must not suddenly resume after EPAS recovers.
      # Wait for a new, independent indicator request once the planner is idle.
      self._v211_alc_epas_abort = True
    if not op_alc_requested:
      self._v211_alc_epas_abort = False
    blocked_alc = bool(hybrid_native_ap and
                       (alc_route in ('NATIVE_LKAS_BLOCK', 'WAIT_EPAS') or self._v211_alc_epas_abort))
    if blocked_alc or native_alc_owns_steer or native_alc_handback:
      self.apply_angle_last = float(CS.out.steeringAngleDeg)
    if alc_route != self._v211_alc_route or (blocked_alc and int(self.frame) - self._v211_alc_last_notice_frame >= 100):
      cloudlog.warning(f'[XNOR_V211_ALC] route={alc_route} native_lkas={int(native_ap_lateral_active)} '
                       f'native_state={int(getattr(CS, "_native_alc_state", 31))} '
                       f'op_request={int(op_alc_requested)} v212_ready={int(v212_ready)} real_eac={eac_status_raw} '
                       f'error={eac_error_raw} hands={hands_on_level} blocked={int(blocked_alc)}')
      self._v211_alc_last_notice_frame = int(self.frame)
    self._v211_alc_route = alc_route
    # Steering (50Hz)
    if self.frame % 2 == 0:
      if ((not lat_active) or human_control_blocks_lateral or steer_inhibit or
          native_alc_owns_steer or native_alc_handback or blocked_alc or
          (int(self.frame) < int(self._steer_warmup_until_frame)) or
          (hybrid_native_ap and int(self.frame) < int(self._hybrid_drop_warmup_until_frame))):
        apply_angle = float(CS.out.steeringAngleDeg)
      elif hybrid_native_ap and self._hybrid_coop_rearm:
        # Keep the requested angle pinned to the measured wheel through both the physical override
        # and the EPAS re-arm phase. Panda temporarily converts the car-facing carrier to type 0;
        # once real EPAS health is stable, type 1 resumes from the driver's actual wheel position.
        apply_angle = float(CS.out.steeringAngleDeg)
      else:
        desired_angle = self._lane_positioned_target_angle(
          float(actuators.steeringAngleDeg),
          float(CS.out.steeringAngleDeg),
          float(getattr(CS.out, "vEgoRaw", CS.out.vEgo)),
        )
        apply_angle = float(apply_std_steer_angle_limits(
          float(desired_angle),
          float(self.apply_angle_last),
          float(getattr(CS.out, "vEgoRaw", CS.out.vEgo)),
          float(CS.out.steeringAngleDeg),
          lat_active,
          CarControllerParams.ANGLE_LIMITS,
        ))
        steer_guard_deg = float(np.interp(
          float(getattr(CS.out, "vEgoRaw", CS.out.vEgo)),
          [0.0, 10.0, 20.0, 30.0],
          [34.0, 42.0, 52.0, 62.0],
        ))
        # Keep a measured-angle guard, but widen it with speed so the car can
        # build angle earlier into sharper corners instead of washing wide.
        apply_angle = float(np.clip(
          apply_angle,
          float(CS.out.steeringAngleDeg) - steer_guard_deg,
          float(CS.out.steeringAngleDeg) + steer_guard_deg,
        ))

      self.apply_angle_last = float(apply_angle)

      if self.CP.carFingerprint in LEGACY_CARS:
        counter = (self.frame // 2) % 16
        # In Hybrid mode this bus-0 packet is a TEMPLATE only: panda validates/caches it, blocks
        # direct TX, then substitutes it onto the next genuine native-AP 0x488 while preserving
        # native cadence/counter. Use a fixed non-zero template counter because panda discards it;
        # this avoids the overlay cache's zero-counter guard dropping every 16th template.
        if ((not hybrid_native_ap) or lat_active) and not (native_alc_owns_steer or native_alc_handback or blocked_alc):
          template_counter = 1 if hybrid_native_ap else counter
          can_sends.append(
            self.tesla_can.create_steering_control(template_counter, self.apply_angle_last, lat_active)
          )
      else:
        can_sends.append(
          self.tesla_can.create_steering_control(self.apply_angle_last, lat_active)
        )

    # EPAS handshake ownership:
    # - Normal/direct OP keeps its existing continuous APS_eacMonitor stream.
    # - Hybrid normally leaves Tesla's native EPAS lifecycle untouched.
    # - V186 exception: after a *proven* native-Autosteer falling edge while OP remains engaged,
    #   send OP's normal APS_eacAllow=1 stream until native Autosteer comes back or OP disengages.
    # - V199 also sends allow=1 during the state-driven co-op re-arm, beginning at the physical
    #   takeover rather than waiting for native 0x488 to fall.
    if (self.CP.carFingerprint in LEGACY_CARS) and (self.frame % 2 == 0):
      counter = (self.frame // 2) % 16
      # V213: Tesla retains the sole 0x27D handshake while its native 0x488 is active.
      # OP may take it only after real native release, avoiding competing allow streams.
      # V218: match the original OP-only lifecycle whenever genuine Tesla
      # Autosteer is idle. The original continuously sent allow=1 in 0x27D.
      # V217 forwarded native idle allow=0 but withheld OP allow=1 outside a
      # special recovery latch, leaving OP-only without its EPAS-allow owner.
      # V218 grants only one owner and retains genuine EPAS fault handling.
      hybrid_recovery = (bool(op_enabled) and not self._hybrid_coop_failed and
                         not native_ap_lateral_active)
      if (not hybrid_native_ap) or hybrid_recovery:
        can_sends.append(self.tesla_can.create_steering_allowed(counter))

    # V226: Hybrid exclusive longitudinal has ONE owner in every lifecycle state.
    # When OP is inactive/cancelled, DO NOT transmit a second powertrain DAS_control
    # with ACC_ON(4) (or a lingering ACC_CANCEL) beside native Tesla ACC_OFF.
    # Internal 0x659 above announces the owner transition immediately on a changed
    # bit6, before this 25 Hz TX block. Panda then releases the native AP carrier;
    # neither OP chassis templates nor the PT frame are sent without ownership.
    # The original continuous sender remains unchanged in non-Hybrid/legacy modes.
    # V229 replaces both legacy DAS_control senders below with Unity-style templates.
    if (not self._v229_stopgo_selected) and self.CP.openpilotLongitudinalControl and (self.frame % 4 == 0):
      # Only cancel when OP is NOT engaged. With pcmCruise=False (Autopilot-Disabled) controlsd sets
      # cruiseControl.cancel for the whole engagement, which previously encoded ACC_CANCEL (13)
      # into every DAS_control while OP was driving.
      state = 13 if (CC.cruiseControl.cancel and not CC.enabled) else 4
      accel = float(np.clip(
        float(actuators.accel),
        CarControllerParams.ACCEL_MIN,
        CarControllerParams.ACCEL_MAX
      ))
      counter = (self.frame // 4) % 8

      native_acc = bool(hybrid_native_ap) or bool(self._cached_enable_acc) or bool(getattr(CS, "enableACC", False))
      long_active = bool(CC.longActive) and ((not autopilot_disabled) or native_acc)
      send_op_powertrain = _v226_powertrain_tx_allowed(
        bool(hybrid_native_ap), bool(self._v221_hil_selected), bool(v221_hil_request)
      )

      # LONG/ACC virtual stalk still manages native cruise SET independently;
      # direct OP acceleration is emitted only while OP holds the Hybrid owner.
      if send_op_powertrain:
        long_frame = self.tesla_can.create_longitudinal_command(
          state, accel, counter, float(CS.out.vEgo), long_active,
        )
        can_sends.append(long_frame)
        if bool(getattr(CS, '_xnor_experimental_direct_active', False)) and (self.frame % 100 == 0):
          cloudlog.info(f'[XNOR_V226_LONG_TX] ccEnabled={int(bool(CC.enabled))} '
                        f'longActive={int(bool(CC.longActive))} encodedActive={int(long_active)} '
                        f'owner={int(bool(v221_hil_request))} '
                        f'native_tacc={str(getattr(CS,"stock_cruise_state","unknown"))} '
                        f'accel={accel:.3f} accState={state} '
                        f'addr={int(long_frame[0]):#x} bus={int(long_frame[2])} '
                        'stage=userspace_requested_not_ECU_ack')
      elif self.frame % 100 == 0:
        cloudlog.info(f'[XNOR_V226_LONG_RELEASE] native_only=1 owner=0 '
                      f'longActive={int(bool(CC.longActive))} '
                      f'cancel={int(bool(CC.cruiseControl.cancel))} '
                      f'native_tacc={str(getattr(CS,"stock_cruise_state","unknown"))}')

      # Keep the old direct chassis/stalk low-speed experiment restricted to non-Hybrid. Hybrid
      # reproduces the pre-rollback powertrain 0x2BF path and does not introduce a second 0x2B9.
      if (not hybrid_native_ap) and long_active and native_acc and (self.CP.carFingerprint in LEGACY_CARS) and \
         hasattr(self.tesla_can, "create_longitudinal_command_chassis"):
        if _ARM_ENABLE_CHASSIS:
          can_sends.append(
            self.tesla_can.create_longitudinal_command_chassis(
              state,
              accel,
              counter,
              float(CS.out.vEgo),
              long_active
            )
          )
        try:
          _ARM_PULSE_MAX = 3
          _ARM_PULSE_GAP = 18
          _ARM_WINDOW_FRAMES = 250

          if not bool(self._arm_long_active_prev):
            self._arm_pulse_count = 0
            self._arm_engage_frame = int(self.frame)
          self._arm_long_active_prev = True

          di_armed = bool(getattr(CS, "stock_cruise_enabled", False))
          v_ego_mph = float(CS.out.vEgo) * CV.MS_TO_MPH
          within_window = (int(self.frame) - int(self._arm_engage_frame)) <= _ARM_WINDOW_FRAMES
          want_arm = (
            _ARM_ENABLE_STALK and
            (not di_armed) and (v_ego_mph < 20.0) and within_window and
            (int(self._arm_pulse_count) < _ARM_PULSE_MAX)
          )
          if want_arm and (int(self.frame) - int(self._arm_pulse_last_frame) >= _ARM_PULSE_GAP):
            if self._send_stw(CS, can_sends, BTN_MAIN, bus=int(self._stw_bus(CS))):
              self._stw_release_frame = int(self.frame) + 1
              self._stw_release_bus = int(self._stw_bus(CS))
              self._arm_pulse_last_frame = int(self.frame)
              self._arm_pulse_count = int(self._arm_pulse_count) + 1
        except Exception:
          pass
      else:
        self._arm_long_active_prev = False

    # Unity-style chassis DAS_control ownership for stop/go. This frame is NOT put on the wire
    # directly: tesla_legacy safety captures the userspace 0x2B9 as a validated template and
    # merges it onto the next genuine AP 0x2B9 while preserving the AP rolling counter/timing.
    # Send templates at 50 Hz so every ~40 Hz stock 0x2B9 has a fresh desired payload available.
    if (
      (not self._v229_stopgo_selected)
      and ((not hybrid_native_ap) or v221_hil_request)
      and _UNITY_2B9_OVERLAY_TEMPLATE
      and self.CP.openpilotLongitudinalControl
      and (self.CP.carFingerprint in LEGACY_CARS)
      and hasattr(self.tesla_can, "create_longitudinal_command_chassis")
      and (self.frame % 2 == 0)
    ):
      native_acc_overlay = bool(self._cached_enable_acc) or bool(getattr(CS, "enableACC", False))
      long_active_overlay = bool(CC.longActive) and ((not autopilot_disabled) or native_acc_overlay)
      if long_active_overlay and (native_acc_overlay or v221_hil_request):
        overlay_state = 13 if (CC.cruiseControl.cancel and not CC.enabled) else 4
        overlay_accel = float(np.clip(
          float(actuators.accel),
          CarControllerParams.ACCEL_MIN,
          CarControllerParams.ACCEL_MAX,
        ))
        # Counter=1 is deliberate: this packet is only a cache template. Panda strips this
        # template counter and preserves the genuine AP 0x2B9 counter before recomputing checksum.
        can_sends.append(self.tesla_can.create_longitudinal_command_chassis(
          overlay_state,
          overlay_accel,
          1,
          float(CS.out.vEgo),
          True,
        ))

    # V229 Unity-style stop-and-go DAS_control templates (both pandas).
    if self._v229_stopgo_selected and self.CP.openpilotLongitudinalControl:
      self._v229_stopgo_long(CC, CS, can_sends)

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = float(self.apply_angle_last)

    self.frame += 1
    return new_actuators, can_sends
