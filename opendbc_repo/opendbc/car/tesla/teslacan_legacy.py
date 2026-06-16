# /data/openpilot/opendbc/car/tesla/teslacan_legacy.py
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import V_CRUISE_MAX
from opendbc.car.tesla.values import CANBUS, CarControllerParams


class TeslaCANRaven:
  """
  Legacy Tesla CAN helpers (HW2 "legacy" stack).

  Note: This class name historically said "Raven" in some forks. For clarity, we provide:
    TeslaCANLegacy = TeslaCANRaven
  """

  def __init__(self, packers):
    self.packers = packers

  @staticmethod
  def checksum(msg_id, dat):
    ret = (msg_id & 0xFF) + ((msg_id >> 8) & 0xFF)
    ret += sum(dat)
    return ret & 0xFF

  def create_steering_control(self, counter, angle, enabled):
    values = {
      "DAS_steeringControlCounter": counter,
      "DAS_steeringAngleRequest": -angle,
      "DAS_steeringHapticRequest": 0,
      "DAS_steeringControlType": 1 if enabled else 0,
    }

    data = self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)[1]
    values["DAS_steeringControlChecksum"] = self.checksum(0x488, data[:3])
    return self.packers[CANBUS.party].make_can_msg("DAS_steeringControl", CANBUS.party, values)

  def create_longitudinal_command(self, acc_state, accel, counter, v_ego, active, gas_pressed: bool = False, set_speed_kph: float | None = None):
    if set_speed_kph is not None:
      set_speed = max(float(set_speed_kph), 0.0)
    else:
      set_speed = max(v_ego * CV.MS_TO_KPH, 0)
      if active:
        set_speed = 0 if accel < 0 else V_CRUISE_MAX

    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": 0,
      "DAS_jerkMin": CarControllerParams.JERK_LIMIT_MIN,
      "DAS_jerkMax": CarControllerParams.JERK_LIMIT_MAX,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0),
      "DAS_controlCounter": counter,
    }

    # Powertrain DBC uses 0x2BF for DAS_control (legacy HW2)
    data = self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)[1]
    values["DAS_controlChecksum"] = self.checksum(0x2BF, data[:7])
    return self.packers[CANBUS.powertrain].make_can_msg("DAS_control", CANBUS.powertrain, values)

  def create_steering_allowed(self, counter):
    values = {
      "APS_eacMonitorCounter": counter,
      "APS_eacAllow": 1,
    }

    data = self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)[1]
    values["APS_eacMonitorChecksum"] = self.checksum(0x27D, data[:2])
    return self.packers[CANBUS.party].make_can_msg("APS_eacMonitor", CANBUS.party, values)

  # --- HUD ownership (AEB-flash suppression) ---
  # DAS_status (0x399) and DAS_status2 (0x389) are the two frames the IC renders the
  # factory AEB/FCW flash from. On an external panda the stock frames reach the IC without
  # passing through the panda, so they can't be scrubbed in transit. Instead we transmit a
  # complete, clean copy from userspace every cycle so OP becomes the most-recent coherent
  # source the IC latches. Warning fields held at 0; counter rolls 0-15; checksum computed
  # in Python (external panda does not recompute on TX). Emit the WHOLE group together and
  # only while engaged (see carcontroller._process_hud_status) to avoid IC oscillation.
  def create_das_status(self, counter, op_status, fcw, ldw, hands_on_state, alca_state,
                        blind_left, blind_right, speed_limit, fleet_state):
    # 0x399 on this chassis DBC is message "AutopilotStatus" with state field
    # "autopilotStatus" (low nibble) — NOT "DAS_status"/"DAS_autopilotState".
    # Wrong names make CANPacker fall back to addr 0x0, so the clean frame never
    # overrides the stock 0x399 and the IC keeps flashing AEB/FCW.
    values = {
      "autopilotStatus": op_status,                          # 0-4 low nibble; 5 ~ engaged
      "DAS_forwardCollisionWarning": 0x01 if fcw else 0x03,  # 0x03 = SNA = no warning (match stock AP / 0x389 longColl SNA parity). 0=NONE renders the AEB/FCW icon on this IC.
      "DAS_blindSpotRearLeft": 1 if blind_left else 0,
      "DAS_blindSpotRearRight": 1 if blind_right else 0,
      "DAS_fusedSpeedLimit": speed_limit,
      "DAS_visionOnlySpeedLimit": speed_limit,
      "DAS_laneDepartureWarning": ldw,
      "DAS_autopilotHandsOnState": hands_on_state,
      "DAS_autoLaneChangeState": alca_state,
      "DAS_fleetSpeedState": fleet_state,
      "DAS_sideCollisionWarning": 0,
      "DAS_sideCollisionAvoid": 0,
      "DAS_statusCounter": counter,
      "DAS_statusChecksum": 0,
    }
    data = self.packers[CANBUS.party].make_can_msg("AutopilotStatus", CANBUS.party, values)[1]
    values["DAS_statusChecksum"] = self.checksum(0x399, data[:7])
    return self.packers[CANBUS.party].make_can_msg("AutopilotStatus", CANBUS.party, values)

  def create_das_status2(self, counter, acc_speed_limit, fcw):
    values = {
      "DAS_accSpeedLimit": acc_speed_limit,
      # [PMM FIX] full Unity-parity 'healthy DAS' frame so the IC latches THIS (sev=0) over the
      # factory's spurious sev=6. Missing radarTelemetry/csaState/ppOffsetDesiredRamp were why the
      # earlier clean-frame transmit (Option A) didn't override the factory copy.
      "DAS_pmmObstacleSeverity": 0,                          # PMM_NONE (explicit)
      "DAS_pmmLoggingRequest": 0,
      "DAS_activationFailureStatus": 0,                      # AEB/activation HUD trigger -> hold 0
      "DAS_pmmUltrasonicsFaultReason": 0,
      "DAS_pmmRadarFaultReason": 0,
      "DAS_pmmSysFaultReason": 0,
      "DAS_pmmCameraFaultReason": 0,
      "DAS_ACC_report": 1,
      "DAS_csaState": 2,                                     # CSA_EXTERNAL_STATE_ENABLE (Unity, engaged)
      "DAS_radarTelemetry": 1,                               # RADAR_TELEMETRY_NORMAL (Unity)
      "DAS_robState": 2,                                     # ROB_STATE_ACTIVE
      "DAS_driverInteractionLevel": 0,
      "DAS_ppOffsetDesiredRamp": 0x80,                       # PP_NO_OFFSET (Unity)
      "DAS_longCollisionWarning": 0x01 if fcw else 0x0F,     # 0x0F = SNA = no warning (Unity)
      "DAS_status2Counter": counter,
      "DAS_status2Checksum": 0,
    }
    data = self.packers[CANBUS.party].make_can_msg("DAS_status2", CANBUS.party, values)[1]
    values["DAS_status2Checksum"] = self.checksum(0x389, data[:7])
    return self.packers[CANBUS.party].make_can_msg("DAS_status2", CANBUS.party, values)


# Clarity alias: HW2 is "legacy"
TeslaCANLegacy = TeslaCANRaven
