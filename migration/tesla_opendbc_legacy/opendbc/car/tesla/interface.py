from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, DBC, FSD_14_FW, Ecu, TeslaLegacyParams, LEGACY_CARS
from opendbc.car.tesla.radar_interface import RadarInterface, RADAR_START_ADDR
from openpilot.common.params import Params
import cereal.messaging as messaging


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def __init__(self, CP: structs.CarParams):
    super().__init__(CP)
    self._model_v2_sock = messaging.sub_sock('modelV2', conflate=True)
    self._ic_model_data = None

  def post_update(self, c: structs.CarControl, ret: structs.CarState) -> None:
    try:
      self.CS.human_control = bool(self.CS.hso_controller.update_stat(self.CS, bool(getattr(c, 'latActive', False)), c.actuators, int(self.CS._param_frame)))
    except Exception:
      self.CS.human_control = False

    # Unity IC integration uses modelV2 lane probabilities and the model path. Keep the
    # last good sample on CarState so CarController can render the IC lanes at its own cadence.
    # This is deliberately independent of ALC: IC lane rendering must still work with ALC off.
    model_msg = messaging.recv_one_or_none(self._model_v2_sock)
    if model_msg is not None:
      try:
        model_v2 = model_msg.modelV2
        self._ic_model_data = {
          'lane_probs': tuple(float(v) for v in model_v2.laneLineProbs),
          'position_x': tuple(float(v) for v in model_v2.position.x),
          'position_y': tuple(float(v) for v in model_v2.position.y),
        }
      except Exception:
        pass
    self.CS.ic_model_data = self._ic_model_data

    if not getattr(self.CS, 'enableALC', False):
      return

    # ALC is already updated from CarState's model subscriber. An additional
    # call here (usually with no new model frame) reset its phase every cycle.
    # OP auto-lane-change operates independently in Hybrid.
    if (not bool(getattr(self.CS, 'hybrid_native_ap', False))) and getattr(self.CS, 'alca_need_engagement', False):
      ret.steeringPressed = True
      if int(getattr(self.CS, 'alca_direction', 0)) == 1:
        ret.steeringTorque = 2.0
      elif int(getattr(self.CS, 'alca_direction', 0)) == 2:
        ret.steeringTorque = -2.0

  @staticmethod
  def _apply_xnor_safety_flags(ret: structs.CarParams) -> None:
    params = Params()
    autopilot_disabled = bool(params.get_bool("TinklaAutopilotDisabled"))
    hybrid_native_ap = bool(params.get_bool("TinklaHybridNativeAP")) and not autopilot_disabled
    # Both xnor lateral modes use the physical MAIN/CANCEL stalk as the OP engagement source.
    # Set this statically in CarParams so panda can honour the stalk immediately after safety init;
    # do not depend on the runtime 0x659 hybrid bit arriving first.
    if autopilot_disabled or hybrid_native_ap:
      for cfg in ret.safetyConfigs:
        cfg.safetyParam |= int(TeslaSafetyFlags.OP_STALK_ENABLE)

    if params.get_bool("TinklaIgnoreStockAeb"):
      for cfg in ret.safetyConfigs:
        cfg.safetyParam |= int(TeslaSafetyFlags.IGNORE_STOCK_AEB)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    if candidate in LEGACY_CARS:
      return CarInterface._get_params_sx(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs)

    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]
    CarInterface._apply_xnor_safety_flags(ret)

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    if 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in DBC[candidate]

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

      ret.vEgoStopping = 0.1
      ret.vEgoStarting = 0.1
      ret.stoppingDecelRate = 0.3

    fsd_14 = any(fw.ecu == Ecu.eps and fw.fwVersion in FSD_14_FW.get(candidate, []) for fw in car_fw)
    if fsd_14:
      ret.flags |= TeslaFlags.FSD_14.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.FSD_14.value

    return ret

  @staticmethod
  def _get_params_sx(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    if not any(0x201 in f for f in fingerprint.values()):
      ret.flags |= TeslaLegacyParams.NO_SDM1.value

    if candidate in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW1)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_X_HW2):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]
    elif candidate in (CAR.TESLA_MODEL_S_HW3,):
      ret.safetyConfigs = [
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3)),
        get_safety_config(structs.CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)),
      ]

    CarInterface._apply_xnor_safety_flags(ret)

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = candidate in (CAR.TESLA_MODEL_S_HW2,)

    ret.alphaLongitudinalAvailable = True
    params = Params()
    hybrid_native_ap = bool(params.get_bool("TinklaHybridNativeAP")) and not bool(params.get_bool("TinklaAutopilotDisabled"))

    # XNOR_V201_V198_ENGAGEMENT_OP_VCRUISE:
    # Preserve V198's proven Hybrid engagement contract exactly: MAIN raises CarState's independent
    # cruise latch and the normal PCM rising edge engages selfdrive. selfdrive/car/cruise.py splits
    # only vCruise ownership onto OP's original non-PCM logic; this flag is not changed to achieve
    # set-speed ownership and therefore cannot regress the engagement edge again.
    ret.openpilotLongitudinalControl = True
    ret.pcmCruise = hybrid_native_ap

    # Apply LONG_CONTROL to EVERY safety config. HW2 has a main panda for the established
    # non-Hybrid 0x2B9 path and an external panda that transmits OP's 0x2BF. Hybrid does not
    # author 0x2B9, but its external panda still requires LONG_CONTROL for the direct 0x2BF path.
    for cfg in ret.safetyConfigs:
      cfg.safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    # OP-owned longitudinal is allowed from standstill in every xnor longitudinal mode.
    ret.minEnableSpeed = -1.
    ret.vEgoStopping = 0.1
    ret.vEgoStarting = 0.1
    ret.stoppingDecelRate = 0.3

    return ret
