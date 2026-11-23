from opendbc.car import Bus, get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.tesla.carcontroller import CarController
from opendbc.car.tesla.carstate import CarState
from opendbc.car.tesla.values import TeslaSafetyFlags, TeslaFlags, CANBUS, CAR, DBC, FSD_14_FW, Ecu, TeslaLegacyParams, LEGACY_CARS
from opendbc.car.tesla.radar_interface import RadarInterface, RADAR_START_ADDR


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def _hybrid_selection() -> tuple[bool, bool, bool]:
    """Read only persistent Tesla configuration; this is not an authority signal.

    The packet heartbeat and the hardware-specific safety dispatcher still
    determine steering/longitudinal ownership independently.
    """
    # Standalone opendbc unit tests may not have compiled openpilot Params.
    # Fall back to *no Hybrid request*, never an assumed ownership grant.
    try:
      from openpilot.common.params import Params
      p = Params()
    except (ImportError, OSError):
      return False, False, False
    disabled = bool(p.get_bool("TinklaAutopilotDisabled"))
    hybrid = bool(p.get_bool("TinklaHybridNativeAP")) and not disabled
    # Do not suppress stock AEB warnings in this unqualified native port.
    # The historical HUD override and native DAS_lssState layouts conflict.
    ignore_hud_aeb = False
    return disabled, hybrid, ignore_hud_aeb

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    if candidate in LEGACY_CARS:
      return CarInterface._get_params_sx(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs)

    ret.brand = "tesla"

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.tesla)]

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle

    # Model X and HW 2.5 vehicles are missing DAS_settings
    if 0x293 not in fingerprint[CANBUS.autopilot_party]:
      ret.flags |= TeslaFlags.MISSING_DAS_SETTINGS.value

    # Radar support is intended to work for:
    # - Tesla Model 3 vehicles built approximately mid-2017 through early-2021
    # - Tesla Model Y vehicles built approximately mid-2020 through early-2021
    # - Vehicles equipped with the Continental ARS4-B radar (used on HW2 / HW2.5 / early HW3)
    # - Radar CAN lines must be tapped and connected to CAN bus 1 (normally not used for tesla vehicles)
    ret.radarUnavailable = RADAR_START_ADDR not in fingerprint[1] or Bus.radar not in DBC[candidate]

    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value

    fsd_14 = any(fw.ecu == Ecu.eps and fw.fwVersion in FSD_14_FW.get(candidate, []) for fw in car_fw)
    if fsd_14:
      ret.flags |= TeslaFlags.FSD_14.value
      ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.FSD_14.value

      # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret

  @staticmethod
  def _get_params_sx(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "tesla"

    if not any(0x201 in f for f in fingerprint.values()):
      ret.flags |= TeslaLegacyParams.NO_SDM1.value

    if candidate in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
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

    ret.steerLimitTimer = 0.4
    ret.steerActuatorDelay = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = candidate in (CAR.TESLA_MODEL_S_HW2, )

    ret.alphaLongitudinalAvailable = True
    # Legacy HW1/HW2/HW3 uses two distinct PANDA safety contexts for HW2/3.
    # Both require LONG_CONTROL. The stock XNOR interface set it only on the
    # main Panda, leaving the external 0x2BF sender without matching ownership.
    ret.openpilotLongitudinalControl = True
    autopilot_disabled, hybrid_native_ap, ignore_hud_aeb = CarInterface._hybrid_selection()
    ret.pcmCruise = hybrid_native_ap
    for cfg in ret.safetyConfigs:
      cfg.safetyParam |= int(TeslaSafetyFlags.LONG_CONTROL)
      if autopilot_disabled or hybrid_native_ap:
        cfg.safetyParam |= int(TeslaSafetyFlags.OP_STALK_ENABLE)
      if ignore_hud_aeb:
        cfg.safetyParam |= int(TeslaSafetyFlags.IGNORE_STOCK_AEB)

    # Independent safety checks still gate actual longitudinal control.
    ret.minEnableSpeed = -1.
    # vEgoStopping/vEgoStarting/stoppingDecelRate are deprecated group
    # members in pinned 0.11.2 capnp; setting them as root fields causes
    # CarInterface.get_non_essential_params() to raise AttributeError.
    # Use the native 0.11.2 longitudinal defaults until replay-qualified.

    # ret.dashcamOnly = candidate in (CAR.TESLA_MODEL_X) # dashcam only, pending find invalidLkasSetting signal

    return ret
