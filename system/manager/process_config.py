import os
import operator
import platform

from cereal import car, custom
from openpilot.common.params import Params
from openpilot.system.hardware import PC, TICI
from openpilot.system.manager.process import PythonProcess, NativeProcess, DaemonProcess
# V235: the model selector is optional. Never let a selector import regression
# prevent manager from starting the stock openpilot process set.
try:
  from openpilot.sunnypilot.models.helpers import get_active_model_runner
  XNOR_MODEL_SELECTOR_AVAILABLE = True
  XNOR_MODEL_SELECTOR_IMPORT_ERROR = ""
except Exception as e:
  XNOR_MODEL_SELECTOR_AVAILABLE = False
  XNOR_MODEL_SELECTOR_IMPORT_ERROR = repr(e)
  print(f"XNOR_MODEL_SELECTOR: disabled for this boot: {XNOR_MODEL_SELECTOR_IMPORT_ERROR}")

  def get_active_model_runner(params: Params | None = None, force_check: bool = False):
    return custom.ModelManagerSP.Runner.stock

WEBCAM = os.getenv("USE_WEBCAM") is not None

def driverview(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started or params.get_bool("IsDriverViewEnabled")

def notcar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and CP.notCar

def iscar(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not CP.notCar

def logging(started: bool, params: Params, CP: car.CarParams) -> bool:
  run = (not CP.notCar) or not params.get_bool("DisableLogging")
  return started and run

def ublox_available() -> bool:
  return os.path.exists('/dev/ttyHS0') and not os.path.exists('/persist/comma/use-quectel-gps')

def ublox(started: bool, params: Params, CP: car.CarParams) -> bool:
  use_ublox = ublox_available()
  if use_ublox != params.get_bool("UbloxAvailable"):
    params.put_bool("UbloxAvailable", use_ublox)
  return started and use_ublox

def joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("JoystickDebugMode")

def not_joystick(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("JoystickDebugMode")

def long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and params.get_bool("LongitudinalManeuverMode")

def not_long_maneuver(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not params.get_bool("LongitudinalManeuverMode")

def qcomgps(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started and not ublox_available()

def always_run(started: bool, params: Params, CP: car.CarParams) -> bool:
  return True

def only_onroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return started

def only_offroad(started: bool, params: Params, CP: car.CarParams) -> bool:
  return not started

def tesla_vision_speed_limit(started: bool, params: Params, CP: car.CarParams) -> bool:
  """Run the optional V1-UK sign recogniser only for Tesla while onroad."""
  try:
    return bool(started and getattr(CP, "brand", "") == "tesla" and params.get_bool("VisionSpeedLimitDetection"))
  except Exception:
    return False

_XNOR_MODEL_SELECTOR_RUNTIME_FAILED = False
_XNOR_MODEL_SELECTOR_RUNTIME_ERROR = ""

def _safe_active_model_runner(started: bool, params: Params):
  """Never allow the optional model selector to take manager down.

  V236: V235 protected only the initial import.  The selector is also called from
  process should_run predicates, so an API/runtime mismatch there must fail closed
  to the stock model rather than escaping through ensure_running().
  """
  global _XNOR_MODEL_SELECTOR_RUNTIME_FAILED, _XNOR_MODEL_SELECTOR_RUNTIME_ERROR
  if not XNOR_MODEL_SELECTOR_AVAILABLE or _XNOR_MODEL_SELECTOR_RUNTIME_FAILED:
    return custom.ModelManagerSP.Runner.stock

  try:
    return get_active_model_runner(params, force_check=not started)
  except Exception as e:
    _XNOR_MODEL_SELECTOR_RUNTIME_FAILED = True
    _XNOR_MODEL_SELECTOR_RUNTIME_ERROR = repr(e)
    print(f"XNOR_MODEL_SELECTOR: runtime failure; forcing stock model for this boot: {_XNOR_MODEL_SELECTOR_RUNTIME_ERROR}")
    try:
      params.remove("ModelRunnerTypeCache")
    except Exception:
      pass
    return custom.ModelManagerSP.Runner.stock

def model_selector_healthy(started: bool, params: Params, CP: car.CarParams) -> bool:
  return XNOR_MODEL_SELECTOR_AVAILABLE and not _XNOR_MODEL_SELECTOR_RUNTIME_FAILED

def is_tinygrad_model(started: bool, params: Params, CP: car.CarParams) -> bool:
  return _safe_active_model_runner(started, params) == custom.ModelManagerSP.Runner.tinygrad

def is_stock_model(started: bool, params: Params, CP: car.CarParams) -> bool:
  return _safe_active_model_runner(started, params) == custom.ModelManagerSP.Runner.stock

def or_(*fns):
  return lambda *args: operator.or_(*(fn(*args) for fn in fns))

def and_(*fns):
  return lambda *args: operator.and_(*(fn(*args) for fn in fns))

procs = [
  DaemonProcess("manage_athenad", "system.athena.manage_athenad", "AthenadPid"),

  NativeProcess("loggerd", "system/loggerd", ["./loggerd"], logging),
  NativeProcess("encoderd", "system/loggerd", ["./encoderd"], only_onroad),
  NativeProcess("stream_encoderd", "system/loggerd", ["./encoderd", "--stream"], notcar),
  PythonProcess("logmessaged", "system.logmessaged", always_run),

  NativeProcess("camerad", "system/camerad", ["./camerad"], driverview, enabled=not WEBCAM),
  PythonProcess("webcamerad", "tools.webcam.camerad", driverview, enabled=WEBCAM),
  PythonProcess("proclogd", "system.proclogd", only_onroad, enabled=platform.system() != "Darwin"),
  PythonProcess("journald", "system.journald", only_onroad, platform.system() != "Darwin"),
  PythonProcess("micd", "system.micd", iscar),
  PythonProcess("timed", "system.timed", always_run, enabled=not PC),

  PythonProcess("modeld", "selfdrive.modeld.modeld", and_(only_onroad, is_stock_model)),
  PythonProcess("dmonitoringmodeld", "selfdrive.modeld.dmonitoringmodeld", driverview, enabled=(WEBCAM or not PC)),

  PythonProcess("sensord", "system.sensord.sensord", only_onroad, enabled=not PC),
  PythonProcess("ui", "selfdrive.ui.ui", always_run, restart_if_crash=True),
  PythonProcess("soundd", "selfdrive.ui.soundd", driverview),
  PythonProcess("locationd", "selfdrive.locationd.locationd", only_onroad),
  NativeProcess("mapd", "selfdrive", ["./mapd"], always_run),
  PythonProcess("speedlimitvisiond", "selfdrive.speed_limit_vision_uk", tesla_vision_speed_limit),
  NativeProcess("_pandad", "selfdrive/pandad", ["./pandad"], always_run, enabled=False),
  PythonProcess("calibrationd", "selfdrive.locationd.calibrationd", only_onroad),
  PythonProcess("torqued", "selfdrive.locationd.torqued", only_onroad),
  PythonProcess("controlsd", "selfdrive.controls.controlsd", and_(not_joystick, iscar)),
  PythonProcess("joystickd", "tools.joystick.joystickd", or_(joystick, notcar)),
  PythonProcess("selfdrived", "selfdrive.selfdrived.selfdrived", only_onroad),
  PythonProcess("card", "selfdrive.car.card", only_onroad),
  PythonProcess("deleter", "system.loggerd.deleter", always_run),
  PythonProcess("dmonitoringd", "selfdrive.monitoring.dmonitoringd", driverview, enabled=(WEBCAM or not PC)),
  PythonProcess("qcomgpsd", "system.qcomgpsd.qcomgpsd", qcomgps, enabled=TICI),
  PythonProcess("pandad", "selfdrive.pandad.pandad", always_run),
  PythonProcess("paramsd", "selfdrive.locationd.paramsd", only_onroad),
  PythonProcess("lagd", "selfdrive.locationd.lagd", only_onroad),
  PythonProcess("ubloxd", "system.ubloxd.ubloxd", ublox, enabled=TICI),
  PythonProcess("pigeond", "system.ubloxd.pigeond", ublox, enabled=TICI),
  PythonProcess("plannerd", "selfdrive.controls.plannerd", not_long_maneuver),
  PythonProcess("maneuversd", "tools.longitudinal_maneuvers.maneuversd", long_maneuver),
  PythonProcess("radard", "selfdrive.controls.radard", only_onroad),
  PythonProcess("hardwared", "system.hardware.hardwared", always_run),
  PythonProcess("tombstoned", "system.tombstoned", always_run, enabled=not PC),
  PythonProcess("updated", "system.updated.updated", only_offroad, enabled=not PC),
  PythonProcess("uploader", "system.loggerd.uploader", always_run),
  PythonProcess("statsd", "system.statsd", always_run),
  PythonProcess("feedbackd", "selfdrive.ui.feedback.feedbackd", only_onroad),

  # debug procs
  NativeProcess("bridge", "cereal/messaging", ["./bridge"], notcar),
  PythonProcess("webrtcd", "system.webrtc.webrtcd", notcar),
  PythonProcess("webjoystick", "tools.bodyteleop.web", notcar),
  PythonProcess("joystick", "tools.joystick.joystick_control", and_(joystick, iscar)),
]

# XNOR Sunnypilot model selector port
procs += [
  PythonProcess("models_manager", "sunnypilot.models.manager", and_(only_offroad, model_selector_healthy), enabled=XNOR_MODEL_SELECTOR_AVAILABLE),
  NativeProcess("modeld_tinygrad", "sunnypilot/modeld_v2", ["./modeld"], and_(only_onroad, is_tinygrad_model), enabled=XNOR_MODEL_SELECTOR_AVAILABLE),
]

managed_processes = {p.name: p for p in procs}
