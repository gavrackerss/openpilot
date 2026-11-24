"""SunnyPilot downloaded-model runtime adapted to XNOR openpilot 0.11.2.

Only verified bundles selected by ModelManagerSP reach this process. Stock
modeld remains a separate manager process and is the fail-closed default.
"""
from __future__ import annotations

import os
from openpilot.common.hardware import COMMA_HARDWARE
os.environ.setdefault("DEV", "QCOM" if COMMA_HARDWARE else "CPU")

import pickle
import time
import numpy as np
from tinygrad.device import Device
from tinygrad.tensor import Tensor

import openpilot.cereal.messaging as messaging
from openpilot.cereal import log
from openpilot.cereal.messaging import PubMaster, SubMaster
from openpilot.cereal.visionipc import VisionStreamType
from msgq.visionipc import VisionIpcClient, VisionBuf
from opendbc.car.car_helpers import get_demo_car_params
from opendbc.car.structs import car

from openpilot.common.file_chunker import open_file_chunked
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.drive_helpers import get_accel_from_plan, get_curvature_from_plan, should_stop, smooth_value
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.fill_model_msg import fill_driving_model_data, fill_model_msg, fill_pose_msg, PublishState
from openpilot.selfdrive.modeld.parse_model_outputs import Parser as CurrentParser
from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
from openpilot.sunnypilot.modeld_v2.parse_model_outputs import Parser as LegacyCombinedParser
from openpilot.sunnypilot.modeld_v2.parse_model_outputs_split import Parser as SplitParser
from openpilot.sunnypilot.modeld_v2.runtime_helpers import derive_frame_skip, detect_desire_key, make_split_input_queues, make_supercombo_input_queues
from openpilot.sunnypilot.models.helpers import get_active_bundle
from openpilot.sunnypilot.models.paths import model_root
from openpilot.sunnypilot.models.split_model_constants import SplitModelConstants

PROCESS_NAME = "openpilot.sunnypilot.modeld_v2.modeld"


def _pkl_exists(path: str) -> bool:
  from openpilot.common.file_chunker import get_manifest_path
  return os.path.isfile(path) or os.path.isfile(get_manifest_path(path))


def _find_driving_pkl(bundle) -> str | None:
  override = os.environ.get("COMBINED_MODEL_PKL")
  if override and _pkl_exists(override):
    return override
  if bundle is None:
    return None
  for model in bundle.models:
    name = getattr(model.artifact, "fileName", "")
    if name:
      path = os.path.join(model_root(), name)
      if _pkl_exists(path):
        return path
  return None


class FrameMeta:
  frame_id = 0
  timestamp_sof = 0
  timestamp_eof = 0

  def __init__(self, vipc=None):
    if vipc is not None:
      self.frame_id = vipc.frame_id
      self.timestamp_sof = vipc.timestamp_sof
      self.timestamp_eof = vipc.timestamp_eof


class SunnyPilotModelState:
  def __init__(self, cam_w: int, cam_h: int):
    self.bundle = get_active_bundle()
    if self.bundle is None:
      raise RuntimeError("No active SunnyPilot model bundle")
    self.generation = int(self.bundle.generation)
    self.is_20hz = bool(self.bundle.is20hz)
    overrides = {o.key: o.value for o in self.bundle.overrides}
    self.lat_smooth = float(overrides.get("lat", "0") or 0)
    self.long_smooth = float(overrides.get("long", "0") or 0)
    self.min_lat_speed = 0.3
    self.planplus_control = 1.0

    pkl_path = _find_driving_pkl(self.bundle)
    if pkl_path is None:
      raise FileNotFoundError("Selected SunnyPilot combined PKL is not installed")

    cloudlog.warning("loading SunnyPilot bundle %s (%s) from %s",
                     self.bundle.internalName, self.bundle.ref, pkl_path)
    with open_file_chunked(pkl_path) as stream:
      jits = pickle.load(stream)

    metadata = jits["metadata"]
    if "model" in metadata:
      self.combined_type = "supercombo"
      model_meta = metadata["model"]
      self.output_slices = model_meta["output_slices"]
      self.policy_slices = []
      self.policy_keys = []
      self.vision_input_names = [k for k in model_meta["input_shapes"] if "img" in k]
      frame_skip = derive_frame_skip({}, model_meta["input_shapes"])
      self.input_queues, self.npy = make_supercombo_input_queues(model_meta["input_shapes"], frame_skip, Device.DEFAULT)
      # Older v17 supercombos use MHP plan heads; current Gen12 bundles do not.
      self.parser = CurrentParser() if self.generation >= 12 else LegacyCombinedParser()
    else:
      vision_meta = metadata["vision"]
      self.policy_keys = [k for k in metadata if k != "vision"]
      if not self.policy_keys:
        raise ValueError("SunnyPilot split bundle contains no policy metadata")
      self.combined_type = "split" if self.policy_keys == ["policy"] else "multi_policy"
      self.output_slices = vision_meta["output_slices"]
      self.policy_slices = [metadata[k]["output_slices"] for k in self.policy_keys]
      self.has_on_policy = any("on" in k.lower() for k in self.policy_keys)
      first_policy = metadata[self.policy_keys[0]]
      self.vision_input_names = [k for k in vision_meta["input_shapes"] if "img" in k]
      frame_skip = derive_frame_skip(vision_meta["input_shapes"], first_policy["input_shapes"])
      self.input_queues, self.npy = make_split_input_queues(
        vision_meta["input_shapes"], first_policy["input_shapes"], frame_skip, Device.DEFAULT)
      self.parser = SplitParser()

    if len(self.vision_input_names) < 1:
      raise ValueError("SunnyPilot bundle has no image inputs")

    self.constants = SplitModelConstants if self.is_20hz else ModelConstants
    self.model_freq = int(getattr(self.constants, "MODEL_FREQ", ModelConstants.MODEL_RUN_FREQ))
    self.desire_len = int(self.constants.DESIRE_LEN)
    self.desire_key = detect_desire_key(self.npy)
    self.prev_desire = np.zeros(self.desire_len, dtype=np.float32)
    self.full_frames: dict[str, Tensor] = {}
    self.blob_cache: dict[tuple[str, int], Tensor] = {}
    nv12 = get_nv12_info(cam_w, cam_h)
    self.frame_buf_params = dict.fromkeys(self.vision_input_names, nv12)

    camera_jits = jits.get((cam_w, cam_h))
    if camera_jits is None:
      raise ValueError(f"SunnyPilot PKL has no JIT for camera resolution {cam_w}x{cam_h}")
    self.run_policy = camera_jits["run_policy"]
    self.warp_enqueue = camera_jits["warp_enqueue"]

    road_name = next((k for k in self.vision_input_names if "big" not in k), self.vision_input_names[0])
    yuv_size = self.frame_buf_params[road_name][3]
    blank = Tensor(np.zeros(yuv_size, dtype=np.uint8), device=Device.DEFAULT).contiguous().realize()
    self.warp_enqueue(**self.input_queues, frame=blank, big_frame=blank)

  def _slice(self, raw: np.ndarray, slices: dict) -> dict[str, np.ndarray]:
    return {k: raw[np.newaxis, v] for k, v in slices.items()}

  def run(self, bufs: dict[str, VisionBuf], transforms: dict[str, np.ndarray],
          inputs: dict[str, np.ndarray], prepare_only: bool) -> dict[str, np.ndarray] | None:
    for key, buf in bufs.items():
      ptr = np.frombuffer(buf.data, dtype=np.uint8).ctypes.data
      yuv_size = self.frame_buf_params[key][3]
      cache_key = (key, ptr)
      if cache_key not in self.blob_cache:
        self.blob_cache[cache_key] = Tensor.from_blob(ptr, (yuv_size,), dtype="uint8", device=Device.DEFAULT)
      self.full_frames[key] = self.blob_cache[cache_key]

    desire = inputs[self.desire_key]
    self.npy[self.desire_key][:] = np.where(desire - self.prev_desire > .99, desire, 0)
    self.prev_desire[:] = desire

    for key in ("traffic_convention", "lateral_control_params", "action_t"):
      if key in self.npy and key in inputs:
        self.npy[key][:] = inputs[key]

    road_key = next((k for k in bufs if "big" not in k), next(iter(bufs)))
    wide_key = next((k for k in bufs if "big" in k), road_key)
    self.npy["tfm"][:, :] = transforms[road_key].reshape(3, 3)
    self.npy["big_tfm"][:, :] = transforms[wide_key].reshape(3, 3)

    if prepare_only:
      self.warp_enqueue(**self.input_queues, frame=self.full_frames[road_key], big_frame=self.full_frames[wide_key])
      return None

    raw_outputs = self.run_policy(**self.input_queues, frame=self.full_frames[road_key], big_frame=self.full_frames[wide_key])

    if self.combined_type == "supercombo":
      raw = raw_outputs.numpy().flatten()
      return self.parser.parse_outputs(self._slice(raw, self.output_slices))

    vision_raw = raw_outputs[0].numpy().flatten()
    outputs = self.parser.parse_vision_outputs(self._slice(vision_raw, self.output_slices))
    for i, policy_slices in enumerate(self.policy_slices):
      policy_raw = raw_outputs[i + 1].numpy().flatten()
      parsed = self.parser.parse_policy_outputs(self._slice(policy_raw, policy_slices))
      if "off" in self.policy_keys[i].lower() and getattr(self, "has_on_policy", False):
        parsed.pop("plan", None)
      outputs.update(parsed)

    if "planplus" in outputs and "plan" in outputs:
      outputs["plan"] = outputs["plan"] + outputs["planplus"]

    if "desired_curvature" in outputs and "prev_desired_curv" in self.npy:
      buf = self.npy["prev_desired_curv"]
      buf[0, :-1] = buf[0, 1:]
      buf[0, -1, :] = outputs["desired_curvature"][0, :]
    return outputs

  def action(self, output: dict[str, np.ndarray], prev: log.ModelDataV2.Action,
             lat_action_t: float, long_action_t: float, v_ego: float) -> log.ModelDataV2.Action:
    if "action" in output:
      desired_accel = float(output["action"][0, 1])
      desired_curvature = float(output["action"][0, 0]) / max(1.0, v_ego) ** 2
    else:
      plan = output["plan"][0]
      desired_accel = float(get_accel_from_plan(
        plan[:, Plan.VELOCITY][:, 0], plan[:, Plan.ACCELERATION][:, 0],
        self.constants.T_IDXS, action_t=long_action_t))
      curvature_plan = plan
      if "planplus" in output and self.planplus_control != 1.0:
        curvature_plan = plan + (self.planplus_control - 1.0) * output["planplus"][0]
      if "desired_curvature" in output:
        desired_curvature = float(output["desired_curvature"][0, 0])
      else:
        desired_curvature = float(get_curvature_from_plan(
          curvature_plan[:, Plan.T_FROM_CURRENT_EULER][:, 2],
          curvature_plan[:, Plan.ORIENTATION_RATE][:, 2],
          self.constants.T_IDXS, v_ego, lat_action_t))

    desired_accel = smooth_value(desired_accel, prev.desiredAcceleration, self.long_smooth)
    if v_ego > self.min_lat_speed:
      desired_curvature = smooth_value(desired_curvature, prev.desiredCurvature, self.lat_smooth)
    else:
      desired_curvature = prev.desiredCurvature

    return log.ModelDataV2.Action(
      desiredCurvature=float(desired_curvature),
      desiredAcceleration=float(desired_accel),
      shouldStop=bool(should_stop(v_ego, desired_accel)),
    )


def main(demo: bool = False):
  cloudlog.warning("SunnyPilot modeld init")
  if not COMMA_HARDWARE:
    raise RuntimeError("SunnyPilot QCOM runtime is only enabled on comma hardware")
  config_realtime_process(7, 54)

  while True:
    streams = VisionIpcClient.available_streams("camerad", block=False)
    if streams:
      use_extra = VisionStreamType.VISION_STREAM_WIDE_ROAD in streams and VisionStreamType.VISION_STREAM_NARROW_ROAD in streams
      main_wide = VisionStreamType.VISION_STREAM_NARROW_ROAD not in streams
      break
    time.sleep(.1)

  main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide else VisionStreamType.VISION_STREAM_NARROW_ROAD
  vipc_main = VisionIpcClient("camerad", main_stream, True)
  vipc_extra = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_WIDE_ROAD, False)
  while not vipc_main.connect(False):
    time.sleep(.1)
  while use_extra and not vipc_extra.connect(False):
    time.sleep(.1)

  cloudlog.warning("SunnyPilot camera %dx%d extra=%s", vipc_main.width, vipc_main.height, use_extra)
  model = SunnyPilotModelState(vipc_main.width, vipc_main.height)
  cloudlog.warning("SunnyPilot model loaded: %s", model.bundle.internalName)

  pm = PubMaster(["modelV2", "drivingModelData", "cameraOdometry"])
  sm = SubMaster(["deviceState", "carState", "narrowRoadCameraState", "extrinsicsCalibration",
                  "driverMonitoringState", "carControl", "lateralDelay"])
  params = Params()
  publish_state = PublishState()

  frame_filter = FirstOrderFilter(0., 10., 1. / model.model_freq)
  frame_id = last_vipc_frame_id = run_count = 0
  transform_main = np.zeros((3, 3), dtype=np.float32)
  transform_extra = np.zeros((3, 3), dtype=np.float32)
  extrinsics_seen = False
  buf_main = buf_extra = None
  meta_main, meta_extra = FrameMeta(), FrameMeta()

  CP = get_demo_car_params() if demo else messaging.log_from_bytes(params.get("CarParams", block=True), car.CarParams)
  long_delay = CP.longitudinalActuatorDelay + model.long_smooth
  prev_action = log.ModelDataV2.Action()
  desire_helper = DesireHelper()

  while True:
    while meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
      buf_main = vipc_main.recv()
      meta_main = FrameMeta(vipc_main)
      if buf_main is None:
        break
    if buf_main is None:
      continue

    if use_extra:
      while True:
        buf_extra = vipc_extra.recv()
        meta_extra = FrameMeta(vipc_extra)
        if buf_extra is None or meta_main.timestamp_sof < meta_extra.timestamp_sof + 25000000:
          break
      if buf_extra is None:
        continue
    else:
      buf_extra, meta_extra = buf_main, meta_main

    sm.update(0)
    desire = desire_helper.desire
    is_rhd = sm["driverMonitoringState"].isRHD
    frame_id = sm["narrowRoadCameraState"].frameId
    v_ego = max(sm["carState"].vEgo, 0.)
    lat_delay = sm["lateralDelay"].lateralDelay + model.lat_smooth

    if sm.updated["extrinsicsCalibration"] and sm.seen["narrowRoadCameraState"] and sm.seen["deviceState"]:
      euler = np.array(sm["extrinsicsCalibration"].rpyCalib, dtype=np.float32)
      dc = DEVICE_CAMERAS[(str(sm["deviceState"].deviceType), str(sm["narrowRoadCameraState"].sensor))]
      main_intrinsics = dc.wide_road.intrinsics if main_wide else dc.narrow_road.intrinsics
      extra_intrinsics = dc.wide_road.intrinsics if (use_extra or main_wide) else dc.narrow_road.intrinsics
      transform_main = get_warp_matrix(euler, main_intrinsics, False).astype(np.float32)
      transform_extra = get_warp_matrix(euler, extra_intrinsics, True).astype(np.float32)
      extrinsics_seen = True

    traffic = np.zeros(2, dtype=np.float32)
    traffic[int(is_rhd)] = 1
    vec_desire = np.zeros(model.desire_len, dtype=np.float32)
    if 0 <= desire < model.desire_len:
      vec_desire[desire] = 1

    dropped = max(0, meta_main.frame_id - last_vipc_frame_id - 1)
    frames_dropped = frame_filter.update(min(dropped, 10))
    if run_count < 10:
      frame_filter.x = 0.
      frames_dropped = 0.
    run_count += 1
    drop_ratio = frames_dropped / (1 + frames_dropped)
    prepare_only = dropped > 0

    bufs = {name: buf_extra if "big" in name else buf_main for name in model.vision_input_names}
    transforms = {name: transform_extra if "big" in name else transform_main for name in model.vision_input_names}
    frame_delay = DT_MDL
    action_delay = DT_MDL / 2
    lat_action_t = lat_delay + frame_delay + action_delay
    long_action_t = long_delay + frame_delay + action_delay
    inputs = {
      model.desire_key: vec_desire,
      "traffic_convention": traffic,
      "lateral_control_params": np.array([v_ego, lat_delay], dtype=np.float32),
      "action_t": np.array([lat_action_t, long_action_t], dtype=np.float32),
    }

    start = time.perf_counter()
    output = model.run(bufs, transforms, inputs, prepare_only)
    execution_time = time.perf_counter() - start

    if output is not None:
      model_msg = messaging.new_message("modelV2")
      driving_msg = messaging.new_message("drivingModelData")
      pose_msg = messaging.new_message("cameraOdometry")

      action = model.action(output, prev_action, lat_action_t, long_action_t, v_ego)
      prev_action = action
      fill_model_msg(model_msg, output, action, publish_state,
                     meta_main.frame_id, meta_extra.frame_id, frame_id,
                     drop_ratio, meta_main.timestamp_eof, execution_time, extrinsics_seen)
      model_msg.modelV2.big = False

      desire_state = model_msg.modelV2.meta.desireState
      lane_change_prob = desire_state[log.Desire.laneChangeLeft] + desire_state[log.Desire.laneChangeRight]
      desire_helper.update(sm["carState"], sm["carControl"].latActive, lane_change_prob)
      model_msg.modelV2.meta.laneChangeState = desire_helper.lane_change_state
      model_msg.modelV2.meta.laneChangeDirection = desire_helper.lane_change_direction

      fill_driving_model_data(driving_msg, model_msg)
      fill_pose_msg(pose_msg, output, meta_main.frame_id, dropped, meta_main.timestamp_eof, extrinsics_seen)
      pm.send("modelV2", model_msg)
      pm.send("drivingModelData", driving_msg)
      pm.send("cameraOdometry", pose_msg)

    last_vipc_frame_id = meta_main.frame_id


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument("--demo", action="store_true")
  args = parser.parse_args()
  main(demo=args.demo)
