#!/usr/bin/env python3
from __future__ import annotations

import os
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog


# XNOR Vision Speed Limit V1-UK
#
# Runtime design is based on the speed-limit vision pipeline in StarPilot
# (firestar5683/StarPilot, Dom branch), but deliberately uses only the legacy
# single ONNX detector. V1-UK adds a UK red-ring geometry gate, full-frame
# scanning, conservative temporal confirmation and lower-only integration in
# Tesla CarState. It recognises numeric 20/30/40/50/60/70 mph signs.
#
# National Speed Limit and non-numeric restrictions are intentionally not
# handled in V1.

MODEL_PATH = Path(__file__).resolve().parent / "assets" / "vision_models" / "speed_limit_vision.onnx"

RUNTIME_HZ = 20
NORMAL_INFERENCE_INTERVAL = 0.40
FOLLOWUP_INFERENCE_INTERVAL = 0.20
FOLLOWUP_WINDOW_SECONDS = 2.0
MEMORY_PRESSURE_INTERVAL = 1.50
MEMORY_PRESSURE_PERCENT = 88.0
MEMORY_CRITICAL_PERCENT = 94.0
PUBLISHED_HOLD_SECONDS = 300.0
HISTORY_SECONDS = 2.5
MIN_MODEL_CONFIDENCE = 0.10
MIN_CONFIRMED_CONFIDENCE = 0.18
HIGHER_RELEASE_CONFIDENCE = 0.55
INITIAL_REQUIRED_READS = 2
CHANGE_REQUIRED_READS = 2
HIGHER_RELEASE_REQUIRED_READS = 3
MAX_PROPOSALS = 8
MAX_BOX_AREA_RATIO = 0.18
MIN_BOX_WIDTH = 10
MIN_BOX_HEIGHT = 14
DETECTOR_INPUT_SIZE = 640

SUPPORTED_UK_LIMITS_MPH = frozenset((20, 30, 40, 50, 60, 70))
SPEED_LIMIT_CLASSES = {
  6: 20,
  7: 30,
  8: 40,
  9: 50,
  10: 60,
  11: 70,
}


@dataclass
class Detection:
  speed_limit_mph: int
  confidence: float
  model_confidence: float = 0.0
  ring_score: float = 0.0
  bbox: tuple[int, int, int, int] | None = None


@dataclass
class NV12Frame:
  y: np.ndarray
  uv: np.ndarray
  width: int
  height: int


@dataclass
class HistoryEntry:
  speed_limit_mph: int
  confidence: float
  created_at: float


class SpeedLimitVisionUK:
  def __init__(self):
    self.params = Params()
    self.Image = None
    self.VisionIpcClient = None
    self.VisionStreamType = None
    self.net = None
    self.model_input_name = ""
    self.client = None
    self.stream_type = None
    self.stream_name = ""

    self.history: deque[HistoryEntry] = deque()
    self.published_speed_limit_mph = 0
    self.published_confidence = 0.0
    self.last_detection_at = 0.0
    self.last_inference_at = -1e9
    self.followup_until = 0.0
    self.last_status = ""
    self._last_raw_log_signature = None
    self._last_raw_log_at = 0.0

    self.sm = None
    self.runtime_error = ""

  def _set_status(self, text: str) -> None:
    if text == self.last_status:
      return
    self.last_status = text
    try:
      self.params.put_nonblocking("VisionSpeedLimitStatus", text[:160])
    except Exception:
      pass

  def _log_raw_proposal(self, decision: str, speed_limit_mph: int, model_confidence: float,
                        ring_score: float, combined_confidence: float = 0.0,
                        bbox: tuple[int, int, int, int] | None = None) -> None:
    """Log detector/ring-gate state without flooding rlogs with identical frames."""
    now = time.monotonic()
    signature = (
      str(decision),
      int(speed_limit_mph),
      round(float(model_confidence), 2),
      round(float(ring_score), 2),
      round(float(combined_confidence), 2),
    )
    # Rejected raw proposals can repeat every inference. Log a changed proposal
    # immediately, otherwise at most once per second.
    if signature == self._last_raw_log_signature and (now - self._last_raw_log_at) < 1.0:
      return
    self._last_raw_log_signature = signature
    self._last_raw_log_at = now
    bbox_text = "none" if bbox is None else ",".join(str(int(v)) for v in bbox)
    cloudlog.info(
      f"[XNOR_VSL_V1UK] proposal={int(speed_limit_mph)} "
      f"model={float(model_confidence):.3f} ring={float(ring_score):.3f} "
      f"combined={float(combined_confidence):.3f} decision={decision} bbox={bbox_text}"
    )

  def _log_temporal_candidate(self, detection: Detection, count: int, required: int, decision: str) -> None:
    cloudlog.info(
      f"[XNOR_VSL_V1UK] candidate={int(detection.speed_limit_mph)} "
      f"model={float(detection.model_confidence):.3f} ring={float(detection.ring_score):.3f} "
      f"combined={float(detection.confidence):.3f} count={int(count)}/{int(required)} "
      f"decision={decision}"
    )

  def _publish(self, speed_limit_mph: int, confidence: float) -> None:
    now = time.monotonic()
    self.published_speed_limit_mph = int(speed_limit_mph)
    self.published_confidence = float(confidence)
    self.last_detection_at = now
    try:
      self.params.put_nonblocking("VisionSpeedLimit", float(speed_limit_mph) * CV.MPH_TO_MS)
      self.params.put_nonblocking("VisionSpeedLimitConfidence", float(confidence))
      self.params.put_nonblocking("VisionSpeedLimitTimestamp", float(now))
    except Exception:
      pass
    self._set_status(f"UK vision: {speed_limit_mph} mph ({confidence * 100.0:.0f}%)")
    cloudlog.info(f"[XNOR_VSL_V1UK] publish={speed_limit_mph}mph confidence={confidence:.3f}")

  def _refresh_publish_timestamp(self) -> None:
    if self.published_speed_limit_mph <= 0:
      return
    now = time.monotonic()
    self.last_detection_at = now
    try:
      self.params.put_nonblocking("VisionSpeedLimitTimestamp", float(now))
    except Exception:
      pass

  def _clear_publish(self, reason: str) -> None:
    old = self.published_speed_limit_mph
    self.published_speed_limit_mph = 0
    self.published_confidence = 0.0
    self.last_detection_at = 0.0
    self.history.clear()
    try:
      self.params.put_nonblocking("VisionSpeedLimit", 0.0)
      self.params.put_nonblocking("VisionSpeedLimitConfidence", 0.0)
      self.params.put_nonblocking("VisionSpeedLimitTimestamp", 0.0)
    except Exception:
      pass
    self._set_status(f"UK vision: scanning ({reason})")
    if old > 0:
      cloudlog.info(f"[XNOR_VSL_V1UK] clear previous={old}mph reason={reason}")

  def _load_runtime(self) -> bool:
    try:
      from PIL import Image
      from cereal import messaging
      from msgq.visionipc import VisionIpcClient, VisionStreamType
      from tinygrad.nn.onnx import OnnxRunner

      self.Image = Image
      self.VisionIpcClient = VisionIpcClient
      self.VisionStreamType = VisionStreamType
      self.sm = messaging.SubMaster(["deviceState"])
      try:
        os.nice(10)
      except Exception:
        pass
    except Exception as exc:
      self.runtime_error = f"Tinygrad/Pillow/VisionIPC unavailable: {type(exc).__name__}"
      self._set_status(self.runtime_error)
      cloudlog.exception("[XNOR_VSL_V1UK] runtime dependency unavailable")
      return False

    if not MODEL_PATH.is_file():
      self.runtime_error = f"Vision model missing: {MODEL_PATH.name}"
      self._set_status(self.runtime_error)
      cloudlog.error(f"[XNOR_VSL_V1UK] {self.runtime_error}")
      return False

    try:
      self.net = OnnxRunner(str(MODEL_PATH))
      inputs = list(self.net.graph_inputs.items())
      if len(inputs) != 1:
        raise RuntimeError(f"expected one model input, found {len(inputs)}")
      self.model_input_name, input_spec = inputs[0]
      cloudlog.info(
        f"[XNOR_VSL_V1UK] Tinygrad ONNX input={self.model_input_name} "
        f"shape={tuple(input_spec.shape)} dtype={input_spec.dtype}"
      )
      self.runtime_error = ""
      self._set_status("UK vision: ready")
      cloudlog.info(f"[XNOR_VSL_V1UK] loaded Tinygrad ONNX model {MODEL_PATH}")
      return True
    except Exception as exc:
      self.runtime_error = f"Vision model load failed: {type(exc).__name__}"
      self._set_status(self.runtime_error)
      cloudlog.exception("[XNOR_VSL_V1UK] model load failed")
      return False

  def _disconnect_camera(self) -> None:
    self.client = None
    self.stream_type = None
    self.stream_name = ""

  def _connect_camera(self) -> bool:
    if self.VisionIpcClient is None or self.VisionStreamType is None:
      return False

    try:
      streams = self.VisionIpcClient.available_streams("camerad", block=False)
    except Exception:
      streams = []

    desired = None
    name = ""
    if self.VisionStreamType.VISION_STREAM_ROAD in streams:
      desired = self.VisionStreamType.VISION_STREAM_ROAD
      name = "road"
    elif self.VisionStreamType.VISION_STREAM_WIDE_ROAD in streams:
      desired = self.VisionStreamType.VISION_STREAM_WIDE_ROAD
      name = "wide"

    if desired is None:
      self._disconnect_camera()
      return False

    try:
      if self.client is None or self.stream_type != desired:
        self.client = self.VisionIpcClient("camerad", desired, True)
        self.stream_type = desired
        self.stream_name = name

      if not self.client.is_connected():
        self.client.connect(True)
      return bool(self.client.is_connected())
    except Exception:
      self._disconnect_camera()
      return False

  def _receive_frame_nv12(self):
    client = self.client
    if client is None:
      return None

    try:
      buf = client.recv()
      if buf is None:
        return None
      raw = np.frombuffer(buf.data, dtype=np.uint8)
      needed = int(client.stride) * int(client.height) * 3 // 2
      if raw.size < needed:
        return None
      image = raw[:needed].reshape((int(client.height) * 3 // 2, int(client.stride)))
      y = image[:int(client.height), :int(client.width)]
      uv = image[int(client.height):int(client.height) + int(client.height) // 2, :int(client.width)]
      return NV12Frame(y=y, uv=uv, width=int(client.width), height=int(client.height))
    except Exception:
      return None

  def _resize_luma(self, plane: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    if self.Image is None:
      raise RuntimeError("Pillow is not loaded")
    img = self.Image.fromarray(np.ascontiguousarray(plane), mode="L")
    return np.asarray(img.resize(size, self.Image.Resampling.BILINEAR), dtype=np.uint8)

  def _nv12_region_to_rgb(self, frame: NV12Frame, x1: int, y1: int, x2: int, y2: int,
                          target_size: tuple[int, int] | None = None) -> np.ndarray:
    x1 = int(np.clip(x1, 0, frame.width - 1))
    y1 = int(np.clip(y1, 0, frame.height - 1))
    x2 = int(np.clip(x2, x1 + 1, frame.width))
    y2 = int(np.clip(y2, y1 + 1, frame.height))

    y_plane = frame.y[y1:y2, x1:x2]

    # NV12 stores one interleaved U/V sample per 2x2 luma block.
    uv_x1 = x1 // 2
    uv_x2 = (x2 + 1) // 2
    uv_y1 = y1 // 2
    uv_y2 = (y2 + 1) // 2
    uv = frame.uv[uv_y1:uv_y2, uv_x1 * 2:uv_x2 * 2]
    u_plane = uv[:, 0::2]
    v_plane = uv[:, 1::2]

    out_w = int(target_size[0]) if target_size is not None else (x2 - x1)
    out_h = int(target_size[1]) if target_size is not None else (y2 - y1)
    if out_w <= 0 or out_h <= 0:
      raise ValueError("invalid NV12 region size")

    if y_plane.shape != (out_h, out_w):
      y_plane = self._resize_luma(y_plane, (out_w, out_h))
    else:
      y_plane = np.asarray(y_plane, dtype=np.uint8)

    u_plane = self._resize_luma(u_plane, (out_w, out_h))
    v_plane = self._resize_luma(v_plane, (out_w, out_h))

    # ITU-R BT.601 limited-range NV12 -> RGB, equivalent to the conversion
    # previously performed by OpenCV for our purposes.
    y32 = y_plane.astype(np.int32)
    u32 = u_plane.astype(np.int32) - 128
    v32 = v_plane.astype(np.int32) - 128
    c = np.maximum(y32 - 16, 0)

    r = (298 * c + 409 * v32 + 128) >> 8
    g = (298 * c - 100 * u32 - 208 * v32 + 128) >> 8
    b = (298 * c + 516 * u32 + 128) >> 8
    return np.stack((r, g, b), axis=-1).clip(0, 255).astype(np.uint8)

  def _prepare_detector_input(self, frame: NV12Frame):
    ratio = min(DETECTOR_INPUT_SIZE / frame.width, DETECTOR_INPUT_SIZE / frame.height)
    new_width = int(round(frame.width * ratio))
    new_height = int(round(frame.height * ratio))
    pad_width = (DETECTOR_INPUT_SIZE - new_width) / 2
    pad_height = (DETECTOR_INPUT_SIZE - new_height) / 2
    left = int(round(pad_width - 0.1))
    top = int(round(pad_height - 0.1))

    rgb = self._nv12_region_to_rgb(frame, 0, 0, frame.width, frame.height, (new_width, new_height))
    letterboxed = np.full((DETECTOR_INPUT_SIZE, DETECTOR_INPUT_SIZE, 3), 114, dtype=np.uint8)
    letterboxed[top:top + new_height, left:left + new_width] = rgb

    # StarPilot/Ultralytics model input: RGB, NCHW float32, [0,1].
    blob = letterboxed.transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    return blob, ratio, left, top

  @staticmethod
  def _rgb_to_opencv_hsv_channels(rgb: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return H/S/V channels on OpenCV's H=0..179, S/V=0..255 scale."""
    rgbf = rgb.astype(np.float32)
    r = rgbf[:, :, 0]
    g = rgbf[:, :, 1]
    b = rgbf[:, :, 2]

    vmax = np.maximum(np.maximum(r, g), b)
    vmin = np.minimum(np.minimum(r, g), b)
    delta = vmax - vmin

    hue = np.zeros_like(vmax, dtype=np.float32)
    nz = delta > 1e-6
    rmax = nz & (vmax == r)
    gmax = nz & (vmax == g)
    bmax = nz & (vmax == b)

    hue[rmax] = np.mod((g[rmax] - b[rmax]) / delta[rmax], 6.0) * 30.0
    hue[gmax] = (((b[gmax] - r[gmax]) / delta[gmax]) + 2.0) * 30.0
    hue[bmax] = (((r[bmax] - g[bmax]) / delta[bmax]) + 4.0) * 30.0
    hue = np.mod(hue, 180.0)

    sat = np.zeros_like(vmax, dtype=np.float32)
    nonblack = vmax > 1e-6
    sat[nonblack] = (delta[nonblack] / vmax[nonblack]) * 255.0
    return hue, sat, vmax

  def _uk_red_ring_score(self, sign_crop) -> float:
    if sign_crop is None or sign_crop.size == 0:
      return 0.0

    h, w = sign_crop.shape[:2]
    if h < 12 or w < 12:
      return 0.0
    aspect = w / max(h, 1)
    if aspect < 0.60 or aspect > 1.50:
      return 0.0

    hue, sat, val = self._rgb_to_opencv_hsv_channels(sign_crop)

    red = ((((hue <= 12) | (hue >= 168)) & (sat >= 70) & (val >= 55))).astype(np.uint8)
    white = ((val >= 125) & (sat <= 85)).astype(np.uint8)
    dark = ((val <= 120) & (sat <= 145)).astype(np.uint8)

    red_ratio = float(red.mean())
    white_ratio = float(white.mean())
    dark_ratio = float(dark.mean())
    if red_ratio < 0.025 or red_ratio > 0.50:
      return 0.0
    if white_ratio < 0.16 or dark_ratio < 0.004:
      return 0.0

    yy, xx = np.mgrid[0:h, 0:w]
    nx = (xx - (w - 1) / 2.0) / max(w / 2.0, 1.0)
    ny = (yy - (h - 1) / 2.0) / max(h / 2.0, 1.0)
    rr = np.sqrt(nx * nx + ny * ny)

    ring_zone = (rr >= 0.52) & (rr <= 1.05)
    centre_zone = rr <= 0.48
    if not ring_zone.any() or not centre_zone.any():
      return 0.0

    ring_red = float(red[ring_zone].mean())
    centre_red = float(red[centre_zone].mean())
    centre_white = float(white[centre_zone].mean())
    centre_dark = float(dark[centre_zone].mean())

    if ring_red < 0.055:
      return 0.0
    if ring_red < (centre_red * 1.35 + 0.01):
      return 0.0
    if centre_white < 0.18 or centre_dark < 0.006:
      return 0.0

    score = (
      min(ring_red / 0.28, 1.0) * 0.45 +
      min(centre_white / 0.65, 1.0) * 0.30 +
      min(centre_dark / 0.18, 1.0) * 0.15 +
      min(red_ratio / 0.20, 1.0) * 0.10
    )
    return float(np.clip(score, 0.0, 1.0))

  def _detect(self, frame: NV12Frame):
    if self.net is None or not self.model_input_name:
      return None

    frame_h, frame_w = frame.height, frame.width
    blob, ratio, pad_x, pad_y = self._prepare_detector_input(frame)

    try:
      outputs = self.net({self.model_input_name: blob})
      if not outputs:
        raise RuntimeError("ONNX runner returned no outputs")
      output = next(iter(outputs.values()))
      predictions = np.squeeze(output.numpy())
    except Exception:
      cloudlog.exception("[XNOR_VSL_V1UK] Tinygrad detector forward failed")
      return None

    if predictions.ndim != 2:
      return None
    if predictions.shape[0] < predictions.shape[1]:
      predictions = predictions.T

    max_area = frame_w * frame_h * MAX_BOX_AREA_RATIO
    candidates = []
    raw_supported = []

    for prediction in predictions:
      if len(prediction) <= 4:
        continue
      class_scores = prediction[4:]
      class_id = int(np.argmax(class_scores))
      speed_mph = SPEED_LIMIT_CLASSES.get(class_id)
      if speed_mph not in SUPPORTED_UK_LIMITS_MPH:
        continue

      model_conf = float(class_scores[class_id])
      if model_conf < MIN_MODEL_CONFIDENCE:
        continue

      cx, cy, bw, bh = [float(x) for x in prediction[:4]]
      x1 = max(int((cx - bw / 2.0 - pad_x) / ratio), 0)
      y1 = max(int((cy - bh / 2.0 - pad_y) / ratio), 0)
      x2 = min(int((cx + bw / 2.0 - pad_x) / ratio), frame_w)
      y2 = min(int((cy + bh / 2.0 - pad_y) / ratio), frame_h)
      if x2 <= x1 or y2 <= y1:
        continue

      box_w = x2 - x1
      box_h = y2 - y1
      if box_w < MIN_BOX_WIDTH or box_h < MIN_BOX_HEIGHT:
        continue
      if box_w * box_h > max_area:
        continue
      if y1 > frame_h * 0.88:
        continue

      try:
        crop = self._nv12_region_to_rgb(frame, x1, y1, x2, y2)
      except Exception:
        continue
      uk_score = self._uk_red_ring_score(crop)
      bbox = (x1, y1, x2, y2)
      raw_supported.append((model_conf, speed_mph, uk_score, bbox))
      if uk_score <= 0.0:
        continue

      confidence = float(np.clip(model_conf * 0.72 + uk_score * 0.28, 0.0, 0.99))
      candidates.append(Detection(speed_mph, confidence, model_conf, uk_score, bbox))

    if not candidates:
      if raw_supported:
        model_conf, speed_mph, uk_score, bbox = max(raw_supported, key=lambda item: item[0])
        self._log_raw_proposal("ring_reject", speed_mph, model_conf, uk_score, 0.0, bbox)
      return None

    candidates.sort(key=lambda d: d.confidence, reverse=True)
    best = candidates[0]
    self._log_raw_proposal(
      "ring_accept",
      best.speed_limit_mph,
      best.model_confidence,
      best.ring_score,
      best.confidence,
      best.bbox,
    )
    return best

  def _prune_history(self, now: float) -> None:
    while self.history and now - self.history[0].created_at > HISTORY_SECONDS:
      self.history.popleft()

  def _update_detection(self, detection: Detection) -> None:
    now = time.monotonic()
    self.followup_until = max(self.followup_until, now + FOLLOWUP_WINDOW_SECONDS)
    self.history.append(HistoryEntry(detection.speed_limit_mph, detection.confidence, now))
    self._prune_history(now)

    counts = Counter(x.speed_limit_mph for x in self.history)
    count = counts.get(detection.speed_limit_mph, 0)
    confs = [x.confidence for x in self.history if x.speed_limit_mph == detection.speed_limit_mph]
    best_conf = max(confs) if confs else 0.0
    if best_conf < MIN_CONFIRMED_CONFIDENCE:
      self._log_temporal_candidate(detection, count, INITIAL_REQUIRED_READS, "confidence_reject")
      self._set_status(f"UK candidate: {detection.speed_limit_mph} mph")
      return

    current = self.published_speed_limit_mph
    if current <= 0:
      if count >= INITIAL_REQUIRED_READS:
        self._log_temporal_candidate(detection, count, INITIAL_REQUIRED_READS, "publish")
        self._publish(detection.speed_limit_mph, best_conf)
      else:
        self._log_temporal_candidate(detection, count, INITIAL_REQUIRED_READS, "waiting")
        self._set_status(f"UK candidate: {detection.speed_limit_mph} mph ({count}/{INITIAL_REQUIRED_READS})")
      return

    if detection.speed_limit_mph == current:
      self._log_temporal_candidate(detection, count, 1, "refresh")
      self._refresh_publish_timestamp()
      self._set_status(f"UK vision: {current} mph ({best_conf * 100.0:.0f}%)")
      return

    if detection.speed_limit_mph < current:
      if count >= CHANGE_REQUIRED_READS:
        self._log_temporal_candidate(detection, count, CHANGE_REQUIRED_READS, "publish_lower")
        self._publish(detection.speed_limit_mph, best_conf)
      else:
        self._log_temporal_candidate(detection, count, CHANGE_REQUIRED_READS, "waiting_lower")
        self._set_status(f"UK lower candidate: {detection.speed_limit_mph} mph ({count}/{CHANGE_REQUIRED_READS})")
      return

    # V1 never uses a higher vision value as a new speed target. A strongly
    # confirmed higher sign only releases the existing lower vision override;
    # Tesla/map speed-limit logic then becomes authoritative again.
    if count >= HIGHER_RELEASE_REQUIRED_READS and best_conf >= HIGHER_RELEASE_CONFIDENCE:
      self._log_temporal_candidate(detection, count, HIGHER_RELEASE_REQUIRED_READS, "release_higher")
      self._clear_publish(f"confirmed higher sign {detection.speed_limit_mph} mph")
      self.followup_until = now + 1.0
    else:
      decision = "waiting_higher" if best_conf >= HIGHER_RELEASE_CONFIDENCE else "higher_confidence_reject"
      self._log_temporal_candidate(detection, count, HIGHER_RELEASE_REQUIRED_READS, decision)
      self._set_status(
        f"UK higher candidate: {detection.speed_limit_mph} mph "
        f"({count}/{HIGHER_RELEASE_REQUIRED_READS})"
      )

  def _memory_usage_percent(self) -> float:
    if self.sm is None:
      return 0.0
    try:
      self.sm.update(0)
      return float(self.sm["deviceState"].memoryUsagePercent)
    except Exception:
      return 0.0

  def run(self) -> None:
    if not self._load_runtime():
      while True:
        time.sleep(10.0)

    rk = Ratekeeper(RUNTIME_HZ, None)

    while True:
      now = time.monotonic()
      mem = self._memory_usage_percent()

      if mem >= MEMORY_CRITICAL_PERCENT:
        self._disconnect_camera()
        self._set_status(f"UK vision paused - memory {mem:.0f}%")
        rk.keep_time()
        continue

      if not self._connect_camera():
        self._set_status("UK vision: waiting for camera")
        rk.keep_time()
        continue

      interval = MEMORY_PRESSURE_INTERVAL if mem >= MEMORY_PRESSURE_PERCENT else NORMAL_INFERENCE_INTERVAL
      if now < self.followup_until:
        interval = max(FOLLOWUP_INFERENCE_INTERVAL, min(interval, NORMAL_INFERENCE_INTERVAL))

      if now - self.last_inference_at < interval:
        if self.published_speed_limit_mph > 0 and now - self.last_detection_at > PUBLISHED_HOLD_SECONDS:
          self._clear_publish("stale")
        rk.keep_time()
        continue

      frame = self._receive_frame_nv12()
      self.last_inference_at = now
      if frame is None:
        rk.keep_time()
        continue

      detection = self._detect(frame)
      if detection is not None:
        self._update_detection(detection)
      elif self.published_speed_limit_mph > 0:
        if now - self.last_detection_at > PUBLISHED_HOLD_SECONDS:
          self._clear_publish("stale")
        else:
          self._set_status(f"UK vision: holding {self.published_speed_limit_mph} mph")
      else:
        self._set_status(f"UK vision: scanning {self.stream_name}")

      frame = None
      rk.keep_time()


def main() -> None:
  SpeedLimitVisionUK().run()


if __name__ == "__main__":
  main()
