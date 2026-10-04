#!/usr/bin/env python3
"""XNOR Vision Speed Limit V1-UK.

Recognizes numeric UK red-circle limits (20/30/40/50/60/70 mph) using the
legacy StarPilot single-model detector and UK-specific red-ring validation.

The runtime publishes a confirmed vision limit only after independent-frame
confirmation. V1 intentionally keeps a lower active limit latched when a
higher sign is later seen; CLEAR in Tesla settings releases that latch.
National Speed Limit is intentionally unsupported in V1.
"""
from __future__ import annotations

import math
import time
from collections import Counter, deque
from dataclasses import dataclass

import numpy as np

from cereal import messaging
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.vision_speed_limit_common import VISION_MODEL_PATH, prepare_cv2_path


INFERENCE_INTERVAL = 0.40
FOLLOWUP_INTERVAL = 0.20
FOLLOWUP_SECONDS = 1.6
PRESSURE_INTERVAL = 2.0

MEM_PRESSURE_AVAILABLE_KB = 512 * 1024
MEM_CRITICAL_AVAILABLE_KB = 256 * 1024
MEM_PRESSURE_PERCENT = 88
MEM_CRITICAL_PERCENT = 94

MODEL_INPUT_SIZE = 640
MODEL_MIN_CONFIDENCE = 0.05
MODEL_MAX_BOX_AREA_RATIO = 0.20
MODEL_MIN_WIDTH = 10
MODEL_MIN_HEIGHT = 16
MODEL_MAX_Y_RATIO = 0.88

SUPPORTED_SPEEDS_MPH = frozenset((20, 30, 40, 50, 60, 70))
SPEED_LIMIT_CLASSES = {
  2: 10,
  3: 100,
  4: 110,
  5: 120,
  6: 20,
  7: 30,
  8: 40,
  9: 50,
  10: 60,
  11: 70,
  12: 80,
  13: 90,
}

HISTORY_SECONDS = 2.0
BASE_CONFIRMATIONS = 2
LARGE_DROP_CONFIRMATIONS = 3
LARGE_DROP_MPH = 20.0
MIN_CONFIRMED_SCORE = 0.20

RED_LOW_HUE_MAX = 12
RED_HIGH_HUE_MIN = 168
RED_SAT_MIN = 75
RED_VALUE_MIN = 55
WHITE_VALUE_MIN = 130
WHITE_SAT_MAX = 90
DARK_VALUE_MAX = 125
RING_MIN_RED_RATIO = 0.20
CORE_MIN_WHITE_RATIO = 0.20
CORE_MIN_DARK_RATIO = 0.015
CORE_MAX_RED_RATIO = 0.18


@dataclass
class Detection:
  speed_mph: int
  confidence: float
  model_confidence: float
  ring_score: float


@dataclass
class HistoryEntry:
  speed_mph: int
  confidence: float
  created_at: float


def _mem_available_kb() -> int | None:
  try:
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
      for line in f:
        if line.startswith("MemAvailable:"):
          return int(line.split()[1])
  except (OSError, ValueError, IndexError):
    pass
  return None


class UKSpeedLimitVision:
  def __init__(self, cv2):
    self.cv2 = cv2
    self.params = Params()
    self.sm = messaging.SubMaster(["deviceState", "carState", "mapdOut"], ignore_avg_freq=True)

    self.net = cv2.dnn.readNetFromONNX(str(VISION_MODEL_PATH))
    self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
    self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
    try:
      cv2.setNumThreads(1)
    except Exception:
      pass

    self.client = None
    self.stream_type = None
    self.stream_name = ""
    self.history: deque[HistoryEntry] = deque()
    self.published_speed_mph = 0
    self.published_confidence = 0.0
    self.last_inference_at = -float("inf")
    self.followup_until = 0.0
    self.last_status = ""
    self.last_memory_state = "normal"

    self._clear_published("Scanning UK speed-limit signs")

  def _put_if_changed(self, key: str, value) -> None:
    try:
      if self.params.get(key) != value:
        self.params.put(key, value)
    except Exception:
      pass

  def _set_status(self, text: str) -> None:
    if text == self.last_status:
      return
    self.last_status = text
    self._put_if_changed("VisionSpeedLimitStatus", text)
    cloudlog.info(f"[XNOR_VISION_SL_V1] {text}")

  def _clear_published(self, status: str) -> None:
    self.published_speed_mph = 0
    self.published_confidence = 0.0
    self.history.clear()
    self._put_if_changed("VisionSpeedLimit", 0.0)
    self._put_if_changed("VisionSpeedLimitConfidence", 0.0)
    self._put_if_changed("VisionSpeedLimitSupportCount", 0)
    self._put_if_changed("VisionSpeedLimitSupportSpeed", 0.0)
    try:
      if self.params.get_bool("VisionSpeedLimitReset"):
        self.params.put_bool("VisionSpeedLimitReset", False)
    except Exception:
      pass
    self._set_status(status)

  def _handle_manual_reset(self) -> None:
    try:
      if self.params.get_bool("VisionSpeedLimitReset"):
        self._clear_published("Cleared manually - scanning")
    except Exception:
      pass

  def _memory_state(self) -> str:
    available = _mem_available_kb()
    usage = None
    try:
      if self.sm.valid.get("deviceState", False):
        usage = int(self.sm["deviceState"].memoryUsagePercent)
    except Exception:
      usage = None

    if ((available is not None and available <= MEM_CRITICAL_AVAILABLE_KB) or
        (usage is not None and usage >= MEM_CRITICAL_PERCENT)):
      return "critical"
    if ((available is not None and available <= MEM_PRESSURE_AVAILABLE_KB) or
        (usage is not None and usage >= MEM_PRESSURE_PERCENT)):
      return "pressure"
    return "normal"

  def _connect_camera(self) -> bool:
    if self.client is not None and self.client.is_connected():
      return True

    from msgq.visionipc import VisionIpcClient, VisionStreamType

    try:
      streams = VisionIpcClient.available_streams("camerad", block=False)
    except Exception:
      streams = []

    if VisionStreamType.VISION_STREAM_ROAD in streams:
      desired = VisionStreamType.VISION_STREAM_ROAD
      name = "road camera"
    elif VisionStreamType.VISION_STREAM_WIDE_ROAD in streams:
      desired = VisionStreamType.VISION_STREAM_WIDE_ROAD
      name = "wide road camera"
    else:
      self._disconnect_camera()
      return False

    if self.client is None or self.stream_type != desired:
      self.client = VisionIpcClient("camerad", desired, True)
      self.stream_type = desired
      self.stream_name = name

    if not self.client.is_connected():
      self.client.connect(True)
    return self.client.is_connected()

  def _disconnect_camera(self) -> None:
    self.client = None
    self.stream_type = None
    self.stream_name = ""

  def _receive_frame(self):
    client = self.client
    if client is None:
      return None
    buf = client.recv()
    if buf is None:
      return None
    data = buf.data
    if data is None or len(data) == 0:
      return None

    yuv = np.frombuffer(data, dtype=np.uint8).reshape((len(data) // client.stride, client.stride))
    nv12 = yuv[:client.height * 3 // 2, :client.width]
    return self.cv2.cvtColor(nv12, self.cv2.COLOR_YUV2BGR_NV12)

  @staticmethod
  def _letterbox(cv2, image):
    h, w = image.shape[:2]
    ratio = min(MODEL_INPUT_SIZE / max(h, 1), MODEL_INPUT_SIZE / max(w, 1))
    new_w = max(int(round(w * ratio)), 1)
    new_h = max(int(round(h * ratio)), 1)
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    dw = (MODEL_INPUT_SIZE - new_w) / 2.0
    dh = (MODEL_INPUT_SIZE - new_h) / 2.0
    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))
    boxed = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return boxed, ratio, dw, dh

  def _uk_red_ring_score(self, crop) -> float:
    cv2 = self.cv2
    if crop.size == 0:
      return 0.0
    h, w = crop.shape[:2]
    if h < 12 or w < 12:
      return 0.0
    aspect = w / max(h, 1)
    if not 0.55 <= aspect <= 1.55:
      return 0.0

    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    red = ((((hue <= RED_LOW_HUE_MAX) | (hue >= RED_HIGH_HUE_MIN))) &
           (sat >= RED_SAT_MIN) & (val >= RED_VALUE_MIN)).astype(np.uint8)

    contours, _ = cv2.findContours(red * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
      return 0.0

    best = 0.0
    for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:4]:
      x, y, rw, rh = cv2.boundingRect(contour)
      if rw < max(8, int(w * 0.15)) or rh < max(8, int(h * 0.15)):
        continue
      red_aspect = rw / max(rh, 1)
      if not 0.60 <= red_aspect <= 1.45:
        continue

      pad_x = max(int(rw * 0.06), 1)
      pad_y = max(int(rh * 0.06), 1)
      x1, y1 = max(x - pad_x, 0), max(y - pad_y, 0)
      x2, y2 = min(x + rw + pad_x, w), min(y + rh + pad_y, h)
      sign = crop[y1:y2, x1:x2]
      if sign.size == 0:
        continue

      sh, sw = sign.shape[:2]
      hsv_sign = cv2.cvtColor(sign, cv2.COLOR_BGR2HSV)
      hh, ss, vv = cv2.split(hsv_sign)
      red_sign = ((((hh <= RED_LOW_HUE_MAX) | (hh >= RED_HIGH_HUE_MIN))) &
                  (ss >= RED_SAT_MIN) & (vv >= RED_VALUE_MIN))
      white_sign = (vv >= WHITE_VALUE_MIN) & (ss <= WHITE_SAT_MAX)
      dark_sign = vv <= DARK_VALUE_MAX

      yy, xx = np.ogrid[:sh, :sw]
      nx = (xx - (sw - 1) / 2.0) / max(sw / 2.0, 1.0)
      ny = (yy - (sh - 1) / 2.0) / max(sh / 2.0, 1.0)
      radius = np.sqrt(nx * nx + ny * ny)

      ring_ratio = 0.0
      for inner, outer in ((0.45, 0.68), (0.55, 0.78), (0.65, 0.88), (0.72, 1.02)):
        band = (radius >= inner) & (radius < outer)
        if np.any(band):
          ring_ratio = max(ring_ratio, float(red_sign[band].mean()))

      core = radius < 0.45
      if not np.any(core):
        continue
      core_red = float(red_sign[core].mean())
      core_white = float(white_sign[core].mean())
      core_dark = float(dark_sign[core].mean())

      if ring_ratio < RING_MIN_RED_RATIO:
        continue
      if core_white < CORE_MIN_WHITE_RATIO or core_dark < CORE_MIN_DARK_RATIO:
        continue
      if core_red > CORE_MAX_RED_RATIO and core_red > ring_ratio * 0.55:
        continue

      quality = min(1.0, ring_ratio / 0.55) * 0.70
      quality += min(1.0, core_white / 0.45) * 0.20
      quality += min(1.0, core_dark / 0.10) * 0.10
      best = max(best, min(1.0, quality))

    return best

  def _detect(self, frame) -> Detection | None:
    cv2 = self.cv2
    frame_h, frame_w = frame.shape[:2]
    boxed, ratio, pad_w, pad_h = self._letterbox(cv2, frame)
    blob = cv2.dnn.blobFromImage(
      boxed, scalefactor=1.0 / 255.0,
      size=(MODEL_INPUT_SIZE, MODEL_INPUT_SIZE),
      swapRB=True, crop=False,
    )
    self.net.setInput(blob)
    predictions = np.squeeze(self.net.forward())
    if predictions.ndim != 2:
      return None
    if predictions.shape[0] < predictions.shape[1]:
      predictions = predictions.T

    candidates: list[Detection] = []
    for prediction in predictions:
      class_scores = prediction[4:]
      if class_scores.size == 0:
        continue
      class_id = int(np.argmax(class_scores))
      speed_mph = SPEED_LIMIT_CLASSES.get(class_id)
      if speed_mph not in SUPPORTED_SPEEDS_MPH:
        continue
      model_conf = float(class_scores[class_id])
      if not math.isfinite(model_conf) or model_conf < MODEL_MIN_CONFIDENCE:
        continue

      center_x, center_y, width, height = [float(v) for v in prediction[:4]]
      x1 = max(int((center_x - width / 2.0 - pad_w) / ratio), 0)
      y1 = max(int((center_y - height / 2.0 - pad_h) / ratio), 0)
      x2 = min(int((center_x + width / 2.0 - pad_w) / ratio), frame_w)
      y2 = min(int((center_y + height / 2.0 - pad_h) / ratio), frame_h)
      if x2 <= x1 or y2 <= y1:
        continue

      bw, bh = x2 - x1, y2 - y1
      if bw < MODEL_MIN_WIDTH or bh < MODEL_MIN_HEIGHT:
        continue
      if bw * bh > frame_w * frame_h * MODEL_MAX_BOX_AREA_RATIO:
        continue
      if y1 > frame_h * MODEL_MAX_Y_RATIO:
        continue

      px = int(bw * 0.08)
      py = int(bh * 0.08)
      crop = frame[max(y1 - py, 0):min(y2 + py, frame_h), max(x1 - px, 0):min(x2 + px, frame_w)]
      ring_score = self._uk_red_ring_score(crop)
      if ring_score <= 0.0:
        continue

      confidence = min(0.99, model_conf * 0.60 + ring_score * 0.40)
      candidates.append(Detection(int(speed_mph), float(confidence), model_conf, ring_score))

    return max(candidates, key=lambda d: d.confidence, default=None)

  def _reference_speed_mph(self) -> float:
    references = []
    try:
      cruise_kph = float(self.sm["carState"].vCruise)
      if 1.0 < cruise_kph < 200.0:
        references.append(cruise_kph * CV.KPH_TO_MPH)
    except Exception:
      pass
    try:
      map_ms = float(self.sm["mapdOut"].speedLimit)
      if map_ms > 0.0:
        references.append(map_ms * CV.MS_TO_MPH)
    except Exception:
      pass
    return max(references, default=0.0)

  def _prune_history(self, now: float) -> None:
    while self.history and now - self.history[0].created_at > HISTORY_SECONDS:
      self.history.popleft()

  def _publish_confirmed(self, detection: Detection, support: int) -> None:
    speed = int(detection.speed_mph)

    if self.published_speed_mph > 0 and speed > self.published_speed_mph:
      self._set_status(
        f"Higher {speed} mph sign confirmed - V1 holding {self.published_speed_mph} mph until CLEAR"
      )
      return

    if self.published_speed_mph > 0 and speed == self.published_speed_mph:
      self.published_confidence = max(self.published_confidence, detection.confidence)
    else:
      old = self.published_speed_mph
      self.published_speed_mph = speed
      self.published_confidence = detection.confidence
      cloudlog.info(
        f"[XNOR_VISION_SL_V1] publish old_mph={old} new_mph={speed} "
        f"confidence={detection.confidence:.3f} support={support} "
        f"model={detection.model_confidence:.3f} ring={detection.ring_score:.3f}"
      )

    limit_ms = self.published_speed_mph * CV.MPH_TO_MS
    self._put_if_changed("VisionSpeedLimit", float(limit_ms))
    self._put_if_changed("VisionSpeedLimitConfidence", float(self.published_confidence))
    self._put_if_changed("VisionSpeedLimitSupportCount", int(support))
    self._put_if_changed("VisionSpeedLimitSupportSpeed", float(limit_ms))
    self._set_status(
      f"Holding {self.published_speed_mph} mph - UK vision confirmed "
      f"({self.published_confidence * 100:.0f}%, {support} reads)"
    )

  def _update_detection(self, detection: Detection, now: float) -> None:
    self.history.append(HistoryEntry(detection.speed_mph, detection.confidence, now))
    self._prune_history(now)

    counts = Counter(entry.speed_mph for entry in self.history)
    support = int(counts[detection.speed_mph])
    matching = [entry for entry in self.history if entry.speed_mph == detection.speed_mph]
    avg_conf = float(sum(entry.confidence for entry in matching) / max(len(matching), 1))

    reference = self.published_speed_mph or self._reference_speed_mph()
    drop = max(0.0, float(reference) - float(detection.speed_mph)) if reference > 0.0 else 0.0
    required = LARGE_DROP_CONFIRMATIONS if drop >= LARGE_DROP_MPH else BASE_CONFIRMATIONS

    if support >= required and avg_conf >= MIN_CONFIRMED_SCORE:
      confirmed = Detection(
        detection.speed_mph,
        max(avg_conf, detection.confidence),
        detection.model_confidence,
        detection.ring_score,
      )
      self._publish_confirmed(confirmed, support)
    else:
      self.followup_until = max(self.followup_until, now + FOLLOWUP_SECONDS)
      self._set_status(
        f"Candidate {detection.speed_mph} mph - confirming "
        f"{support}/{required} ({avg_conf * 100:.0f}%)"
      )

  def run(self) -> None:
    self._set_status("Scanning UK speed-limit signs")

    while True:
      self.sm.update(0)
      self._handle_manual_reset()

      memory_state = self._memory_state()
      if memory_state != self.last_memory_state:
        cloudlog.info(f"[XNOR_VISION_SL_V1] memory_state={memory_state}")
        self.last_memory_state = memory_state

      if memory_state == "critical":
        self._disconnect_camera()
        self._set_status("Paused - critical memory pressure")
        time.sleep(0.25)
        continue

      if not self._connect_camera():
        self._set_status("Waiting for road camera")
        time.sleep(0.25)
        continue

      frame = self._receive_frame()
      if frame is None:
        continue

      now = time.monotonic()
      interval = PRESSURE_INTERVAL if memory_state == "pressure" else (
        FOLLOWUP_INTERVAL if now < self.followup_until else INFERENCE_INTERVAL
      )
      if now - self.last_inference_at < interval:
        continue
      self.last_inference_at = now

      try:
        detection = self._detect(frame)
      except Exception as exc:
        cloudlog.warning(f"[XNOR_VISION_SL_V1] inference failed: {exc!r}")
        self._set_status(f"Vision inference error - {type(exc).__name__}")
        time.sleep(0.25)
        continue

      if detection is not None:
        self._update_detection(detection, now)
      else:
        self._prune_history(now)
        if self.published_speed_mph == 0 and now >= self.followup_until:
          self._set_status("Scanning UK speed-limit signs")


def main() -> None:
  params = Params()
  prepare_cv2_path()
  try:
    import cv2
  except Exception as exc:
    try:
      params.put("VisionSpeedLimitStatus", f"OpenCV runtime unavailable - {type(exc).__name__}")
    except Exception:
      pass
    cloudlog.exception("[XNOR_VISION_SL_V1] failed to import cv2")
    return

  while True:
    try:
      daemon = UKSpeedLimitVision(cv2)
      daemon.run()
    except KeyboardInterrupt:
      raise
    except Exception as exc:
      cloudlog.exception("[XNOR_VISION_SL_V1] runtime failure")
      try:
        params.put("VisionSpeedLimitStatus", f"Vision runtime restarting - {type(exc).__name__}")
        params.put("VisionSpeedLimit", 0.0)
        params.put("VisionSpeedLimitConfidence", 0.0)
        params.put("VisionSpeedLimitSupportCount", 0)
        params.put("VisionSpeedLimitSupportSpeed", 0.0)
      except Exception:
        pass
      time.sleep(5.0)


if __name__ == "__main__":
  main()
