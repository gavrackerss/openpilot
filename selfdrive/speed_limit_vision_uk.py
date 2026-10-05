#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import importlib
import os
import shutil
import sys
import time
import zipfile
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.speed_limit_uk_reader import UKNationalSpeedLimitReader, UKSpeedValueReader


# XNOR Vision Speed Limit V1.2-UK
#
# Runtime design is based on the speed-limit vision pipeline in StarPilot
# (firestar5683/StarPilot, Dom branch), but deliberately uses only the legacy
# single ONNX detector. V1-UK adds a UK red-ring geometry gate, full-frame
# scanning, conservative temporal confirmation and lower-only integration in
# Tesla CarState. It recognises numeric 20/30/40/50/60/70 mph signs.
#
# National Speed Limit and non-numeric restrictions are intentionally not
# handled in V1.2 with scoped mapd road context.

MODEL_PATH = Path(__file__).resolve().parent / "assets" / "vision_models" / "speed_limit_vision.onnx"

# The comma/AGNOS Python environment does not automatically re-resolve
# pyproject.toml when a changed-files overlay is installed. Carry a pinned
# official ARM64 opencv-python-headless wheel with V1-UK and unpack it once to
# persistent media when cv2 is not already available.
OPENCV_VERSION = "4.13.0.92"
OPENCV_WHEEL_NAME = "opencv_python_headless-4.13.0.92-cp37-abi3-manylinux2014_aarch64.manylinux_2_17_aarch64.whl"
OPENCV_WHEEL_SHA256 = "5c8cfc8e87ed452b5cecb9419473ee5560a989859fe1d10d1ce11ae87b09a2cb"
OPENCV_WHEEL_PATH = Path(__file__).resolve().parent / "assets" / "vision_runtime" / OPENCV_WHEEL_NAME
OPENCV_RUNTIME_DIR = Path("/data/media/0/xnor_speed_limit_runtime") / f"opencv-{OPENCV_VERSION}-aarch64"
OPENCV_RUNTIME_MARKER = OPENCV_RUNTIME_DIR / ".xnor_vsl_opencv_complete"

RUNTIME_HZ = 20
NORMAL_INFERENCE_INTERVAL = 0.40
FOLLOWUP_INFERENCE_INTERVAL = 0.20
FOLLOWUP_WINDOW_SECONDS = 2.0
TRACK_FRAME_INTERVAL = 0.10
TRACK_MAX_AGE_SECONDS = 1.50
TRACK_MIN_FEATURE_COUNT = 4
TRACK_MAX_FAILED_READS = 3
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
NATIONAL_SCAN_MAX_WIDTH = 960
NATIONAL_HOUGH_MIN_RADIUS = 7
NATIONAL_HOUGH_MAX_RADIUS_RATIO = 0.09
NATIONAL_MIN_SCORE = 0.68

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
  value_confidence: float = 0.0
  legacy_speed_mph: int = 0
  bbox: tuple[int, int, int, int] | None = None
  source: str = "numeric"


@dataclass
class SignTrack:
  source: str
  speed_limit_mph: int
  bbox: tuple[int, int, int, int]
  previous_gray: np.ndarray
  points: np.ndarray
  started_at: float
  last_frame_at: float
  failed_reads: int = 0


@dataclass
class HistoryEntry:
  speed_limit_mph: int
  confidence: float
  created_at: float


class SpeedLimitVisionUK:
  def __init__(self):
    self.params = Params()
    self.cv2 = None
    self.VisionIpcClient = None
    self.VisionStreamType = None
    self.net = None
    self.value_reader = None
    self.national_reader = None
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
    self.track: SignTrack | None = None
    self.last_candidate_at = 0.0
    self._last_national_log_at = 0.0

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

  def _log_raw_proposal(self, decision: str, legacy_speed_mph: int, model_confidence: float,
                        ring_score: float, value_speed_mph: int = 0, value_confidence: float = 0.0,
                        combined_confidence: float = 0.0,
                        bbox: tuple[int, int, int, int] | None = None) -> None:
    """Log detector/ring-gate state without flooding rlogs with identical frames."""
    now = time.monotonic()
    signature = (
      str(decision),
      int(legacy_speed_mph),
      int(value_speed_mph),
      round(float(model_confidence), 2),
      round(float(ring_score), 2),
      round(float(value_confidence), 2),
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
      f"[XNOR_VSL_V12UK] legacy={int(legacy_speed_mph)} "
      f"model={float(model_confidence):.3f} ring={float(ring_score):.3f} "
      f"ukValue={int(value_speed_mph)} valueConf={float(value_confidence):.3f} "
      f"combined={float(combined_confidence):.3f} decision={decision} bbox={bbox_text}"
    )

  def _log_temporal_candidate(self, detection: Detection, count: int, required: int, decision: str) -> None:
    cloudlog.info(
      f"[XNOR_VSL_V12UK] candidate={int(detection.speed_limit_mph)} "
      f"legacy={int(detection.legacy_speed_mph)} model={float(detection.model_confidence):.3f} "
      f"ring={float(detection.ring_score):.3f} valueConf={float(detection.value_confidence):.3f} "
      f"combined={float(detection.confidence):.3f} count={int(count)}/{int(required)} "
      f"source={detection.source} decision={decision}"
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
    cloudlog.info(f"[XNOR_VSL_V12UK] publish={speed_limit_mph}mph confidence={confidence:.3f}")

  def _publish_lower_candidate(self, detection: Detection) -> None:
    """Publish an accepted sign as a provisional lower-only candidate.

    CarState decides whether it is actually lower than the current trusted
    Tesla/map/DAS limit. If it is not lower, this candidate can never raise the
    effective limit.
    """
    now = time.monotonic()
    self.last_candidate_at = now
    try:
      self.params.put_nonblocking("VisionSpeedLimitCandidate", float(detection.speed_limit_mph) * CV.MPH_TO_MS)
      self.params.put_nonblocking("VisionSpeedLimitCandidateConfidence", float(detection.confidence))
      self.params.put_nonblocking("VisionSpeedLimitCandidateTimestamp", float(now))
    except Exception:
      pass
    cloudlog.info(
      f"[XNOR_VSL_V12UK] lower_candidate={int(detection.speed_limit_mph)}mph "
      f"confidence={float(detection.confidence):.3f} source={detection.source}"
    )

  def _clear_lower_candidate(self, reason: str) -> None:
    had_candidate = self.last_candidate_at > 0.0
    self.last_candidate_at = 0.0
    try:
      self.params.put_nonblocking("VisionSpeedLimitCandidate", 0.0)
      self.params.put_nonblocking("VisionSpeedLimitCandidateConfidence", 0.0)
      self.params.put_nonblocking("VisionSpeedLimitCandidateTimestamp", 0.0)
    except Exception:
      pass
    if had_candidate:
      cloudlog.info(f"[XNOR_VSL_V12UK] clear lower candidate reason={reason}")

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
    self._clear_lower_candidate(reason)
    self._set_status(f"UK vision: scanning ({reason})")
    if old > 0:
      cloudlog.info(f"[XNOR_VSL_V12UK] clear previous={old}mph reason={reason}")

  @staticmethod
  def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
      for chunk in iter(lambda: f.read(1024 * 1024), b""):
        digest.update(chunk)
    return digest.hexdigest()

  def _load_cv2(self):
    """Import system cv2, or activate the pinned ARM64 wheel carried by V1-UK."""
    try:
      import cv2
      cloudlog.info(f"[XNOR_VSL_V12UK] using system OpenCV {getattr(cv2, '__version__', 'unknown')}")
      return cv2
    except ModuleNotFoundError:
      pass

    if not OPENCV_WHEEL_PATH.is_file():
      raise FileNotFoundError(f"vendored OpenCV wheel missing: {OPENCV_WHEEL_PATH}")

    wheel_hash = self._sha256_file(OPENCV_WHEEL_PATH)
    if wheel_hash != OPENCV_WHEEL_SHA256:
      raise RuntimeError(f"vendored OpenCV wheel checksum mismatch: {wheel_hash}")

    marker_ok = False
    try:
      marker_ok = OPENCV_RUNTIME_MARKER.read_text().strip() == OPENCV_WHEEL_SHA256
    except (FileNotFoundError, OSError):
      pass

    if not marker_ok:
      tmp_dir = OPENCV_RUNTIME_DIR.with_name(OPENCV_RUNTIME_DIR.name + ".tmp")
      shutil.rmtree(tmp_dir, ignore_errors=True)
      tmp_dir.parent.mkdir(parents=True, exist_ok=True)
      tmp_dir.mkdir(parents=True, exist_ok=True)
      try:
        with zipfile.ZipFile(OPENCV_WHEEL_PATH, "r") as zf:
          zf.extractall(tmp_dir)
        (tmp_dir / ".xnor_vsl_opencv_complete").write_text(OPENCV_WHEEL_SHA256)
        shutil.rmtree(OPENCV_RUNTIME_DIR, ignore_errors=True)
        os.replace(tmp_dir, OPENCV_RUNTIME_DIR)
      except Exception:
        shutil.rmtree(tmp_dir, ignore_errors=True)
        raise

    runtime_path = str(OPENCV_RUNTIME_DIR)
    if runtime_path not in sys.path:
      sys.path.insert(0, runtime_path)
    importlib.invalidate_caches()

    import cv2
    cloudlog.info(
      f"[XNOR_VSL_V12UK] using vendored OpenCV {getattr(cv2, '__version__', 'unknown')} "
      f"from {OPENCV_RUNTIME_DIR}"
    )
    return cv2

  def _load_runtime(self) -> bool:
    try:
      cv2 = self._load_cv2()
      from cereal import messaging
      from msgq.visionipc import VisionIpcClient, VisionStreamType

      self.cv2 = cv2
      self.VisionIpcClient = VisionIpcClient
      self.VisionStreamType = VisionStreamType
      self.sm = messaging.SubMaster(["deviceState", "mapdOut"])
      cv2.setNumThreads(1)
      try:
        os.nice(10)
      except Exception:
        pass
    except Exception as exc:
      self.runtime_error = f"OpenCV/VisionIPC unavailable: {type(exc).__name__}: {exc}"
      self._set_status(self.runtime_error)
      cloudlog.exception("[XNOR_VSL_V12UK] runtime dependency unavailable")
      return False

    if not MODEL_PATH.is_file():
      self.runtime_error = f"Vision model missing: {MODEL_PATH.name}"
      self._set_status(self.runtime_error)
      cloudlog.error(f"[XNOR_VSL_V12UK] {self.runtime_error}")
      return False

    try:
      self.net = self.cv2.dnn.readNetFromONNX(str(MODEL_PATH))
      self.net.setPreferableBackend(self.cv2.dnn.DNN_BACKEND_OPENCV)
      self.net.setPreferableTarget(self.cv2.dnn.DNN_TARGET_CPU)
      self.value_reader = UKSpeedValueReader(self.cv2)
      self.national_reader = UKNationalSpeedLimitReader(self.cv2)
      self.runtime_error = ""
      self._set_status("UK vision V1.2: ready")
      cloudlog.info(f"[XNOR_VSL_V12UK] loaded proposal model {MODEL_PATH}; UK crop/tracking/national readers active")
      return True
    except Exception as exc:
      self.runtime_error = f"Vision model load failed: {type(exc).__name__}"
      self._set_status(self.runtime_error)
      cloudlog.exception("[XNOR_VSL_V12UK] model load failed")
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

  def _receive_frame_bgr(self):
    client = self.client
    if client is None:
      return None

    try:
      buf = client.recv()
      if buf is None:
        return None
      data = buf.data
      if not data.any():
        return None
      image = np.frombuffer(data, dtype=np.uint8).reshape((len(data) // client.stride, client.stride))
      return self.cv2.cvtColor(image[:client.height * 3 // 2, :client.width], self.cv2.COLOR_YUV2BGR_NV12)
    except Exception:
      return None

  @staticmethod
  def _letterbox(cv2, image, shape=(DETECTOR_INPUT_SIZE, DETECTOR_INPUT_SIZE), color=(114, 114, 114)):
    image_height, image_width = image.shape[:2]
    target_height, target_width = shape
    ratio = min(target_width / image_width, target_height / image_height)
    new_width = int(round(image_width * ratio))
    new_height = int(round(image_height * ratio))
    resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)

    pad_width = (target_width - new_width) / 2
    pad_height = (target_height - new_height) / 2
    top = int(round(pad_height - 0.1))
    bottom = int(round(pad_height + 0.1))
    left = int(round(pad_width - 0.1))
    right = int(round(pad_width + 0.1))
    result = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return result, ratio, left, top

  def _uk_red_ring_score(self, sign_crop) -> float:
    if sign_crop is None or sign_crop.size == 0:
      return 0.0

    h, w = sign_crop.shape[:2]
    if h < 12 or w < 12:
      return 0.0
    aspect = w / max(h, 1)
    if aspect < 0.60 or aspect > 1.50:
      return 0.0

    hsv = self.cv2.cvtColor(sign_crop, self.cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]

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

  @staticmethod
  def _resolve_national_limit_from_mapd_values(tile_loaded: bool, road_context: str, one_way: bool) -> tuple[int, str]:
    """Resolve only road classes the bundled mapd schema can distinguish safely."""
    if not tile_loaded:
      return 0, "map_unavailable"
    context = str(road_context).lower()
    if "freeway" in context:
      return 70, "motorway"
    if "city" in context and not bool(one_way):
      # mapd's public schema has no explicit single-carriageway enum. A
      # non-freeway bidirectional OSM way is the deliberately narrow proxy used
      # for the requested single-carriageway 60 mph case.
      return 60, "single_carriageway"
    return 0, "unresolved"

  def _mapd_national_limit(self) -> tuple[int, str, str, bool, int]:
    if self.sm is None:
      return 0, "map_unavailable", "unknown", False, 0
    try:
      if not self.sm.valid.get("mapdOut", False) or not self.sm.alive.get("mapdOut", False):
        return 0, "map_unavailable", "unknown", False, 0
      msg = self.sm["mapdOut"]
      tile_loaded = bool(msg.tileLoaded)
      context = str(msg.roadContext)
      one_way = bool(msg.oneWay)
      lanes = int(msg.lanes)
      limit, road_class = self._resolve_national_limit_from_mapd_values(tile_loaded, context, one_way)
      return limit, road_class, context, one_way, lanes
    except Exception:
      return 0, "map_unavailable", "unknown", False, 0

  def _detect_national_sign(self, frame_bgr) -> Detection | None:
    if self.national_reader is None:
      return None

    resolved_mph, road_class, context, one_way, lanes = self._mapd_national_limit()
    if resolved_mph <= 0:
      return None

    frame_h, frame_w = frame_bgr.shape[:2]
    scale = min(1.0, NATIONAL_SCAN_MAX_WIDTH / max(frame_w, 1))
    if scale < 1.0:
      scan = self.cv2.resize(
        frame_bgr,
        (max(int(round(frame_w * scale)), 1), max(int(round(frame_h * scale)), 1)),
        interpolation=self.cv2.INTER_AREA,
      )
    else:
      scan = frame_bgr

    # The sign can appear on either side of UK roads, so scan the full width but
    # exclude the very bottom of the frame where wheels/road markings dominate.
    scan_h, scan_w = scan.shape[:2]
    roi_h = max(int(scan_h * 0.88), 1)
    gray = self.cv2.cvtColor(scan[:roi_h], self.cv2.COLOR_BGR2GRAY)
    gray = self.cv2.GaussianBlur(gray, (5, 5), 1.2)
    max_radius = max(int(min(scan_w, roi_h) * NATIONAL_HOUGH_MAX_RADIUS_RATIO), NATIONAL_HOUGH_MIN_RADIUS + 2)

    circles = self.cv2.HoughCircles(
      gray,
      self.cv2.HOUGH_GRADIENT,
      dp=1.25,
      minDist=max(int(scan_h * 0.025), 18),
      param1=110,
      param2=22,
      minRadius=NATIONAL_HOUGH_MIN_RADIUS,
      maxRadius=max_radius,
    )
    if circles is None:
      return None

    best = None
    for cx, cy, radius in np.round(circles[0]).astype(int)[:24]:
      if radius <= 0:
        continue
      # Expand slightly beyond the Hough circle so the entire sign face/edge is
      # available to the stripe reader.
      pad_r = max(int(round(radius * 1.12)), radius + 1)
      sx1 = max(cx - pad_r, 0)
      sy1 = max(cy - pad_r, 0)
      sx2 = min(cx + pad_r, scan_w)
      sy2 = min(cy + pad_r, roi_h)
      if sx2 <= sx1 or sy2 <= sy1:
        continue

      if scale < 1.0:
        x1 = max(int(round(sx1 / scale)), 0)
        y1 = max(int(round(sy1 / scale)), 0)
        x2 = min(int(round(sx2 / scale)), frame_w)
        y2 = min(int(round(sy2 / scale)), frame_h)
      else:
        x1, y1, x2, y2 = sx1, sy1, sx2, sy2

      crop = frame_bgr[y1:y2, x1:x2]
      national = self.national_reader.read(crop)
      if national is None or national.confidence < NATIONAL_MIN_SCORE:
        continue

      detection = Detection(
        speed_limit_mph=int(resolved_mph),
        confidence=float(national.confidence),
        model_confidence=0.0,
        ring_score=0.0,
        value_confidence=float(national.confidence),
        legacy_speed_mph=0,
        bbox=(x1, y1, x2, y2),
        source="national",
      )
      if best is None or detection.confidence > best.confidence:
        best = detection

    if best is not None:
      now = time.monotonic()
      if now - self._last_national_log_at >= 0.5:
        bbox_text = ",".join(str(int(v)) for v in best.bbox) if best.bbox is not None else "none"
        cloudlog.info(
          f"[XNOR_VSL_V12UK] national=1 score={best.confidence:.3f} "
          f"mapClass={road_class} mapContext={context} oneWay={int(one_way)} lanes={lanes} "
          f"resolved={best.speed_limit_mph}mph bbox={bbox_text}"
        )
        self._last_national_log_at = now
    return best

  @staticmethod
  def _clamp_bbox(bbox, width: int, height: int):
    x1, y1, x2, y2 = bbox
    result = (
      max(int(round(x1)), 0),
      max(int(round(y1)), 0),
      min(int(round(x2)), int(width)),
      min(int(round(y2)), int(height)),
    )
    return result if result[2] > result[0] and result[3] > result[1] else None

  def _track_feature_points(self, gray: np.ndarray, bbox):
    height, width = gray.shape[:2]
    x1, y1, x2, y2 = bbox
    bw = x2 - x1
    bh = y2 - y1
    pad_x = max(int(bw * 0.25), 3)
    pad_y = max(int(bh * 0.25), 3)
    mask = np.zeros_like(gray)
    mask[max(y1-pad_y, 0):min(y2+pad_y, height), max(x1-pad_x, 0):min(x2+pad_x, width)] = 255
    return self.cv2.goodFeaturesToTrack(
      gray,
      mask=mask,
      maxCorners=48,
      qualityLevel=0.004,
      minDistance=3,
      blockSize=5,
    )

  def _start_track(self, frame_bgr: np.ndarray, detection: Detection, now: float) -> None:
    if detection.bbox is None:
      return
    try:
      gray = self.cv2.cvtColor(frame_bgr, self.cv2.COLOR_BGR2GRAY)
      points = self._track_feature_points(gray, detection.bbox)
      if points is None or len(points) < TRACK_MIN_FEATURE_COUNT:
        return
      self.track = SignTrack(
        source="national" if detection.source.startswith("national") else "numeric",
        speed_limit_mph=int(detection.speed_limit_mph),
        bbox=detection.bbox,
        previous_gray=gray,
        points=points,
        started_at=float(now),
        last_frame_at=float(now),
      )
      cloudlog.info(
        f"[XNOR_VSL_V12UK] track_start source={self.track.source} "
        f"speed={self.track.speed_limit_mph}mph features={len(points)}"
      )
    except Exception:
      self.track = None

  def _clear_track(self, reason: str) -> None:
    if self.track is not None:
      cloudlog.info(
        f"[XNOR_VSL_V12UK] track_clear source={self.track.source} "
        f"speed={self.track.speed_limit_mph}mph reason={reason}"
      )
    self.track = None

  def _track_detection(self, frame_bgr: np.ndarray, now: float) -> Detection | None:
    track = self.track
    if track is None:
      return None
    if now - track.started_at > TRACK_MAX_AGE_SECONDS:
      self._clear_track("age")
      return None

    current_gray = self.cv2.cvtColor(frame_bgr, self.cv2.COLOR_BGR2GRAY)
    points = track.points
    if points is None or len(points) < TRACK_MIN_FEATURE_COUNT:
      points = self._track_feature_points(track.previous_gray, track.bbox)
    if points is None or len(points) < TRACK_MIN_FEATURE_COUNT:
      self._clear_track("features")
      return None

    next_points, status, errors = self.cv2.calcOpticalFlowPyrLK(
      track.previous_gray,
      current_gray,
      points,
      None,
      winSize=(25, 25),
      maxLevel=3,
      criteria=(self.cv2.TERM_CRITERIA_EPS | self.cv2.TERM_CRITERIA_COUNT, 20, 0.03),
    )
    if next_points is None or status is None:
      self._clear_track("flow")
      return None

    good = status.reshape(-1).astype(bool)
    if errors is not None:
      good &= errors.reshape(-1) < 35.0
    old = points.reshape(-1, 2)[good]
    new = next_points.reshape(-1, 2)[good]
    if len(old) < TRACK_MIN_FEATURE_COUNT:
      self._clear_track("flow_features")
      return None

    transform, inliers = self.cv2.estimateAffinePartial2D(
      old, new, method=self.cv2.RANSAC, ransacReprojThreshold=3.0
    )
    if transform is None or inliers is None or int(inliers.sum()) < TRACK_MIN_FEATURE_COUNT:
      self._clear_track("transform")
      return None

    scale = float(np.hypot(transform[0, 0], transform[0, 1]))
    if not 0.78 <= scale <= 1.35:
      self._clear_track("scale")
      return None

    x1, y1, x2, y2 = track.bbox
    corners = np.float32(((x1, y1), (x2, y1), (x2, y2), (x1, y2))).reshape(-1, 1, 2)
    moved = self.cv2.transform(corners, transform).reshape(-1, 2)
    bbox = self._clamp_bbox(
      (moved[:, 0].min(), moved[:, 1].min(), moved[:, 0].max(), moved[:, 1].max()),
      frame_bgr.shape[1],
      frame_bgr.shape[0],
    )
    if bbox is None:
      self._clear_track("bbox")
      return None

    inlier_points = new[inliers.reshape(-1).astype(bool)].reshape(-1, 1, 2)
    track.bbox = bbox
    track.previous_gray = current_gray
    track.points = inlier_points
    track.last_frame_at = float(now)

    bx1, by1, bx2, by2 = bbox
    bw, bh = bx2 - bx1, by2 - by1
    px, py = max(int(bw * 0.08), 1), max(int(bh * 0.08), 1)
    crop_bbox = self._clamp_bbox(
      (bx1-px, by1-py, bx2+px, by2+py),
      frame_bgr.shape[1],
      frame_bgr.shape[0],
    )
    if crop_bbox is None:
      return None
    cx1, cy1, cx2, cy2 = crop_bbox
    crop = frame_bgr[cy1:cy2, cx1:cx2]

    detection = None
    if track.source == "numeric":
      ring = self._uk_red_ring_score(crop)
      read = self.value_reader.read(crop) if ring > 0.0 and self.value_reader is not None else None
      if read is not None and int(read.speed_limit_mph) == int(track.speed_limit_mph):
        confidence = float(np.clip(read.confidence * 0.65 + ring * 0.35, 0.0, 0.99))
        detection = Detection(
          speed_limit_mph=int(read.speed_limit_mph),
          confidence=confidence,
          model_confidence=0.0,
          ring_score=float(ring),
          value_confidence=float(read.confidence),
          legacy_speed_mph=0,
          bbox=bbox,
          source="numeric_track",
        )
    else:
      national = self.national_reader.read(crop) if self.national_reader is not None else None
      resolved_mph, road_class, context, one_way, lanes = self._mapd_national_limit()
      if (
        national is not None and
        resolved_mph == int(track.speed_limit_mph) and
        national.confidence >= NATIONAL_MIN_SCORE
      ):
        detection = Detection(
          speed_limit_mph=int(resolved_mph),
          confidence=float(national.confidence),
          model_confidence=0.0,
          ring_score=0.0,
          value_confidence=float(national.confidence),
          legacy_speed_mph=0,
          bbox=bbox,
          source="national_track",
        )

    if detection is None:
      track.failed_reads += 1
      if track.failed_reads >= TRACK_MAX_FAILED_READS:
        self._clear_track("read_fail")
      return None

    track.failed_reads = 0
    cloudlog.info(
      f"[XNOR_VSL_V12UK] track_read source={detection.source} "
      f"speed={detection.speed_limit_mph}mph confidence={detection.confidence:.3f}"
    )
    return detection

  def _detect(self, frame_bgr):
    if self.net is None:
      return None

    frame_h, frame_w = frame_bgr.shape[:2]
    lb, ratio, pad_x, pad_y = self._letterbox(self.cv2, frame_bgr)
    blob = self.cv2.dnn.blobFromImage(
      lb,
      scalefactor=1.0 / 255.0,
      size=(DETECTOR_INPUT_SIZE, DETECTOR_INPUT_SIZE),
      swapRB=True,
      crop=False,
    )
    self.net.setInput(blob)

    try:
      predictions = np.squeeze(self.net.forward())
    except Exception:
      cloudlog.exception("[XNOR_VSL_V12UK] detector forward failed")
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

      crop = frame_bgr[y1:y2, x1:x2]
      uk_score = self._uk_red_ring_score(crop)
      bbox = (x1, y1, x2, y2)
      raw_supported.append((model_conf, speed_mph, uk_score, bbox))
      if uk_score <= 0.0:
        continue

      value_read = self.value_reader.read(crop) if self.value_reader is not None else None
      if value_read is None:
        self._log_raw_proposal(
          "value_reject", speed_mph, model_conf, uk_score, 0, 0.0, 0.0, bbox
        )
        continue

      final_speed_mph = int(value_read.speed_limit_mph)
      value_conf = float(value_read.confidence)
      confidence = float(np.clip(
        value_conf * 0.60 +
        uk_score * 0.30 +
        model_conf * 0.10,
        0.0,
        0.99,
      ))
      self._log_raw_proposal(
        "value_accept", speed_mph, model_conf, uk_score,
        final_speed_mph, value_conf, confidence, bbox,
      )
      candidates.append(Detection(
        final_speed_mph,
        confidence,
        model_conf,
        uk_score,
        value_conf,
        speed_mph,
        bbox,
      ))

    if not candidates:
      if raw_supported:
        model_conf, legacy_speed_mph, uk_score, bbox = max(raw_supported, key=lambda item: item[0])
        if uk_score <= 0.0:
          self._log_raw_proposal(
            "ring_reject", legacy_speed_mph, model_conf, uk_score, 0, 0.0, 0.0, bbox
          )
      return self._detect_national_sign(frame_bgr)

    candidates.sort(key=lambda d: d.confidence, reverse=True)
    return candidates[0]

  def _prune_history(self, now: float) -> None:
    while self.history and now - self.history[0].created_at > HISTORY_SECONDS:
      self.history.popleft()

  def _update_detection(self, detection: Detection) -> None:
    now = time.monotonic()
    self._publish_lower_candidate(detection)
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

      if self.last_candidate_at > 0.0 and now - self.last_candidate_at > PUBLISHED_HOLD_SECONDS:
        self._clear_lower_candidate("stale")

      if self.published_speed_limit_mph > 0 and now - self.last_detection_at > PUBLISHED_HOLD_SECONDS:
        self._clear_publish("stale")

      if mem >= MEMORY_CRITICAL_PERCENT:
        self._clear_track("memory")
        self._disconnect_camera()
        self._set_status(f"UK vision paused - memory {mem:.0f}%")
        rk.keep_time()
        continue

      if not self._connect_camera():
        self._clear_track("camera")
        self._set_status("UK vision: waiting for camera")
        rk.keep_time()
        continue

      detector_interval = MEMORY_PRESSURE_INTERVAL if mem >= MEMORY_PRESSURE_PERCENT else NORMAL_INFERENCE_INTERVAL
      if now < self.followup_until:
        detector_interval = max(FOLLOWUP_INFERENCE_INTERVAL, min(detector_interval, NORMAL_INFERENCE_INTERVAL))

      detector_due = (now - self.last_inference_at) >= detector_interval
      track_due = self.track is not None and (now - self.track.last_frame_at) >= TRACK_FRAME_INTERVAL
      if not detector_due and not track_due:
        rk.keep_time()
        continue

      frame_bgr = self._receive_frame_bgr()
      if frame_bgr is None:
        rk.keep_time()
        continue

      # Cheap optical-flow confirmation runs between expensive full-frame
      # detector passes. A successful tracked read is a genuinely later camera
      # frame of the same physical sign and therefore counts toward 2/2 or 3/3.
      tracked_detection = None
      if track_due:
        tracked_detection = self._track_detection(frame_bgr, now)
        if tracked_detection is not None:
          self._update_detection(tracked_detection)

      # Do not count a detector result from the same frame as an additional
      # temporal read. If tracking succeeded, wait for the next frame.
      detection = None
      if tracked_detection is None and detector_due:
        self.last_inference_at = now
        detection = self._detect(frame_bgr)
        if detection is not None:
          self._update_detection(detection)
          self._start_track(frame_bgr, detection, now)

      if tracked_detection is None and detection is None:
        if self.published_speed_limit_mph > 0:
          self._set_status(f"UK vision: holding {self.published_speed_limit_mph} mph")
        elif self.last_candidate_at > 0.0:
          self._set_status("UK vision: provisional lower candidate active")
        else:
          self._set_status(f"UK vision: scanning {self.stream_name}")

      frame_bgr = None
      rk.keep_time()


def main() -> None:
  SpeedLimitVisionUK().run()


if __name__ == "__main__":
  main()
