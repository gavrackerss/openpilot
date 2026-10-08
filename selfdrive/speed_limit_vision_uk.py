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


# XNOR Vision Speed Limit V1.10-UK
#
# Runtime design is based on the speed-limit vision pipeline in StarPilot
# (firestar5683/StarPilot, Dom branch), but deliberately uses only the legacy
# single ONNX detector. V1-UK adds a UK red-ring geometry gate, full-frame
# scanning, conservative temporal confirmation and lower-only integration in
# Tesla CarState. It recognises numeric 20/30/40/50/60/70 mph signs.
#
# V1.10 adds fragmented/partial red-ring handling without weakening the normal
# strict-ring path. A proposal with >=50% ring evidence may reach OCR, but it is
# accepted only when the digit read is exceptionally clear and internally
# consistent; it still requires the normal independent numeric 2/2 confirmation.

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
NUMERIC_NMS_IOU_THRESHOLD = 0.55
NUMERIC_RING_RETRY_PADDING = 0.18
NUMERIC_RING_RECENTER_PADDING = 0.08
NUMERIC_RING_MIN_BOX_RATIO = 0.035
NUMERIC_PARTIAL_RING_MIN_SCORE = 0.50
NUMERIC_PARTIAL_OCR_MIN_CONFIDENCE = 0.78
NUMERIC_PARTIAL_FIRST_DIGIT_MIN = 0.48
NUMERIC_PARTIAL_ZERO_MIN = 0.48
NUMERIC_PARTIAL_MARGIN_MIN = 0.055
NUMERIC_PARTIAL_WHOLE_MIN = 0.32
NUMERIC_TRACK_INTERVAL = 0.05
NUMERIC_TRACK_OCR_INTERVAL = 0.10
NUMERIC_TRACK_MIN_DELAY = 0.08
NUMERIC_TRACK_MAX_AGE = 1.50
NUMERIC_TRACK_MAX_FRAMES = 24
NUMERIC_TRACK_ARM_MIN_CONFIDENCE = 0.40
NUMERIC_TRACK_MIN_CONFIDENCE = 0.52
NUMERIC_TRACK_INITIAL_SEARCH_PADDING = 2.00
NUMERIC_TRACK_SEARCH_PADDING = 1.00
NUMERIC_TRACK_INITIAL_MIN_SCALE = 0.40
NUMERIC_TRACK_INITIAL_MAX_SCALE = 3.00
NUMERIC_TRACK_MIN_SCALE = 0.55
NUMERIC_TRACK_MAX_SCALE = 2.20
NUMERIC_TRACK_INITIAL_MAX_CENTER_SHIFT = 2.50
NUMERIC_TRACK_MAX_CENTER_SHIFT = 1.50
NUMERIC_TRACK_MIN_RING_SCORE = 0.20
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
HIGHER_CHANGE_CONFIDENCE = 0.55
INITIAL_REQUIRED_READS = 2
CHANGE_REQUIRED_READS = 2
NATIONAL_REQUIRED_READS = 3
MAX_PROPOSALS = 8
MAX_BOX_AREA_RATIO = 0.18
MIN_BOX_WIDTH = 10
MIN_BOX_HEIGHT = 14
DETECTOR_INPUT_SIZE = 640
NATIONAL_SCAN_MAX_WIDTH = 960
NATIONAL_HOUGH_MIN_RADIUS = 7
NATIONAL_HOUGH_MAX_RADIUS_RATIO = 0.09
NATIONAL_MIN_SCORE = 0.78

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
class PendingNumeric:
  speed_limit_mph: int
  bbox: tuple[int, int, int, int]
  created_at: float
  last_track_at: float
  last_ocr_at: float
  frames: int = 0
  ocr_attempts: int = 0
  tracked_once: bool = False
  first_confidence: float = 0.0


@dataclass
class HistoryEntry:
  speed_limit_mph: int
  confidence: float
  created_at: float
  source_family: str


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
    self.pending_numeric: PendingNumeric | None = None
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
    if self.pending_numeric is not None and self.pending_numeric.speed_limit_mph == int(speed_limit_mph):
      self._clear_pending_numeric("published")

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
    # A newly-seen provisional lower candidate may be fresher than an older
    # confirmed publish that is expiring, so its own 300 s lifetime is managed
    # independently rather than being cleared here.
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
      self._set_status("UK vision V1.10: ready")
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

  def _numeric_red_mask(self, sign_crop):
    hsv = self.cv2.cvtColor(sign_crop, self.cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    return (
      (((hue <= 12) | (hue >= 168))) &
      (sat >= 60) &
      (val >= 48)
    ).astype(np.uint8) * 255

  def _find_numeric_red_bbox(self, sign_crop):
    """Locate a red sign perimeter, including fragmented red arcs."""
    if sign_crop is None or sign_crop.size == 0:
      return None, "none"

    h, w = sign_crop.shape[:2]
    if h < 12 or w < 12:
      return None, "small"

    try:
      red = self._numeric_red_mask(sign_crop)
      close3 = self.cv2.morphologyEx(
        red, self.cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8)
      )
      contours, _hier = self.cv2.findContours(
        close3, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE
      )
    except Exception:
      return None, "error"

    crop_area = float(max(w * h, 1))
    candidates = []

    # Existing behaviour: a single plausible near-square red component.
    for contour in contours:
      x, y, cw, ch = self.cv2.boundingRect(contour)
      if cw < 6 or ch < 6:
        continue
      aspect = cw / max(ch, 1)
      ratio = float(cw * ch) / crop_area
      if 0.50 <= aspect <= 1.80 and ratio >= NUMERIC_RING_MIN_BOX_RATIO:
        squareness = min(aspect, 1.0 / max(aspect, 1e-6))
        candidates.append((
          float(cw * ch) * (0.75 + 0.25 * squareness),
          (x, y, cw, ch),
          "single",
        ))

    # V1.10: join nearby red arcs before giving up. This is deliberately local
    # to an ONNX sign proposal; it does not perform a new full-frame search.
    significant = []
    min_piece_area = max(crop_area * 0.0015, 2.0)
    for contour in contours:
      area = float(self.cv2.contourArea(contour))
      if area < min_piece_area:
        continue
      x, y, cw, ch = self.cv2.boundingRect(contour)
      significant.append((x, y, cw, ch, area))

    if len(significant) >= 2:
      ux1 = min(x for x, _y, _cw, _ch, _a in significant)
      uy1 = min(y for _x, y, _cw, _ch, _a in significant)
      ux2 = max(x + cw for x, _y, cw, _ch, _a in significant)
      uy2 = max(y + ch for _x, y, _cw, ch, _a in significant)
      uw, uh = ux2 - ux1, uy2 - uy1
      aspect = uw / max(uh, 1)
      ratio = float(uw * uh) / crop_area
      red_area = sum(a for _x, _y, _cw, _ch, a in significant)
      if 0.50 <= aspect <= 1.80 and NUMERIC_RING_MIN_BOX_RATIO <= ratio <= 0.95:
        squareness = min(aspect, 1.0 / max(aspect, 1e-6))
        fill = min(red_area / max(float(uw * uh) * 0.20, 1.0), 1.0)
        candidates.append((
          float(uw * uh) * (0.60 + 0.25 * squareness + 0.15 * fill),
          (ux1, uy1, uw, uh),
          "fragment_union",
        ))

    # A slightly wider morphological bridge can reconnect blurred ring arcs.
    k = 5 if min(h, w) >= 32 else 3
    try:
      kernel = self.cv2.getStructuringElement(self.cv2.MORPH_ELLIPSE, (k, k))
      joined = self.cv2.morphologyEx(red, self.cv2.MORPH_CLOSE, kernel)
      joined_contours, _hier = self.cv2.findContours(
        joined, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE
      )
      for contour in joined_contours:
        x, y, cw, ch = self.cv2.boundingRect(contour)
        if cw < 7 or ch < 7:
          continue
        aspect = cw / max(ch, 1)
        ratio = float(cw * ch) / crop_area
        if 0.50 <= aspect <= 1.80 and NUMERIC_RING_MIN_BOX_RATIO <= ratio <= 0.95:
          squareness = min(aspect, 1.0 / max(aspect, 1e-6))
          candidates.append((
            float(cw * ch) * (0.65 + 0.35 * squareness),
            (x, y, cw, ch),
            "fragment_close",
          ))
    except Exception:
      pass

    if not candidates:
      return None, "none"
    candidates.sort(key=lambda item: item[0], reverse=True)
    _score, bbox, mode = candidates[0]
    return bbox, mode

  def _normalize_numeric_sign_crop(self, sign_crop):
    """Recenter a proposal on a whole or reconstructed red sign perimeter."""
    if sign_crop is None or sign_crop.size == 0:
      return sign_crop

    h, w = sign_crop.shape[:2]
    best, _mode = self._find_numeric_red_bbox(sign_crop)
    if best is None:
      return sign_crop

    x, y, cw, ch = best
    pad_x = max(int(round(cw * NUMERIC_RING_RECENTER_PADDING)), 2)
    pad_y = max(int(round(ch * NUMERIC_RING_RECENTER_PADDING)), 2)
    x1 = max(x - pad_x, 0)
    y1 = max(y - pad_y, 0)
    x2 = min(x + cw + pad_x, w)
    y2 = min(y + ch + pad_y, h)
    if x2 <= x1 or y2 <= y1:
      return sign_crop
    return sign_crop[y1:y2, x1:x2]

  def _uk_red_ring_assessment(self, sign_crop):
    """Return strict score, partial evidence, reject reason and diagnostics."""
    if sign_crop is None or sign_crop.size == 0:
      return 0.0, 0.0, "empty", "empty"

    ring_bbox, ring_mode = self._find_numeric_red_bbox(sign_crop)
    sign_crop = self._normalize_numeric_sign_crop(sign_crop)
    if sign_crop is None or sign_crop.size == 0:
      return 0.0, 0.0, "normalize", f"mode={ring_mode}"

    h, w = sign_crop.shape[:2]
    if h < 12 or w < 12:
      return 0.0, 0.0, "small", f"mode={ring_mode} size={w}x{h}"
    aspect = w / max(h, 1)
    if aspect < 0.55 or aspect > 1.70:
      return 0.0, 0.0, "aspect", f"mode={ring_mode} aspect={aspect:.2f}"

    hsv = self.cv2.cvtColor(sign_crop, self.cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    red = ((((hue <= 12) | (hue >= 168)) & (sat >= 60) & (val >= 48))).astype(np.uint8)
    white = ((val >= 120) & (sat <= 92)).astype(np.uint8)
    dark = ((val <= 125) & (sat <= 155)).astype(np.uint8)

    red_ratio = float(red.mean())
    white_ratio = float(white.mean())
    dark_ratio = float(dark.mean())

    yy, xx = np.mgrid[0:h, 0:w]
    nx = (xx - (w - 1) / 2.0) / max(w / 2.0, 1.0)
    ny = (yy - (h - 1) / 2.0) / max(h / 2.0, 1.0)
    rr = np.sqrt(nx * nx + ny * ny)

    ring_zone = (rr >= 0.50) & (rr <= 1.06)
    centre_zone = rr <= 0.48
    if not ring_zone.any() or not centre_zone.any():
      return 0.0, 0.0, "zones", f"mode={ring_mode}"

    ring_red = float(red[ring_zone].mean())
    centre_red = float(red[centre_zone].mean())
    centre_white = float(white[centre_zone].mean())
    centre_dark = float(dark[centre_zone].mean())

    # Measure angular coverage so several disconnected arcs can still provide
    # useful ring evidence even if no single contour is a complete circle.
    angles = np.arctan2(ny, nx)
    arc_bins = 16
    arc_hits = 0
    for idx in range(arc_bins):
      lo = -np.pi + (2.0 * np.pi * idx / arc_bins)
      hi = -np.pi + (2.0 * np.pi * (idx + 1) / arc_bins)
      sector = ring_zone & (angles >= lo) & (angles < hi)
      if sector.any() and float(red[sector].mean()) >= 0.045:
        arc_hits += 1
    arc_coverage = float(arc_hits) / float(arc_bins)

    evidence = float(np.clip(
      min(ring_red / 0.20, 1.0) * 0.40 +
      arc_coverage * 0.22 +
      min(centre_white / 0.58, 1.0) * 0.20 +
      min(centre_dark / 0.14, 1.0) * 0.10 +
      min(red_ratio / 0.18, 1.0) * 0.08,
      0.0,
      1.0,
    ))

    details = (
      f"mode={ring_mode} evidence={evidence:.3f} redRatio={red_ratio:.3f} "
      f"ringRed={ring_red:.3f} arc={arc_coverage:.3f} centreRed={centre_red:.3f} "
      f"centreWhite={centre_white:.3f} centreDark={centre_dark:.3f} "
      f"whiteRatio={white_ratio:.3f} darkRatio={dark_ratio:.3f}"
    )

    reason = "ok"
    if red_ratio < 0.025 or red_ratio > 0.50:
      reason = "red_ratio"
    elif white_ratio < 0.16:
      reason = "white_ratio"
    elif dark_ratio < 0.004:
      reason = "dark_ratio"
    elif ring_red < 0.055:
      reason = "ring_red"
    elif ring_red < (centre_red * 1.35 + 0.01):
      reason = "ring_contrast"
    elif centre_white < 0.18:
      reason = "centre_white"
    elif centre_dark < 0.006:
      reason = "centre_dark"

    if reason != "ok":
      return 0.0, evidence, reason, details

    strict_score = (
      min(ring_red / 0.28, 1.0) * 0.45 +
      min(centre_white / 0.65, 1.0) * 0.30 +
      min(centre_dark / 0.18, 1.0) * 0.15 +
      min(red_ratio / 0.20, 1.0) * 0.10
    )
    return float(np.clip(strict_score, 0.0, 1.0)), evidence, "ok", details

  def _uk_red_ring_score(self, sign_crop) -> float:
    strict, _evidence, _reason, _details = self._uk_red_ring_assessment(sign_crop)
    return float(strict)

  @staticmethod
  def _partial_numeric_read_is_clear(read) -> bool:
    if read is None:
      return False
    return bool(
      float(read.confidence) >= NUMERIC_PARTIAL_OCR_MIN_CONFIDENCE and
      float(read.first_digit_score) >= NUMERIC_PARTIAL_FIRST_DIGIT_MIN and
      float(read.zero_score) >= NUMERIC_PARTIAL_ZERO_MIN and
      float(read.first_digit_margin) >= NUMERIC_PARTIAL_MARGIN_MIN and
      float(read.whole_value_score) >= NUMERIC_PARTIAL_WHOLE_MIN
    )

  def _log_ring_assessment(self, decision: str, legacy_speed_mph: int,
                           model_confidence: float, strict_score: float,
                           evidence_score: float, reason: str, details: str,
                           bbox=None) -> None:
    bbox_text = "none" if bbox is None else ",".join(str(int(v)) for v in bbox)
    cloudlog.info(
      f"[XNOR_VSL_V12UK] ring_detail decision={decision} legacy={int(legacy_speed_mph)} "
      f"model={float(model_confidence):.3f} strict={float(strict_score):.3f} "
      f"evidence={float(evidence_score):.3f} reason={reason} "
      f"bbox={bbox_text} {details}"
    )

  @staticmethod
  def _resolve_national_limit_from_mapd_values(tile_loaded: bool, road_context: str,
                                                one_way: bool, lanes: int,
                                                way_ref: str) -> tuple[int, str]:
    """Resolve UK NSL only where the bundled mapd outputs are specific enough.

    mapd's 'freeway' context is an inferred matching class, not an OSM motorway
    tag. Require a UK motorway reference (M + digit) before assigning 70.
    Single-carriageway is scoped to a bidirectional, non-freeway way with at
    most three mapped lanes; dual carriageways are normally separate one-way
    OSM ways and therefore intentionally remain unresolved here.
    """
    if not tile_loaded:
      return 0, "map_unavailable"

    context = str(road_context).lower()
    lane_count = max(int(lanes), 0)
    refs = [part.strip().upper().replace(" ", "") for part in str(way_ref or "").split(";")]
    motorway_ref = any(len(ref) >= 2 and ref[0] == "M" and ref[1].isdigit() for ref in refs)

    if "freeway" in context and motorway_ref:
      return 70, "motorway"

    # V1.3: missing lane metadata is not evidence of a single carriageway.
    # Accept lane_count=0 only when OSM supplied a non-motorway road reference.
    # This blocks the observed false-positive context:
    #   mapContext=city oneWay=0 lanes=0 ref=
    non_motorway_ref = any(ref and not (len(ref) >= 2 and ref[0] == "M" and ref[1].isdigit()) for ref in refs)
    single_carriageway_metadata = (1 <= lane_count <= 3) or (lane_count == 0 and non_motorway_ref)
    if "freeway" not in context and not bool(one_way) and not motorway_ref and single_carriageway_metadata:
      return 60, "single_carriageway"

    return 0, "unresolved"

  def _mapd_national_limit(self) -> tuple[int, str, str, bool, int, str]:
    if self.sm is None:
      return 0, "map_unavailable", "unknown", False, 0, ""
    try:
      if not self.sm.valid.get("mapdOut", False) or not self.sm.alive.get("mapdOut", False):
        return 0, "map_unavailable", "unknown", False, 0, ""
      msg = self.sm["mapdOut"]
      tile_loaded = bool(msg.tileLoaded)
      context = str(msg.roadContext)
      one_way = bool(msg.oneWay)
      lanes = int(msg.lanes)
      way_ref = str(msg.wayRef)
      limit, road_class = self._resolve_national_limit_from_mapd_values(
        tile_loaded, context, one_way, lanes, way_ref
      )
      return limit, road_class, context, one_way, lanes, way_ref
    except Exception:
      return 0, "map_unavailable", "unknown", False, 0, ""

  def _detect_national_sign(self, frame_bgr) -> Detection | None:
    if self.national_reader is None:
      return None

    resolved_mph, road_class, context, one_way, lanes, way_ref = self._mapd_national_limit()
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
          f"mapClass={road_class} mapContext={context} oneWay={int(one_way)} lanes={lanes} ref={way_ref} "
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

  @staticmethod
  def _bbox_iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(ix2 - ix1, 0), max(iy2 - iy1, 0)
    inter = float(iw * ih)
    if inter <= 0.0:
      return 0.0
    area_a = float(max(ax2 - ax1, 0) * max(ay2 - ay1, 0))
    area_b = float(max(bx2 - bx1, 0) * max(by2 - by1, 0))
    union = area_a + area_b - inter
    return inter / union if union > 0.0 else 0.0

  @classmethod
  def _expand_bbox(cls, bbox, width: int, height: int, padding_ratio: float):
    x1, y1, x2, y2 = bbox
    bw, bh = x2 - x1, y2 - y1
    px = max(int(round(bw * padding_ratio)), 2)
    py = max(int(round(bh * padding_ratio)), 2)
    return cls._clamp_bbox((x1 - px, y1 - py, x2 + px, y2 + py), width, height)

  @classmethod
  def _dedupe_numeric_proposals(cls, proposals):
    """Keep one ONNX proposal per physical sign before running digit OCR."""
    if not proposals:
      return []

    # Ring geometry is a stronger UK-sign cue than the legacy class score.
    ordered = sorted(
      proposals,
      key=lambda item: (
        float(item[2]) > 0.0,
        float(item[2]),
        float(item[4]) if len(item) > 4 else 0.0,
        float(item[0]),
      ),
      reverse=True,
    )
    kept = []
    for proposal in ordered:
      bbox = proposal[3]
      if any(cls._bbox_iou(bbox, existing[3]) >= NUMERIC_NMS_IOU_THRESHOLD for existing in kept):
        continue
      kept.append(proposal)
      if len(kept) >= MAX_PROPOSALS:
        break
    return kept

  def _clear_pending_numeric(self, reason: str) -> None:
    pending = self.pending_numeric
    if pending is not None:
      cloudlog.info(
        f"[XNOR_VSL_V12UK] numeric_pending_clear speed={pending.speed_limit_mph}mph "
        f"frames={pending.frames} ocr={pending.ocr_attempts} reason={reason}"
      )
    self.pending_numeric = None

  def _arm_pending_numeric(self, detection: Detection, now: float | None = None) -> None:
    if detection.bbox is None or not detection.source.startswith("numeric"):
      return
    if detection.confidence < NUMERIC_TRACK_ARM_MIN_CONFIDENCE:
      return
    if self.published_speed_limit_mph == int(detection.speed_limit_mph):
      self._clear_pending_numeric("already_published")
      return

    stamp = time.monotonic() if now is None else float(now)
    self.pending_numeric = PendingNumeric(
      speed_limit_mph=int(detection.speed_limit_mph),
      bbox=detection.bbox,
      created_at=stamp,
      last_track_at=stamp,
      last_ocr_at=stamp,
      frames=0,
      ocr_attempts=0,
      tracked_once=False,
      first_confidence=float(detection.confidence),
    )
    bbox_text = ",".join(str(int(v)) for v in detection.bbox)
    cloudlog.info(
      f"[XNOR_VSL_V12UK] numeric_pending_arm speed={detection.speed_limit_mph}mph "
      f"confidence={detection.confidence:.3f} bbox={bbox_text}"
    )

  def _find_tracked_numeric_ring(self, frame_bgr: np.ndarray, bbox, initial: bool = False):
    """Follow the already-detected red ring in a small local ROI.

    This is intentionally not a second full-frame detector. Candidate geometry
    must remain close in position/scale to the previously accepted physical
    sign and must still pass the normal UK red-ring scorer.
    """
    frame_h, frame_w = frame_bgr.shape[:2]
    search_padding = NUMERIC_TRACK_INITIAL_SEARCH_PADDING if initial else NUMERIC_TRACK_SEARCH_PADDING
    min_scale = NUMERIC_TRACK_INITIAL_MIN_SCALE if initial else NUMERIC_TRACK_MIN_SCALE
    max_scale = NUMERIC_TRACK_INITIAL_MAX_SCALE if initial else NUMERIC_TRACK_MAX_SCALE
    max_shift = NUMERIC_TRACK_INITIAL_MAX_CENTER_SHIFT if initial else NUMERIC_TRACK_MAX_CENTER_SHIFT

    search_bbox = self._expand_bbox(
      bbox, frame_w, frame_h, search_padding
    )
    if search_bbox is None:
      return None

    sx1, sy1, sx2, sy2 = search_bbox
    search = frame_bgr[sy1:sy2, sx1:sx2]
    if search.size == 0:
      return None

    try:
      hsv = self.cv2.cvtColor(search, self.cv2.COLOR_BGR2HSV)
      hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
      red = (
        (((hue <= 12) | (hue >= 168))) &
        (sat >= 65) &
        (val >= 50)
      ).astype(np.uint8) * 255
      red = self.cv2.morphologyEx(
        red, self.cv2.MORPH_CLOSE, np.ones((3, 3), dtype=np.uint8)
      )
      contours, _hier = self.cv2.findContours(
        red, self.cv2.RETR_EXTERNAL, self.cv2.CHAIN_APPROX_SIMPLE
      )
    except Exception:
      return None

    px1, py1, px2, py2 = bbox
    prev_w = max(px2 - px1, 1)
    prev_h = max(py2 - py1, 1)
    prev_cx = (px1 + px2) * 0.5
    prev_cy = (py1 + py2) * 0.5

    best = None
    best_rank = -1.0
    for contour in contours:
      x, y, cw, ch = self.cv2.boundingRect(contour)
      if cw < 8 or ch < 8:
        continue
      aspect = cw / max(ch, 1)
      if aspect < 0.55 or aspect > 1.65:
        continue

      gx1, gy1 = sx1 + x, sy1 + y
      gx2, gy2 = gx1 + cw, gy1 + ch
      scale_w = cw / float(prev_w)
      scale_h = ch / float(prev_h)
      if not (
        min_scale <= scale_w <= max_scale and
        min_scale <= scale_h <= max_scale
      ):
        continue

      cx = (gx1 + gx2) * 0.5
      cy = (gy1 + gy2) * 0.5
      dx = abs(cx - prev_cx) / float(prev_w)
      dy = abs(cy - prev_cy) / float(prev_h)
      if dx > max_shift or dy > max_shift:
        continue

      # Add a little white margin around the red contour so ring/centre
      # geometry and digit OCR see the complete circular sign face.
      pad_x = max(int(round(cw * NUMERIC_RING_RECENTER_PADDING)), 2)
      pad_y = max(int(round(ch * NUMERIC_RING_RECENTER_PADDING)), 2)
      candidate = self._clamp_bbox(
        (gx1 - pad_x, gy1 - pad_y, gx2 + pad_x, gy2 + pad_y),
        frame_w,
        frame_h,
      )
      if candidate is None:
        continue
      x1, y1, x2, y2 = candidate
      crop = frame_bgr[y1:y2, x1:x2]
      strict_ring, partial_ring, ring_reason, _ring_details = self._uk_red_ring_assessment(crop)
      ring = float(strict_ring if strict_ring > 0.0 else partial_ring)
      partial = strict_ring <= 0.0
      if partial:
        if partial_ring < NUMERIC_PARTIAL_RING_MIN_SCORE:
          continue
      elif ring < NUMERIC_TRACK_MIN_RING_SCORE:
        continue

      motion = float(np.hypot(dx, dy))
      proximity = max(0.0, 1.0 - motion / max(max_shift * 1.414, 1e-3))
      squareness = min(aspect, 1.0 / max(aspect, 1e-6))
      rank = float(ring) * 0.70 + proximity * 0.20 + squareness * 0.10
      if rank > best_rank:
        best_rank = rank
        best = (candidate, float(ring), bool(partial), str(ring_reason))

    return best

  def _reacquire_pending_numeric(self, frame_bgr: np.ndarray, now: float) -> Detection | None:
    pending = self.pending_numeric
    if pending is None:
      return None

    age = now - pending.created_at
    if age > NUMERIC_TRACK_MAX_AGE:
      self._clear_pending_numeric("age")
      return None
    if age < NUMERIC_TRACK_MIN_DELAY:
      return None
    if now - pending.last_track_at < NUMERIC_TRACK_INTERVAL:
      return None

    pending.last_track_at = float(now)
    pending.frames += 1

    found = self._find_tracked_numeric_ring(
      frame_bgr, pending.bbox, initial=not pending.tracked_once
    )
    if found is None:
      if pending.frames <= 3 or pending.frames % 5 == 0:
        cloudlog.info(
          f"[XNOR_VSL_V12UK] numeric_track_miss speed={pending.speed_limit_mph}mph "
          f"frame={pending.frames}/{NUMERIC_TRACK_MAX_FRAMES}"
        )
      if pending.frames >= NUMERIC_TRACK_MAX_FRAMES:
        self._clear_pending_numeric("frames")
      return None

    tracked_bbox, ring, ring_partial, ring_reason = found
    pending.bbox = tracked_bbox
    pending.tracked_once = True
    bbox_text = ",".join(str(int(v)) for v in tracked_bbox)

    # Track at up to 20 Hz, but the template digit reader is the more expensive
    # part. Re-read digits at up to 10 Hz on the best continuously-followed box.
    if now - pending.last_ocr_at < NUMERIC_TRACK_OCR_INTERVAL:
      if pending.frames <= 3 or pending.frames % 5 == 0:
        cloudlog.info(
          f"[XNOR_VSL_V12UK] numeric_track speed={pending.speed_limit_mph}mph "
          f"frame={pending.frames} ring={ring:.3f} bbox={bbox_text}"
        )
      return None

    pending.last_ocr_at = float(now)
    pending.ocr_attempts += 1
    x1, y1, x2, y2 = tracked_bbox
    crop = frame_bgr[y1:y2, x1:x2]
    read = self.value_reader.read(crop) if self.value_reader is not None else None

    if read is not None:
      self._log_ocr_accept("numeric_track", read, tracked_bbox)

    if read is None:
      cloudlog.info(
        f"[XNOR_VSL_V12UK] numeric_track_read_miss speed={pending.speed_limit_mph}mph "
        f"frame={pending.frames} ocr={pending.ocr_attempts} ring={ring:.3f}"
      )
      self._log_ocr_reject("numeric_track", tracked_bbox)
      return None

    value_conf = float(read.confidence)
    read_speed = int(read.speed_limit_mph)

    if ring_partial and not self._partial_numeric_read_is_clear(read):
      cloudlog.info(
        f"[XNOR_VSL_V12UK] numeric_track_partial_reject expected={pending.speed_limit_mph}mph "
        f"read={read_speed}mph ring={ring:.3f} reason={ring_reason} "
        f"valueConf={value_conf:.3f} first={float(read.first_digit_score):.3f} "
        f"zero={float(read.zero_score):.3f} margin={float(read.first_digit_margin):.3f} "
        f"whole={float(read.whole_value_score):.3f}"
      )
      return None

    confidence = float(np.clip(value_conf * 0.65 + ring * 0.35, 0.0, 0.99))

    if read_speed != int(pending.speed_limit_mph):
      cloudlog.info(
        f"[XNOR_VSL_V12UK] numeric_track_mismatch expected={pending.speed_limit_mph}mph "
        f"read={read_speed}mph confidence={confidence:.3f} frame={pending.frames}"
      )
      return None

    if confidence < NUMERIC_TRACK_MIN_CONFIDENCE:
      cloudlog.info(
        f"[XNOR_VSL_V12UK] numeric_track_low_conf speed={read_speed}mph "
        f"confidence={confidence:.3f} frame={pending.frames}"
      )
      return None

    detection = Detection(
      speed_limit_mph=read_speed,
      confidence=confidence,
      model_confidence=0.0,
      ring_score=float(ring),
      value_confidence=value_conf,
      legacy_speed_mph=0,
      bbox=tracked_bbox,
      source="numeric_reacquire_partial" if ring_partial else "numeric_reacquire",
    )
    cloudlog.info(
      f"[XNOR_VSL_V12UK] numeric_reacquire speed={detection.speed_limit_mph}mph "
      f"confidence={detection.confidence:.3f} ring={ring:.3f} "
      f"valueConf={value_conf:.3f} frame={pending.frames} "
      f"ocr={pending.ocr_attempts} bbox={bbox_text}"
    )
    self._clear_pending_numeric("confirmed")
    return detection

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
    # V1.10 confirmation comes only from independent detector passes on fresh
    # camera frames. The previous numeric optical-flow path could fail before
    # 2/2 even after a strong first read, while tracking one bad object could
    # also make confirmation less independent.
    if detection.source.startswith(("numeric", "national")):
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
      resolved_mph, road_class, context, one_way, lanes, way_ref = self._mapd_national_limit()
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

  def _log_ocr_reject(self, source: str, bbox=None) -> None:
    if self.value_reader is None:
      return
    reason = str(getattr(self.value_reader, "last_reject_reason", "") or "")
    debug = str(getattr(self.value_reader, "last_debug", "") or "")
    if not reason and not debug:
      return
    elapsed_ms = float(getattr(self.value_reader, "last_elapsed_ms", 0.0) or 0.0)
    bbox_text = "none" if bbox is None else ",".join(str(int(v)) for v in bbox)
    cloudlog.info(
      f"[XNOR_VSL_V12UK] ocr_reject source={source} bbox={bbox_text} "
      f"elapsedMs={elapsed_ms:.1f} reason={reason} debug={debug}"
    )

  def _log_ocr_accept(self, source: str, read, bbox=None) -> None:
    if read is None or self.value_reader is None:
      return
    elapsed_ms = float(getattr(self.value_reader, "last_elapsed_ms", 0.0) or 0.0)
    method = str(getattr(read, "method", "") or "legacy")
    bbox_text = "none" if bbox is None else ",".join(str(int(v)) for v in bbox)
    cloudlog.info(
      f"[XNOR_VSL_V12UK] ocr_accept source={source} speed={int(read.speed_limit_mph)}mph "
      f"confidence={float(read.confidence):.3f} method={method} "
      f"elapsedMs={elapsed_ms:.1f} bbox={bbox_text}"
    )

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
    raw_supported = []

    # Stage 1: cheap proposal/ring pass only. Do not run digit OCR repeatedly
    # over the several nearly-identical boxes the legacy detector often emits
    # for one physical sign.
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

      bbox = (x1, y1, x2, y2)
      crop = frame_bgr[y1:y2, x1:x2]
      uk_score, partial_score, ring_reason, ring_details = self._uk_red_ring_assessment(crop)

      # A tight model box can clip the perimeter. Retry once with a modest
      # expansion and retain whichever crop has stronger total ring evidence.
      retry_bbox = self._expand_bbox(
        bbox, frame_w, frame_h, NUMERIC_RING_RETRY_PADDING
      )
      if retry_bbox is not None and retry_bbox != bbox:
        rx1, ry1, rx2, ry2 = retry_bbox
        retry_crop = frame_bgr[ry1:ry2, rx1:rx2]
        retry_strict, retry_partial, retry_reason, retry_details = self._uk_red_ring_assessment(retry_crop)
        current_rank = (float(uk_score) > 0.0, float(uk_score), float(partial_score))
        retry_rank = (float(retry_strict) > 0.0, float(retry_strict), float(retry_partial))
        if retry_rank > current_rank:
          uk_score = float(retry_strict)
          partial_score = float(retry_partial)
          ring_reason = str(retry_reason)
          ring_details = str(retry_details)
          bbox = retry_bbox

      raw_supported.append((
        model_conf, speed_mph, uk_score, bbox,
        partial_score, ring_reason, ring_details,
      ))

    if not raw_supported:
      return self._detect_national_sign(frame_bgr)

    proposals = self._dedupe_numeric_proposals(raw_supported)
    if len(proposals) < len(raw_supported):
      cloudlog.info(
        f"[XNOR_VSL_V12UK] numeric_dedupe raw={len(raw_supported)} kept={len(proposals)}"
      )

    # Stage 2: digit OCR only on spatially distinct proposals. Strict-ring
    # candidates follow the established path. Partial-ring candidates may reach
    # OCR at >=0.50 evidence, but require the stronger clear-read gate.
    candidates = []
    for model_conf, speed_mph, uk_score, bbox, partial_score, ring_reason, ring_details in proposals:
      x1, y1, x2, y2 = bbox
      crop = frame_bgr[y1:y2, x1:x2]

      partial_path = uk_score <= 0.0 and partial_score >= NUMERIC_PARTIAL_RING_MIN_SCORE
      if uk_score <= 0.0 and not partial_path:
        self._log_raw_proposal(
          "ring_reject", speed_mph, model_conf, uk_score, 0, 0.0, 0.0, bbox
        )
        self._log_ring_assessment(
          "reject", speed_mph, model_conf, uk_score, partial_score,
          ring_reason, ring_details, bbox,
        )
        continue

      value_read = self.value_reader.read(crop) if self.value_reader is not None else None
      if value_read is None:
        self._log_raw_proposal(
          "partial_value_reject" if partial_path else "value_reject",
          speed_mph, model_conf, partial_score if partial_path else uk_score,
          0, 0.0, 0.0, bbox,
        )
        if partial_path:
          self._log_ring_assessment(
            "partial_ocr_reject", speed_mph, model_conf, uk_score, partial_score,
            ring_reason, ring_details, bbox,
          )
        self._log_ocr_reject("detector_partial" if partial_path else "detector", bbox)
        continue

      self._log_ocr_accept("detector_partial" if partial_path else "detector", value_read, bbox)

      if partial_path and not self._partial_numeric_read_is_clear(value_read):
        self._log_raw_proposal(
          "partial_clear_reject", speed_mph, model_conf, partial_score,
          int(value_read.speed_limit_mph), float(value_read.confidence), 0.0, bbox,
        )
        self._log_ring_assessment(
          "partial_clear_reject", speed_mph, model_conf, uk_score, partial_score,
          ring_reason, ring_details, bbox,
        )
        cloudlog.info(
          f"[XNOR_VSL_V12UK] partial_clear_reject read={int(value_read.speed_limit_mph)}mph "
          f"valueConf={float(value_read.confidence):.3f} "
          f"first={float(value_read.first_digit_score):.3f} "
          f"zero={float(value_read.zero_score):.3f} "
          f"margin={float(value_read.first_digit_margin):.3f} "
          f"whole={float(value_read.whole_value_score):.3f}"
        )
        continue

      final_speed_mph = int(value_read.speed_limit_mph)
      value_conf = float(value_read.confidence)
      effective_ring = float(partial_score if partial_path else uk_score)
      confidence = float(np.clip(
        value_conf * 0.60 +
        effective_ring * 0.30 +
        model_conf * 0.10,
        0.0,
        0.99,
      ))
      self._log_raw_proposal(
        "partial_value_accept" if partial_path else "value_accept",
        speed_mph, model_conf, effective_ring,
        final_speed_mph, value_conf, confidence, bbox,
      )
      if partial_path:
        self._log_ring_assessment(
          "partial_accept", speed_mph, model_conf, uk_score, partial_score,
          ring_reason, ring_details, bbox,
        )
      candidates.append(Detection(
        final_speed_mph,
        confidence,
        model_conf,
        effective_ring,
        value_conf,
        speed_mph,
        bbox,
        "numeric_partial" if partial_path else "numeric",
      ))

    if not candidates:
      return self._detect_national_sign(frame_bgr)

    candidates.sort(key=lambda d: d.confidence, reverse=True)
    return candidates[0]

  def _prune_history(self, now: float) -> None:
    while self.history and now - self.history[0].created_at > HISTORY_SECONDS:
      self.history.popleft()

  def _update_detection(self, detection: Detection) -> None:
    now = time.monotonic()
    self.followup_until = max(self.followup_until, now + FOLLOWUP_WINDOW_SECONDS)

    is_national = detection.source.startswith("national")
    source_family = "national" if is_national else "numeric"
    self.history.append(HistoryEntry(
      detection.speed_limit_mph,
      detection.confidence,
      now,
      source_family,
    ))
    self._prune_history(now)

    counts = Counter(
      (x.speed_limit_mph, x.source_family)
      for x in self.history
    )
    count = counts.get((detection.speed_limit_mph, source_family), 0)
    confs = [
      x.confidence for x in self.history
      if x.speed_limit_mph == detection.speed_limit_mph and x.source_family == source_family
    ]
    best_conf = max(confs) if confs else 0.0
    initial_required = NATIONAL_REQUIRED_READS if is_national else INITIAL_REQUIRED_READS
    change_required = NATIONAL_REQUIRED_READS if is_national else CHANGE_REQUIRED_READS

    # Numeric signs retain the existing immediate lower-only provisional path.
    # NSL is heuristic-only, so do not expose it to CarState until all 3 fresh
    # full-frame confirmations have succeeded.
    if not is_national or count >= NATIONAL_REQUIRED_READS:
      self._publish_lower_candidate(detection)

    if best_conf < MIN_CONFIRMED_CONFIDENCE:
      self._log_temporal_candidate(detection, count, initial_required, "confidence_reject")
      self._set_status(f"UK candidate: {detection.speed_limit_mph} mph")
      return

    current = self.published_speed_limit_mph
    if current <= 0:
      if count >= initial_required:
        self._log_temporal_candidate(detection, count, initial_required, "publish")
        self._publish(detection.speed_limit_mph, best_conf)
      else:
        self._log_temporal_candidate(detection, count, initial_required, "waiting")
        self._set_status(f"UK candidate: {detection.speed_limit_mph} mph ({count}/{initial_required})")
      return

    if detection.speed_limit_mph == current:
      self._log_temporal_candidate(detection, count, 1, "refresh")
      self._refresh_publish_timestamp()
      self._set_status(f"UK vision: {current} mph ({best_conf * 100.0:.0f}%)")
      return

    if detection.speed_limit_mph < current:
      if count >= change_required:
        self._log_temporal_candidate(detection, count, change_required, "publish_lower")
        self._publish(detection.speed_limit_mph, best_conf)
      else:
        self._log_temporal_candidate(detection, count, change_required, "waiting_lower")
        self._set_status(f"UK lower candidate: {detection.speed_limit_mph} mph ({count}/{change_required})")
      return

    # V1.10: an independently confirmed higher camera sign is authoritative.
    # Numeric signs require the normal 2/2 plus a strong confidence floor;
    # NSL already requires 3/3 and its stricter 0.78 geometry threshold.
    higher_confidence_ok = is_national or best_conf >= HIGHER_CHANGE_CONFIDENCE
    if count >= change_required and higher_confidence_ok:
      self._log_temporal_candidate(detection, count, change_required, "publish_higher")
      self._publish(detection.speed_limit_mph, best_conf)
    else:
      decision = "waiting_higher" if higher_confidence_ok else "higher_confidence_reject"
      self._log_temporal_candidate(detection, count, change_required, decision)
      self._set_status(
        f"UK higher candidate: {detection.speed_limit_mph} mph "
        f"({count}/{change_required})"
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
        self._clear_pending_numeric("memory")
        self._disconnect_camera()
        self._set_status(f"UK vision paused - memory {mem:.0f}%")
        rk.keep_time()
        continue

      if not self._connect_camera():
        self._clear_track("camera")
        self._clear_pending_numeric("camera")
        self._set_status("UK vision: waiting for camera")
        rk.keep_time()
        continue

      detector_interval = MEMORY_PRESSURE_INTERVAL if mem >= MEMORY_PRESSURE_PERCENT else NORMAL_INFERENCE_INTERVAL
      if now < self.followup_until:
        detector_interval = max(FOLLOWUP_INFERENCE_INTERVAL, min(detector_interval, NORMAL_INFERENCE_INTERVAL))

      detector_due = (now - self.last_inference_at) >= detector_interval
      track_due = self.track is not None and (now - self.track.last_frame_at) >= TRACK_FRAME_INTERVAL
      pending_due = (
        self.pending_numeric is not None and
        (now - self.pending_numeric.created_at) >= NUMERIC_TRACK_MIN_DELAY and
        (now - self.pending_numeric.last_track_at) >= NUMERIC_TRACK_INTERVAL
      )
      if not detector_due and not track_due and not pending_due:
        rk.keep_time()
        continue

      frame_bgr = self._receive_frame_bgr()
      if frame_bgr is None:
        rk.keep_time()
        continue

      # V1.7 follows an already-accepted numeric sign locally at up to 20 Hz
      # and reruns digit OCR at up to 10 Hz. A successful read is from a fresh
      # camera frame and skips full ONNX on that same frame.
      reacquired_detection = None
      if pending_due:
        reacquired_detection = self._reacquire_pending_numeric(frame_bgr, time.monotonic())
        if reacquired_detection is not None:
          self._update_detection(reacquired_detection)

      # Legacy track support remains for diagnostics. Numeric/NSL tracks are
      # not armed by V1.5, so normal confirmation comes from reacquisition or
      # independent full detector passes.
      tracked_detection = None
      if reacquired_detection is None and track_due:
        # recv() and detector work can be expensive; use a fresh timestamp for
        # track age rather than the loop timestamp captured before camera I/O.
        tracked_detection = self._track_detection(frame_bgr, time.monotonic())
        if tracked_detection is not None:
          self._update_detection(tracked_detection)

      # Do not count a detector result from the same frame as an additional
      # temporal read. If tracking succeeded, wait for the next frame.
      detection = None
      if reacquired_detection is None and tracked_detection is None and detector_due:
        detection = self._detect(frame_bgr)
        # Stamp the completed inference. V1.2 stamped before inference, so a
        # slow detector pass could consume the entire 1.5 s track lifetime.
        self.last_inference_at = time.monotonic()
        if detection is not None:
          self._update_detection(detection)
          if detection.source == "numeric":
            self._arm_pending_numeric(detection, self.last_inference_at)
          # Do not optical-flow this detection into a second confirmation.
          # followup_until still schedules fresh detector passes as a fallback.

      if reacquired_detection is None and tracked_detection is None and detection is None:
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
