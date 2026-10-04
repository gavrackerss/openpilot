"""Shared constants/readiness helpers for XNOR Vision Speed Limit V1-UK.

Standard-library only because manager pre-imports PythonProcess modules at boot.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

VISION_MODEL_DIR = Path("/data/media/0/models")
VISION_MODEL_PATH = VISION_MODEL_DIR / "xnor_speed_limit_uk_v1.onnx"
VISION_MODEL_MARKER = VISION_MODEL_DIR / "xnor_speed_limit_uk_v1.verified"
VISION_MODEL_URL = (
  "https://raw.githubusercontent.com/firestar5683/StarPilot/"
  "c2109496a72e85ca702ecef7aff53ff45adcfb8c/"
  "starpilot/assets/vision_models/speed_limit_vision.onnx"
)
VISION_MODEL_SIZE = 12_249_823
VISION_MODEL_GIT_BLOB_SHA1 = "eadeda70187b34a083591da704dc66a4f9e328d5"

VISION_RUNTIME_ROOT = Path("/data/media/0/xnor_vision_runtime")
VISION_CV2_MARKER = VISION_RUNTIME_ROOT / ".opencv_ready"
VISION_CV2_PACKAGE = "opencv-python-headless==4.13.0.92"


def vision_model_ready() -> bool:
  try:
    return (
      VISION_MODEL_PATH.is_file()
      and VISION_MODEL_PATH.stat().st_size == VISION_MODEL_SIZE
      and VISION_MODEL_MARKER.is_file()
      and VISION_MODEL_MARKER.read_text(encoding="utf-8").strip() == VISION_MODEL_GIT_BLOB_SHA1
    )
  except OSError:
    return False


def _system_cv2_ready() -> bool:
  try:
    return importlib.util.find_spec("cv2") is not None
  except Exception:
    return False


def _private_cv2_ready() -> bool:
  try:
    return VISION_CV2_MARKER.is_file() and (VISION_RUNTIME_ROOT / "cv2").exists()
  except OSError:
    return False


def vision_cv2_ready() -> bool:
  return _system_cv2_ready() or _private_cv2_ready()


def prepare_cv2_path() -> None:
  if _system_cv2_ready():
    return
  if _private_cv2_ready():
    root = str(VISION_RUNTIME_ROOT)
    if root not in sys.path:
      sys.path.insert(0, root)
