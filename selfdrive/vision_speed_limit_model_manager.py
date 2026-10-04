#!/usr/bin/env python3
"""Offroad installer for XNOR Vision Speed Limit V1-UK.

Downloads the pinned StarPilot legacy detector and, only when the base image
does not already provide cv2, installs a private headless OpenCV runtime under
/data/media/0. Nothing is installed into the system Python environment.
"""
from __future__ import annotations

import hashlib
import os
import shutil
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.vision_speed_limit_common import (
  VISION_CV2_MARKER,
  VISION_CV2_PACKAGE,
  VISION_MODEL_GIT_BLOB_SHA1,
  VISION_MODEL_MARKER,
  VISION_MODEL_PATH,
  VISION_MODEL_SIZE,
  VISION_MODEL_URL,
  VISION_RUNTIME_ROOT,
  vision_cv2_ready,
  vision_model_ready,
)

RETRY_SECONDS = 30.0


def _put_status(params: Params, value: str) -> None:
  try:
    if params.get("VisionSpeedLimitStatus") != value:
      params.put("VisionSpeedLimitStatus", value)
  except Exception:
    pass


def _git_blob_sha1(path: Path) -> str:
  size = path.stat().st_size
  digest = hashlib.sha1()
  digest.update(f"blob {size}\0".encode("ascii"))
  with path.open("rb") as f:
    while chunk := f.read(1024 * 1024):
      digest.update(chunk)
  return digest.hexdigest()


def _verify_or_mark_existing_model() -> bool:
  try:
    if not VISION_MODEL_PATH.is_file() or VISION_MODEL_PATH.stat().st_size != VISION_MODEL_SIZE:
      return False
    if _git_blob_sha1(VISION_MODEL_PATH) != VISION_MODEL_GIT_BLOB_SHA1:
      return False
    VISION_MODEL_MARKER.write_text(VISION_MODEL_GIT_BLOB_SHA1, encoding="utf-8")
    return True
  except OSError:
    return False


def _download_model(params: Params) -> bool:
  VISION_MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
  part = VISION_MODEL_PATH.with_suffix(".onnx.part")
  try:
    part.unlink(missing_ok=True)
  except OSError:
    pass

  _put_status(params, "Downloading UK vision speed-limit model...")
  try:
    request = urllib.request.Request(VISION_MODEL_URL, headers={"User-Agent": "XNOR-Vision-Speed-Limit/1"})
    with urllib.request.urlopen(request, timeout=45) as response, part.open("wb") as out:
      while True:
        block = response.read(1024 * 1024)
        if not block:
          break
        out.write(block)
      out.flush()
      os.fsync(out.fileno())

    if part.stat().st_size != VISION_MODEL_SIZE:
      raise RuntimeError(f"model size mismatch: {part.stat().st_size} != {VISION_MODEL_SIZE}")
    actual_sha = _git_blob_sha1(part)
    if actual_sha != VISION_MODEL_GIT_BLOB_SHA1:
      raise RuntimeError(f"model Git blob SHA mismatch: {actual_sha}")

    os.replace(part, VISION_MODEL_PATH)
    VISION_MODEL_MARKER.write_text(VISION_MODEL_GIT_BLOB_SHA1, encoding="utf-8")
    cloudlog.info("[XNOR_VISION_SL_V1] model download verified")
    return True
  except Exception as exc:
    cloudlog.warning(f"[XNOR_VISION_SL_V1] model download failed: {exc!r}")
    _put_status(params, f"Model download failed - retrying: {type(exc).__name__}")
    try:
      part.unlink(missing_ok=True)
    except OSError:
      pass
    return False


def _test_private_cv2() -> bool:
  env = os.environ.copy()
  old_path = env.get("PYTHONPATH", "")
  env["PYTHONPATH"] = str(VISION_RUNTIME_ROOT) + (os.pathsep + old_path if old_path else "")
  try:
    result = subprocess.run(
      [sys.executable, "-c", "import cv2; print(cv2.__version__)"],
      env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
      text=True, timeout=30, check=False,
    )
    if result.returncode == 0:
      VISION_CV2_MARKER.write_text(result.stdout.strip() or VISION_CV2_PACKAGE, encoding="utf-8")
      return True
  except Exception:
    pass
  return False


def _install_private_cv2(params: Params) -> bool:
  if vision_cv2_ready():
    return True

  _put_status(params, "Installing private OpenCV vision runtime...")
  try:
    if VISION_RUNTIME_ROOT.exists():
      shutil.rmtree(VISION_RUNTIME_ROOT)
    VISION_RUNTIME_ROOT.mkdir(parents=True, exist_ok=True)
  except OSError as exc:
    _put_status(params, f"OpenCV runtime directory failed: {type(exc).__name__}")
    return False

  commands = [
    [sys.executable, "-m", "pip", "install", "--disable-pip-version-check", "--no-deps",
     "--target", str(VISION_RUNTIME_ROOT), VISION_CV2_PACKAGE],
  ]
  uv = shutil.which("uv")
  if uv:
    commands.append([uv, "pip", "install", "--no-deps", "--target", str(VISION_RUNTIME_ROOT), VISION_CV2_PACKAGE])

  for command in commands:
    try:
      result = subprocess.run(
        command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, timeout=300, check=False,
      )
      if result.returncode == 0 and _test_private_cv2():
        cloudlog.info("[XNOR_VISION_SL_V1] private OpenCV runtime verified")
        return True
      cloudlog.warning(
        f"[XNOR_VISION_SL_V1] OpenCV installer failed rc={result.returncode}: "
        f"{result.stdout[-1000:] if result.stdout else ''}"
      )
    except Exception as exc:
      cloudlog.warning(f"[XNOR_VISION_SL_V1] OpenCV installer error: {exc!r}")

  _put_status(params, "OpenCV runtime install failed - retrying offroad")
  return False


def main() -> None:
  params = Params()
  while params.get_bool("TinklaVisionSpeedLimitEnabled"):
    model_ok = vision_model_ready() or _verify_or_mark_existing_model()
    if not model_ok:
      model_ok = _download_model(params)

    cv2_ok = vision_cv2_ready()
    if model_ok and not cv2_ok:
      cv2_ok = _install_private_cv2(params)

    if model_ok and cv2_ok:
      _put_status(params, "UK V1 model ready - starts automatically onroad")
      return

    time.sleep(RETRY_SECONDS)


if __name__ == "__main__":
  main()
