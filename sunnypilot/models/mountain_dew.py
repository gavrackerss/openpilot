"""Stage the official comma.ai Mountain Dew big-model weights for inspection.

This is NOT a runnable xnor-dev/Qualcomm model profile. The upstream model is
a big-model tinygrad artifact intended for a different AMD/chestnut execution
path. No code here registers the file with modeld or changes the active bundle.
"""
from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import requests

from openpilot.common.params import Params
from openpilot.system.hardware.hw import Paths

MODEL_NAME = "Mountain Dew (comma.ai master, 2026-10-10)"
MODEL_COMMIT = "2713630e7950a61dc072467b5394a1043e34547f"
MODEL_FILENAME = "big_driving_tinygrad.pkl"
MODEL_SHA256 = "256f2178366479a211933701c03518621ed3970289e6a8a43e5a2bcb15e55b5f"
MODEL_SIZE = 799942179
MODEL_URL = (
  f"https://media.githubusercontent.com/media/commaai/openpilot/{MODEL_COMMIT}/"
  "openpilot/selfdrive/modeld/models/big_driving_tinygrad.pkl"
)
STAGE_SUBDIR = "experimental_mountain_dew"


def _verified(path: Path) -> bool:
  if not path.is_file() or path.stat().st_size != MODEL_SIZE:
    return False
  digest = hashlib.sha256()
  with path.open('rb') as f:
    for block in iter(lambda: f.read(1024 * 1024), b''):
      digest.update(block)
  return digest.hexdigest() == MODEL_SHA256


def stage_mountain_dew(root: str | Path | None = None, *,
                       source_file: str | Path | None = None,
                       params: Params | None = None) -> Path:
  """Fetch or import exact upstream weights; never install as active model."""
  params = params or Params()
  root = Path(root) if root is not None else Path(Paths.model_root())
  destination = root / STAGE_SUBDIR / MODEL_FILENAME
  destination.parent.mkdir(parents=True, exist_ok=True)
  if _verified(destination):
    params.put("ModelManager_MountainDewStageStatus", "Verified locally; not compatible with XNOR modeld")
    return destination

  if params.get_bool("IsOnroad") or not params.get_bool("IsOffroad"):
    raise RuntimeError("Mountain Dew staging is permitted off-road only")

  staged = destination.with_suffix(destination.suffix + ".partial")
  params.put("ModelManager_MountainDewStageStatus", "Downloading / importing; not active")
  try:
    h = hashlib.sha256()
    total = 0
    if source_file is not None:
      with Path(source_file).open("rb") as incoming, staged.open("wb") as out:
        for chunk in iter(lambda: incoming.read(1024 * 1024), b""):
          if params.get_bool("IsOnroad") or not params.get_bool("IsOffroad"):
            raise RuntimeError("Download cancelled: vehicle is on-road")
          total += len(chunk)
          if total > MODEL_SIZE:
            raise ValueError("Model file exceeds pinned size")
          h.update(chunk)
          out.write(chunk)
    else:
      # GitHub LFS media endpoint serves file bytes; a pointer alone fails the
      # exact size and SHA-256 checks below.
      with requests.get(MODEL_URL, stream=True, allow_redirects=True,
                        timeout=(20, 120)) as response:
        response.raise_for_status()
        with staged.open("wb") as out:
          for chunk in response.iter_content(chunk_size=1024 * 1024):
            if not chunk:
              continue
            if params.get_bool("IsOnroad") or not params.get_bool("IsOffroad"):
              raise RuntimeError("Download cancelled: vehicle is on-road")
            total += len(chunk)
            if total > MODEL_SIZE:
              raise ValueError("Model transfer larger than expected")
            h.update(chunk)
            out.write(chunk)

    if total != MODEL_SIZE or h.hexdigest() != MODEL_SHA256:
      raise ValueError(f"Mountain Dew model size/hash mismatch (received {total} bytes)")
    os.replace(staged, destination)
    params.put("ModelManager_MountainDewStageStatus", "Verified locally; staging only, not active")
    return destination
  except Exception as e:
    params.put("ModelManager_MountainDewStageStatus", f"Staging failed: {str(e)[:150]}")
    raise
  finally:
    staged.unlink(missing_ok=True)


def main() -> None:
  parser = argparse.ArgumentParser(description="Verify/cache Mountain Dew from comma.ai; never activates")
  parser.add_argument("--source", help="Previously downloaded upstream .pkl file for offline import")
  parser.add_argument("--dest-root", help="Root to stage under, defaults to Paths.model_root()")
  args = parser.parse_args()
  path = stage_mountain_dew(root=args.dest_root, source_file=args.source)
  print(f"Verified: {path}")
  print("This big model is NOT compatible with the current XNOR modeld runner and was not activated.")


if __name__ == "__main__":
  main()
