#!/usr/bin/env python3
"""Read-only hardware inventory for an existing comma installation.

No Params writes, model execution, CAN messages, process stops, firmware
operations, or source changes. Output excludes serial numbers, network
addresses, VINs, tokens, and environment variables.
"""
from pathlib import Path
import json
import platform
import shutil
import subprocess
import sys


def read(path):
  try:
    return Path(path).read_text().strip()
  except (OSError, UnicodeError):
    return None


def git(root, *args):
  try:
    return subprocess.check_output(["git", "-C", str(root), *args],
                                   text=True, stderr=subprocess.DEVNULL,
                                   timeout=5).strip()
  except (OSError, subprocess.SubprocessError):
    return None


def report(root):
  root = root.resolve()
  mem = {}
  for line in (read("/proc/meminfo") or "").splitlines():
    name, _, value = line.partition(":")
    if name in ("MemTotal", "MemAvailable", "SwapTotal", "SwapFree"):
      mem[name] = value.strip()
  states = {}
  for name in ("panda", "opendbc_repo", "tinygrad_repo"):
    states[name] = {"gitlink": git(root, "rev-parse", f"HEAD:{name}"),
                    "actual": git(root / name, "rev-parse", "HEAD")
                    if (root / name / ".git").exists() else None}
  # Do not enumerate USB devices or report serial-number-bearing filenames.
  processes = set()
  for item in Path("/proc").glob("[0-9]*/comm"):
    name = read(item)
    if name in {"modeld", "pandad", "manager", "controlsd", "card", "camerad"}:
      processes.add(name)
  model_dir = next((p for p in (root / "openpilot/selfdrive/modeld/models",
                                root / "selfdrive/modeld/models") if p.is_dir()), None)
  models = {}
  if model_dir:
    for filename in ("driving_supercombo.onnx", "driving_tinygrad.pkl",
                     "driving_vision.onnx", "driving_on_policy.onnx",
                     "driving_off_policy.onnx", "tg_input_devices.json"):
      path = model_dir / filename
      parts = sorted(model_dir.glob(filename + ".chunk*"))
      models[filename] = {"direct_bytes": path.stat().st_size if path.is_file() else None,
                         "chunk_files": sum(p.is_file() and p.name != filename + ".chunkmanifest" for p in parts)}
  return {"scope": "read-only capability inventory, not qualification",
          "agnos_marker": Path("/AGNOS").exists(), "machine": platform.machine(),
          "kernel": platform.release(), "python": platform.python_version(),
          "kgsl_present": Path("/dev/kgsl-3d0").exists(),
          "available_disk_bytes": shutil.disk_usage(root).free,
          "memory": mem, "source_commit": git(root, "rev-parse", "HEAD"),
          "branch": git(root, "branch", "--show-current"), "dependencies": states,
          "detected_process_names": sorted(processes), "models": models,
          "device_qualified": False}


if __name__ == "__main__":
  if len(sys.argv) != 2:
    raise SystemExit("Usage: python3 xnor_v0112_device_inventory.py /data/openpilot")
  root = Path(sys.argv[1])
  if not root.is_dir():
    raise SystemExit("Checkout does not exist")
  print(json.dumps(report(root), indent=2, sort_keys=True))
