#!/usr/bin/env python3
"""Export only passive, additive Tesla opendbc source changes as a patch.

Never modifies the active opendbc_repo submodule or Panda firmware.
Safety.h is copied unchanged for comparison, and explicitly excluded
from the exported patch. The patch is for a future writable opendbc fork,
NOT for flashing or activating vehicle controls.

Usage:
  git submodule update --init opendbc_repo
  python3 tools/xnor_v0112_export_tesla_opendbc_patch.py --output /tmp/tesla-native-passive.patch

For review only:
  cd opendbc_repo && git apply --check /tmp/tesla-native-passive.patch
"""
from __future__ import annotations

import argparse
import difflib
from pathlib import Path

from xnor_v0112_tesla_panda_preflight import PINNED, OVERLAY, audit

ROOT = Path(__file__).resolve().parents[1]
PATCH_FILES = ("car/tesla/values.py", "dbc/tesla_can.dbc")


def build_patch() -> str:
  report = audit()
  if report["ready_to_flash_panda"] or report["ready_to_enable_tesla_longitudinal"]:
    raise RuntimeError("Passive export must never qualify vehicle control")
  parts = []
  for rel in PATCH_FILES:
    original = (PINNED / rel).read_text(encoding="utf-8").splitlines(keepends=True)
    staged = (OVERLAY / rel).read_text(encoding="utf-8").splitlines(keepends=True)
    diff = list(difflib.unified_diff(
      original, staged, fromfile=f"a/opendbc/{rel}",
      tofile=f"b/opendbc/{rel}", n=3,
    ))
    if not diff:
      raise RuntimeError(f"No staged changes for expected overlay {rel}")
    parts.extend(diff)
  patch = "".join(parts)
  headers = [line for line in patch.splitlines() if line.startswith(("--- ", "+++ "))]
  expected = [
    "--- a/opendbc/car/tesla/values.py", "+++ b/opendbc/car/tesla/values.py",
    "--- a/opendbc/dbc/tesla_can.dbc", "+++ b/opendbc/dbc/tesla_can.dbc",
  ]
  if headers != expected or "safety/modes/" in patch or "OP_STALK_ENABLE" in patch or "IGNORE_STOCK_AEB" in patch:
    raise RuntimeError("Patch scope violation: only passive values and telemetry DBC allowed")
  if "BO_ 937 DAS_telemetry: 8 XXX" not in patch or "class CruiseButtons:" not in patch:
    raise RuntimeError("Expected additive Tesla changes absent")
  return patch


def main(argv=None):
  parser = argparse.ArgumentParser()
  parser.add_argument("--output", type=Path, required=True,
                      help="Destination for passive review patch; never inside a git submodule")
  opts = parser.parse_args(argv)
  target = opts.output.resolve()
  if target.is_relative_to((ROOT / "opendbc_repo").resolve()) or target.is_relative_to((ROOT / "panda").resolve()):
    parser.error("Refusing to write inside active Panda/opendbc submodules")
  if target.exists():
    parser.error(f"Refusing to overwrite existing patch: {target}")
  text = build_patch()
  target.parent.mkdir(parents=True, exist_ok=True)
  target.write_text(text, encoding="utf-8")
  print(f"Passive review-only opendbc patch: {target}")
  print("Does not modify Panda safety, CAN transmitters or 0x659 ownership.")


if __name__ == "__main__":
  main()
