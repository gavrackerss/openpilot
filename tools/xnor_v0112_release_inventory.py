#!/usr/bin/env python3
"""Read-only release inventory. Never builds, installs, merges or flashes.

Reports known packaging/feature blockers without claiming vehicle qualification.
A successful inventory means the report was generated, not release approval.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
BASE_PINS = {
  "opendbc_repo": "3f86206a13c3181f0224e8f1424b836592e2b2d2",
  "panda": "fa83a40ad799c827ab9cbda4560cf3426531ceff",
}


def git(*args):
  return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def inventory():
  head = git("rev-parse", "HEAD")
  dependencies = {}
  blockers = []
  for line in git("ls-tree", "HEAD").splitlines():
    meta, name = line.split("\t", 1)
    mode, kind, sha = meta.split()
    if mode == "160000" and kind == "commit":
      dependencies[name] = sha
  for name, stock in BASE_PINS.items():
    if dependencies.get(name) == stock:
      blockers.append(f"{name}: original native pin; reviewed Hybrid changes are scratch-only")
    elif not dependencies.get(name):
      blockers.append(f"{name}: missing gitlink")
    else:
      blockers.append(f"{name}: changed pin requires independent dependency review")
  evidence = ROOT / "XNOR_0112_RELEASE_READINESS.md"
  modeld = ROOT / "openpilot/selfdrive/modeld/modeld.py"
  hud = ROOT / "openpilot/selfdrive/ui/onroad/hud_renderer.py"
  if "XNORRequestedModelRef" not in modeld.read_text():
    blockers.append("Native model execution is not connected to requested-model preference")
  if "manual_speed_limit" not in hud.read_text():
    blockers.append("V279 live speed controls are not connected to HUD")
  blockers.extend([
    "Exact-head build and test evidence has not been independently reviewed",
    "Real vehicle replay and independent safety qualification are outstanding",
    "LS2 longitudinal behavior remains unqualified",
  ])
  return {
    "source_commit": head,
    "gitlinks": dependencies,
    "readiness_document_sha256": hashlib.sha256(evidence.read_bytes()).hexdigest(),
    "blockers": blockers,
    "production_promotion_approved": False,
    "vehicle_qualified": False,
  }


if __name__ == "__main__":
  print(json.dumps(inventory(), indent=2, sort_keys=True))
