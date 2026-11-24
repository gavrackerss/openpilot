#!/usr/bin/env python3
"""Verify exact source parity with committed Hybrid dependency review revisions.

Runs only in disposable GitHub Actions scratch clones. Does not update the
openpilot gitlinks, create deployable firmware, or touch a vehicle. The caller
must first apply the existing, reviewed patches to the original native pins.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "migration/tesla_v0112/hybrid_dependency_review_pins.json"


def git(path: Path, *args: str) -> str:
  return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


def verify(component: str, path: Path) -> dict:
  if Path("/AGNOS").exists() or os.environ.get("GITHUB_ACTIONS") != "true":
    raise RuntimeError("GitHub Actions scratch verification only")
  tmp = Path(os.environ["RUNNER_TEMP"]).resolve()
  path = path.resolve()
  if not path.is_relative_to(tmp) or path == tmp or path == ROOT or path.is_relative_to(ROOT):
    raise RuntimeError("Expected a separate runner scratch dependency clone")
  if Path(git(path, "rev-parse", "--show-toplevel")).resolve() != path:
    raise RuntimeError("Expected a standalone dependency repository")
  spec = json.loads(MANIFEST.read_text())["dependencies"][component]
  base = spec["native_base"]
  review = spec["review_commit"]
  if git(path, "rev-parse", "HEAD") != base:
    raise RuntimeError(f"{component}: unexpected original dependency pin")
  git(path, "add", "--all")
  patched_tree = git(path, "write-tree")
  git(path, "fetch", "--no-tags", spec["repository"], review)
  if git(path, "rev-parse", "FETCH_HEAD") != review:
    raise RuntimeError(f"{component}: fetched a different review revision")
  if git(path, "rev-parse", f"{review}^") != base:
    raise RuntimeError(f"{component}: unexpected review parent")
  review_tree = git(path, "rev-parse", f"{review}^{{tree}}")
  if patched_tree != review_tree:
    raise RuntimeError(f"{component}: reviewed Git tree differs from the tested patch tree")
  # Only after whole-tree equality, use the exact immutable review commit in
  # the disposable CI clone. No source changes or production gitlink changes.
  git(path, "reset", "--hard", review)
  if git(path, "status", "--porcelain") or git(path, "rev-parse", "HEAD") != review:
    raise RuntimeError(f"{component}: reviewed scratch checkout is not clean")
  return {"component": component, "native_base": base, "review_commit": review,
          "review_tree": review_tree, "patch_tree_matches": True,
          "vehicle_qualified": False}


if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  parser.add_argument("--panda", required=True, type=Path)
  parser.add_argument("--opendbc", required=True, type=Path)
  parser.add_argument("--report", required=True, type=Path)
  args = parser.parse_args()
  report = {"dependencies": [verify("panda", args.panda),
                            verify("opendbc_repo", args.opendbc)],
            "production_gitlinks_updated": False,
            "vehicle_qualified": False}
  args.report.write_text(json.dumps(report, indent=2) + "\n")
  print(json.dumps(report, indent=2))
