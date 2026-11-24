#!/usr/bin/env python3
"""Assemble the complete Tesla Hybrid *development* opendbc/Panda port in a disposable checkout.

This command is deliberately not an installer. It never edits the original
openpilot tree, native gitlinks, Panda flash, /data/openpilot, or /AGNOS.
It requires the XNOR 0.11.2 candidate source branch and its exact submodule
pins. The exported patches are applied to scratch Git clones, then a manifest
is written. It can optionally run host-only tests but never sends CAN.

Example on a development workstation (NOT a comma device):
  git submodule update --init panda opendbc_repo
  python3 tools/xnor_v0112_assemble_hybrid_candidate.py \
    --output-dir /tmp/xnor-v0112-hybrid-assembled

Building source is not proof that steering, radar, AEB or LS2 braking is safe.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from xnor_v0112_full_hybrid_panda_preflight import ROOT, SOURCES, audit
from xnor_v0112_export_full_hybrid_panda_patch import export, PATCH_NAMES

PINS = {
  "opendbc_repo": "3f86206a13c3181f0224e8f1424b836592e2b2d2",
  "panda": "fa83a40ad799c827ab9cbda4560cf3426531ceff",
}


def run(*args: str, cwd: Path | None = None, env: dict | None = None) -> str:
  p = subprocess.run(args, cwd=cwd, env=env, check=True, text=True,
                     stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  return p.stdout.strip()


def prohibit_live_install(destination: Path) -> None:
  if Path("/AGNOS").exists():
    raise RuntimeError("Refusing to assemble on AGNOS/comma hardware")
  target = destination.resolve()
  src = ROOT.resolve()
  if target == src or target.is_relative_to(src) or src.is_relative_to(target):
    raise RuntimeError("Output must be in a different directory, not the active openpilot tree")
  for blocked in ("/data", "/persist", "/dev", "/sys", "/proc"):
    root = Path(blocked)
    if target == root or target.is_relative_to(root):
      raise RuntimeError(f"Refusing to write into device/system path {blocked}")
  if destination.exists():
    raise FileExistsError(f"Refusing to replace an existing candidate: {destination}")
  if not destination.parent.is_dir():
    raise FileNotFoundError(f"Output parent does not exist: {destination.parent}")


def assemble(destination: Path, *, run_host_tests: bool = False) -> dict:
  prohibit_live_install(destination)
  report = audit()
  native_head = run("git", "rev-parse", "HEAD", cwd=ROOT)
  native_branch = run("git", "branch", "--show-current", cwd=ROOT)
  required = "upgrade/xnor-v0112-xnor-base"
  ci_base = os.getenv("GITHUB_BASE_REF", "")
  ci_ref = os.getenv("GITHUB_REF_NAME", "")
  ci_verification = (os.getenv("GITHUB_ACTIONS") == "true" and
                     os.getenv("GITHUB_REPOSITORY") == "gavrackerss/openpilot" and
                     (ci_base == required or ci_ref in (
                       required, "upgrade/xnor-v0112-hybrid-build-verification")))
  if native_branch != required and not ci_verification:
    raise RuntimeError("Assemble only the XNOR 0.11.2 staging branch or its CI verification PR")

  # Source must really be a checkout of the exact XNOR dependency commit,
  # not an accidental checkout with an uncommitted or unrelated Panda port.
  for component, expected in PINS.items():
    sub = ROOT / component
    if not (sub / ".git").exists():
      raise RuntimeError(f"Initialize pinned submodule first: {component}")
    actual = run("git", "rev-parse", "HEAD", cwd=sub)
    if actual != expected:
      raise RuntimeError(f"Wrong {component} source {actual} (expected {expected})")
    if run("git", "status", "--porcelain", cwd=sub):
      raise RuntimeError(f"Original submodule has local changes: {component}")

  scratch = Path(tempfile.mkdtemp(
    prefix="xnor-v0112-hybrid-build-", dir=destination.parent))
  try:
    out = scratch / "openpilot"
    run("git", "clone", "--no-hardlinks", "--quiet", "--no-checkout",
        str(ROOT), str(out))
    # The source CI checkout may be a detached PR merge commit. Never rely on
    # the local clone's default-branch heuristic to select the correct tree.
    run("git", "checkout", "--detach", native_head, cwd=out)
    if run("git", "rev-parse", "HEAD", cwd=out) != native_head:
      raise RuntimeError("Scratch openpilot clone does not match original commit")

    patches = scratch / "patches"
    exported = export(patches)  # read-only, git apply --check in ORIGINAL submodules
    for sub, component in (("opendbc_repo", "opendbc"), ("panda", "panda")):
      target = out / sub
      # A gitlink checkout contains an empty directory; clone exact local
      # submodule source into it instead of recursively fetching a moving ref.
      if target.exists():
        if any(target.iterdir()):
          raise RuntimeError(f"Scratch gitlink unexpectedly populated: {target}")
        target.rmdir()
      run("git", "clone", "--no-hardlinks", "--quiet",
          str(SOURCES[component]), str(target))
      if run("git", "rev-parse", "HEAD", cwd=target) != PINS[sub]:
        raise RuntimeError(f"Scratch {sub} source drift")
      patch = exported[component]
      run("git", "apply", "--check", str(patch), cwd=target)
      run("git", "apply", str(patch), cwd=target)
      if not run("git", "status", "--porcelain", cwd=target):
        raise RuntimeError(f"Candidate source patch unexpectedly empty: {sub}")

    if run_host_tests:
      # Execute only isolated source and safety tests. Full openpilot scons
      # and complete vehicle simulation remain separately qualified gates.
      run(sys.executable,
          "tests/test_xnor_v0112_full_hybrid_panda.py", "-v", cwd=out)
      run(sys.executable,
          "tests/test_xnor_v0112_tesla_vehicle_contract.py", "-v", cwd=out)
      run("cc", "-Wall", "-Wextra", "-Werror", "-std=gnu11", "-DALLOW_DEBUG",
          "-I", str(out / "opendbc_repo"),
          str(out / "migration/tesla_v0112/hybrid_panda_candidate/host_tests/test_hybrid_panda.c"),
          "-o", str(scratch / "tesla-hybrid-host-smoke"), "-lm", cwd=out)
      run(str(scratch / "tesla-hybrid-host-smoke"))

    manifest = {
      "source_branch": native_branch,
      "source_commit": native_head,
      "panda_pin": PINS["panda"],
      "opendbc_pin": PINS["opendbc_repo"],
      "patches": {key: path.name for key, path in exported.items()},
      "panda_and_opendbc_patched_in_scratch": True,
      "source_audit": report,
      "host_source_and_c_smoke_requested": run_host_tests,
      "device_flash_performed": False,
      "vehicle_authority_qualified": False,
      "onroad_install_supported": False,
      "note": "Scratch source checkout for compilation and independent review only.",
    }
    (out / "XNOR_HYBRID_ASSEMBLY_MANIFEST.json").write_text(
      json.dumps(manifest, sort_keys=True, indent=2) + "\n", encoding="utf-8")
    # Keep the exact review diffs beside the assembled checkout.
    shutil.copytree(patches, out / "_XNOR_HYBRID_REVIEW_PATCHES")
    os.replace(out, destination)
    return manifest
  finally:
    shutil.rmtree(scratch, ignore_errors=True)


def main(argv: list[str] | None = None) -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--output-dir", required=True, type=Path)
  parser.add_argument("--run-host-tests", action="store_true",
                      help="Run source tests and compiled host-C smoke in scratch source")
  opts = parser.parse_args(argv)
  manifest = assemble(opts.output_dir, run_host_tests=opts.run_host_tests)
  print(json.dumps(manifest, indent=2, sort_keys=True))
  print("Scratch candidate assembled. Never flash/install without safety validation.")


if __name__ == "__main__":
  main()
