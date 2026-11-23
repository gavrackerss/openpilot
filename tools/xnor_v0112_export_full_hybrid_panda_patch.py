#!/usr/bin/env python3
"""Export XNOR 0.11.2 full Tesla Hybrid Panda as two reviewable git patches.

Reads only known pinned opendbc and panda submodules, verifies exact source
Git blob hashes, and exports diffs from the complete staged replacement files.

This does NOT run git apply, change a submodule pointer, build firmware,
flash Panda, enable sendcan or make the car drive. 'git apply --check' is
automatically run as a read-only applicability test.

Run ONLY in a disposable workstation checkout, never on the live comma:
  git submodule update --init opendbc_repo panda
  python3 tools/xnor_v0112_export_full_hybrid_panda_patch.py \
    --output-dir /tmp/xnor-v0112-hybrid-review

Then copy each patch to a *writable test clone* of the matching dependency,
apply there, compile and run both Panda safety and vehicle replay tests.
"""
from __future__ import annotations

import argparse
import difflib
from pathlib import Path
import subprocess

from xnor_v0112_full_hybrid_panda_preflight import BASE, candidate_path, ROOT, SOURCES, NATIVE_BLOBS, NEW_TESTS, NEW_VEHICLE_FILES, audit

PATCH_FILES = {
  "opendbc": tuple(NATIVE_BLOBS["opendbc"]) + tuple(NEW_TESTS) + tuple(NEW_VEHICLE_FILES),
  "panda": tuple(NATIVE_BLOBS["panda"]),
}
PATCH_NAMES = {
  "opendbc": "xnor-v0112-tesla-hybrid-opendbc.patch",
  "panda": "xnor-v0112-tesla-hybrid-panda.patch",
}


def patch_for(component: str) -> str:
  if component not in PATCH_FILES:
    raise ValueError("Unexpected dependency requested")
  chunks: list[str] = []
  for rel in PATCH_FILES[component]:
    a = SOURCES[component] / rel
    b = candidate_path(component, rel)
    is_new = rel in (*NEW_TESTS, *NEW_VEHICLE_FILES)
    old = [] if is_new else a.read_text(encoding="utf-8").splitlines(keepends=True)
    new = b.read_text(encoding="utf-8").splitlines(keepends=True)
    diff = list(difflib.unified_diff(
      old, new, fromfile="/dev/null" if is_new else f"a/{rel}",
      tofile=f"b/{rel}", n=3,
    ))
    if not diff:
      raise RuntimeError(f"Expected staged change missing: {component}/{rel}")
    chunks.append(f"diff --git a/{rel} b/{rel}\n")
    if is_new:
      chunks.append("new file mode 100644\n")
    chunks.extend(diff)
  return "".join(chunks)


def export(output_dir: Path) -> dict[str, Path]:
  audit()  # source identity, safety invariants, all required new C APIs
  output = output_dir.resolve()
  for src in (SOURCES.values()):
    if output == src.resolve() or output.is_relative_to(src.resolve()):
      raise ValueError("Refusing to write inside active Panda/opendbc gitlinks")
  if output == ROOT or output.is_relative_to(ROOT):
    raise ValueError("Use an external scratch output directory, not the active repo")
  for name in PATCH_NAMES.values():
    if (output / name).exists():
      raise FileExistsError(f"Refusing to overwrite existing patch: {output / name}")

  patches = {component: patch_for(component) for component in PATCH_FILES}
  output.mkdir(parents=True, exist_ok=True)
  result = {}
  for component, data in patches.items():
    file = output / PATCH_NAMES[component]
    file.write_text(data, encoding="utf-8")
    result[component] = file
  for component, file in result.items():
    checked = subprocess.run(
      ["git", "apply", "--check", str(file)],
      cwd=SOURCES[component], capture_output=True, text=True, check=False,
    )
    if checked.returncode != 0:
      raise RuntimeError(f"{component} patch does not apply to pinned native source: {checked.stderr}")
  return result


def main(argv=None) -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--output-dir", type=Path, required=True)
  opts = parser.parse_args(argv)
  for component, path in export(opts.output_dir).items():
    print(f"{component} review patch: {path} (git apply --check PASSED)")
  print("Firmware build/flash: NOT performed. Tesla Hybrid authority: NOT qualified.")


if __name__ == "__main__":
  main()
