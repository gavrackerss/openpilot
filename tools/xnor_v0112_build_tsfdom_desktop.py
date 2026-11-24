#!/usr/bin/env python3
"""Desktop-only comparison and compile of exact upstream TSFDOM ONNX source.

Uses trusted, pinned commaai source, not downloaded SunnyPilot pickles.
Builds into an isolated temporary directory. Never replaces the native stock
model, starts modeld, publishes model outputs, or changes vehicle authority.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from openpilot.common.file_chunker import open_file_chunked
from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
from openpilot.sunnypilot.models.native_abi import describe_metadata, compare_native_abi
from openpilot.sunnypilot.models.native_envelope import inspect_envelope

ROOT = Path(__file__).resolve().parents[1]
STOCK = ROOT / "openpilot/selfdrive/modeld/models/driving_supercombo.onnx"
COMPILER = ROOT / "openpilot/selfdrive/modeld/compile_modeld.py"
CANDIDATE_REF = "0394d7284cfc229f772ee4fa73ff724ba8245110"
CANDIDATE_PATH = "openpilot/selfdrive/modeld/models/driving_supercombo.onnx"
CANDIDATE_BLOB = "0a48017d3d0d0e08649df44b78b3b7435f82ac76"


def git(source, *args):
  return subprocess.check_output(["git", "-C", str(source), *args], text=True).strip()


def copy_and_hash(source, dest):
  digest = hashlib.sha256()
  with open_file_chunked(source) as inp, dest.open("wb") as out:
    while data := inp.read(1024 * 1024):
      digest.update(data)
      out.write(data)
  return digest.hexdigest()


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--source-checkout", type=Path, required=True)
  parser.add_argument("--report", type=Path, default=Path("tsfdom-native-build.json"))
  args = parser.parse_args()
  source = args.source_checkout.resolve()
  assert git(source, "rev-parse", "HEAD") == CANDIDATE_REF
  assert git(source, "rev-parse", f"HEAD:{CANDIDATE_PATH}") == CANDIDATE_BLOB
  assert not Path("/AGNOS").exists(), "Desktop CI only"
  report = {"model": "TSFDOM", "upstream_ref": CANDIDATE_REF,
            "upstream_git_blob": CANDIDATE_BLOB, "compiled": False,
            "vehicle_qualified": False}
  try:
    with tempfile.TemporaryDirectory(prefix="xnor-tsfdom-") as work:
      temp = Path(work)
      stock_onnx, candidate_onnx = temp / "stock.onnx", temp / "candidate.onnx"
      report["stock_sha256"] = copy_and_hash(STOCK, stock_onnx)
      report["candidate_sha256"] = copy_and_hash(source / CANDIDATE_PATH, candidate_onnx)
      reference = describe_metadata(make_metadata_dict(str(stock_onnx)))
      candidate = describe_metadata(make_metadata_dict(str(candidate_onnx)))
      issues = compare_native_abi(reference, candidate)
      report.update(stock_abi=reference, candidate_abi=candidate, abi_issues=issues)
      if issues:
        report["outcome"] = "Different native ABI; requires separate model runner"
        return
      target = temp / "candidate.pkl"
      env = dict(os.environ, DEV="CPU:LLVM")
      subprocess.run([sys.executable, str(COMPILER),
                      "--model-size", "512x256",
                      "--camera-resolutions", "1928x1208", "1344x760",
                      "--onnx", str(candidate_onnx),
                      "--output", str(target), "--frame-skip", "4"],
                     cwd=temp, env=env, check=True, timeout=1200)
      with target.open("rb") as stream:
        framing = inspect_envelope(stream)
      report.update(compiled=True, opcode_bytes=framing.opcode_bytes,
                    buffer_count=framing.buffer_count,
                    outcome="Native desktop compiler and its JIT round-trip tests passed")
  except Exception as exc:
    report["error"] = str(exc)
    raise
  finally:
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()
