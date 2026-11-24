#!/usr/bin/env python3
"""Inspect pinned real SunnyPilot model artifacts in disposable desktop CI.

Downloads only the explicitly pinned OPM10V3 and TSFDOM chunks, verifies all
digests, and examines the framing without unpickling or running either model.
No device state, Params, manager, CAN or native modeld changes.
"""
from __future__ import annotations

from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from urllib.parse import urlsplit
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from openpilot.sunnypilot.models.native_envelope import inspect_envelope

PINS = ROOT / "migration/sunnypilot_legacy/v23_candidate_pins.json"
MAX_CHUNK = 128 * 1024 * 1024
MAX_BUNDLE = 512 * 1024 * 1024


def inspect_candidate(bundle):
  assert bundle["is_big"] is False and bundle["runner"] == "tinygrad"
  artifact = bundle["models"][0]["artifact"]
  base = artifact["download_uri"]["url"]
  assert urlsplit(base).scheme == "https" and urlsplit(base).hostname == "huggingface.co"
  chunks = artifact["chunks"]
  total = 0
  full_hash = hashlib.sha256()
  with tempfile.TemporaryFile() as out:
    for i, chunk in enumerate(chunks):
      suffix = f".chunk{i+1:02d}of{len(chunks):02d}"
      assert chunk["file_name"] == artifact["file_name"] + suffix
      digest = hashlib.sha256()
      length = 0
      with urlopen(base + suffix, timeout=90) as stream:
        while data := stream.read(1024 * 1024):
          length += len(data)
          total += len(data)
          if length > MAX_CHUNK or total > MAX_BUNDLE:
            raise ValueError("Pinned artifact exceeds inspection budget")
          digest.update(data)
          full_hash.update(data)
          out.write(data)
      if digest.hexdigest() != chunk["sha256"]:
        raise ValueError(f"{chunk['file_name']}: checksum mismatch")
    if full_hash.hexdigest() != artifact["download_uri"]["sha256"]:
      raise ValueError("Combined artifact checksum mismatch")
    out.seek(0)
    prefix = out.read(16)
    out.seek(0)
    report = {"model": bundle["short_name"], "ref": bundle["ref"],
              "verified_bytes": total, "verified_chunks": len(chunks),
              "native_abi_verified": False, "custom_model_authority": False}
    try:
      report["native_envelope"] = asdict(inspect_envelope(out))
      report["format"] = "native-envelope-framing-only"
    except (ValueError, OverflowError) as exc:
      report["format"] = "plain-pickle" if prefix.startswith(b"\x80") else "other"
      report["native_envelope_issue"] = str(exc)
    return report


def main():
  pins = json.loads(PINS.read_text())
  results = [inspect_candidate(b) for b in pins["bundles"]]
  report = {"catalog_blob_sha": pins["catalog_blob_sha"],
            "catalog_tinygrad_ref": pins["tinygrad_ref"], "candidates": results}
  Path("sunnypilot-candidate-report.json").write_text(json.dumps(report, indent=2) + "\n")
  print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()
