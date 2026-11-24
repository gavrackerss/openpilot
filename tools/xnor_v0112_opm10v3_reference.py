#!/usr/bin/env python3
"""Pinned OPM10V3 split-model reference inference on a desktop only.

Uses exact upstream ONNX graphs and their original parser. This is not an
AGNOS model runner, and never calls modeld.main or publishes any predictions.
"""
from __future__ import annotations

import argparse
import base64
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import pickle
import subprocess
import sys
import time

REF = "7d4c295c27bc3f72727c49ec73b98e45745b588d"
FILES = {
  "vision": ("selfdrive/modeld/models/driving_vision.onnx", "ea8d73b6531339f9741a929c55e9a623b0d69f1e"),
  "on_policy": ("selfdrive/modeld/models/driving_on_policy.onnx", "f9e060692fbad8397e426ad97827e86daa62dc7b"),
  "off_policy": ("selfdrive/modeld/models/driving_off_policy.onnx", "1abfb62ae7e6ce163f5074e04a9e4ce70b0e9862"),
}
PARSER = "selfdrive/modeld/parse_model_outputs.py"
PARSER_BLOB = "802f0ad85998b09405d563e03bd8465e0dd6085c"


def git(root, *args):
  return subprocess.check_output(["git", "-C", str(root), *args])


def verify_file(root, rel, blob):
  path = root / rel
  if git(root, "rev-parse", f"HEAD:{rel}").decode().strip() != blob:
    raise ValueError(f"Unexpected pinned source: {rel}")
  original = git(root, "show", f"HEAD:{rel}")
  digest = hashlib.sha256()
  if original.startswith(b"version https://git-lfs.github.com/spec/v1"):
    expected = next(line.split(b":", 1)[1].decode() for line in original.splitlines() if line.startswith(b"oid sha256:"))
    with path.open("rb") as f:
      while data := f.read(1024 * 1024):
        digest.update(data)
    if digest.hexdigest() != expected:
      raise ValueError(f"LFS payload mismatch: {rel}")
    return expected
  if git(root, "hash-object", str(path)).decode().strip() != blob:
    raise ValueError(f"Working-tree source mismatch: {rel}")
  return hashlib.sha256(path.read_bytes()).hexdigest()


class SliceUnpickler(pickle.Unpickler):
  def find_class(self, module, name):
    if module == "builtins" and name == "slice":
      return slice
    raise pickle.UnpicklingError("Unexpected metadata global")


def output_slices(session):
  raw = session.get_modelmeta().custom_metadata_map["output_slices"]
  slices = SliceUnpickler(io.BytesIO(base64.b64decode(raw))).load()
  if not isinstance(slices, dict) or not all(isinstance(k, str) and isinstance(v, slice) for k, v in slices.items()):
    raise ValueError("Invalid output slice metadata")
  return slices


def input_shapes(session):
  shapes = {}
  for item in session.get_inputs():
    if any(not isinstance(d, int) or d <= 0 for d in item.shape):
      raise ValueError(f"Unsupported dynamic input: {item.name}")
    shapes[item.name] = tuple(item.shape)
  return shapes


def run_session(session, inputs, slices):
  import numpy as np
  out, = session.run(None, inputs)
  if out.ndim != 2 or out.shape[0] != 1 or not np.all(np.isfinite(out)):
    raise ValueError("Invalid raw model output")
  return {name: out[:, sl].astype(np.float32, copy=True) for name, sl in slices.items()}, out.shape


def main():
  if Path("/AGNOS").exists() or Path("/data/openpilot").exists():
    raise RuntimeError("Desktop only")
  p = argparse.ArgumentParser()
  p.add_argument("--source-checkout", type=Path, required=True)
  p.add_argument("--report", type=Path, default=Path("opm10v3-reference.json"))
  args = p.parse_args()
  root = args.source_checkout.resolve()
  report = {"model": "OPM10V3", "ref": REF, "synthetic_only": True,
            "runtime": "ONNX Runtime CPU reference", "vehicle_qualified": False,
            "inference_passed": False}
  try:
    assert git(root, "rev-parse", "HEAD").decode().strip() == REF
    import numpy as np
    import onnxruntime as ort
    report["source_sha256"] = {k: verify_file(root, *v) for k, v in FILES.items()}
    verify_file(root, PARSER, PARSER_BLOB)
    # Use the original upstream parser and constants, not the new supercombo parser.
    sys.path.insert(0, str(root))
    from openpilot.selfdrive.modeld.parse_model_outputs import Parser
    assert Path(sys.modules[Parser.__module__].__file__).resolve() == (root / PARSER).resolve()
    parser = Parser()
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = 2
    opts.inter_op_num_threads = 1
    sessions = {k: ort.InferenceSession(str(root / v[0]), sess_options=opts, providers=["CPUExecutionProvider"])
                for k, v in FILES.items()}
    shapes = {k: input_shapes(s) for k, s in sessions.items()}
    slices = {k: output_slices(s) for k, s in sessions.items()}
    report["input_shapes"] = shapes
    report["output_names"] = {k: sorted(v) for k, v in slices.items()}
    assert set(shapes["vision"]) == {"img", "big_img"}
    assert shapes["on_policy"] == shapes["off_policy"]
    policy = shapes["on_policy"]
    assert set(policy) == {"features_buffer", "desire_pulse", "traffic_convention"}
    assert policy["features_buffer"][0] == 1 and policy["features_buffer"][2] == 512
    assert policy["desire_pulse"][0] == 1 and policy["desire_pulse"][2] == 8
    assert policy["traffic_convention"] == (1, 2)
    # Preserve the original 20Hz -> 5Hz recurrent sampling contract.
    history = np.zeros((1, policy["features_buffer"][1] * 4, 512), dtype=np.float32)
    desires = np.zeros((1, policy["desire_pulse"][1] * 4, 8), dtype=np.float32)
    previous = None
    changes = 0
    rng = np.random.default_rng(20261124)
    frames = []
    dtype_map = {"tensor(uint8)": np.uint8, "tensor(float)": np.float32, "tensor(float16)": np.float16}
    for i in range(12):
      start = time.perf_counter()
      vision_input = {}
      for item in sessions["vision"].get_inputs():
        dtype = dtype_map[item.type]
        vision_input[item.name] = rng.integers(0, 256, size=shapes["vision"][item.name], dtype=np.uint8).astype(dtype)
      v, v_shape = run_session(sessions["vision"], vision_input, slices["vision"])
      v = parser.parse_vision_outputs(v)
      features = v["hidden_state"].reshape(1, 1, 512)
      if previous is not None and not np.array_equal(previous, features):
        changes += 1
      previous = features.copy()
      history[:, :-1] = history[:, 1:].copy()
      history[:, -1:] = features
      desires[:, :-1] = desires[:, 1:].copy()
      desires[:, -1:] = 0
      if i == 3:
        desires[0, -1, 1] = 1
      policy_input = {"features_buffer": history[:, 3::4].copy(),
                      "desire_pulse": desires.reshape(1, -1, 4, 8).max(axis=2),
                      "traffic_convention": np.array([[1, 0]], dtype=np.float32)}
      results = {}
      for head, parse in (("on_policy", parser.parse_policy_outputs), ("off_policy", parser.parse_off_policy_outputs)):
        cast = {x.name: policy_input[x.name].astype(dtype_map[x.type], copy=False) for x in sessions[head].get_inputs()}
        outs, raw_shape = run_session(sessions[head], cast, slices[head])
        results[head] = parse(outs)
      off = results["off_policy"]
      on = results["on_policy"]
      off.pop("plan")
      merged = {**v, **off, **on}
      if "planplus" in merged:
        merged["plan"] = merged["plan"] + merged["planplus"]
      assert merged["plan"].shape == (1, 33, 15)
      assert merged["lead"].shape == (1, 3, 6, 4)
      assert all(np.all(np.isfinite(a)) for a in merged.values())
      frames.append({"frame": i, "inference_ms": (time.perf_counter() - start) * 1000.})
    assert changes > 0
    report.update(inference_passed=True, frames=frames, feature_changes=changes,
                  action_semantics_verified=False, camera_warp_verified=False)
  except Exception as exc:
    report["error"] = repr(exc)
    raise
  finally:
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()
