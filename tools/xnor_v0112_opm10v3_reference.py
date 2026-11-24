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
import tempfile

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
  replay = p.add_mutually_exclusive_group()
  replay.add_argument("--replay-npz", type=Path)
  replay.add_argument("--synthetic-nv12", action="store_true")
  p.add_argument("--predictions", type=Path, default=Path("opm10v3-paired-replay.json"))
  args = p.parse_args()
  root = args.source_checkout.resolve()
  report = {"model": "OPM10V3", "ref": REF, "synthetic_only": args.replay_npz is None,
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
    replay_arrays = None
    camera_history = None
    if args.replay_npz is not None or args.synthetic_nv12:
      from xnor_v0112_opm10v3_camera import CameraHistory, self_test
      from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
      self_test()
      loader_path = Path(__file__).resolve().parents[1] / "openpilot/sunnypilot/models/desktop_paired_replay.py"
      loader_spec = importlib.util.spec_from_file_location("paired_replay_loader", loader_path)
      loader = importlib.util.module_from_spec(loader_spec)
      loader_spec.loader.exec_module(loader)
      if args.synthetic_nv12:
        with tempfile.TemporaryDirectory(prefix="opm10v3-nv12-") as temp:
          fixture = Path(temp) / "fixture.npz"
          loader.make_synthetic_replay(fixture)
          camera, replay_arrays = loader.load_replay(fixture)
      else:
        camera, replay_arrays = loader.load_replay(args.replay_npz)
      camera_history = CameraHistory(camera, get_nv12_info(*camera))
      report["camera_size"] = camera
      report["camera_warp_reference"] = "NumPy, upstream nearest-neighbor clip and five-frame queue"
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
    predictions = []
    prev_desire = np.zeros(8, dtype=np.float32)
    dtype_map = {"tensor(uint8)": np.uint8, "tensor(float)": np.float32, "tensor(float16)": np.float16}
    for i in range(len(replay_arrays["road"]) if replay_arrays is not None else 12):
      start = time.perf_counter()
      vision_input = {}
      prepared = camera_history.update(replay_arrays["road"][i], replay_arrays["wide"][i],
                                       replay_arrays["road_tfm"][i], replay_arrays["wide_tfm"][i]) if camera_history else None
      for item in sessions["vision"].get_inputs():
        dtype = dtype_map[item.type]
        image = prepared[item.name] if prepared is not None else rng.integers(0, 256, size=shapes["vision"][item.name], dtype=np.uint8)
        assert image.shape == shapes["vision"][item.name]
        vision_input[item.name] = image.astype(dtype, copy=False)
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
      if replay_arrays is not None:
        desire = replay_arrays["desire_pulse"][i].copy()
        desire[0] = 0
        desires[0, -1] = np.where(desire - prev_desire > .99, desire, 0)
        prev_desire[:] = desire
        convention = replay_arrays["traffic_convention"][i][None, :]
      else:
        if i == 3:
          desires[0, -1, 1] = 1
        convention = np.array([[1, 0]], dtype=np.float32)
      policy_input = {"features_buffer": history[:, 3::4].copy(),
                      "desire_pulse": desires.reshape(1, -1, 4, 8).max(axis=2),
                      "traffic_convention": convention}
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
      if replay_arrays is not None:
        predictions.append({"frame": i, "plan": merged["plan"][0].tolist(),
                            "lead": merged["lead"][0].tolist(),
                            "lead_prob": merged["lead_prob"][0].tolist(), "raw_action": None})
    if args.replay_npz is None:
      assert changes > 0
    if replay_arrays is not None:
      args.predictions.write_text(json.dumps({"camera_size": camera, "frames": len(predictions),
        "opm10v3": predictions, "synthetic_only": args.synthetic_nv12,
        "vehicle_qualified": False}, indent=2) + "\n")
      report["paired_replay_frames"] = len(predictions)
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
