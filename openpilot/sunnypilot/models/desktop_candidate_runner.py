"""Isolated desktop runner for a locally compiled, pinned native candidate.

This module is not used by modeld. It has no publishers, Params, CAN, vehicle
state or actuator path. Raw action values are inspected, never interpreted.
Only use with locally generated, trusted compiler output.
"""
from __future__ import annotations

import os
from pathlib import Path
import time


def inspect_compiled_candidate(path: Path, expected_abi: dict) -> dict:
  if Path("/AGNOS").exists():
    raise RuntimeError("Candidate bench is desktop-only")
  if os.environ.get("DEV") != "CPU:LLVM":
    raise RuntimeError("Set DEV=CPU:LLVM before importing Tinygrad")
  import numpy as np
  from tinygrad.tensor import Tensor
  from tinygrad.device import Device
  from openpilot.selfdrive.modeld.helpers import load_oob
  from openpilot.selfdrive.modeld.compile_modeld import make_input_queues, WARP_INPUTS, POLICY_INPUTS
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  from openpilot.sunnypilot.models.native_abi import describe_metadata, compare_native_abi

  with path.open("rb") as stream:
    bundle = load_oob(stream)
  meta = bundle["metadata"]
  assert not compare_native_abi(expected_abi, describe_metadata(meta))
  assert Device.DEFAULT.startswith("CPU"), Device.DEFAULT
  shapes = meta["input_shapes"]
  slices = meta["output_slices"]
  assert slices["action"].stop - slices["action"].start == 4
  assert shapes["features_buffer"] == (1, 32, 512)
  assert shapes["desire_pulse"] == (1, 33, 8)

  results = []
  for width, height in ((1928, 1208), (1344, 760)):
    queues, npy = make_input_queues(shapes, 4, device=Device.DEFAULT)
    warp = bundle[(width, height)]
    run = bundle["run_policy"]
    nv12_size = get_nv12_info(width, height)[3]
    rng = np.random.default_rng(42)
    durations = []
    previous_features = None
    feature_changes = 0
    for _i in range(12):
      # Distinct synthetic frames, not actual driving or camera capture.
      frame = Tensor(rng.integers(0, 256, nv12_size, dtype=np.uint8), device=Device.DEFAULT).realize()
      wide = Tensor(rng.integers(0, 256, nv12_size, dtype=np.uint8), device=Device.DEFAULT).realize()
      npy["tfm"][:] = np.eye(3, dtype=np.float32)
      npy["big_tfm"][:] = np.eye(3, dtype=np.float32)
      npy["traffic_convention"][:] = [1., 0.]
      npy["action_t"][:] = [.2, .3]
      npy["desire"][:] = 0.
      start = time.perf_counter()
      warped = warp(**{k: queues[k] for k in WARP_INPUTS}, frame=frame, big_frame=wide)
      out, = run(**{k: queues[k] for k in POLICY_INPUTS}, warped=warped)
      raw = out.numpy()[0]
      durations.append((time.perf_counter() - start) * 1000.)
      assert raw.shape == (2580,) and np.all(np.isfinite(raw))
      features = raw[slices["hidden_state"]].copy()
      if previous_features is not None and not np.array_equal(previous_features, features):
        feature_changes += 1
      previous_features = features
      npy["prev_feat"][:] = features
      parsed = Parser().parse_outputs({k: raw[np.newaxis, s].copy() for k, s in slices.items()})
      assert parsed["plan"].shape == (1, 33, 15)
      assert parsed["lead"].shape == (1, 3, 6, 4)
      assert all(np.all(np.isfinite(v)) for v in parsed.values())
    assert feature_changes > 0, "Recurrent features did not change"
    results.append({"camera": [width, height], "frames": len(durations),
                    "feature_changes": feature_changes,
                    "inference_ms": durations, "raw_action_width": 4})
  return {"camera_results": results, "synthetic_only": True,
          "action_semantics_verified": False, "vehicle_qualified": False}
