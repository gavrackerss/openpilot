"""Runtime-only queue helpers for SunnyPilot combined Tinygrad PKLs.

These are intentionally separated from the legacy compiler. The downloaded
artifacts are already compiled; the on-car runtime only recreates their input
queues and never recompiles a model while driving.
"""
from __future__ import annotations

import numpy as np
from tinygrad.tensor import Tensor


def detect_desire_key(input_shapes: dict) -> str:
  key = next((k for k in input_shapes if k.startswith("desire")), None)
  if key is None:
    raise ValueError(f"No desire input in {list(input_shapes)}")
  return key


def make_split_input_queues(vision_input_shapes: dict, policy_input_shapes: dict, frame_skip: int, device: str):
  road_key = next((k for k in vision_input_shapes if "img" in k and "big" not in k), None)
  if road_key is None:
    raise ValueError("No road image input in SunnyPilot vision metadata")

  img = vision_input_shapes[road_key]
  n_frames = img[1] // 6
  img_buf_shape = (frame_skip * (n_frames - 1) + 1, 6, img[2], img[3])

  fb = policy_input_shapes["features_buffer"]
  desire_key = detect_desire_key(policy_input_shapes)
  desire = policy_input_shapes[desire_key]
  traffic = policy_input_shapes.get("traffic_convention", (1, 2))

  npy = {
    desire_key: np.zeros(desire[2], dtype=np.float32),
    "traffic_convention": np.zeros(traffic, dtype=np.float32),
    "tfm": np.zeros((3, 3), dtype=np.float32),
    "big_tfm": np.zeros((3, 3), dtype=np.float32),
  }

  handled = {"features_buffer", desire_key, "traffic_convention"}
  for key, shape in policy_input_shapes.items():
    if key not in handled:
      npy[key] = np.zeros(shape, dtype=np.float32)

  queues = {
    "img_q": Tensor(np.zeros(img_buf_shape, dtype=np.uint8), device=device).contiguous().realize(),
    "big_img_q": Tensor(np.zeros(img_buf_shape, dtype=np.uint8), device=device).contiguous().realize(),
    "feat_q": Tensor(np.zeros((frame_skip * (fb[1] - 1) + 1, fb[0], fb[2]), dtype=np.float32),
                     device=device).contiguous().realize(),
    "desire_q": Tensor(np.zeros((frame_skip * desire[1], desire[0], desire[2]), dtype=np.float32),
                       device=device).contiguous().realize(),
    **{k: Tensor(v, device="NPY").realize() for k, v in npy.items()},
  }
  return queues, npy


def derive_frame_skip(vision_input_shapes: dict, policy_input_shapes: dict) -> int:
  fb = policy_input_shapes.get("features_buffer")
  if fb is None:
    return 1
  return 1 if fb[1] >= 99 else 4


def make_supercombo_input_queues(input_shapes: dict, frame_skip: int, device: str):
  img_shape = input_shapes.get("img", input_shapes.get("input_imgs"))
  if img_shape is None:
    raise ValueError("No road image input in SunnyPilot supercombo metadata")

  n_frames = img_shape[1] // 6
  img_buf_shape = (frame_skip * (n_frames - 1) + 1, 6, img_shape[2], img_shape[3])

  npy: dict[str, np.ndarray] = {}
  queue_keys = {}
  desire_key = detect_desire_key(input_shapes)

  for key, shape in input_shapes.items():
    if "img" in key:
      continue
    if len(shape) == 3 and shape[1] > 1:
      if key == desire_key:
        npy[key] = np.zeros(shape[2], dtype=np.float32)
        queue_keys["desire_q"] = Tensor(
          np.zeros((frame_skip * shape[1], shape[0], shape[2]), dtype=np.float32),
          device=device).contiguous().realize()
      elif key == "features_buffer":
        queue_keys["feat_q"] = Tensor(
          np.zeros((frame_skip * (shape[1] - 1) + 1, shape[0], shape[2]), dtype=np.float32),
          device=device).contiguous().realize()
      else:
        npy[key] = np.zeros(shape, dtype=np.float32)
    elif len(shape) == 2:
      npy[key] = np.zeros(shape, dtype=np.float32)

  npy.setdefault("traffic_convention", np.zeros(input_shapes.get("traffic_convention", (1, 2)), dtype=np.float32))
  npy["tfm"] = np.zeros((3, 3), dtype=np.float32)
  npy["big_tfm"] = np.zeros((3, 3), dtype=np.float32)

  queues = {
    "img_q": Tensor(np.zeros(img_buf_shape, dtype=np.uint8), device=device).contiguous().realize(),
    "big_img_q": Tensor(np.zeros(img_buf_shape, dtype=np.uint8), device=device).contiguous().realize(),
    **queue_keys,
    **{k: Tensor(v, device="NPY").realize() for k, v in npy.items()},
  }
  return queues, npy
