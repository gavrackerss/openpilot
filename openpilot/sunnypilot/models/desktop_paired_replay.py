"""Read-only paired desktop replay for locally compiled stock and candidate models.

The input is an explicitly prepared NPZ with paired native NV12 buffers and
recorded model inputs. No camera devices, publishers, Params, actuator commands,
or vehicle control. The outputs are model predictions, not control decisions.
"""
from __future__ import annotations

import os
from pathlib import Path
import time
import zipfile

MAX_FRAMES = 256
MAX_ARCHIVE_BYTES = 2 * 1024**3


def make_synthetic_replay(path: Path):
  """Deterministic fixture for CI, not evidence of driving behavior."""
  import numpy as np
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  camera = (1344, 760)
  nbytes = get_nv12_info(*camera)[3]
  n = 12
  rng = np.random.default_rng(20261124)
  tfm = np.repeat(np.eye(3, dtype=np.float32)[None, :, :], n, axis=0)
  desire = np.zeros((n, 8), dtype=np.float32)
  desire[3:6, 1] = 1
  np.savez(path, camera_size=np.array(camera, dtype=np.int32),
           road=rng.integers(0, 256, size=(n, nbytes), dtype=np.uint8),
           wide=rng.integers(0, 256, size=(n, nbytes), dtype=np.uint8),
           road_tfm=tfm, wide_tfm=tfm.copy(), desire_pulse=desire,
           traffic_convention=np.repeat(np.array([[1, 0]], dtype=np.float32), n, axis=0),
           action_t=np.repeat(np.array([[.2, .3]], dtype=np.float32), n, axis=0))


def load_replay(path: Path):
  import numpy as np
  path = Path(path)
  if not path.is_file() or path.stat().st_size > MAX_ARCHIVE_BYTES:
    raise ValueError("Missing replay or file exceeds size limit")
  keys = {"camera_size", "road", "wide", "road_tfm", "wide_tfm",
          "desire_pulse", "traffic_convention", "action_t"}
  with zipfile.ZipFile(path) as z:
    infos = z.infolist()
    if {i.filename for i in infos} != {k + ".npy" for k in keys} or len(infos) != len(keys):
      raise ValueError("Unexpected or duplicate replay arrays")
    if sum(i.file_size for i in infos) > MAX_ARCHIVE_BYTES:
      raise ValueError("Replay exceeds decompressed size limit")
  with np.load(path, allow_pickle=False) as source:
    arrays = {k: source[k] for k in keys}
  size = arrays["camera_size"]
  if size.shape != (2,) or size.dtype.kind not in "iu":
    raise ValueError("Invalid camera size")
  camera = tuple(int(v) for v in size)
  if camera not in ((1928, 1208), (1344, 760)):
    raise ValueError("Unsupported camera size")
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  nbytes = get_nv12_info(*camera)[3]
  road = arrays["road"]
  if road.ndim != 2 or not 1 <= len(road) <= MAX_FRAMES:
    raise ValueError("Expected 1 to 256 paired frames")
  n = len(road)
  for key in ("road", "wide"):
    if arrays[key].shape != (n, nbytes) or arrays[key].dtype != np.uint8:
      raise ValueError(f"{key}: invalid native NV12 buffer shape or dtype")
  for key, shape in (("road_tfm", (n, 3, 3)), ("wide_tfm", (n, 3, 3)),
                     ("desire_pulse", (n, 8)), ("traffic_convention", (n, 2)),
                     ("action_t", (n, 2))):
    a = arrays[key]
    if a.shape != shape or a.dtype != np.float32 or not np.all(np.isfinite(a)):
      raise ValueError(f"{key}: invalid recorded input shape, dtype or values")
  return camera, arrays


def replay_compiled(path: Path, expected_abi: dict, camera: tuple[int, int], arrays: dict):
  if Path("/AGNOS").exists() or os.environ.get("DEV") != "CPU:LLVM":
    raise RuntimeError("Desktop CPU replay only")
  import numpy as np
  from tinygrad.tensor import Tensor
  from tinygrad.device import Device
  from openpilot.selfdrive.modeld.helpers import load_oob
  from openpilot.selfdrive.modeld.compile_modeld import make_input_queues, WARP_INPUTS, POLICY_INPUTS
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  from openpilot.sunnypilot.models.native_abi import describe_metadata, compare_native_abi
  if not Device.DEFAULT.startswith("CPU"):
    raise RuntimeError("Expected CPU backend")
  with Path(path).open("rb") as stream:
    bundle = load_oob(stream)
  meta = bundle["metadata"]
  if compare_native_abi(expected_abi, describe_metadata(meta)):
    raise ValueError("Compiled model ABI does not match expected reference")
  queues, npy = make_input_queues(meta["input_shapes"], 4, device=Device.DEFAULT)
  slices = meta["output_slices"]
  parser = Parser()
  prev_desire = np.zeros(8, dtype=np.float32)
  frames = []
  for i in range(len(arrays["road"])):
    road = Tensor(arrays["road"][i], device=Device.DEFAULT).contiguous().realize()
    wide = Tensor(arrays["wide"][i], device=Device.DEFAULT).contiguous().realize()
    npy["tfm"][:] = arrays["road_tfm"][i]
    npy["big_tfm"][:] = arrays["wide_tfm"][i]
    npy["traffic_convention"][:] = arrays["traffic_convention"][i]
    npy["action_t"][:] = arrays["action_t"][i]
    desire = arrays["desire_pulse"][i].copy()
    desire[0] = 0
    npy["desire"][:] = np.where(desire - prev_desire > .99, desire, 0)
    prev_desire[:] = desire
    start = time.perf_counter()
    warped = bundle[camera](**{k: queues[k] for k in WARP_INPUTS}, frame=road, big_frame=wide)
    out, = bundle["run_policy"](**{k: queues[k] for k in POLICY_INPUTS}, warped=warped)
    raw = out.numpy()[0]
    elapsed = (time.perf_counter() - start) * 1000.
    if raw.shape != tuple(meta["output_shapes"]["outputs"][1:]) or not np.all(np.isfinite(raw)):
      raise ValueError(f"Invalid model output at frame {i}")
    npy["prev_feat"][:] = raw[slices["hidden_state"]]
    parsed = parser.parse_outputs({k: raw[np.newaxis, s].copy() for k, s in slices.items()})
    if not all(np.all(np.isfinite(v)) for v in parsed.values()):
      raise ValueError(f"Nonfinite parsed outputs at frame {i}")
    frames.append({"frame": i, "inference_ms": elapsed,
                   "plan": parsed["plan"][0].tolist(),
                   "lead": parsed["lead"][0].tolist(),
                   "lead_prob": parsed["lead_prob"][0].tolist(),
                   "raw_action": raw[slices["action"]].tolist() if "action" in slices else None})
  return frames
