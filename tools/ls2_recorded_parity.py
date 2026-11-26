#!/usr/bin/env python3
"""Real-road LS2 Pixel/reference parity gate.

This is the XNOR equivalent of Zoompilot's scripts/verify_parity.py, but its
capture input is a recorded openpilot segment rather than synthetic images.

The capture stage:
  * decodes paired fcamera/ecamera HEVC frames as NV12,
  * restores camerad's padded NV12 layout,
  * uses the recorded calibration and the exact resident 0.11.2 ModelState warp,
  * sends the resulting (2,6,128,256) tensors to the prepared Pixel LS2 engine,
  * saves the exact warped/packed inputs and Pixel outputs for 32+ frames.

The reference stage runs those exact inputs through the unmodified LS2 ONNX
with ONNX Runtime. The compare stage gates the heads involved in the reported
road behavior: plan, lead, lead_prob, road_edges, lane_lines and action.

Capture is intended to run OFFROAD on the comma with the Pixel Jetlink server
enabled and no modeld owning the connection. Reference can run on another
machine with the same checkout and the LS2 ONNX.

Examples:
  python3 tools/ls2_recorded_parity.py capture \
    --fcamera /data/drive--0--fcamera.hevc \
    --ecamera /data/drive--0--ecamera.hevc \
    --rlog /data/drive--0--rlog.zst \
    --dir /data/ls2-road-parity --n 32

  python3 tools/ls2_recorded_parity.py reference \
    --onnx /path/to/big_driving_supercombo.onnx \
    --dir /data/ls2-road-parity

  python3 tools/ls2_recorded_parity.py compare --dir /data/ls2-road-parity
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

LS2_SHA256 = "909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8"
LS2_NBYTES = 768_749_892
REQUIRED_HEADS = ("plan", "lead", "lead_prob", "road_edges", "lane_lines", "action")
MIN_CORR = 0.999
MIN_SAMPLES = 16
QUIET_FRACTION = 0.05
TINY_TOLERANCE = 0.10
CONSTANT_FRACTION = 1e-3

# Raw MDN head layouts used by openpilot's Parser. Split these into homogeneous
# columns so a large position column cannot hide a bad angle/velocity column.
MDN_LAYOUTS = {
  "plan": [(0, 0, 15), (5, 1, 15)],
  "lane_lines": [(0, 0, 2)],
  "road_edges": [(0, 0, 2)],
  "lead": [(0, 0, 4), (2, 3, 4)],
}


def _sha256(path: Path) -> str:
  h = hashlib.sha256()
  with path.open("rb") as f:
    while chunk := f.read(8 * 1024 * 1024):
      h.update(chunk)
  return h.hexdigest()


def _record_context(rlog: str) -> dict:
  """Read enough route state to build the same camera transforms as modeld."""
  from openpilot.tools.lib.logreader import LogReader

  rpy = None
  device_type = None
  sensor = None
  is_rhd = True
  first_camera_id = None
  moving_camera_index = None
  v_ego = 0.0
  camera_samples = []

  for msg in LogReader(rlog, sort_by_time=True):
    which = msg.which()
    if which == "carState":
      v_ego = float(msg.carState.vEgo)
    elif which in ("extrinsicsCalibration", "liveCalibration"):
      cal = getattr(msg, which)
      value = np.asarray(cal.rpyCalib, dtype=np.float32)
      if value.shape == (3,) and np.isfinite(value).all():
        rpy = value
    elif which == "deviceState":
      device_type = str(msg.deviceState.deviceType)
    elif which == "driverMonitoringState":
      is_rhd = bool(msg.driverMonitoringState.isRHD)
    elif which == "narrowRoadCameraState":
      state = msg.narrowRoadCameraState
      sensor = str(state.sensor)
      frame_id = int(state.frameId)
      if first_camera_id is None:
        first_camera_id = frame_id
      idx = frame_id - first_camera_id
      if idx >= 0:
        camera_samples.append((idx, v_ego))
        if moving_camera_index is None and v_ego >= 3.0:
          moving_camera_index = idx

  if rpy is None:
    raise RuntimeError("rlog contains no usable extrinsics/live calibration")
  if device_type is None:
    raise RuntimeError("rlog contains no deviceState.deviceType")
  if sensor is None:
    raise RuntimeError("rlog contains no narrowRoadCameraState.sensor")

  return {
    "rpy_calib": rpy,
    "device_type": device_type,
    "sensor": sensor,
    "is_rhd": is_rhd,
    "moving_camera_index": int(moving_camera_index or 0),
    "camera_samples": camera_samples,
  }


def _tight_to_camerad_nv12(frame: np.ndarray, width: int, height: int) -> np.ndarray:
  """Put ffmpeg's tight NV12 frame back into camerad's Venus padded layout."""
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

  stride, y_height, uv_height, size = get_nv12_info(width, height)
  frame = np.asarray(frame, dtype=np.uint8).reshape(-1)
  tight = width * height * 3 // 2
  if frame.size != tight:
    raise ValueError(f"decoded NV12 is {frame.size} bytes, expected {tight}")

  out = np.zeros(size, dtype=np.uint8)
  y = frame[:width * height].reshape(height, width)
  uv = frame[width * height:].reshape(height // 2, width)

  y_dst = out[:stride * y_height].reshape(y_height, stride)
  uv_off = stride * y_height
  uv_dst = out[uv_off:uv_off + stride * uv_height].reshape(uv_height, stride)
  y_dst[:height, :width] = y
  uv_dst[:height // 2, :width] = uv
  return out


def _transforms(context: dict) -> tuple[np.ndarray, np.ndarray]:
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.common.transformations.model import get_warp_matrix

  key = (context["device_type"], context["sensor"])
  if key not in DEVICE_CAMERAS:
    raise RuntimeError(f"camera calibration table has no {key!r}")
  dc = DEVICE_CAMERAS[key]
  rpy = context["rpy_calib"]
  # Paired fcamera/ecamera means modeld's main stream is the narrow road camera
  # and the extra stream is the wide road camera.
  main = get_warp_matrix(rpy, dc.narrow_road.intrinsics, False).astype(np.float32)
  wide = get_warp_matrix(rpy, dc.wide_road.intrinsics, True).astype(np.float32)
  return main, wide


def _packed_inputs(spec, is_rhd: bool, frame_no: int) -> np.ndarray:
  packed = np.zeros(spec.packed_nelem, dtype=np.float32)
  for name, (at, _shape) in spec.packed_layout.items():
    n = at.stop - at.start
    if name == "traffic_convention":
      value = np.array([0.0, 1.0] if is_rhd else [1.0, 0.0], dtype=np.float32)
      packed[at] = value[:n]
    elif name == "action_t":
      # Representative real modeld horizons; identical on Pixel/reference.
      packed[at] = np.array([
        0.25 + 0.03 * np.sin(frame_no * 0.2),
        0.35 + 0.03 * np.cos(frame_no * 0.2),
      ], dtype=np.float32)[:n]
    elif name.startswith("desire"):
      # No lane-change desire during the parity gate. Zero is a valid pulse.
      packed[at] = 0.0
    else:
      raise RuntimeError(f"unhandled LS2 packed input {name!r}")
  return packed


def capture(args) -> int:
  from jetlink.client import JetlinkClient
  from openpilot.jetlink_authority import (
    MODEL_NBYTES, MODEL_SHA, PORT, discover_phone, validate_spec,
  )
  from openpilot.jetlink_native_authority import (
    JETLINK_FRAME_FAILURE_S, NativeModeldWarp, PIXEL_SERVER_TIMEOUT_S,
    _model_spec_checks, _pixel_transport,
  )
  from openpilot.selfdrive.modeld.modeld import ModelState
  from openpilot.tools.lib.framereader import FrameReader

  if args.n < 32:
    raise SystemExit("real-road parity requires at least 32 frames")

  context = _record_context(args.rlog)
  fcam = FrameReader(args.fcamera, pix_fmt="nv12", hwaccel="auto")
  ecam = FrameReader(args.ecamera, pix_fmt="nv12", hwaccel="auto")
  if (fcam.w, fcam.h) != (ecam.w, ecam.h):
    raise SystemExit(f"camera dimensions differ: f={fcam.w}x{fcam.h}, e={ecam.w}x{ecam.h}")

  start = context["moving_camera_index"] if args.start_frame is None else args.start_frame
  end = start + args.n
  if start < 0 or end > min(fcam.frame_count, ecam.frame_count):
    raise SystemExit(
      f"requested frames {start}:{end}, but cameras have "
      f"{fcam.frame_count}/{ecam.frame_count} frames"
    )

  out = Path(args.dir)
  out.mkdir(parents=True, exist_ok=True)

  # Load the resident stock ModelState solely to reuse the exact 0.11.2
  # compiled QCOM warp. Authority markers are irrelevant to this process.
  model = ModelState(fcam.w, fcam.h, False)
  model.warmup()
  warp = NativeModeldWarp(model)
  main_tfm, wide_tfm = _transforms(context)

  host = args.host or discover_phone()
  client = JetlinkClient(
    _pixel_transport(host, args.port or PORT),
    deadline=JETLINK_FRAME_FAILURE_S,
    name="XNOR-LS2-recorded-parity",
    want_hidden=False,
  )
  try:
    hello = client.hello(timeout=3.0)
    if not client.t.datagrams:
      raise RuntimeError("Pixel did not negotiate USB datagram frame upload")
    spec = client.ensure_engine(
      MODEL_SHA, MODEL_NBYTES, frame_skip=4,
      build_timeout=PIXEL_SERVER_TIMEOUT_S,
    )
    _model_spec_checks(spec)
    validate_spec(spec.to_dict())

    (out / "spec.json").write_text(json.dumps(spec.to_dict(), indent=2))
    meta = {
      "model_sha256": MODEL_SHA,
      "model_nbytes": MODEL_NBYTES,
      "server": {
        "backend": hello.get("backend"),
        "runtime_version": hello.get("runtime_version", hello.get("trt_version")),
        "device": hello.get("device"),
      },
      "source": {
        "fcamera": str(Path(args.fcamera).resolve()),
        "ecamera": str(Path(args.ecamera).resolve()),
        "rlog": str(Path(args.rlog).resolve()),
        "start_frame": start,
        "frames": args.n,
        "width": fcam.w,
        "height": fcam.h,
      },
      "calibration": {
        "device_type": context["device_type"],
        "sensor": context["sensor"],
        "rpy_calib": context["rpy_calib"].tolist(),
        "is_rhd": context["is_rhd"],
      },
      "heads": list(REQUIRED_HEADS),
    }
    (out / "capture.json").write_text(json.dumps(meta, indent=2))

    for n, idx in enumerate(range(start, end)):
      road = _tight_to_camerad_nv12(fcam.get(idx), fcam.w, fcam.h)
      wide = _tight_to_camerad_nv12(ecam.get(idx), ecam.w, ecam.h)
      warped = np.array(warp.run_arrays(road, wide, main_tfm, wide_tfm), copy=True)
      packed = _packed_inputs(spec, context["is_rhd"], n)
      pred = client.infer(warped, packed, frame_id=n, reset=(n == 0))

      np.save(out / f"in_warped_{n:03d}.npy", warped)
      np.save(out / f"in_packed_{n:03d}.npy", packed)
      np.save(out / f"out_link_{n:03d}.npy", np.asarray(pred, np.float32))
      gpu_us, queue_us, total_us = client.last_timings
      print(
        f"frame {n:03d} source={idx:04d} "
        f"gpu={gpu_us/1000:.1f}ms total={total_us/1000:.1f}ms "
        f"finite={bool(np.isfinite(pred).all())}"
      )
  finally:
    client.close()

  print(f"captured {args.n} real-road LS2 frames to {out}")
  return 0


_ORT_DTYPES = {
  "tensor(uint8)": np.uint8,
  "tensor(int32)": np.int32,
  "tensor(int64)": np.int64,
  "tensor(float16)": np.float16,
  "tensor(float)": np.float32,
  "tensor(double)": np.float64,
  "tensor(bool)": np.bool_,
}


def _ort_feed_dtypes(sess) -> dict[str, np.dtype]:
  out = {}
  for value in sess.get_inputs():
    if value.type not in _ORT_DTYPES:
      raise SystemExit(f"unsupported ONNX input type {value.name}: {value.type}")
    out[value.name] = _ORT_DTYPES[value.type]
  return out


def _captured_count(directory: Path, stem: str) -> int:
  return len(sorted(directory.glob(f"{stem}_*.npy")))


def reference(args) -> int:
  import onnxruntime as ort
  from jetlink.spec import ModelSpec, DRIVING_OUTPUT, spec_from_onnx

  d = Path(args.dir)
  link_spec = ModelSpec.load(d / "spec.json")
  onnx = Path(args.onnx)
  digest = _sha256(onnx)
  if digest != LS2_SHA256:
    raise SystemExit(f"wrong ONNX: sha256 {digest}, expected LS2 {LS2_SHA256}")

  # Derive the complete recurrent graph contract from the untouched ONNX.
  # The Pixel's ENGINE_RESP is still checked for the wire-facing contract, but
  # the reference must not guess which state_<q>/next_state_<q> tensors exist.
  spec = spec_from_onnx(str(onnx), frame_skip=link_spec.frame_skip,
                        sha256=digest, nbytes=onnx.stat().st_size)
  if spec.output_slices != link_spec.output_slices:
    raise SystemExit("LS2 ONNX output_slices do not match the Pixel engine spec")
  if spec.model_hw != link_spec.model_hw:
    raise SystemExit(f"LS2 ONNX model geometry {spec.model_hw} != Pixel {link_spec.model_hw}")

  n = _captured_count(d, "in_warped")
  if n < 32:
    raise SystemExit(f"need >=32 captured frames, found {n}")

  sess = ort.InferenceSession(str(onnx), providers=["CPUExecutionProvider"])
  dtypes = _ort_feed_dtypes(sess)
  shapes = {i.name: tuple(i.shape) for i in sess.get_inputs()}

  if spec.stateful:
    names = [o.name for o in sess.get_outputs()]
    state = {name: np.zeros(shapes[name], dtypes[name]) for name in spec.state_pairs}
    for i in range(n):
      warped = np.load(d / f"in_warped_{i:03d}.npy")
      packed = np.load(d / f"in_packed_{i:03d}.npy")
      feed = {
        "new_img": warped,
        **state,
        **{name: packed[at] for name, (at, _) in spec.packed_layout.items()},
      }
      missing = set(dtypes) - set(feed)
      if missing:
        raise SystemExit(f"reference has no input for {sorted(missing)}")
      feed = {
        k: np.ascontiguousarray(v, dtype=dtypes[k]).reshape(shapes[k])
        for k, v in feed.items()
      }
      outs = dict(zip(names, sess.run(None, feed), strict=True))
      pred = np.asarray(outs[DRIVING_OUTPUT], np.float32).reshape(-1)
      np.save(d / f"out_ref_{i:03d}.npy", pred)
      state = {name: outs[next_name] for name, next_name in spec.state_pairs.items()}
      print(f"reference frame {i:03d}: {pred.size} values")
  else:
    from jetlink.queues import PolicyQueues

    queues = PolicyQueues(spec)
    queues.reset()
    for i in range(n):
      warped = np.load(d / f"in_warped_{i:03d}.npy")
      packed = np.load(d / f"in_packed_{i:03d}.npy")
      feed = queues.step(warped, packed)
      missing = set(dtypes) - set(feed)
      if missing:
        raise SystemExit(f"reference queues have no input for {sorted(missing)}")
      feed = {k: np.ascontiguousarray(v, dtype=dtypes[k]) for k, v in feed.items()}
      pred = np.asarray(sess.run(None, feed)[0], np.float32).reshape(-1)
      np.save(d / f"out_ref_{i:03d}.npy", pred)
      queues.after_run({DRIVING_OUTPUT: pred})
      print(f"reference frame {i:03d}: {pred.size} values")

  return 0


def _corr(a: np.ndarray, b: np.ndarray) -> float:
  if a.std() == 0 and b.std() == 0:
    return 1.0
  if a.std() == 0 or b.std() == 0:
    return 1.0 if np.allclose(a, b) else 0.0
  return float(np.corrcoef(a, b)[0, 1])


def _columns(name: str, a: np.ndarray) -> dict[str, np.ndarray] | None:
  for hypotheses, selection, width in MDN_LAYOUTS.get(name, ()):
    rows = max(hypotheses, 1)
    if a.size % rows:
      continue
    raw = a.reshape(rows, -1)
    n = (raw.shape[1] - selection) // 2
    if n <= 0 or (raw.shape[1] - selection) % 2 or n % width:
      continue
    mu = raw[:, :n].reshape(-1, width)
    std = raw[:, n:2*n].reshape(-1, width)
    cols = {f"mu[{j}]": mu[:, j] for j in range(width)}
    cols.update({f"std[{j}]": std[:, j] for j in range(width)})
    if selection:
      cols["sel"] = raw[:, 2*n:].reshape(-1)
    return cols
  return None


def _tolerance(ref: np.ndarray, spread: float) -> float:
  return max(
    TINY_TOLERANCE * float(np.abs(ref).max(initial=0.0)),
    CONSTANT_FRACTION * spread,
  )


def _judge(a: np.ndarray, b: np.ndarray, per_frame: int, spread: float | None = None) -> tuple[bool, str]:
  if per_frame < MIN_SAMPLES or (spread is not None and b.std() < QUIET_FRACTION * spread):
    bound = _tolerance(b, b.std() if spread is None else spread)
    err = float(np.abs(a - b).max(initial=0.0))
    return err <= bound, f"maxerr={err:.5g} bound={bound:.5g}"
  corr = _corr(a, b)
  return corr >= MIN_CORR, f"corr={corr:.6f}"


def compare(args) -> int:
  from jetlink.spec import ModelSpec

  d = Path(args.dir)
  spec = ModelSpec.load(d / "spec.json")
  n = _captured_count(d, "out_link")
  if n < 32:
    raise SystemExit(f"need >=32 Pixel outputs, found {n}")
  if _captured_count(d, "out_ref") != n:
    raise SystemExit("reference outputs missing/incomplete; run reference mode first")

  links = [np.load(d / f"out_link_{i:03d}.npy").reshape(-1) for i in range(n)]
  refs = [np.load(d / f"out_ref_{i:03d}.npy").reshape(-1) for i in range(n)]

  failed = []
  print(f"pooled over {n} real-road frames:")
  for name in REQUIRED_HEADS:
    if name not in spec.output_slices:
      failed.append(f"{name}: missing slice")
      print(f"  {name:16} MISSING")
      continue
    sl = spec.output_slices[name]
    a = np.concatenate([x[sl] for x in links])
    b = np.concatenate([x[sl] for x in refs])
    per_frame = links[0][sl].size
    ok, why = _judge(a, b, per_frame)
    details = []

    col_link = [_columns(name, x[sl]) for x in links]
    if col_link[0]:
      col_ref = [_columns(name, x[sl]) for x in refs]
      for key in col_link[0]:
        ca = np.concatenate([x[key] for x in col_link])
        cb = np.concatenate([x[key] for x in col_ref])
        col_ok, col_why = _judge(ca, cb, col_link[0][key].size, b.std())
        if not col_ok:
          ok = False
          details.append(f"{key} {col_why}")

    max_abs = float(np.abs(a - b).max(initial=0.0))
    mean_abs = float(np.abs(a - b).mean())
    suffix = "" if ok else "  <-- FAIL"
    extra = "" if not details else " | " + "; ".join(details[:8])
    print(
      f"  {name:16} {_corr(a,b):.6f} {why:26} "
      f"max={max_abs:.5f} mean={mean_abs:.6f}{suffix}{extra}"
    )
    if not ok:
      failed.append(name)

  report = {
    "frames": n,
    "required_heads": list(REQUIRED_HEADS),
    "min_corr": MIN_CORR,
    "passed": not failed,
    "failed": failed,
  }
  (d / "result.json").write_text(json.dumps(report, indent=2))

  if failed:
    print("\nFAIL: " + ", ".join(failed))
    return 1
  print("\nPASS: Pixel LS2 matches the unmodified LS2 ONNX on every required head.")
  return 0


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = parser.add_subparsers(dest="mode", required=True)

  p = sub.add_parser("capture")
  p.add_argument("--fcamera", required=True)
  p.add_argument("--ecamera", required=True)
  p.add_argument("--rlog", required=True)
  p.add_argument("--dir", required=True)
  p.add_argument("--n", type=int, default=32)
  p.add_argument("--start-frame", type=int)
  p.add_argument("--host")
  p.add_argument("--port", type=int, default=5599)

  p = sub.add_parser("reference")
  p.add_argument("--onnx", required=True)
  p.add_argument("--dir", required=True)

  p = sub.add_parser("compare")
  p.add_argument("--dir", required=True)

  args = parser.parse_args()
  return {"capture": capture, "reference": reference, "compare": compare}[args.mode](args)


if __name__ == "__main__":
  sys.exit(main())
