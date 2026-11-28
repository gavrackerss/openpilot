#!/usr/bin/env python3
"""Exact Clarity/Zoompilot-style Cinque Terre V2 synthetic parity on our Pixel USB-NCM path.

Runs the published 32-frame seed-0 style workload, but connects through XNOR's
verified Pixel USB-NCM transport.  The comparison logic is the upstream
verify_parity.py gate: every output slice, every MDN column, per-frame and
pooled, MIN_CORR 0.999 with the same quiet/small absolute-error fallback.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

import numpy as np

from jetlink.spec import DRIVING_OUTPUT, ModelSpec

V2_SHA256 = "09d080f36965bb2a0790500452bd328aa03c484d0222aa79d1ad9f021a522aec"
V2_NBYTES = 766_040_736
V2_REF = "37bfa1413edcdc2e8844984b83727c33f81d8f46"
MIN_CORR = 0.999
MIN_SAMPLES = 16
QUIET_FRACTION = 0.05
TINY_TOLERANCE = 0.10
CONSTANT_FRACTION = 1e-3

MDN_LAYOUTS = {
  "plan": [(0, 0, 15), (5, 1, 15)],
  "lane_lines": [(0, 0, 2)],
  "road_edges": [(0, 0, 2)],
  "lead": [(0, 0, 4), (2, 3, 4)],
  "pose": [(0, 0, 6)],
  "wide_from_device_euler": [(0, 0, 3)],
  "road_transform": [(0, 0, 6)],
  "sim_pose": [(0, 0, 6)],
}


def _sha256(path: Path) -> str:
  h = hashlib.sha256()
  with path.open("rb") as f:
    while chunk := f.read(8 * 1024 * 1024):
      h.update(chunk)
  return h.hexdigest()


def _validate_v2_spec(spec: ModelSpec) -> None:
  if spec.sha256 != V2_SHA256 or int(spec.nbytes) != V2_NBYTES:
    raise ValueError("Pixel engine is not exact Cinque Terre V2")
  if spec.stateful:
    raise ValueError("Cinque Terre V2 must use the queued graph contract")
  if tuple(spec.model_hw) != (128, 256):
    raise ValueError(f"unexpected V2 geometry {spec.model_hw}")
  if spec.packed_nelem != 12:
    raise ValueError(f"unexpected V2 packed scalar count {spec.packed_nelem}")
  if tuple(spec.output_shapes.get(DRIVING_OUTPUT, ())) != (1, 18452):
    raise ValueError(f"unexpected V2 output shape {spec.output_shapes.get(DRIVING_OUTPUT)}")
  if spec.hidden_range is None or spec.hidden_range[1] - spec.hidden_range[0] != 16384:
    raise ValueError(f"unexpected V2 hidden slice {spec.hidden_range}")


def make_inputs(spec: ModelSpec, n: int, seed: int = 0):
  rng = np.random.default_rng(seed)
  h, w = spec.model_hw
  yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
  frames = []
  for i in range(n):
    warped = np.empty(spec.warped_shape, np.uint8)
    for cam in range(warped.shape[0]):
      for ch in range(warped.shape[1]):
        phase = 0.7 * i + 1.3 * ch + 2.1 * cam
        base = (96 + 64 * np.sin(xx / 37.0 + phase) + 48 * np.cos(yy / 23.0 - phase)
                + 24 * np.sin((xx + yy) / 61.0))
        warped[cam, ch] = np.clip(base + rng.normal(0, 6, (h, w)), 0, 255).astype(np.uint8)
    packed = np.zeros(spec.packed_nelem, np.float32)
    for name, (at, _) in spec.packed_layout.items():
      size = at.stop - at.start
      if name == "traffic_convention":
        packed[at] = np.array([1.0, 0.0][:size])
      elif name == "action_t":
        packed[at] = np.array([0.25 + 0.1 * np.sin(0.5 * i),
                               0.35 + 0.1 * np.cos(0.5 * i)][:size])
      elif name == "desire":
        if i % 8 == 0:
          packed[at.start + 1 + (i // 8) % (size - 1)] = 1.0
    frames.append((warped, packed))
  return frames


def capture(args) -> int:
  from jetlink.client import JetlinkClient
  from openpilot.jetlink_authority import PORT, discover_phone
  from openpilot.jetlink_native_authority import (
    JETLINK_FRAME_FAILURE_S, PIXEL_SERVER_TIMEOUT_S, _pixel_transport,
  )

  if args.n < 32:
    raise SystemExit("V2 synthetic control requires at least 32 frames")
  out = Path(args.dir)
  out.mkdir(parents=True, exist_ok=True)

  host = args.host or discover_phone()
  client = JetlinkClient(
    _pixel_transport(host, args.port or PORT),
    deadline=JETLINK_FRAME_FAILURE_S,
    name="XNOR-V2-synthetic-parity",
    want_hidden=True,
  )
  try:
    hello = client.hello(timeout=3.0)
    server_backend = hello.get("backend")
    server_device = hello.get("device")
    server_runtime = hello.get("runtime_version", hello.get("trt_version"))
    print(f"server: backend={server_backend} device={server_device} runtime={server_runtime}")
    if args.expect_backend is not None and server_backend != args.expect_backend:
      raise RuntimeError(f"Pixel backend {server_backend!r} != expected {args.expect_backend!r}")
    if args.expect_device is not None and server_device != args.expect_device:
      raise RuntimeError(f"Pixel device {server_device!r} != expected {args.expect_device!r}")
    if args.expect_device_prefix is not None and not str(server_device or "").startswith(args.expect_device_prefix):
      raise RuntimeError(f"Pixel device {server_device!r} does not start with expected prefix {args.expect_device_prefix!r}")
    if not client.t.datagrams:
      raise RuntimeError("Pixel did not negotiate USB datagram frame upload")
    spec = client.ensure_engine(
      V2_SHA256, V2_NBYTES, frame_skip=4,
      build_timeout=PIXEL_SERVER_TIMEOUT_S,
      fetch=True, ref=V2_REF,
    )
    _validate_v2_spec(spec)
    engine_accelerator = (client._engine_state or {}).get("accelerator")
    print(f"engine: accelerator={engine_accelerator}")
    if args.expect_accelerator is not None and engine_accelerator != args.expect_accelerator:
      raise RuntimeError(
        f"Pixel engine accelerator {engine_accelerator!r} != expected {args.expect_accelerator!r}"
      )
    (out / "spec.json").write_text(json.dumps(spec.to_dict(), indent=2))
    (out / "capture.json").write_text(json.dumps({
      "model_sha256": V2_SHA256,
      "model_nbytes": V2_NBYTES,
      "seed": args.seed,
      "frames": args.n,
      "server": {
        "backend": hello.get("backend"),
        "runtime_version": hello.get("runtime_version", hello.get("trt_version")),
        "device": hello.get("device"),
      },
    }, indent=2))

    for i, (warped, packed) in enumerate(make_inputs(spec, args.n, args.seed)):
      pred = client.infer(warped, packed, frame_id=i, reset=(i == 0))
      np.save(out / f"in_warped_{i:03d}.npy", warped)
      np.save(out / f"in_packed_{i:03d}.npy", packed)
      np.save(out / f"out_link_{i:03d}.npy", np.asarray(pred, np.float32))
      gpu_us, queue_us, total_us = client.last_timings
      print(f"frame {i:03d}: {pred.size} values gpu={gpu_us/1000:.1f}ms total={total_us/1000:.1f}ms "
            f"finite={bool(np.isfinite(pred).all())}")
  finally:
    client.close()
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


def _ort_feed_dtypes(sess):
  out = {}
  for value in sess.get_inputs():
    if value.type not in _ORT_DTYPES:
      raise SystemExit(f"unsupported ONNX input type {value.name}: {value.type}")
    out[value.name] = _ORT_DTYPES[value.type]
  return out


def reference(args) -> int:
  os.environ.setdefault("OMP_NUM_THREADS", "1")
  os.environ.setdefault("OMP_WAIT_POLICY", "PASSIVE")
  import onnxruntime as ort
  from jetlink.queues import PolicyQueues

  d = Path(args.dir)
  spec = ModelSpec.load(d / "spec.json")
  onnx = Path(args.onnx)
  digest = _sha256(onnx)
  if digest != V2_SHA256 or onnx.stat().st_size != V2_NBYTES:
    raise SystemExit(f"wrong V2 ONNX: sha256={digest} bytes={onnx.stat().st_size}")

  if args.ort_mode == "default":
    print("creating DEFAULT ONNX Runtime CPU session (normal graph optimisations/arena)...", flush=True)
    sess = ort.InferenceSession(str(onnx), providers=["CPUExecutionProvider"])
  else:
    so = ort.SessionOptions()
    so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1
    so.enable_cpu_mem_arena = False
    so.enable_mem_pattern = False
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_DISABLE_ALL
    try:
      so.add_session_config_entry("session.intra_op.allow_spinning", "0")
      so.add_session_config_entry("session.inter_op.allow_spinning", "0")
    except Exception:
      pass
    print("creating LOWMEM ONNX Runtime CPU session...", flush=True)
    sess = ort.InferenceSession(str(onnx), sess_options=so, providers=["CPUExecutionProvider"])
  print(f"ONNX Runtime session ready ({args.ort_mode})", flush=True)

  dtypes = _ort_feed_dtypes(sess)
  queues = PolicyQueues(spec)
  queues.reset()
  n = len(list(d.glob("in_warped_*.npy")))
  if n < 32:
    raise SystemExit(f"need >=32 captured frames, found {n}")

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


def _corr(a, b):
  if a.std() == 0 and b.std() == 0:
    return 1.0
  if a.std() == 0 or b.std() == 0:
    return 1.0 if np.allclose(a, b) else 0.0
  return float(np.corrcoef(a, b)[0, 1])


def _columns(name, a):
  for hyp, sel, width in MDN_LAYOUTS.get(name, ()):
    rows = max(hyp, 1)
    if a.size % rows:
      continue
    raw = a.reshape(rows, -1)
    n = (raw.shape[1] - sel) // 2
    if n <= 0 or (raw.shape[1] - sel) % 2 or n % width:
      continue
    mu = raw[:, :n].reshape(-1, width)
    std = raw[:, n:2*n].reshape(-1, width)
    cols = {f"mu[{j}]": mu[:, j] for j in range(width)}
    cols.update({f"std[{j}]": std[:, j] for j in range(width)})
    if sel:
      cols["sel"] = raw[:, 2*n:].reshape(-1)
    return cols
  return None


def _tolerance(ref, spread):
  return max(TINY_TOLERANCE * float(np.abs(ref).max(initial=0.0)),
             CONSTANT_FRACTION * spread)


def _judge(a, b, per_frame, spread=None):
  if per_frame < MIN_SAMPLES or (spread is not None and b.std() < QUIET_FRACTION * spread):
    bound = _tolerance(b, b.std() if spread is None else spread)
    err = float(np.abs(a - b).max(initial=0.0))
    return err <= bound, f"by error, max abs {err:.4g} {'<=' if err <= bound else '>'} {bound:.4g}"
  c = _corr(a, b)
  return c >= MIN_CORR, f"{c:.6f}"


def _report_slices(spec, links, refs):
  passed = {}
  for name, sl in sorted(spec.output_slices.items()):
    a = np.concatenate([x[sl] for x in links])
    b = np.concatenate([x[sl] for x in refs])
    whole = _corr(a, b)
    ok, why = _judge(a, b, links[0][sl].size)
    detail = f"{why if why.startswith('by error') else '(compared whole)':38}"
    cols_a = [_columns(name, x[sl]) for x in links]
    if cols_a[0]:
      cols_b = [_columns(name, x[sl]) for x in refs]
      worst_c, worst_k, failed, by_error = 2.0, "", [], 0
      for k in cols_a[0]:
        ca = np.concatenate([c[k] for c in cols_a])
        cb = np.concatenate([c[k] for c in cols_b])
        col_ok, col_why = _judge(ca, cb, cols_a[0][k].size, b.std())
        if col_why.startswith("by error"):
          by_error += 1
        elif float(col_why) < worst_c:
          worst_c, worst_k = float(col_why), k
        if not col_ok:
          failed.append(f"{k} {col_why}")
      ok &= not failed
      worst = f"worst col {worst_c:8.6f} {worst_k:8}" if worst_k else f"{'no column correlated':27}"
      detail = f"{worst} {f'({by_error} by error)' if by_error else '':14}"
      if failed:
        detail += "  cols: " + ", ".join(failed)
    passed[name] = ok
    flag = "" if ok else "   <-- FAIL"
    print(f"    {name:24} corr {whole:8.6f}  {detail}  max abs {np.abs(a-b).max():8.4f}  "
          f"mean abs {np.abs(a-b).mean():7.5f}{flag}")
  return passed


def compare(args) -> int:
  d = Path(args.dir)
  spec = ModelSpec.load(d / "spec.json")
  n = len(list(d.glob("out_link_*.npy")))
  if n < 32:
    raise SystemExit(f"need >=32 captured outputs, found {n}")
  links, refs = [], []
  for i in range(n):
    links.append(np.load(d / f"out_link_{i:03d}.npy").reshape(-1))
    refs.append(np.load(d / f"out_ref_{i:03d}.npy").reshape(-1))

  frame_fail = {}
  for i, (link, ref) in enumerate(zip(links, refs, strict=True)):
    print(f"\nframe {i}: corr {_corr(link, ref):.6f}  max abs {np.abs(link-ref).max():.4f}")
    for name, sl in sorted(spec.output_slices.items()):
      a, b = link[sl], ref[sl]
      c = _corr(a, b)
      gated = a.size >= MIN_SAMPLES
      if gated and c < MIN_CORR:
        frame_fail[name] = min(frame_fail.get(name, 2.0), c)

  print(f"\npooled over {n} frames, per slice and per column:")
  passed = _report_slices(spec, links, refs)
  bad = [f"{k} {v:.6f} on one frame" for k, v in sorted(frame_fail.items(), key=lambda kv: kv[1])]
  bad += [f"{k} pooled" for k, ok in passed.items() if not ok and k not in frame_fail]
  if bad:
    print(f"\nFAIL: {len(bad)} slice(s) below corr {MIN_CORR} or past their error bound: " + ", ".join(bad))
    return 1
  print(f"\nOK: every slice and every column at or above corr {MIN_CORR}, or within "
        f"{TINY_TOLERANCE:.0%} of its largest value where too small/quiet to correlate")
  return 0


def main() -> int:
  p = argparse.ArgumentParser(description=__doc__)
  sub = p.add_subparsers(dest="mode", required=True)

  c = sub.add_parser("capture")
  c.add_argument("--dir", required=True)
  c.add_argument("--n", type=int, default=32)
  c.add_argument("--seed", type=int, default=0)
  c.add_argument("--host")
  c.add_argument("--port", type=int, default=5599)
  c.add_argument("--expect-backend", help="abort unless HELLO reports this backend (for A/B qualification)")
  c.add_argument("--expect-device", help="abort unless HELLO reports this exact device tag (for A/B qualification)")
  c.add_argument("--expect-device-prefix", help="abort unless HELLO device tag starts with this prefix")
  c.add_argument("--expect-accelerator", help="abort unless the ready engine reports this actual accelerator")

  r = sub.add_parser("reference")
  r.add_argument("--onnx", required=True)
  r.add_argument("--dir", required=True)
  r.add_argument("--ort-mode", choices=("lowmem", "default"), default="lowmem",
                 help="reference session: lowmem fits comma RAM; default reproduces normal ORT execution")

  q = sub.add_parser("compare")
  q.add_argument("--dir", required=True)

  args = p.parse_args()
  return {"capture": capture, "reference": reference, "compare": compare}[args.mode](args)


if __name__ == "__main__":
  sys.exit(main())
