#!/usr/bin/env python3
"""Compile and reload a pinned native TSFDOM artifact on an isolated bench.

Never replaces stock artifacts, launches modeld, modifies Params, publishes
predictions or uses CAN. QCOM use requires a physically disconnected bench
and a separate checkout with no active openpilot manager or control processes.
Desktop mode is available for development regression tests.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
COMPILER = ROOT / "openpilot/selfdrive/modeld/compile_modeld.py"
COMPILER_BLOB = "2d27a41496e8b4071ab8deeda286dde3804d6c74"
TINYGRAD_PIN = "8611fe22a7fcc7d1928bbde19ded66277cb12f3e"
MODEL_REF = "0394d7284cfc229f772ee4fa73ff724ba8245110"
SOURCE_SHA256 = "baa30b01cc980a2e616bca5c94f64b64cccce5acfd2456c48bd29da274f4d78b"
CAMERAS = ((1928, 1208), (1344, 760))
QCOM_FLAGS = {"DEV": "QCOM", "IMAGE": "1", "FLOAT16": "1", "NOLOCALS": "1",
              "JIT_BATCH_SIZE": "0", "OPENPILOT_HACKS": "1"}
RUNTIME_NAMES = {"manager", "modeld", "pandad", "controlsd", "card", "camerad",
                 "selfdrived", "dmonitoringmodeld", "boardd", "launch_openpilot.sh"}


def git(path, *args):
  return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()


def sha256(path):
  h = hashlib.sha256()
  with path.open("rb") as f:
    while chunk := f.read(1024 * 1024):
      h.update(chunk)
  return h.hexdigest()


def active_runtime(proc=Path("/proc")):
  found = set()
  for entry in proc.iterdir():
    if not entry.name.isdigit():
      continue
    try:
      comm = (entry / "comm").read_text().strip()
      cmd = (entry / "cmdline").read_bytes().decode("utf-8", errors="replace").split("\0")
    except OSError:
      continue
    if comm in RUNTIME_NAMES:
      found.add(comm)
    for arg in cmd:
      name = Path(arg).name.removesuffix(".py")
      if name in RUNTIME_NAMES:
        found.add(name)
      elif name.startswith(("openpilot.", "selfdrive.", "system.")) and name.rsplit(".", 1)[-1] in RUNTIME_NAMES:
        found.add(name.rsplit(".", 1)[-1])
  return sorted(found)


def require_isolated(backend, output, disconnected):
  live = Path("/data/openpilot").resolve()
  for path in (ROOT, output.resolve()):
    if path == live or path.is_relative_to(live):
      raise RuntimeError("Never run this tool in the active vehicle installation")
  if output.exists():
    raise FileExistsError("Choose a new output directory")
  if backend == "QCOM":
    if not disconnected or not Path("/AGNOS").exists() or not Path("/dev/kgsl-3d0").exists():
      raise RuntimeError("QCOM requires physically disconnected AGNOS hardware and explicit bench confirmation")
    running = active_runtime()
    if running:
      raise RuntimeError("Vehicle runtime is present: " + ", ".join(running))
  elif Path("/AGNOS").exists():
    raise RuntimeError("CPU development mode is not permitted on AGNOS")


def inspect_local_artifact(path, expected, backend, camera):
  # Reuse the native compiler's exact queue and warp contract.
  import numpy as np
  from tinygrad import Tensor, Device
  from openpilot.selfdrive.modeld.helpers import load_oob
  from openpilot.selfdrive.modeld.compile_modeld import make_input_queues, WARP_INPUTS, POLICY_INPUTS
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  from openpilot.sunnypilot.models.native_abi import describe_metadata, compare_native_abi
  from openpilot.sunnypilot.models.native_envelope import inspect_envelope
  if not Device.DEFAULT.startswith(backend):
    raise RuntimeError("Unexpected Tinygrad device " + Device.DEFAULT)
  with path.open("rb") as stream:
    inspect_envelope(stream)
  with path.open("rb") as stream:
    bundle = load_oob(stream)
  meta = bundle["metadata"]
  if compare_native_abi(expected, describe_metadata(meta)):
    raise ValueError("Native model ABI mismatch")
  if any(not callable(bundle[c]) for c in CAMERAS):
    raise ValueError("Missing compiled camera warp")
  queues, npy = make_input_queues(meta["input_shapes"], 4, device=Device.DEFAULT)
  parser = Parser()
  nbytes = get_nv12_info(*camera)[3]
  rng = np.random.default_rng(42)
  times = []
  for _ in range(3):
    frames = [Tensor(rng.integers(0, 256, nbytes, dtype=np.uint8), device=Device.DEFAULT).realize() for _ in range(2)]
    npy["tfm"][:] = np.eye(3, dtype=np.float32)
    npy["big_tfm"][:] = np.eye(3, dtype=np.float32)
    npy["traffic_convention"][:] = [1., 0.]
    npy["action_t"][:] = [.2, .3]
    npy["desire"][:] = 0.
    start = time.perf_counter()
    warped = bundle[camera](**{k: queues[k] for k in WARP_INPUTS}, frame=frames[0], big_frame=frames[1])
    out, = bundle["run_policy"](**{k: queues[k] for k in POLICY_INPUTS}, warped=warped)
    raw = out.numpy()[0]
    times.append((time.perf_counter() - start) * 1000)
    if raw.shape != (2580,) or not np.all(np.isfinite(raw)):
      raise ValueError("Invalid model output")
    npy["prev_feat"][:] = raw[meta["output_slices"]["hidden_state"]]
    parsed = parser.parse_outputs({k: raw[np.newaxis, s].copy() for k, s in meta["output_slices"].items()})
    if parsed["plan"].shape != (1, 33, 15) or parsed["lead"].shape != (1, 3, 6, 4):
      raise ValueError("Unexpected parsed output dimensions")
    if not all(np.all(np.isfinite(v)) for v in parsed.values()):
      raise ValueError("Nonfinite parsed prediction")
  return {"backend": Device.DEFAULT, "camera": list(camera), "synthetic_frames": len(times),
          "inference_ms": times, "raw_action_width": 4}


def main():
  p = argparse.ArgumentParser(description=__doc__)
  p.add_argument("--onnx", type=Path, required=True, help="Exact upstream TSFDOM ONNX payload")
  p.add_argument("--output-dir", type=Path, required=True)
  p.add_argument("--backend", choices=["CPU", "QCOM"], required=True)
  p.add_argument("--camera", choices=["1928x1208", "1344x760"], default="1928x1208")
  p.add_argument("--disconnected-bench", action="store_true")
  args = p.parse_args()
  out = args.output_dir.resolve()
  require_isolated(args.backend, out, args.disconnected_bench)
  if git(ROOT, "hash-object", str(COMPILER)) != COMPILER_BLOB:
    raise RuntimeError("Native compiler revision mismatch")
  tg = ROOT / "tinygrad_repo"
  if not (tg / ".git").exists() or git(tg, "rev-parse", "HEAD") != TINYGRAD_PIN:
    raise RuntimeError("Exact native tinygrad source required")
  if git(tg, "status", "--porcelain", "--untracked-files=no"):
    raise RuntimeError("Modified tinygrad source")
  if not args.onnx.is_file() or sha256(args.onnx) != SOURCE_SHA256:
    raise ValueError("TSFDOM source SHA256 mismatch")
  env = {k: v for k, v in os.environ.items() if k not in (*QCOM_FLAGS, "WARP_DEV", "QUEUE_DEV", "USBGPU", "BIG_INTO_SMALL", "AMD_IFACE", "GMMU")}
  flags = QCOM_FLAGS if args.backend == "QCOM" else {"DEV": "CPU:LLVM"}
  env.update(flags)
  env["PYTHONPATH"] = os.pathsep.join((str(ROOT), str(tg)))
  out.mkdir(mode=0o700, parents=False)
  report = {"source_commit": git(ROOT, "rev-parse", "HEAD"), "tinygrad_pin": TINYGRAD_PIN,
            "model_ref": MODEL_REF, "source_sha256": SOURCE_SHA256, "compiler_blob": COMPILER_BLOB,
            "backend_flags": flags, "compiled": False, "loaded": False,
            "vehicle_qualified": False, "runtime_activated": False}
  try:
    import shutil
    src = out / "candidate.onnx"
    shutil.copyfile(args.onnx, src)
    if sha256(src) != SOURCE_SHA256:
      raise ValueError("Copied source mismatch")
    artifact = out / "tsfdom-bench.pkl"
    subprocess.run([sys.executable, str(COMPILER), "--model-size", "512x256",
                    "--camera-resolutions", "1928x1208", "1344x760",
                    "--onnx", str(src), "--output", str(artifact), "--frame-skip", "4"],
                   cwd=out, env=env, check=True, timeout=1200)
    report.update(compiled=True, artifact_sha256=sha256(artifact), artifact_bytes=artifact.stat().st_size)
    # Load only this fresh compiler output, not third-party compiled pickle files.
    os.environ.clear()
    os.environ.update(env)
    sys.path[:0] = [str(ROOT), str(tg)]
    expected = json.loads((ROOT / "migration/sunnypilot_legacy/tsfdom_native_abi_report.json").read_text())["candidate_abi"]
    if args.backend == "QCOM" and active_runtime():
      raise RuntimeError("Vehicle runtime appeared before inference")
    camera = tuple(map(int, args.camera.split("x")))
    report["inference"] = inspect_local_artifact(artifact, expected, args.backend, camera)
    report["loaded"] = True
  except Exception as exc:
    report["error"] = repr(exc)
    raise
  finally:
    (out / "bench-report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
  main()
