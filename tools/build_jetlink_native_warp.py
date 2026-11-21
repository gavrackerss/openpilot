#!/usr/bin/env python3
"""Build the native Jetlink dual-camera QCOM warp on a parked comma.

This builds a separate driving_warp_*_tinygrad.pkl file, NOT the stock
warp_*_tinygrad.pkl used by local Openpilot. V5.9.3's native adapter requires
this file and will not fall back to a CPU warp if it is missing.

The V5.9.3 original called Stock.camera(), which imports the nonexistent
openpilot.common.hardware in this fork. This complete replacement obtains the
camera geometry directly from the same camera definitions used by modeld's
compile_warp.py. Default: comma3 (tici/tizi, 1928x1208).

No vehicle control is enabled or changed by this builder.
"""
from __future__ import annotations

import argparse
import os
import pickle
import tempfile
from pathlib import Path


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--force', action='store_true', help='replace an existing native warp only after backup')
  ap.add_argument('--camera', choices=('comma3', 'comma4'), default='comma3',
                  help='camera generation; comma3 is the default for this device')
  args = ap.parse_args()

  if os.environ.get('DEV') != 'QCOM':
    raise SystemExit('STOP: set DEV=QCOM for the comma QCOM target')
  # The native QCOM warp must be built while parked/offroad and with authority
  # disarmed; fail closed even when the caller forgets to remove the markers.
  flags = ('/data/jetlink_xnor/AUTHORITY_ENABLED', '/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED')
  if any(Path(flag).exists() for flag in flags):
    raise SystemExit('STOP: remove BOTH authority markers before building this experimental warp')

  import numpy as np
  from tinygrad.device import Device
  from tinygrad.tensor import Tensor
  from tinygrad.engine.jit import TinyJit
  from openpilot.selfdrive.modeld.compile_warp import make_frame_prepare
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  from openpilot.common.transformations.model import MEDMODEL_INPUT_SIZE
  from openpilot.common.transformations.camera import _ar_ox_fisheye, _os_fisheye
  from jetlink.openpilot.warp import Warp, Warps
  from jetlink.openpilot.stock import Stock

  # Stock.camera() is deliberately NOT called: its import of
  # openpilot.common.hardware is invalid in this xnor-dev checkout.
  camera = _ar_ox_fisheye if args.camera == 'comma3' else _os_fisheye
  cam_w, cam_h = int(camera.width), int(camera.height)
  model_w, model_h = tuple(int(v) for v in MEDMODEL_INPUT_SIZE)
  if (model_w, model_h) != (512, 256):
    raise SystemExit(f'STOP: expected LS2 warp input 512x256, got {model_w}x{model_h}')
  if (cam_w, cam_h) not in ((1928, 1208), (1344, 760)):
    raise SystemExit(f'STOP: unsupported camera size {cam_w}x{cam_h}')

  op = Stock()
  dest = Path(op.warp_path(cam_w, cam_h, model_w, model_h))
  print(f'Camera: {args.camera}, {cam_w}x{cam_h}')
  print(f'LS2 warp: {model_w}x{model_h} -> (2, 6, 128, 256)')
  print(f'Native warp path: {dest}')
  if dest.exists() and not args.force:
    raise SystemExit(f'STOP: native warp already exists at {dest}; --force requires a backup first')

  frame_n = int(get_nv12_info(cam_w, cam_h)[3])
  frame_prepare = make_frame_prepare(cam_w, cam_h, model_w, model_h)

  @TinyJit
  def drive_warp(*, M_inv, input_frame):
    # Both cameras are warped together. Recurrent state remains on the Pixel.
    road = frame_prepare(input_frame[0], M_inv[0])
    wide = frame_prepare(input_frame[1], M_inv[1])
    return Tensor.stack(road, wide, dim=0).contiguous().realize()

  specs = {
    'input_frame': ((2, frame_n), np.uint8, Device.DEFAULT),
    'M_inv': ((2, 3, 3), np.float32, Device.DEFAULT),
  }
  frames = Tensor(np.zeros((2, frame_n), np.uint8), device=Device.DEFAULT).realize()
  tfm = Tensor(np.tile(np.eye(3, dtype=np.float32), (2, 1, 1)), device=Device.DEFAULT).realize()
  for _ in range(4):
    result = drive_warp(M_inv=tfm, input_frame=frames)
    result.realize()
    Device[Device.DEFAULT].synchronize()
  if drive_warp.captured is None or tuple(drive_warp.captured.ret.shape) != (2, 6, 128, 256):
    raise RuntimeError('Native QCOM warp captured an invalid graph or output shape')

  dest.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(prefix='.jetlink-driving-warp-', suffix='.pkl', dir=dest.parent)
  preflight = dest.parent / (dest.name + '.preflight-tmp')
  try:
    with os.fdopen(fd, 'wb') as f:
      pickle.dump({'run': drive_warp, 'input_specs': specs}, f)
      f.flush()
      os.fsync(f.fileno())
    os.replace(tmp, preflight)
    try:
      class TemporaryStock(Stock):
        def warp_path(self, *_):
          return preflight
      loaded = Warps(TemporaryStock()).load(cam_w, cam_h, model_w, model_h)
      native = Warp(loaded, frame_n)
      a = np.zeros(frame_n, dtype=np.uint8)
      m = np.eye(3, dtype=np.float32)
      native.start(a.ctypes.data, a.ctypes.data, m, m)
      native.wait()
      if memoryview(native.output).nbytes != 2 * 6 * 128 * 256:
        raise RuntimeError('Validated native warp has wrong output size')
      if dest.exists():
        if not args.force:
          raise RuntimeError('Native warp appeared while building; refusing overwrite')
        backup = dest.with_suffix(dest.suffix + '.bak')
        if backup.exists():
          raise RuntimeError(f'Refusing to overwrite existing backup: {backup}')
        os.replace(dest, backup)
        print(f'Prior warp backed up: {backup}')
      os.replace(preflight, dest)
    finally:
      preflight.unlink(missing_ok=True)
  finally:
    Path(tmp).unlink(missing_ok=True)
  print('PASS: native Jetlink QCOM warp generated, validated and executed')
  print('Path:', dest)


if __name__ == '__main__':
  main()
