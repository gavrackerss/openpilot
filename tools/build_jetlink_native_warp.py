#!/usr/bin/env python3
"""Build the *native Jetlink* dual-camera TinyJit warp offline, on the comma.

Unlike the fork's warp_*_tinygrad.pkl (stock image history update), this is
Jetlink's independent driving_warp_*_tinygrad.pkl:
    run(input_frame=(2, NV12_BYTES), M_inv=(2,3,3)) -> (2,6,128,256)

REQUIRES QCOM tinygrad + a parked, offroad comma. Never run while driving.
Build is opt-in: the installer does NOT run or arm this automatically.
"""
from __future__ import annotations

import argparse
import os
import pickle
import tempfile
from pathlib import Path


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('--force', action='store_true', help='replace an existing generated warp')
  args = ap.parse_args()
  if os.environ.get('DEV') != 'QCOM':
    raise SystemExit('STOP: set DEV=QCOM for the comma QCOM target')

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

  assert tuple(MEDMODEL_INPUT_SIZE) == (512, 256), 'Model warp geometry changed'
  op = Stock()
  cam_w, cam_h, model_w, model_h = op.camera()
  assert (model_w, model_h) == (512, 256), 'Current selected model geometry is not LS2-compatible'
  assert (cam_w, cam_h) in [(_ar_ox_fisheye.width, _ar_ox_fisheye.height),
                             (_os_fisheye.width, _os_fisheye.height)]
  dest = Path(op.warp_path(cam_w, cam_h, model_w, model_h))
  if dest.exists() and not args.force:
    raise SystemExit(f'STOP: native warp exists at {dest}; use --force only after backup')
  frame_n = int(get_nv12_info(cam_w, cam_h)[3])
  frame_prepare = make_frame_prepare(cam_w, cam_h, model_w, model_h)

  @TinyJit
  def drive_warp(*, M_inv, input_frame):
    # Both camera frames are warped as a single native TinyJit. No recurrent
    # history is held in this warp (that stays on the Pixel LS2 engine).
    road = frame_prepare(input_frame[0], M_inv[0])
    wide = frame_prepare(input_frame[1], M_inv[1])
    return Tensor.stack(road, wide, dim=0).contiguous().realize()

  specs = {
    'input_frame': ((2, frame_n), np.uint8, Device.DEFAULT),
    'M_inv': ((2, 3, 3), np.float32, Device.DEFAULT),
  }
  frames = Tensor(np.zeros((2, frame_n), np.uint8), device=Device.DEFAULT).realize()
  tfm = Tensor(np.tile(np.eye(3, dtype=np.float32), (2,1,1)), device=Device.DEFAULT).realize()
  for _ in range(4):
    result = drive_warp(M_inv=tfm, input_frame=frames)
    result.realize()
    Device[Device.DEFAULT].synchronize()
  assert drive_warp.captured is not None
  assert tuple(drive_warp.captured.ret.shape) == (2, 6, 128, 256)

  dest.parent.mkdir(parents=True, exist_ok=True)
  fd, tmp = tempfile.mkstemp(prefix='.jetlink-driving-warp-', suffix='.pkl', dir=dest.parent)
  try:
    with os.fdopen(fd, 'wb') as f:
      pickle.dump({'run': drive_warp, 'input_specs': specs}, f)
      f.flush(); os.fsync(f.fileno())
    # Validate the on-disk pickle using Jetlink's own strict loader. A bad
    # pickle NEVER replaces the real native warp or reaches modeld.
    saved = dest.parent / (dest.name + '.preflight-tmp')
    os.replace(tmp, saved)
    try:
      class TemporaryStock(Stock):
        def warp_path(self, *_):
          return saved
      loaded = Warps(TemporaryStock()).load(cam_w, cam_h, model_w, model_h)
      native = Warp(loaded, frame_n)
      a = np.zeros(frame_n, np.uint8)
      t = np.eye(3, dtype=np.float32)
      native.start(a.ctypes.data, a.ctypes.data, t, t)
      native.wait()
      assert memoryview(native.output).nbytes == 2 * 6 * 128 * 256
      os.replace(saved, dest)
    finally:
      saved.unlink(missing_ok=True)
  finally:
    Path(tmp).unlink(missing_ok=True)
  print('PASS: native Jetlink QCOM warp generated, loaded, and executed')
  print('Path:', dest)


if __name__ == '__main__':
  main()
