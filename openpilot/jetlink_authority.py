#!/usr/bin/env python3
"""XNOR Android tether adapter for Zoompilot's authority/fallback implementation.

The model selector, frame holds, fallback and reconnect logic are *not* forked:
they are jetlink.openpilot.joining.JoiningModelState and
jetlink.openpilot.model_state.JetlinkModelState, pinned in gavrackerss/jetlink.

The phone exposes TCP/UDP over Android USB tethering. Zoompilot's stock cable
mode has the inverse USB roles, so the adapter connects outward to the phone.
No separate IPC prediction selector or manager gadget owner is used.
"""
from __future__ import annotations

import ctypes
from pathlib import Path
import subprocess
import numpy as np

from openpilot.common.swaglog import cloudlog

VERSION = 'xn5.9-zoompilot-seam'
SHA256 = '909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8'
NBYTES = 768749892
HOST, PORT = '10.169.232.180', 5599
ARM = Path('/data/jetlink_xnor/AUTHORITY_ENABLED')
QUALIFIED = Path('/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED')
MODEL_HW = (128, 256)


def tether_route_valid() -> bool:
  try:
    res = subprocess.run(['ip', 'route', 'get', HOST], check=True,
                         capture_output=True, text=True, timeout=1.0)
    return ' dev usb0 ' in f' {res.stdout} '
  except (OSError, subprocess.SubprocessError):
    return False


def enabled() -> bool:
  # The independent qualification marker is *not* shipped with this port.
  return ARM.is_file() and QUALIFIED.is_file() and tether_route_valid()


class _Progress:
  def report(self, stage, fraction, message, **kwargs):
    cloudlog.info('jetlink-authority %s %.3f %s', stage, fraction, message)

  def clear(self):
    pass


class _Small:
  """Adapt stock's run(...,prepare_only:bool) to Zoompilot run(...,after_enqueue)."""
  def __init__(self, model):
    object.__setattr__(self, '_model', model)

  def __getattr__(self, name):
    return getattr(self._model, name)

  def __setattr__(self, name, value):
    setattr(self._model, name, value)

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    result = self._model.run(bufs, transforms, inputs, False)
    if after_enqueue is not None:
      after_enqueue()
    return result


class _Warp:
  """Use the *existing V5.8* NV12-to-LS2 mapping, not a new warp transform."""
  def __init__(self, cam_w: int, cam_h: int):
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
    self.cam_w, self.cam_h = cam_w, cam_h
    self.buf_bytes = get_nv12_info(cam_w, cam_h)[3]
    self.output = np.empty((2, 6, *MODEL_HW), dtype=np.uint8)
    self._last = [None, None]
    self._offsets = [None, None]

  def start(self, frame: int, big_frame: int, tfm: np.ndarray, big_tfm: np.ndarray):
    from openpilot.jetlink_shadow_bench import warp_indices
    for i, (ptr, matrix) in enumerate(((frame, tfm), (big_frame, big_tfm))):
      if not ptr or matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('invalid frame pointer or calibration')
      if self._last[i] is None or not np.array_equal(matrix, self._last[i]):
        offsets = warp_indices(self.cam_w, self.cam_h, 256, 512, matrix)
        if offsets.size != self.output[i].size or offsets.min() < 0 or offsets.max() >= self.buf_bytes:
          raise ValueError('NV12 warp outside camera buffer')
        self._offsets[i] = offsets
        self._last[i] = matrix.copy()
      raw = np.ctypeslib.as_array((ctypes.c_uint8 * self.buf_bytes).from_address(int(ptr)))
      np.take(raw, self._offsets[i], out=self.output[i].reshape(-1))

  def wait(self):
    pass


class _Control:
  sm = None

  @staticmethod
  def in_control() -> bool:
    # Missing/stale vehicle information blocks handover, never permits it.
    try:
      import cereal.messaging as messaging
      sm = _Control.sm
      if sm is None:
        _Control.sm = sm = messaging.SubMaster(['carState', 'carControl', 'selfdriveState'])
      sm.update(0)
      services = ['carState', 'carControl', 'selfdriveState']
      return not (sm.all_alive(services) and sm.all_valid(services)) or (
        sm['carControl'].enabled or sm['carControl'].latActive or
        sm['selfdriveState'].enabled or sm['selfdriveState'].active)
    except Exception:
      return True


def _reset_small(small):
  # Legacy split-model histories, preserving GPU tensor identities.
  for array in small.numpy_inputs.values():
    array.fill(0)
  for tensor in small.img_queues.values():
    tensor.assign(0).realize()
  small.full_input_queues.reset()
  small.prev_desire.fill(0)


def attach(small_model, cam_w: int, cam_h: int):
  """Keep local authority unless explicitly enabled and Jetlink is usable."""
  if not enabled():
    return small_model
  from jetlink.client import JetlinkClient
  from jetlink.openpilot.joining import JoiningModelState
  from jetlink.openpilot.model_state import JetlinkModelState
  from jetlink.openpilot.interface import ModelFace
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  from openpilot.selfdrive.modeld.constants import ModelConstants
  from openpilot.selfdrive.modeld.modeld import (
    LAT_SMOOTH_SECONDS, LONG_SMOOTH_SECONDS, get_action_from_model)

  warp = _Warp(cam_w, cam_h)
  face = ModelFace(parser=Parser, frame_size=lambda w, h: warp.buf_bytes,
                   desire_len=ModelConstants.DESIRE_LEN, constants=ModelConstants,
                   lat_smooth_seconds=LAT_SMOOTH_SECONDS,
                   long_smooth_seconds=LONG_SMOOTH_SECONDS,
                   get_action_from_model=get_action_from_model)

  def connect(should_stop=None):
    if not tether_route_valid():
      raise ConnectionError('Android USB tether unavailable')
    client = JetlinkClient.open_tcp(HOST, PORT, deadline=3.0, name='xnor-modeld')
    try:
      hello = client.hello(timeout=5.0)
      if hello.get('loaded') != SHA256:
        raise RuntimeError('Tensor G6 does not have the validated LS2 model loaded')
      spec = client.ensure_engine(SHA256, NBYTES, frame_skip=4, build_timeout=20.0,
                                  should_stop=should_stop or (lambda: False))
      if spec.sha256 != SHA256 or spec.model_hw != MODEL_HW or spec.output_shapes.get('outputs') != (1, 18452):
        raise RuntimeError('Unexpected LS2 output specification')
      return client, spec
    except Exception:
      client.close()
      raise

  def build(client, spec):
    return JetlinkModelState(client, spec, warp, face=face, log=cloudlog,
                             event=lambda *args, **kwargs: None)

  return JoiningModelState(_Small(small_model), connect, build,
                           reset_small=lambda: _reset_small(small_model),
                           progress=_Progress(), log=cloudlog,
                           in_control=_Control.in_control,
                           name=lambda sha: 'LS2 ' + str(sha)[:12])


def handed_over(model) -> bool:
  count = getattr(model, 'handovers', None)
  if count is None:
    return False
  old = getattr(handed_over, '_count', None)
  handed_over._count = count
  return old is not None and old != count


def remote_active(model) -> bool:
  return bool(getattr(model, 'chestnut', False))
