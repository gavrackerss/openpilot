#!/usr/bin/env python3
"""XNOR V5.9 Pixel-app adapter for Zoompilot's model handover/fallback.

Transport and inference replies come from the *existing custom XNOR Android
Jetlink app*, via the tested V5.8 JetlinkTCP/PipelinedInference implementation.
Model selection, handover, demotion, frame-drop response and retry remain with
Zoompilot jetlink.openpilot.joining.JoiningModelState. This is not a separate
prediction publication path. The existing small model remains the fallback.

Experimental integration: authority remains disabled by default. A passing
unit test or successful shadow run does not qualify vehicle-control behavior.
"""
from __future__ import annotations

from collections import deque
from pathlib import Path
from types import SimpleNamespace
import queue
import subprocess
import threading
import time

import numpy as np

from openpilot.common.swaglog import cloudlog

VERSION = '5.9.3-xnor-native-jetlink-ls2'
MODEL_SHA = '909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8'
MODEL_NBYTES = 768749892
MODEL_HW = (128, 256)
HOST, PORT = '10.169.232.180', 5599
ARM = Path('/data/jetlink_xnor/AUTHORITY_ENABLED')
QUALIFIED = Path('/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED')

# Actual V5.8 phone response: 2,066 prediction floats followed by 2 padding
# floats. The 16,384-float hidden state is retained *on the phone*.
REPLY_FLOATS = 2068
PREDICTION_FLOATS = 2066
MAX_REPLY_AGE_NS = 120_000_000
MAX_REPLY_FRAME_LAG = 2
WAIT_FIRST_S = 0.085
WAIT_FRAME_S = 0.037
PENDING_EXPIRE_S = 0.17

# Verified from the V5.8 Android ENGINE_RESP saved in the real drive summaries.
OUTPUT_SLICES = {
  'lane_lines': (0, 528),
  'lane_lines_prob': (528, 536),
  'road_edges': (536, 800),
  'meta': (800, 855),
  'desire_pred': (855, 887),
  'pose': (887, 899),
  'wide_from_device_euler': (899, 905),
  'road_transform': (905, 917),
  'plan': (917, 1907),
  'lead': (1907, 2051),
  'lead_prob': (2051, 2054),
  'desire_state': (2054, 2062),
  # 'action' exists at 2062:2066, but the existing stock planner computes
  # the action from the parsed LS2 plan. Do not guess raw action semantics.
}


def tether_route_valid() -> bool:
  try:
    route = subprocess.run(['ip', 'route', 'get', HOST], check=True,
                           capture_output=True, text=True, timeout=1.0).stdout
    return ' dev usb0 ' in f' {route} '
  except (OSError, subprocess.SubprocessError):
    return False


def enabled() -> bool:
  # The CTM parked benchmark is a separate non-publishing process. Its marker
  # MUST NOT start the older LS2-only JoiningModelState: wrong model contract.
  # The local small model remains the only modeld source for this test.
  if Path('/data/jetlink_xnor/CTM_PARKED_TEST_ENABLED').exists():
    return False
  # Arm only the *joining mechanism*. It must keep retrying in the background
  # if the Pixel is plugged in AFTER modeld starts; the small model drives.
  # The actual transport route is checked in connect() on every attempt.
  return ARM.is_file() and QUALIFIED.is_file()


def validate_spec(spec: dict) -> None:
  """Refuse a different compiled model or an unexpected custom-APK schema."""
  if not isinstance(spec, dict) or spec.get('sha256') != MODEL_SHA:
    raise ValueError('Pixel model SHA is not the validated LS2 model')
  # ENGINE_RESP may omit nbytes; ENGINE_REQ already supplies the exact value.
  if spec.get('nbytes') is not None and int(spec['nbytes']) != MODEL_NBYTES:
    raise ValueError('Pixel LS2 model size changed')
  if int(spec.get('frame_skip', -1)) != 4:
    raise ValueError('Unexpected Pixel recurrent-state stride')
  shapes = spec.get('input_shapes') or {}
  if tuple(shapes.get('new_img', ())) != (2, 6, *MODEL_HW):
    raise ValueError('Unexpected Pixel LS2 image tensor')
  if tuple(shapes.get('desire', ())) != (8,):
    raise ValueError('Unexpected Pixel LS2 desire tensor')
  if tuple(shapes.get('action_t', ())) != (1, 2):
    raise ValueError('Unexpected Pixel LS2 action timing tensor')
  if tuple(shapes.get('traffic_convention', ())) != (1, 2):
    raise ValueError('Unexpected Pixel LS2 traffic-convention tensor')
  if tuple((spec.get('output_shapes') or {}).get('outputs', ())) != (1, 18452):
    raise ValueError('Unexpected Pixel LS2 model output length')
  slices = spec.get('output_slices') or {}
  for key, pair in OUTPUT_SLICES.items():
    if tuple(slices.get(key, ())) != pair:
      raise ValueError(f'Pixel output slice mismatch: {key}')
  if tuple(slices.get('action', ())) != (2062, 2066):
    raise ValueError('Pixel raw action head changed')
  if tuple(slices.get('hidden_state', ())) != (2066, 18450):
    raise ValueError('Pixel returned hidden state rather than phone-local history')
  if tuple(slices.get('pad', ())) != (18450, 18452):
    raise ValueError('Pixel output padding changed')


def extract_prediction_heads(vector: np.ndarray, spec: dict, parser) -> dict:
  """Decode the original custom-app 2068-float INFER_RESP with stock Parser."""
  x = np.asarray(vector, dtype=np.float32).reshape(-1)
  if x.size != REPLY_FLOATS or not np.isfinite(x).all():
    raise ValueError(f'Invalid Pixel LS2 output (expected {REPLY_FLOATS} floats)')
  if not isinstance(spec, dict):
    raise ValueError('Missing server model specification')
  out = {key: x[np.newaxis, start:stop].copy()
         for key, (start, stop) in OUTPUT_SLICES.items()}
  parsed = parser.parse_outputs(out)
  required = {
    'plan': (1, 33, 15),
    'plan_stds': (1, 33, 15),
    'lane_lines': (1, 4, 33, 2),
    'road_edges': (1, 2, 33, 2),
    'lead': (1, 3, 6, 4),
    'lead_prob': (1, 3),
    'desire_state': (1, 8),
  }
  for key, shape in required.items():
    value = parsed.get(key)
    if not isinstance(value, np.ndarray) or value.shape != shape or not np.isfinite(value).all():
      raise ValueError(f'Invalid decoded Pixel prediction: {key}')
  if any(not np.isfinite(a).all() for a in parsed.values() if isinstance(a, np.ndarray)):
    raise ValueError('Pixel prediction contains nonfinite parsed values')
  return parsed


class _Progress:
  def report(self, stage, frac, message, **kwargs):
    cloudlog.info('xnor-jetlink: %s %s', stage, message)

  def clear(self):
    return None


class _Small:
  """Map existing split-model run(...,prepare_only) onto Zoompilot's model."""
  def __init__(self, model, *, normalize_stock_views=False):
    object.__setattr__(self, '_model', model)
    # The pickled stock update_imgs TinyJit requires RESHAPE(BUFFER) queue
    # arguments. After the first real warp, ModelState.run() stores the
    # returned NOOP(BUFFER) tensors. The next local frame (including a
    # remote-to-local fallback) then raises a TinyJit input-info mismatch.
    # Save the ORIGINAL view layout, not its original buffer allocation.
    object.__setattr__(self, '_queue_view_templates',
                       {name: tensor.uop for name, tensor in model.img_queues.items()}
                       if normalize_stock_views else {})

  def _normalize_img_queue_views(self):
    # Restore only the UOp VIEW, wrapping the NEW, already-realized buffer.
    # No Tensor.assign(), allocation, GPU memory copy, or JIT reset occurs.
    # This runs after a successful local inference or prepare-only update,
    # before a subsequent invocation of the precompiled image-warp TinyJit.
    for name, view in self._queue_view_templates.items():
      tensor = self._model.img_queues[name]
      if tuple(tensor.shape) != tuple(view.shape) or tensor.dtype != view.dtype:
        raise RuntimeError(f'Stock local image queue changed shape/dtype: {name}')
      # On this Tinygrad version RESHAPE has TWO source UOps:
      # the underlying image queue buffer and the symbolic shape descriptor.
      # The latter must be preserved to match update_imgs' captured TinyJit
      # expected_input_info (RESHAPE(NOOP, VCONST)).
      if view.op.name != 'RESHAPE' or len(view.src) != 2:
        raise RuntimeError(f'Unexpected stock TinyJit queue view: {name} {view.op} ({len(view.src)} sources)')
      original_buffer = tensor.uop.base
      if original_buffer.buffer is None or not original_buffer.buffer.is_initialized():
        raise RuntimeError(f'Uninitialized image queue buffer: {name}')
      # Keep the NEW realized GPU allocation and its contents, but restore
      # the original RESHAPE operation and its identical shape descriptor.
      # Do not mutate original_buffer or invoke Tensor.assign/realize here.
      tensor.uop = view.replace(src=(original_buffer, view.src[1]))

  def __getattr__(self, name):
    return getattr(self._model, name)

  def __setattr__(self, name, value):
    setattr(self._model, name, value)

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    value = self._model.run(bufs, transforms, inputs, False)
    self._normalize_img_queue_views()
    if after_enqueue is not None:
      after_enqueue()
    return value


class _CameraWarp:
  """Use precisely V5.8's NV12 warp_indices with per-buffer dimensions."""
  def __init__(self, main_w: int, main_h: int):
    self.main_w, self.main_h = int(main_w), int(main_h)
    self.output = np.empty((2, 6, *MODEL_HW), dtype=np.uint8)
    self._calibrations = [None, None]
    self._maps = [None, None]
    self._dimensions = [None, None]

  def run(self, bufs: dict, transforms: dict):
    from openpilot.jetlink_shadow_bench import warp_indices
    # Modeld_v2 bundles can use a different road image key (e.g. new_img);
    # use the source camera designation, not a hardcoded network-input key.
    road = next((k for k in bufs if 'big' not in k), None)
    wide = next((k for k in bufs if 'big' in k), None)
    if road is None or wide is None or road not in transforms or wide not in transforms:
      raise ValueError('Both road and wide camera inputs are required')
    for i, name in enumerate((road, wide)):
      buf, matrix = bufs[name], np.asarray(transforms[name], dtype=np.float32)
      if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('Invalid modeld camera calibration')
      src = np.frombuffer(buf.data, dtype=np.uint8)
      cam_w, cam_h = int(getattr(buf, 'width', self.main_w)), int(getattr(buf, 'height', self.main_h))
      if cam_w <= 0 or cam_h <= 0:
        raise ValueError('Invalid modeld camera dimensions')
      dims = (cam_w, cam_h)
      if self._calibrations[i] is None or self._dimensions[i] != dims or not np.array_equal(self._calibrations[i], matrix):
        indexes = warp_indices(cam_w, cam_h, MODEL_HW[0] * 2, MODEL_HW[1] * 2, matrix)
        if indexes.size != self.output[i].size or indexes.min() < 0 or indexes.max() >= src.size:
          raise ValueError('Pixel camera warp is outside the source frame')
        self._maps[i] = indexes
        self._calibrations[i] = matrix.copy()
        self._dimensions[i] = dims
      if self._maps[i].max() >= src.size:
        raise ValueError('Pixel camera frame size changed during driving')
      np.take(src, self._maps[i], out=self.output[i].reshape(-1))
    return self.output


class _PixelSession:
  """Custom V5.8 TCP control/UDP upload/TCP reply session.

  A pipeline reader thread is created *in the joining background thread*,
  while Zoompilot keeps the small model authoritative. Its first receive is
  deferred until handover, so standby PING has exactly one socket reader.
  """
  def __init__(self, peer, spec: dict):
    from openpilot.jetlink_shadow_bench import PipelinedInference
    self.peer, self.spec = peer, spec
    self.dead = False
    self.active = False
    self._gate = threading.Event()
    self._lock = threading.RLock()
    self.pipeline = PipelinedInference(peer, depth=2, prediction_every=1)
    self.pipeline.worker = threading.Thread(target=self._reader_when_active,
                                            name='xnor-pixel-receiver', daemon=True)
    self.pipeline.start()

  def _reader_when_active(self):
    # This thread was started off modeld's realtime core, before handover.
    try:
      from jetlink.transport.priority import background_thread
      background_thread()
    except Exception:
      pass
    self._gate.wait()
    if not self.pipeline.stop_event.is_set():
      self.pipeline._reader()

  def ping(self, timeout=1.0):
    from openpilot.jetlink_shadow_bench import PING, PONG
    with self._lock:
      if self.active or self.dead:
        return 0.0
      before = time.perf_counter()
      seq = self.peer.send(PING)
      self.peer.expect(PONG, seq, timeout)
      return time.perf_counter() - before

  def activate(self):
    with self._lock:
      if self.dead:
        raise ConnectionError('Pixel session already closed')
      self.active = True
      self._gate.set()

  def leave(self, reason, **measured):
    # Closing a session rather than reusing a stream avoids cross-session
    # replies; Zoompilot's joining state performs the retry/backoff.
    self.dead = True

  def close(self):
    with self._lock:
      self.dead = True
      self._gate.set()
    self.pipeline.close()


class _Control:
  sm = None

  @staticmethod
  def in_control() -> bool:
    # Zoompilot never swaps while engaged or while control state is stale.
    try:
      import cereal.messaging as messaging
      sm = _Control.sm
      if sm is None:
        _Control.sm = sm = messaging.SubMaster(['carState', 'carControl', 'selfdriveState'])
      sm.update(0)
      services = ['carState', 'carControl', 'selfdriveState']
      return not (sm.all_alive(services) and sm.all_valid(services)) or bool(
        sm['carControl'].enabled or getattr(sm['carControl'], 'latActive', False)
        or sm['selfdriveState'].enabled or sm['selfdriveState'].active)
    except Exception:
      return True


def _reset_small(small):
  """Zero the local model's existing history without invalidating TinyJit inputs.

  Stock modeld's update_imgs is a precompiled TinyJit. Tensor.assign(0)
  rewrites the image queue's UOp (RESHAPE(NOOP) -> NOOP), even when the
  underlying GPU buffer remains the same. The next warp call then raises
  JitError: expected_input_info mismatch. Copying zero bytes into the
  ALREADY-ALLOCATED QCOM buffer leaves its UOp, shape and identity intact.

  QCOMAllocator._copyin synchronizes the GPU before writing the buffer.
  This is used only on demotion, not on every model frame.
  """
  for a in small.numpy_inputs.values():
    a.fill(0)
  for t in small.img_queues.values():
    buf = t.uop.base.buffer
    if not buf.is_initialized():
      raise RuntimeError('Local image history GPU buffer not initialized during fallback')
    # Buffer.copyin writes to the same GPU allocation. Crucially, it does
    # not mutate the Tensor.uop captured as an argument by update_imgs.
    buf.copyin(memoryview(bytearray(buf.nbytes)))
  small.full_input_queues.reset()
  small.prev_desire.fill(0)


def attach(small_model, cam_w: int, cam_h: int, *, runner: str = 'stock'):
  """Native JetlinkClient/JetlinkModelState; stock remains the fallback."""
  if not enabled():
    return small_model
  from openpilot.jetlink_native_authority import attach_native
  return attach_native(small_model, cam_w, cam_h, runner=runner)



def prepare_only(model, bufs, transforms, inputs):
  """Retain the local model's dropped-frame warp/queue update while joining.

  JoiningModelState.run does not expose the fork's prepare_only argument.
  When the resident small model is active, preserve ModelState.run(...,True)
  without evaluating an output. Do not run a separate local inference while
  the remote model is active (it owns the frame in that state).
  """
  small = getattr(model, '_small', None)
  if small is not None and getattr(model, '_active', None) is small:
    value = small._model.run(bufs, transforms, inputs, True)
    small._normalize_img_queue_views()
    return value
  return None


def handed_over(model) -> bool:
  count = getattr(model, 'handovers', None)
  if count is None:
    return False
  previous = getattr(handed_over, '_count', None)
  handed_over._count = count
  return previous is not None and previous != count


def remote_active(model) -> bool:
  return bool(getattr(model, 'chestnut', False))
