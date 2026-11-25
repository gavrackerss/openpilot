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

from pathlib import Path
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


def discover_phone() -> str:
  """Get the current Android USB tether gateway; never use stale HOST as peer."""
  from openpilot.jetlink_usb_peer import discover_ls2_host
  return discover_ls2_host()


def tether_route_valid() -> bool:
  try:
    discover_phone()
    return True
  except (OSError, ConnectionError, ValueError):
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
  """Adapt XNOR's resident local model to Zoompilot's joining contract.

  The confirmed 0.11.1 runner used run(..., prepare_only). XNOR 0.11.2's
  stock ModelState uses run(bufs, transforms, inputs). Keep the Jetlink
  joining API unchanged and adapt only this local-model call boundary.
  """

  def __init__(self, model, *, normalize_stock_views=False):
    object.__setattr__(self, "_model", model)
    object.__setattr__(
      self,
      "_v0112",
      hasattr(model, "input_queues") and hasattr(model, "npy") and not hasattr(model, "img_queues"),
    )
    # Retain the final 0.11.1 queue-view repair for old-model regression tests
    # and legacy tooling. Native 0.11.2 does not use img_queues.
    object.__setattr__(
      self,
      "_queue_view_templates",
      {name: tensor.uop for name, tensor in model.img_queues.items()}
      if normalize_stock_views and not self._v0112 else {},
    )

  def _normalize_img_queue_views(self):
    for name, view in self._queue_view_templates.items():
      tensor = self._model.img_queues[name]
      if tuple(tensor.shape) != tuple(view.shape) or tensor.dtype != view.dtype:
        raise RuntimeError(f"Stock local image queue changed shape/dtype: {name}")
      if view.op.name != "RESHAPE" or len(view.src) != 2:
        raise RuntimeError(f"Unexpected stock TinyJit queue view: {name} {view.op} ({len(view.src)} sources)")
      original_buffer = tensor.uop.base
      if original_buffer.buffer is None or not original_buffer.buffer.is_initialized():
        raise RuntimeError(f"Uninitialized image queue buffer: {name}")
      tensor.uop = view.replace(src=(original_buffer, view.src[1]))

  def __getattr__(self, name):
    return getattr(self._model, name)

  def __setattr__(self, name, value):
    setattr(self._model, name, value)

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    if self._v0112:
      value = self._model.run(bufs, transforms, inputs)
    else:
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
    self._calibrations: list[np.ndarray | None] = [None, None]
    self._maps: list[np.ndarray | None] = [None, None]
    self._dimensions: list[tuple[int, int] | None] = [None, None]

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
      calibration = self._calibrations[i]
      if calibration is None or self._dimensions[i] != dims or not np.array_equal(calibration, matrix):
        indexes = warp_indices(cam_w, cam_h, MODEL_HW[0] * 2, MODEL_HW[1] * 2, matrix)
        if indexes.size != self.output[i].size or indexes.min() < 0 or indexes.max() >= src.size:
          raise ValueError('Pixel camera warp is outside the source frame')
        self._maps[i] = indexes
        self._calibrations[i] = matrix.copy()
        self._dimensions[i] = dims
      indexes = self._maps[i]
      if indexes is None:
        raise RuntimeError('Pixel camera warp map was not prepared')
      if indexes.max() >= src.size:
        raise ValueError('Pixel camera frame size changed during driving')
      np.take(src, indexes, out=self.output[i].reshape(-1))
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
      from openpilot.cereal import messaging
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
  """Zero the resident local model history in-place before LS2 fallback.

  0.11.2 stores recurrent queues in input_queues/npy instead of the final
  0.11.1 img_queues/numpy_inputs/full_input_queues layout. Do not rebuild the
  model or replace queue Tensor objects on the realtime fallback frame.
  """
  if hasattr(small, "input_queues") and hasattr(small, "npy"):
    for array in small.npy.values():
      array.fill(0)
    for name in ("img_q", "big_img_q", "feat_q", "desire_q"):
      tensor = small.input_queues.get(name)
      if tensor is None:
        continue
      buf = tensor.uop.base.buffer
      if buf is None or not buf.is_initialized():
        raise RuntimeError(f"Local 0.11.2 history buffer not initialized: {name}")
      buf.copyin(memoryview(bytearray(buf.nbytes)))
    small.prev_desire.fill(0)
    if hasattr(small, "full_frames"):
      small.full_frames.clear()
    if hasattr(small, "_blob_cache"):
      small._blob_cache.clear()
    return

  # Exact final 0.11.1 reset path.
  for array in small.numpy_inputs.values():
    array.fill(0)
  for tensor in small.img_queues.values():
    buf = tensor.uop.base.buffer
    if not buf.is_initialized():
      raise RuntimeError("Local image history GPU buffer not initialized during fallback")
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
  if small is not None and getattr(model, "_active", None) is small:
    if getattr(small, "_v0112", False):
      return small._model.run(bufs, transforms, inputs)
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
