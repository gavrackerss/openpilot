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

VERSION = '5.9.2-xnor-custom-pixel-dual-modeld'
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
  def __init__(self, model):
    object.__setattr__(self, '_model', model)

  def __getattr__(self, name):
    return getattr(self._model, name)

  def __setattr__(self, name, value):
    setattr(self._model, name, value)

  def run(self, bufs, transforms, inputs, after_enqueue=None):
    value = self._model.run(bufs, transforms, inputs, False)
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
  """Reset the already allocated local recurrent buffers after fallback."""
  for a in small.numpy_inputs.values():
    a.fill(0)
  for t in small.img_queues.values():
    t.assign(0).realize()
  small.full_input_queues.reset()
  small.prev_desire.fill(0)


def attach(small_model, cam_w: int, cam_h: int, *, runner: str = 'stock'):
  """Connect the custom APK to upstream Zoompilot's joining/fallback model."""
  if not enabled():
    return small_model
  if runner not in ('stock', 'tinygrad'):
    raise ValueError(f'Unsupported local runner: {runner}')

  from jetlink.openpilot.joining import JoiningModelState
  from jetlink.openpilot.model_state import JetlinkModelState, Trips
  from openpilot.jetlink_shadow_bench import (JetlinkTCP, model_hw, output_reply_count,
                                               packed_scalars)
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  # LS2's action semantics are independent of the selected local bundle.
  # Reuse the stock LS2 plan-to-action implementation for the remote output.
  ls2_action = None
  if runner == 'tinygrad':
    from openpilot.selfdrive.modeld.modeld import get_action_from_model as ls2_action

  warp = _CameraWarp(cam_w, cam_h)

  def connect(should_stop=None):
    if should_stop is not None and should_stop():
      raise InterruptedError('Zoompilot join stopped')
    if not tether_route_valid():
      raise ConnectionError('USB tether lost while connecting to custom XNOR APK')
    peer = JetlinkTCP(HOST, PORT, timeout=3.0)
    try:
      hello = peer.hello(usb_link=True)
      # This is the Android APK's proven UDP frame channel; don't silently
      # substitute stock Jetlink/TCP when the custom endpoint is absent.
      peer.enable_datagrams(hello)
      peer.udp_send_mode = 'sendmmsg'
      spec = peer.load_existing(wait=20.0)
      validate_spec(spec)
      if tuple(model_hw(spec)) != MODEL_HW or output_reply_count(spec) != REPLY_FLOATS:
        raise ValueError('Pixel model geometry or reply length mismatch')
      return _PixelSession(peer, spec), spec
    except Exception:
      peer.close()
      raise

  class _Remote(JetlinkModelState):
    """XNOR framing/output atop Zoompilot's tested hold/demotion policy.

    Reuses JetlinkModelState._note_hold rather than introducing a separate
    fallback threshold or selector. JoiningModelState owns all handovers.
    """
    def __init__(self, client: _PixelSession, spec: dict):
      self.client, self._manifest = client, spec
      # Zoompilot runtime records read spec.sha256 and checkpoint as attributes.
      self.spec = SimpleNamespace(sha256=MODEL_SHA, checkpoint="XNOR Tensor G6 LS2")
      self.chestnut = True
      self.lat_delay = 0.0
      self.PLANPLUS_CONTROL = 1.0
      # During the swap, modeld has already assembled the current frame's
      # buffers using the *local* bundle's keys. Keep those keys stable.
      self.vision_input_names = list(getattr(small_model, 'vision_input_names', ('img', 'big_img')))
      self.constants = getattr(small_model, 'constants', None)
      self.LAT_SMOOTH_SECONDS = getattr(small_model, 'LAT_SMOOTH_SECONDS', 0.)
      self.LONG_SMOOTH_SECONDS = getattr(small_model, 'LONG_SMOOTH_SECONDS', 0.)
      if ls2_action is not None:
        self.get_action_from_model = ls2_action
      self.parser = Parser()
      self.prev_desire = np.zeros(8, dtype=np.float32)
      self._frame_id = 0
      self._holds = deque()
      self._holds_in_a_row = 0
      self.behind = None
      self.held = False
      self.trips = Trips()
      self._last = None
      self._last_frame = 0
      self._last_received_ns = 0
      self.client.activate()

    def _get_recent(self, wait_s: float):
      pipe = self.client.pipeline
      try:
        pipe.expire_udp(older_than_s=PENDING_EXPIRE_S)
        if pipe.error is not None:
          raise RuntimeError('Custom XNOR reply thread failed') from pipe.error
        results = pipe.drain()
        if not results and wait_s > 0:
          try:
            results = [pipe.results.get(timeout=wait_s)] + pipe.drain()
          except queue.Empty:
            results = []
        for item, timing, received_ns in results:
          frame = int(item['frame_id'])
          vector = timing.get('_output')
          if vector is None:
            raise ValueError('Custom Pixel reply missing raw predictions')
          parsed = extract_prediction_heads(vector, self._manifest, self.parser)
          if frame > self._last_frame:
            self._last = parsed
            self._last_frame = frame
            self._last_received_ns = received_ns
        if pipe.error is not None:
          raise RuntimeError('Custom XNOR reply thread stopped') from pipe.error
      except Exception:
        self.client.dead = True
        raise
      if self._last is None:
        return None
      age_ns = time.perf_counter_ns() - self._last_received_ns
      frame_lag = self._frame_id - self._last_frame
      if age_ns < 0 or age_ns > MAX_REPLY_AGE_NS or not 0 <= frame_lag <= MAX_REPLY_FRAME_LAG:
        return None
      return self._last

    def run(self, bufs, transforms, inputs, after_enqueue=None):
      started = time.perf_counter()
      image = warp.run(bufs, transforms)
      # Stock modeld uses desire_pulse; selected tinygrad bundles may use
      # 'desire'. This only adapts input names, not remote output semantics.
      desire_key = next((k for k in inputs if k.startswith('desire')), None)
      if desire_key is None:
        raise ValueError('Local modeld did not supply a desire input')
      desire = np.asarray(inputs[desire_key], dtype=np.float32).reshape(-1)
      traffic = np.asarray(inputs['traffic_convention'], dtype=np.float32).reshape(-1)
      action_t = np.asarray(inputs['action_t'], dtype=np.float32).reshape(-1)
      if desire.size != 8 or traffic.size != 2 or action_t.size != 2:
        raise ValueError('Modeld scalar input dimensions differ from Pixel LS2')
      # Keep the same zeroed 'none' desire bit as stock ModelState.run().
      desire[0] = 0.0
      pulse = np.where(desire - self.prev_desire > 0.99, desire, 0.0)
      self.prev_desire[:] = desire
      packed = np.concatenate((pulse, traffic, action_t)).astype(np.float32)
      self._frame_id += 1
      # Zoompilot publishes the small model from the previous frame on a
      # transient late reply; our input still enters the Pixel's recurrent
      # queue at 20 Hz, with at most two outstanding requests.
      sent = self.client.pipeline.send_frame({'frame_id': self._frame_id}, image,
                                              packed, reset=(self._frame_id == 1))
      if after_enqueue is not None:
        after_enqueue()
      candidate = self._get_recent(WAIT_FIRST_S if self._last is None else WAIT_FRAME_S)
      if candidate is None:
        if self._last is None:
          raise TimeoutError('Pixel did not return its first LS2 prediction in deadline')
        # Stale data must not be published as a fresh driving plan.
        raise TimeoutError('Pixel prediction exceeded frame-age/deadline budget')
      self.held = self._last_frame < self._frame_id or not sent
      self.behind = self._note_hold(self.held)  # Zoompilot's own fallback rule
      self.trips.record(time.perf_counter() - started, 0)
      if self.behind:
        self.client.dead = True
      return candidate

    def close(self):
      self.client.close()

  def build(client: _PixelSession, spec: dict):
    return _Remote(client, spec)

  # No other server, USB gadget owner, or separate publish service is added.
  cloudlog.warning('xnor-jetlink: %s local model with custom Pixel join staged; Zoompilot owns handover', runner)
  return JoiningModelState(_Small(small_model), connect, build,
                           reset_small=lambda: _reset_small(small_model),
                           progress=_Progress(), log=cloudlog,
                           in_control=_Control.in_control,
                           name=lambda sha: 'XNOR LS2 Tensor G6')


def handed_over(model) -> bool:
  count = getattr(model, 'handovers', None)
  if count is None:
    return False
  previous = getattr(handed_over, '_count', None)
  handed_over._count = count
  return previous is not None and previous != count


def remote_active(model) -> bool:
  return bool(getattr(model, 'chestnut', False))
