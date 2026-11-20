#!/usr/bin/env python3
"""Non-publishing *real* parked LS2 handover test using live cameras + a real local model.

Runs in its own process. Does not replace manager modeld, publish modelV2,
write openpilot Params, or enable the remote vehicle-control authority gate.
The custom Pixel APK and V5.9.2 Zoompilot JoiningModelState do real work.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from types import SimpleNamespace

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
os.environ.setdefault('OMP_NUM_THREADS', '1')

import numpy as np
import cereal.messaging as messaging
from cereal import log, car
from msgq.visionipc import VisionIpcClient, VisionStreamType
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.common.transformations.camera import DEVICE_CAMERAS
from openpilot.common.transformations.model import get_warp_matrix
from openpilot import jetlink_authority as jet
from openpilot.jetlink_shadow_bench import camera_frames, parked
from jetlink.comma import gadget

RESULT_DIR = Path('/data/jetlink_v592_park_test')
STOP = False


def stop(_signal, _frame):
  global STOP
  STOP = True


def emit(event, **fields):
  rec = {'event': event, 'time': round(time.monotonic(), 3), **fields}
  print(json.dumps(rec, default=str, allow_nan=False), flush=True)


class StateNotReady(RuntimeError):
  """A subscription has not yet received current, valid vehicle-state data."""


def require_parked(sm) -> None:
  """Fail closed on stale messages, motion, invalid CAN, gear or controls."""
  sm.update(0)
  names = ['deviceState', 'carState', 'carControl', 'selfdriveState',
           'roadCameraState', 'liveCalibration']
  missing = [n for n in names if not sm.seen.get(n, False) or not sm.alive.get(n, False)
             or not sm.valid.get(n, False)]
  if missing:
    raise StateNotReady(f'Live vehicle-state prerequisite missing/stale: {missing}')
  ok, reason = parked(sm)
  if not ok:
    raise RuntimeError(f'Park/controls guard: {reason}')
  if not bool(sm['carState'].canValid):
    raise RuntimeError('Vehicle CAN is invalid; parked handover test stopped')
  if not bool(sm['deviceState'].started):
    raise RuntimeError('Comma is offroad, no live onroad cameras')
  if any((jet.ARM.exists(), jet.QUALIFIED.exists())):
    raise RuntimeError('Remote authority marker unexpectedly present; stop isolated test')


def await_fresh_parked_state(sm, timeout_s: float, phase: str):
  """Resubscribe patiently after blocking camera/model startup, never bypassing safety checks.

  Only StateNotReady is retryable. Motion, gear, engagement, invalid CAN,
  an offroad transition and authority-marker presence fail immediately.
  The frame loop below still calls require_parked for *each live frame*.
  """
  until = time.monotonic() + timeout_s
  last_error = None
  while not STOP:
    try:
      require_parked(sm)
      emit('vehicle_state_verified', phase=phase, gear=str(sm['carState'].gearShifter),
           speed=float(sm['carState'].vEgo), can_valid=True)
      return
    except StateNotReady as exc:
      last_error = exc
      if time.monotonic() >= until:
        raise RuntimeError(f'{phase}: {last_error} after {timeout_s:g}s') from exc
      sm.update(100)  # await messages rather than treating a single poll as definitive
  raise RuntimeError('Parked handover test interrupted')


def identify_runner() -> str:
  found = []
  for path in Path('/proc').glob('[0-9]*/cmdline'):
    try:
      if int(path.parent.name) == os.getpid():
        continue
      cmd = path.read_bytes().replace(b'\x00', b' ').decode(errors='replace')
    except (OSError, ValueError):
      continue
    if 'sunnypilot/modeld_v2/modeld.py' in cmd or 'selfdrive.modeld.modeld_tinygrad' in cmd:
      found.append('tinygrad')
    elif 'selfdrive/modeld/modeld.py' in cmd or ' -m selfdrive.modeld.modeld' in cmd:
      found.append('stock')
  if len(set(found)) != 1:
    raise RuntimeError('Cannot uniquely identify running modeld. Use --runner stock or --runner tinygrad')
  return found[0]


def select_local_model(runner, w, h):
  if runner == 'stock':
    from openpilot.selfdrive.modeld.modeld import ModelState
    return ModelState()
  from openpilot.sunnypilot.modeld_v2.modeld import ModelState
  return ModelState(cam_w=w, cam_h=h)


def build_inputs(model, runner, sm, cp):
  length = model.constants.DESIRE_LEN if runner == 'tinygrad' else 8
  desire = np.zeros(length, dtype=np.float32)
  desire[0] = 1.0
  is_rhd = bool(sm['driverMonitoringState'].isRHD) if sm.seen.get('driverMonitoringState', False) else False
  traffic = np.array([not is_rhd, is_rhd], dtype=np.float32)
  live_lat = float(sm['liveDelay'].lateralDelay) if sm.seen.get('liveDelay', False) else 0.0
  lat_smooth = float(getattr(model, 'LAT_SMOOTH_SECONDS', 0.0)) if runner == 'tinygrad' else 0.0
  long_smooth = float(getattr(model, 'LONG_SMOOTH_SECONDS', 0.3))
  lat_delay = live_lat + lat_smooth
  long_delay = float(cp.longitudinalActuatorDelay) + long_smooth
  key = model.desire_key if runner == 'tinygrad' else 'desire_pulse'
  inputs = {key: desire, 'traffic_convention': traffic,
            'action_t': np.array([lat_delay + 1.5 * DT_MDL, long_delay + 1.5 * DT_MDL], dtype=np.float32)}
  if runner == 'tinygrad' and 'lateral_control_params' in model.numpy_inputs:
    inputs['lateral_control_params'] = np.array([0.0, lat_delay], dtype=np.float32)
  return inputs, lat_delay, long_delay


def live_transforms(sm, main_wide):
  calib = np.asarray(sm['liveCalibration'].rpyCalib, dtype=np.float32)
  if calib.size != 3 or not np.isfinite(calib).all():
    raise RuntimeError('Missing finite live camera calibration')
  dev = str(sm['deviceState'].deviceType)
  sensor = str(sm['roadCameraState'].sensor)
  dc = DEVICE_CAMERAS[(dev, sensor)]
  main = get_warp_matrix(calib, dc.ecam.intrinsics if main_wide else dc.fcam.intrinsics, False).astype(np.float32)
  wide = get_warp_matrix(calib, dc.ecam.intrinsics, True).astype(np.float32)
  if not (np.isfinite(main).all() and np.isfinite(wide).all()):
    raise RuntimeError('Nonfinite live camera warp')
  return main, wide


def validate_model_output(out):
  if not isinstance(out, dict):
    raise ValueError('Model returned no output dictionary')
  shapes = {'plan': (1, 33, 15), 'lane_lines': (1, 4, 33, 2),
            'lead': (1, 3, 6, 4), 'desire_state': (1, 8)}
  for name, shape in shapes.items():
    val = out.get(name)
    if not isinstance(val, np.ndarray) or val.shape != shape or not np.isfinite(val).all():
      raise ValueError(f'Invalid real model prediction {name}: {getattr(val, "shape", None)}')


def test(args):
  if jet.VERSION != '5.9.2-xnor-custom-pixel-dual-modeld':
    raise RuntimeError(f'Expected V5.9.2, installed {jet.VERSION!r}')
  if jet.enabled() or jet.ARM.exists() or jet.QUALIFIED.exists():
    raise RuntimeError('Authority markers must stay ABSENT during parked non-publishing validation')
  if not jet.tether_route_valid():
    raise RuntimeError('Pixel USB tether unavailable')

  # We must not let the test's status record overwrite real modeld's runtime
  # record, consumed by selfdrived. This change is process-local.
  isolated_status = Path(f'/dev/shm/jetlink/modeld-park-test-{os.getpid()}.json')
  gadget.MODELD = isolated_status

  sm = messaging.SubMaster(['deviceState', 'carState', 'carControl', 'selfdriveState',
                            'roadCameraState', 'liveCalibration', 'liveDelay', 'driverMonitoringState'])
  await_fresh_parked_state(sm, args.state_timeout, phase='initial_vehicle_state')

  runner = identify_runner() if args.runner == 'auto' else args.runner
  active_streams = VisionIpcClient.available_streams('camerad', block=False)
  main_wide = VisionStreamType.VISION_STREAM_ROAD not in active_streams
  if not active_streams:
    raise RuntimeError('No live camerad streams; keep the car on but parked')
  if main_wide and VisionStreamType.VISION_STREAM_WIDE_ROAD not in active_streams:
    raise RuntimeError('No usable road or wide camera stream')

  # Reuse V5.8's validated VisionIPC *capture*, which reads actual live road
  # camera NV12 and enforces Park on every camera event. The V5.9.2 adapter
  # performs its OWN image warp and live model inference in this test.
  cam_args = SimpleNamespace(camera_timeout=args.state_timeout, onroad_shadow=False)
  camera = camera_frames(cam_args, {'input_shapes': {'new_img': [2, 6, 128, 256]}})
  first = next(camera)
  b0, b1 = first[3], first[4]
  emit('cameras_ready', runner=runner, road=[b0.width, b0.height], wide=[b1.width, b1.height])
  await_fresh_parked_state(sm, args.state_timeout, phase='after_camera_connection')

  emit('loading_real_local_model', runner=runner)
  local = select_local_model(runner, b0.width, b0.height)
  emit('local_model_loaded')
  await_fresh_parked_state(sm, args.state_timeout, phase='after_local_model_load')
  cp_bytes = Params().get('CarParams', block=False)
  if not cp_bytes:
    raise RuntimeError('CarParams unavailable; cannot use live actuator delays')
  cp = messaging.log_from_bytes(cp_bytes, car.CarParams)

  # Override only enabled() inside this non-publishing test process. Neither
  # marker is touched. The production modeld stays on its local model.
  saved = jet.enabled
  jet.enabled = lambda: True
  try:
    model = jet.attach(local, b0.width, b0.height, runner=runner)
  finally:
    jet.enabled = saved
  if not hasattr(model, 'big_model_state'):
    raise RuntimeError('Zoompilot joiner did not attach to real local model')

  stage = None
  local_frames = remote_frames = fallback_frames = 0
  saw_ready = saw_big = lost_link = saw_fallback = False
  started = time.monotonic()
  last_frame = None
  try:
    for event in camera:
      if STOP or time.monotonic() - started > args.seconds:
        break
      require_parked(sm)
      _, _, received_ns, main_buf, wide_buf, _, _, rhd, skew = event
      if skew > 40:
        continue
      if time.monotonic_ns() - received_ns > 200_000_000:
        continue
      main_mat, wide_mat = live_transforms(sm, main_wide)
      keys = model.vision_input_names
      bufs = {n: wide_buf if 'big' in n else main_buf for n in keys}
      transforms = {n: wide_mat if 'big' in n else main_mat for n in keys}
      inputs, lat_delay, long_delay = build_inputs(model, runner, sm, cp)
      if runner == 'tinygrad':
        # Exactly as the real runner does with liveDelay updates.
        model.lat_delay = lat_delay - float(model.LAT_SMOOTH_SECONDS)
      model.frame_drop_ratio = 0.0  # Test handover only; not latency qualification
      before = bool(jet.remote_active(model))
      output = model.run(bufs, transforms, inputs)
      if output is None:
        raise ValueError('Real model returned None during handover test')
      remote = bool(jet.remote_active(model))
      if remote:
        validate_model_output(output)
      elif not isinstance(output, dict) or 'plan' not in output:
        raise ValueError('Local model produced no driving plan')
      state = model.big_model_state
      if state != stage:
        emit('zoompilot_state', state=state, handovers=model.handovers)
        stage = state
      saw_ready |= state in ('ready', 'running') or model.big_model_available
      if remote:
        saw_big = True
        remote_frames += 1
        if remote_frames == 1:
          emit('remote_real_prediction_accepted', shape=list(output['plan'].shape), frame=event[0])
        # Call the same real action computation but don't publish the action.
        if runner == 'stock':
          from openpilot.selfdrive.modeld.modeld import get_action_from_model
          action = get_action_from_model(output, log.ModelDataV2.Action(), lat_delay + DT_MDL,
                                         long_delay + DT_MDL, 0.0)
        else:
          action = model.get_action_from_model(output, log.ModelDataV2.Action(),
                                               lat_delay + DT_MDL, long_delay + DT_MDL, 0.0)
        if not (math.isfinite(action.desiredCurvature) and math.isfinite(action.desiredAcceleration)):
          raise ValueError('Nonfinite remote LS2 action values')
        if remote_frames >= args.remote_frames and not lost_link:
          # Deliberately cut just this test process's TCP/UDP session.
          # The actual USB tether and production modeld are not touched.
          emit('closing_test_pixel_socket_to_verify_real_fallback')
          model._active.client.peer.close()
          lost_link = True
      else:
        local_frames += 1
        if saw_big and lost_link:
          fallback_frames += 1
          if not saw_fallback:
            emit('local_fallback_confirmed', frame=event[0], handovers=model.handovers)
            saw_fallback = True
          if fallback_frames >= 3:
            break
      last_frame = event[0]
      if local_frames % 20 == 1 and not saw_big:
        emit('waiting_for_live_swap', state=state, local_frames=local_frames,
             eligible=not model.in_control)
    result = {'passed': bool(saw_ready and saw_big and lost_link and saw_fallback),
              'runner': runner, 'ready': saw_ready, 'remote_frames': remote_frames,
              'local_frames': local_frames, 'fallback_frames': fallback_frames,
              'socket_loss_triggered': lost_link, 'fallback': saw_fallback,
              'handovers': model.handovers, 'last_frame': last_frame,
              'elapsed_s': round(time.monotonic() - started, 2),
              'authority_published': False, 'model_messages_published': 0}
    emit('result', **result)
    return 0 if result['passed'] else 2
  finally:
    model.close()
    try:
      isolated_status.unlink(missing_ok=True)
    except OSError:
      pass


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--runner', choices=('auto', 'stock', 'tinygrad'), default='auto')
  parser.add_argument('--seconds', type=float, default=75.0)
  parser.add_argument('--state-timeout', type=float, default=20.0)
  parser.add_argument('--remote-frames', type=int, default=3)
  args = parser.parse_args()
  if not (20 <= args.seconds <= 120) or not (1 <= args.remote_frames <= 10):
    parser.error('Require 20..120 seconds and 1..10 remote frames')
  for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, stop)
  try:
    return test(args)
  except Exception as exc:
    emit('failed', error=f'{type(exc).__name__}: {exc}')
    return 1


if __name__ == '__main__':
  sys.exit(main())
