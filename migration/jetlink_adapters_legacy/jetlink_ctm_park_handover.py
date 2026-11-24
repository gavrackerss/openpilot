#!/usr/bin/env python3
"""Non-publishing *real* parked ORIGINAL CTM handover test using live cameras + a real local model.

Runs in its own process. Does not replace manager modeld, publish modelV2,
write openpilot Params, or enable the remote vehicle-control authority gate.
The signed Pixel v0.7 CTM APK and native Zoompilot JoiningModelState do real work.
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
from openpilot import jetlink_ctm_adapter as jet
from openpilot.jetlink_shadow_bench import parked
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
  """The read-only test has not received fresh *valid* state on a subscription."""
  def __init__(self, missing, details):
    self.missing = tuple(missing)
    self.details = details
    super().__init__(f'State unavailable/invalid: {details}')


# SubMaster.alive uses 10 / service frequency. At carState's 100 Hz this
# is just 100 ms; a live 20 Hz camera/GPU process can trip that repeatedly.
# Use explicit MONOTONIC last-message ages instead: never trust an unbounded
# cached value, and never infer Park from an absent or invalid message.
MAX_STATE_AGE_S = {
  'carState': 0.75,
  'carControl': 0.75,
  'selfdriveState': 0.75,
  'roadCameraState': 1.0,
  'liveCalibration': 3.0,  # normally 4 Hz
  'deviceState': 3.0,
}


def _service_issue(sm, name, calibration_snapshot=None):
  """Check message *age* and validity separately; keep vehicle guards strict.

  Only this isolated parked test may use a previously fresh VALID 3-angle
  calibration when a newer liveCalibration envelope is flagged invalid by
  calibrationd's upstream cameraOdometry/carState health check. In that case
  the service must still be sending fresh messages. No other invalid service
  is accepted, and no cached vehicle speed, gear or control state is allowed.
  """
  if not sm.seen.get(name, False):
    return 'not_received'
  rx_time = sm.recv_time.get(name, 0.)
  age = time.monotonic() - rx_time
  if rx_time <= 0 or age < 0 or age > MAX_STATE_AGE_S[name]:
    return 'stale'
  if not sm.valid.get(name, False):
    if name == 'liveCalibration' and calibration_snapshot is not None:
      return None  # Fresh message; only the previously validated angles are used
    return 'invalid_message'
  return None


def require_parked(sm, calibration_snapshot=None) -> None:
  """Read real vehicle state every time; optionally use validated parked RPY."""
  sm.update(0)

  # Abort immediately on *any observed* motion, gear change or engagement,
  # even if a slower ancillary service (liveCalibration at 4Hz) is invalid.
  if sm.seen.get('carState', False):
    ok, reason = parked(sm)
    if not ok:
      raise RuntimeError(f'Park/controls guard: {reason}')
    if not bool(sm['carState'].canValid):
      raise RuntimeError('Vehicle CAN is invalid; parked handover test stopped')
  if sm.seen.get('carControl', False) and (
      bool(sm['carControl'].enabled) or bool(getattr(sm['carControl'], 'latActive', False))):
    raise RuntimeError('Vehicle controls engaged; parked test stopped')
  if sm.seen.get('selfdriveState', False) and (
      bool(sm['selfdriveState'].enabled) or bool(sm['selfdriveState'].active)):
    raise RuntimeError('Openpilot engaged; parked test stopped')
  if sm.seen.get('deviceState', False) and not bool(sm['deviceState'].started):
    raise RuntimeError('Comma transitioned offroad; parked test stopped')
  if any((jet.ARM.exists(), jet.QUALIFIED.exists(), jet.CTM_QUALIFIED.exists(), jet.CTM_ARM.exists())):
    raise RuntimeError('CTM/LS2 authority marker unexpectedly present; stop isolated test')

  names = ['deviceState', 'carState', 'carControl', 'selfdriveState',
           'roadCameraState', 'liveCalibration']
  problems = {name: issue for name in names if (issue := _service_issue(sm, name, calibration_snapshot))}
  if problems:
    raise StateNotReady(list(problems), problems)


def await_fresh_parked_state(sm, timeout_s: float, phase: str, calibration_snapshot=None):
  """Resubscribe patiently after blocking camera/model startup, never bypassing safety checks.

  Only StateNotReady is retryable. Motion, gear, engagement, invalid CAN,
  an offroad transition and authority-marker presence fail immediately.
  The frame loop below still calls require_parked for *each live frame*.
  """
  until = time.monotonic() + timeout_s
  last_error = None
  previous = None
  while not STOP:
    try:
      require_parked(sm, calibration_snapshot)
      emit('vehicle_state_verified', phase=phase, gear=str(sm['carState'].gearShifter),
           speed=float(sm['carState'].vEgo), can_valid=True)
      return
    except StateNotReady as exc:
      last_error = exc
      if exc.details != previous:
        emit('waiting_for_valid_vehicle_state', phase=phase, issues=exc.details)
        previous = exc.details
      if time.monotonic() >= until:
        raise RuntimeError(f'{phase}: {last_error} after {timeout_s:g}s') from exc
      sm.update(100)  # allow 4Hz calibration to arrive before using a frame
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




def synchronized_live_camera_frames(state_sm, *, timeout_s: float):
  """Capture a *matched, current* road/wide camera pair, never replay backlog.

  The former test created VisionIPC subscribers BEFORE taking 5 seconds to load
  another local model. The road stream was conflated but wide was queued, hence
  the persistent 5700ms skew. This test needs only current frames, so both
  subscribers conflate; the older timestamp is advanced until aligned.
  Production modeld is not modified and remains the only vehicle publisher.
  """
  streams = VisionIpcClient.available_streams('camerad', block=False)
  if not streams:
    raise RuntimeError('No live camerad streams')
  main_wide = VisionStreamType.VISION_STREAM_ROAD not in streams
  main_stream = VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide else VisionStreamType.VISION_STREAM_ROAD
  both = VisionStreamType.VISION_STREAM_ROAD in streams and VisionStreamType.VISION_STREAM_WIDE_ROAD in streams
  # BOTH conflate to newest. A slow local inference must not create a wide
  # camera backlog that makes all subsequent pairs permanently several seconds
  # apart. Timestamp matching, not queue position, establishes synchrony.
  main = VisionIpcClient('camerad', main_stream, True)
  extra = VisionIpcClient('camerad', VisionStreamType.VISION_STREAM_WIDE_ROAD, True) if both else None
  clients = [(main, 'road'), (extra, 'wide')]
  connected_at = time.monotonic()
  for client, name in clients:
    if client is None:
      continue
    while not STOP and not client.connect(False):
      if time.monotonic() - connected_at > timeout_s:
        raise RuntimeError(f'Camera stream {name} connection timed out')
      time.sleep(.1)
  if STOP:
    return

  tolerance_ns = 25_000_000  # same 25ms allowance used by stock modeld pairing
  max_pair_reads = 80
  max_pair_seconds = 4.0
  max_image_age_ns = 200_000_000
  last_main_id = -1
  unmatched_since = time.monotonic()
  aged = 0
  paired = 0

  while not STOP:
    road_buf = main.recv()
    if road_buf is None:
      raise RuntimeError('Road stream ended')
    road_ts, road_eof, road_id = int(main.timestamp_sof), int(main.timestamp_eof), int(main.frame_id)
    wide_buf, wide_ts, wide_eof = road_buf, road_ts, road_eof
    road_advance = wide_advance = 0
    if extra is not None:
      wide_buf = extra.recv()
      if wide_buf is None:
        raise RuntimeError('Wide stream ended')
      wide_ts, wide_eof = int(extra.timestamp_sof), int(extra.timestamp_eof)
      initial_skew_ms = abs(road_ts-wide_ts)/1e6
      begun = time.monotonic()
      while abs(road_ts-wide_ts) > tolerance_ns and not STOP:
        if time.monotonic() - begun > max_pair_seconds or road_advance + wide_advance >= max_pair_reads:
          raise RuntimeError(
            'Unable to synchronize current camera frames: '
            f'initial_skew_ms={initial_skew_ms:.2f}, '
            f'current_skew_ms={abs(road_ts-wide_ts)/1e6:.2f}, '
            f'road_drained={road_advance}, wide_drained={wide_advance}')
        # Check vehicle movement/engagement even when pairing blocks.
        if (road_advance + wide_advance) % 8 == 0:
          state_sm.update(0)
          if state_sm.seen.get('carState', False):
            safe, reason = parked(state_sm)
            if not safe or not bool(state_sm['carState'].canValid):
              raise RuntimeError(f'Vehicle not stationary/valid while synchronizing: {reason}')
          if state_sm.seen.get('deviceState', False) and not bool(state_sm['deviceState'].started):
            raise RuntimeError('Comma transitioned offroad during camera pairing')
        if road_ts > wide_ts:
          wide_buf = extra.recv()
          if wide_buf is None:
            raise RuntimeError('Wide stream ended while synchronizing')
          wide_ts, wide_eof = int(extra.timestamp_sof), int(extra.timestamp_eof)
          wide_advance += 1
        else:
          road_buf = main.recv()
          if road_buf is None:
            raise RuntimeError('Road stream ended while synchronizing')
          road_ts, road_eof, road_id = int(main.timestamp_sof), int(main.timestamp_eof), int(main.frame_id)
          road_advance += 1
      if STOP:
        return
      if (road_advance + wide_advance) > 0 and paired == 0:
        emit('camera_pair_aligned', initial_skew_ms=round(initial_skew_ms, 3),
             aligned_skew_ms=round(abs(road_ts-wide_ts)/1e6, 3),
             skipped_road=road_advance, skipped_wide=wide_advance,
             frame=road_id)
    if road_id <= last_main_id:
      continue
    last_main_id = road_id
    # Check BOTH source capture times, not the receiver arrival timestamp.
    now_ns = time.monotonic_ns()
    ages = ((now_ns-road_eof)/1e6, (now_ns-wide_eof)/1e6)
    if any(not (-100. <= age <= 200.) for age in ages):
      aged += 1
      if aged <= 2 or aged % 20 == 0:
        emit('camera_source_not_current', frame=road_id,
             road_age_ms=round(ages[0],2), wide_age_ms=round(ages[1],2), count=aged)
      if time.monotonic() - unmatched_since > 8.0:
        raise RuntimeError('No current synchronized camera pair available after 8 seconds')
      continue
    paired += 1
    unmatched_since = time.monotonic()
    # Output tuple matches the CTM parked test's existing loop. The buffers
    # are the actual VisionIPC bytes used by the local model and Pixel adapter.
    yield (road_id, road_eof, time.monotonic_ns(), road_buf, wide_buf,
           None, None, False, abs(road_ts-wide_ts)/1e6)


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


def live_transforms(sm, main_wide, calibration_snapshot=None):
  calib = np.asarray(calibration_snapshot if calibration_snapshot is not None else sm['liveCalibration'].rpyCalib, dtype=np.float32)
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
  if jet.VERSION != 'ctm-v0.1-native-queued-parked-only':
    raise RuntimeError(f'CTM adapter revision mismatch: {jet.VERSION!r}')
  if jet.enabled() or jet.ARM.exists() or jet.QUALIFIED.exists() or jet.CTM_QUALIFIED.exists() or jet.CTM_ARM.exists():
    raise RuntimeError('All CTM and CTM authority markers must be absent during CTM parked handover')
  if not jet.PARKED.exists() or not jet.NOT_QUALIFIED.exists() or not jet.STAGED.exists():
    raise RuntimeError('Stage original CTM in parked-only mode first: bash tools/jetlink_ctm_markers.sh stage')
  if os.environ.get('JETLINK_CTM_ISOLATED_PARKED_TEST') != '1':
    raise RuntimeError('Only the isolated CTM parked handover entry point may invoke this test')
  if not jet.tether_route_valid():
    raise RuntimeError('Pixel USB tether unavailable')

  # We must not let the test's status record overwrite real modeld's runtime
  # record, consumed by selfdrived. This change is process-local.
  isolated_status = Path(f'/dev/shm/jetlink/modeld-park-test-{os.getpid()}.json')
  gadget.MODELD = isolated_status

  sm = messaging.SubMaster(['deviceState', 'carState', 'carControl', 'selfdriveState',
                            'roadCameraState', 'liveCalibration', 'liveDelay', 'driverMonitoringState'])
  await_fresh_parked_state(sm, args.state_timeout, phase='initial_vehicle_state')
  # Freeze only camera pose while the car stays Parked. This RPY came from
  # a freshly received, valid liveCalibration message; no guessed geometry.
  calibration_snapshot = np.asarray(sm['liveCalibration'].rpyCalib, dtype=np.float32).reshape(-1).copy()
  if calibration_snapshot.shape != (3,) or not np.isfinite(calibration_snapshot).all():
    raise RuntimeError('Initial valid camera calibration has unusable angles')
  emit('parked_calibration_snapshot_verified', rpy=calibration_snapshot.tolist())

  runner = identify_runner() if args.runner == 'auto' else args.runner
  active_streams = VisionIpcClient.available_streams('camerad', block=False)
  main_wide = VisionStreamType.VISION_STREAM_ROAD not in active_streams
  if not active_streams:
    raise RuntimeError('No live camerad streams; keep the car on but parked')
  if main_wide and VisionStreamType.VISION_STREAM_WIDE_ROAD not in active_streams:
    raise RuntimeError('No usable road or wide camera stream')

  # Subscribe just long enough to obtain the actual camera dimensions.
  # Close BEFORE loading the local model (which takes ~5s), otherwise queued
  # wide-camera frames become ~5s older than the conflated road frames.
  camera = synchronized_live_camera_frames(sm, timeout_s=args.state_timeout)
  first = next(camera)
  b0, b1 = first[3], first[4]
  cam_w, cam_h = b0.width, b0.height
  emit('cameras_ready', runner=runner, road=[cam_w, cam_h],
       wide=[b1.width, b1.height], first_pair_skew_ms=round(first[8], 3))
  camera.close()
  del first, b0, b1
  await_fresh_parked_state(sm, args.state_timeout, phase='after_camera_connection', calibration_snapshot=calibration_snapshot)

  emit('loading_real_local_model', runner=runner)
  local = select_local_model(runner, cam_w, cam_h)
  emit('local_model_loaded')
  await_fresh_parked_state(sm, args.state_timeout, phase='after_local_model_load', calibration_snapshot=calibration_snapshot)
  cp_bytes = Params().get('CarParams', block=False)
  if not cp_bytes:
    raise RuntimeError('CarParams unavailable; cannot use live actuator delays')
  cp = messaging.log_from_bytes(cp_bytes, car.CarParams)

  # Explicit isolated-test entry point: modeld never calls this and its
  # production authority path remains blocked. The adapter itself requires
  # staged CTM-not-qualified flags and this test process identity.
  os.environ['JETLINK_CTM_ISOLATED_PARKED_TEST'] = '1'
  model = jet.attach_parked(local, cam_w, cam_h, runner=runner)
  if not hasattr(model, 'big_model_state'):
    raise RuntimeError('Zoompilot joiner did not attach to real local model')

  stage = None
  local_frames = remote_frames = fallback_frames = 0
  saw_ready = saw_big = lost_link = saw_fallback = False
  started = time.monotonic()
  last_frame = None
  calibration_recoveries = 0
  critical_recoveries = 0
  invalid_calibration_envelopes = 0
  camera_too_old = 0
  camera_skew_skips = 0
  first_run_logged = False

  # The stand-alone parked test already validates the *same* live SubMaster
  # immediately before each inference frame. Feed that observed engagement
  # state to Zoompilot rather than opening a second high-frequency subscriber
  # with a separate, 100ms 'alive' threshold. This change is test-process-only.
  # If this process has not just verified fresh state, refuse the swap.
  last_verified_at = [0.0]
  def test_in_control():
    if STOP or time.monotonic() - last_verified_at[0] > 0.35:
      return True
    if any(_service_issue(sm, n, calibration_snapshot) for n in MAX_STATE_AGE_S):
      return True
    return (abs(float(sm['carState'].vEgo)) >= 0.2
            or str(sm['carState'].gearShifter).split('.')[-1].lower() not in ('park', 'p')
            or not bool(sm['carState'].canValid)
            or not bool(sm['deviceState'].started)
            or bool(sm['carControl'].enabled)
            or bool(getattr(sm['carControl'], 'latActive', False))
            or bool(sm['selfdriveState'].enabled)
            or bool(sm['selfdriveState'].active))
  model._ask_in_control = test_in_control

  # This NEW subscriber starts after model loading, so neither camera can
  # accumulate a several-second backlog during initialization.
  camera = synchronized_live_camera_frames(sm, timeout_s=args.state_timeout)
  try:
    first_live_pair = next(camera)
    emit('fresh_camera_pair_after_model_load', frame=first_live_pair[0],
         pair_skew_ms=round(first_live_pair[8], 3))
    del first_live_pair
    for event in camera:
      if STOP or time.monotonic() - started > args.seconds:
        break
      # Do not discard the camera frame just because the vehicle-state
      # subscriber needs a fresh message. The parked generator may produce
      # frames only every ~0.8 s (observed on this comma, while a second
      # model is resident); carState's 0.75 s age budget is slightly shorter.
      # Dropping that frame on each successful refresh caused infinite zero-
      # inference loops, and eventually the Pixel's idle keepalive timed out.
      try:
        require_parked(sm, calibration_snapshot)
      except StateNotReady as exc:
        critical = any(name != 'liveCalibration' for name in exc.missing)
        if critical:
          critical_recoveries += 1
          if critical_recoveries <= 3 or critical_recoveries % 10 == 0:
            emit('vehicle_state_refresh', count=critical_recoveries,
                 frame=event[0], issues=exc.details)
        else:
          calibration_recoveries += 1
          if calibration_recoveries <= 3 or calibration_recoveries % 10 == 0:
            emit('live_calibration_refresh', count=calibration_recoveries,
                 frame=event[0], issues=exc.details)
        await_fresh_parked_state(sm, min(args.state_timeout, 3.0),
                                 phase='critical_state_recovered' if critical else 'calibration_recovered',
                                 calibration_snapshot=calibration_snapshot)
        # Vehicle state is freshly verified NOW. We may use THIS SAME camera
        # frame if it is still recent and synchronised, otherwise skip it.
        # Never infer using old vehicle state or old camera buffers.
        if critical_recoveries == 1 or calibration_recoveries == 1:
          emit('parked_frame_state_recovered', frame=event[0],
               camera_age_ms=round((time.monotonic_ns()-event[2])/1e6,1))
      last_verified_at[0] = time.monotonic()
      # An invalid calibration envelope can be caused by calibrationd's
      # upstream health check. We use only the initial valid RPY, and only
      # while fresh Park/CAN/control checks continue to pass.
      if sm.seen.get('liveCalibration', False) and not sm.valid.get('liveCalibration', False):
        invalid_calibration_envelopes += 1
        if invalid_calibration_envelopes == 1:
          emit('using_verified_parked_calibration_snapshot', reason='fresh_calibration_envelope_invalid')
      _, _, received_ns, main_buf, wide_buf, _, _, rhd, skew = event
      if skew > 40:
        camera_skew_skips += 1
        if camera_skew_skips <= 2 or camera_skew_skips % 10 == 0:
          emit('camera_pair_skipped', count=camera_skew_skips, skew_ms=round(skew, 2), frame=event[0])
        continue
      age_ns = time.monotonic_ns() - received_ns
      if age_ns < 0 or age_ns > 200_000_000:
        camera_too_old += 1
        if camera_too_old <= 2 or camera_too_old % 10 == 0:
          emit('camera_age_skipped', count=camera_too_old, age_ms=round(age_ns/1e6, 1), frame=event[0])
        continue
      main_mat, wide_mat = live_transforms(sm, main_wide, calibration_snapshot)
      keys = model.vision_input_names
      bufs = {n: wide_buf if 'big' in n else main_buf for n in keys}
      transforms = {n: wide_mat if 'big' in n else main_mat for n in keys}
      inputs, lat_delay, long_delay = build_inputs(model, runner, sm, cp)
      if runner == 'tinygrad':
        # Exactly as the real runner does with liveDelay updates.
        model.lat_delay = lat_delay - float(model.LAT_SMOOTH_SECONDS)
      model.frame_drop_ratio = 0.0  # Test handover only; not latency qualification
      before = bool(jet.remote_active(model))
      if not first_run_logged:
        emit('first_real_local_inference_start', frame=event[0],
             camera_age_ms=round((time.monotonic_ns()-received_ns)/1e6, 1))
        first_run_logged = True
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
          raise ValueError('Nonfinite CTM-derived action values (not published)')
        if remote_frames >= args.remote_frames and not lost_link:
          # Deliberately cut just this test process's TCP/UDP session.
          # The actual USB tether and production modeld are not touched.
          emit('closing_test_pixel_socket_to_verify_real_fallback')
          model._active.client.t.close()
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
              'calibration_recoveries': calibration_recoveries,
              'vehicle_state_refreshes': critical_recoveries,
              'invalid_calibration_envelopes': invalid_calibration_envelopes,
              'camera_age_skips': camera_too_old, 'camera_pair_skips': camera_skew_skips,
              'handovers': model.handovers, 'last_frame': last_frame,
              'elapsed_s': round(time.monotonic() - started, 2),
              'authority_published': False, 'model_messages_published': 0}
    emit('result', **result)
    return 0 if result['passed'] else 2
  finally:
    camera.close()
    model.close()
    try:
      isolated_status.unlink(missing_ok=True)
    except OSError:
      pass


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--runner', choices=('auto', 'stock'), default='stock')
  parser.add_argument('--seconds', type=float, default=75.0)
  parser.add_argument('--state-timeout', type=float, default=20.0)
  parser.add_argument('--remote-frames', type=int, default=3)
  args = parser.parse_args()
  os.environ['JETLINK_CTM_ISOLATED_PARKED_TEST'] = '1'
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
