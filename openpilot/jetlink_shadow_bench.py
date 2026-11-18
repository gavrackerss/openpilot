#!/usr/bin/env python3
"""XNOR Jetlink connected shadow/latency benchmark, v2.

Subscribes to camerad in a separate, non-realtime process; never imports/patches
modeld, publishes messages, accesses CAN, or writes openpilot Params. Synthetic
mode tests the live link and LS2 NPU without camera. Camera mode uses the live
NV12 road/wide frames and the same nearest-neighbour planar YUV layout as
openpilot's compile_warp.py. The CPU implementation is *not* proven numerically
identical to the tinygrad warp; DO NOT use its outputs for driving.

Only run while stationary, in Park, without driving controls engaged.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import queue
import threading
import os
from pathlib import Path
import secrets
import signal
import socket
import struct
import subprocess
import sys
import time
from datetime import datetime, timezone

import numpy as np

VERSION = '2.0'
SHA = '909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8'
NBYTES = 768749892
HEADER = struct.Struct('<IHHIIIQ4x')
INFER_HEADER = struct.Struct('<II')
INFER_REPLY = struct.Struct('<IIIII')
MAGIC = 0x4B4E4C4A
PROTOCOL = 3
HELLO_REQ, HELLO_RESP = 1, 2
ENGINE_REQ, ENGINE_RESP, PROGRESS = 3, 4, 7
INFER_REQ, INFER_RESP, ERROR = 8, 9, 14
PING, PONG, LEAVE = 15, 16, 19
PADDED = 1 << 7
RESET_QUEUES = 1
MAX_BODY = 16 * 1024 * 1024
RUNNING = True


def stop_now(_sig, _frame):
  global RUNNING
  RUNNING = False


def bounded_recv(sock: socket.socket, n: int) -> bytes:
  chunks = bytearray(n)
  view = memoryview(chunks)
  got = 0
  while got < n:
    count = sock.recv_into(view[got:])
    if count == 0:
      raise ConnectionError('Jetlink server disconnected during receive')
    got += count
  return bytes(chunks)


class JetlinkTCP:
  """Small self-contained Jetlink v3 TCP transport; no gadget, no CAN hooks."""
  def __init__(self, host: str, port: int, timeout: float):
    self.sock = socket.create_connection((host, port), timeout=timeout)
    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    self.sock.settimeout(timeout)
    self.seq = 0
    self.spec = None
    self.host = host
    self.port = port

  def send(self, kind: int, payload=b'', flags: int = 0):
    self.seq += 1
    seq = self.seq
    parts = [memoryview(p).cast('B') for p in payload] if isinstance(payload, (tuple, list)) else [memoryview(payload).cast('B')]
    n = sum(p.nbytes for p in parts)
    if (HEADER.size + n) % 512 == 0:
      flags |= PADDED
      parts.append(memoryview(b'\x00'))
    head = memoryview(HEADER.pack(MAGIC, PROTOCOL, kind, seq, flags, n, 0))
    pending = [head] + parts
    while pending:
      # sendmsg gathers the 32-byte header + image + scalar buffers in one syscall.
      sent = self.sock.sendmsg(pending)
      if sent <= 0:
        raise ConnectionError('Jetlink send failed')
      while pending and sent >= len(pending[0]):
        sent -= len(pending[0])
        pending.pop(0)
      if sent and pending:
        pending[0] = pending[0][sent:]
    return seq

  def recv(self):
    h = bounded_recv(self.sock, HEADER.size)
    magic, version, kind, seq, flags, length, _ = HEADER.unpack(h)
    if magic != MAGIC or version != PROTOCOL:
      raise ValueError('Jetlink v3 header mismatch')
    if length > MAX_BODY:
      raise ValueError(f'Refusing Jetlink payload length {length}')
    raw = bounded_recv(self.sock, length + (1 if flags & PADDED else 0))
    return kind, seq, flags, raw[:length]

  def expect(self, expected: int, seq: int, timeout: float):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
      self.sock.settimeout(max(0.1, deadline - time.monotonic()))
      kind, got, flags, payload = self.recv()
      if kind == ERROR:
        try:
          e = json.loads(payload)
        except Exception:
          raise RuntimeError(f'Server error: {payload[:200]!r}')
        raise RuntimeError(f"Jetlink error: {e.get('error')}: {e.get('detail')}")
      if kind == expected and got == seq:
        return payload
      if kind == PROGRESS:
        continue
      if kind == ENGINE_RESP and expected == ENGINE_RESP:
        return payload  # server may send asynchronous engine state updates
      raise RuntimeError(f'Unexpected Jetlink reply type={kind} seq={got}; wanted={expected}/{seq}')
    raise TimeoutError(f'Jetlink {expected} timed out')

  def hello(self):
    client = {'name': 'xnor-connected-shadow-v1', 'nonce': secrets.token_hex(4), 'link': {'kind': 'tcp'}}
    seq = self.send(HELLO_REQ, json.dumps({'client': client}).encode())
    return json.loads(self.expect(HELLO_RESP, seq, 5.0))

  def load_existing(self, wait: float = 90.0):
    # Request the exact compiled LS2 engine already prepared on Tensor G6;
    # no upload, download or alternative model is allowed.
    obj = {'sha256': SHA, 'nbytes': NBYTES, 'frame_skip': 4}
    seq = self.send(ENGINE_REQ, json.dumps(obj).encode())
    state = json.loads(self.expect(ENGINE_RESP, seq, 20.0))
    deadline = time.monotonic() + wait
    while state.get('state') not in ('ready', 'failed', 'need_upload') and time.monotonic() < deadline:
      self.sock.settimeout(min(2.0, max(.1, deadline - time.monotonic())))
      try:
        kind, _, _, raw = self.recv()
      except socket.timeout:
        continue
      if kind == ERROR:
        e = json.loads(raw)
        raise RuntimeError(f"Engine error: {e.get('error')}: {e.get('detail')}")
      if kind == ENGINE_RESP:
        state = json.loads(raw)
    if state.get('state') != 'ready':
      raise RuntimeError(f"LS2 engine unavailable: {state.get('state')} / {state.get('detail')}")
    spec = state.get('spec')
    if not spec or spec.get('sha256') != SHA:
      raise RuntimeError('LS2 server spec hash mismatch or absent')
    self.spec = spec
    return spec

  def infer(self, warped: np.ndarray, packed: np.ndarray, frame_id: int, *, reset: bool = False):
    if self.spec is None:
      raise RuntimeError('ENGINE_REQ must succeed before infer')
    img = np.ascontiguousarray(warped, dtype=np.uint8)
    scalars = np.ascontiguousarray(packed, dtype='<f4')
    if img.nbytes != warped_bytes(self.spec):
      raise ValueError(f'Incorrect warp bytes: got {img.nbytes}, expected {warped_bytes(self.spec)}')
    if scalars.nbytes != scalar_count(self.spec)*4:
      raise ValueError('Incorrect packed scalar size')
    t0 = time.perf_counter_ns()
    seq = self.send(INFER_REQ, (INFER_HEADER.pack(frame_id & 0xffffffff, RESET_QUEUES if reset else 0), img, scalars))
    t_sent = time.perf_counter_ns()
    resp = self.expect(INFER_RESP, seq, 3.0)
    t_recv = time.perf_counter_ns()
    return self.decode_infer_reply(resp, frame_id, t0, t_sent, t_recv)

  def decode_infer_reply(self, resp: bytes, frame_id: int, t0: int, t_sent: int, t_recv: int):
    if len(resp) < INFER_REPLY.size:
      raise RuntimeError('Short LS2 response header')
    got_id, status, gpu_us, queue_us, total_us = INFER_REPLY.unpack_from(resp)
    if got_id != (frame_id & 0xffffffff):
      raise RuntimeError(f'Reply frame {got_id} did not match sent {frame_id}')
    if status != 0:
      raise RuntimeError(f'LS2 model inference failed: status={status} frame={frame_id}')
    expected = output_reply_count(self.spec)
    if len(resp) != INFER_REPLY.size + expected*4:
      raise RuntimeError(f'LS2 output length mismatch {len(resp)} expected {INFER_REPLY.size+expected*4}')
    out = np.frombuffer(resp, dtype='<f4', count=expected, offset=INFER_REPLY.size)
    if not np.isfinite(out).all():
      raise RuntimeError('LS2 returned non-finite outputs')
    wire_ms = (t_recv-t0)/1e6
    return {
      'wire_ms': wire_ms, 'send_ms':(t_sent-t0)/1e6, 'recv_ms':(t_recv-t_sent)/1e6,
      'server_total_ms':total_us/1000, 'server_npu_ms':gpu_us/1000, 'server_queue_ms':queue_us/1000,
      'link_overhead_ms': wire_ms-total_us/1000,
      'output_floats':expected,
    }

  def close(self):
    try:
      self.sock.close()
    except Exception:
      pass



class PipelinedInference:
  """Duplex TCP inference: one camera/sender thread, one receive thread.

  Only two requests may be outstanding by default. This avoids unlimited
  queuing on the phone and allows 20Hz *throughput* even when each response
  takes >50ms; it does not change the per-frame latency or drive the car.
  """
  def __init__(self, peer: JetlinkTCP, depth: int):
    if depth not in (2, 3):
      raise ValueError('Pipeline depth must be 2 or 3')
    self.peer = peer
    self.depth = depth
    self.lock = threading.Lock()
    self.pending = {}
    self.results = queue.Queue()
    self.error = None
    self.stop_event = threading.Event()
    self.worker = threading.Thread(target=self._reader, name='jetlink-shadow-receiver', daemon=True)

  def start(self):
    self.peer.sock.settimeout(3.0)
    self.worker.start()

  def send_frame(self, item: dict, warped: np.ndarray, packed: np.ndarray, *, reset=False):
    if self.error is not None:
      raise RuntimeError(f'Jetlink receive thread failed: {self.error}') from self.error
    img = np.ascontiguousarray(warped, dtype=np.uint8)
    scalars = np.ascontiguousarray(packed, dtype='<f4')
    if img.nbytes != warped_bytes(self.peer.spec) or scalars.nbytes != 4*scalar_count(self.peer.spec):
      raise ValueError('Invalid LS2 frame or scalar input size')
    with self.lock:
      if len(self.pending) >= self.depth:
        return False
      t0 = time.perf_counter_ns()
      seq = self.peer.send(INFER_REQ, (INFER_HEADER.pack(item['frame_id'] & 0xffffffff,
                            RESET_QUEUES if reset else 0), img, scalars))
      sent = time.perf_counter_ns()
      self.pending[seq] = (item, t0, sent)
    return True

  def outstanding(self):
    with self.lock:
      return len(self.pending)

  def _reader(self):
    try:
      while not self.stop_event.is_set():
        try:
          kind, seq, _flags, raw = self.peer.recv()
        except socket.timeout as exc:
          if self.outstanding() > 0:
            raise TimeoutError('Jetlink inference response timed out (3s with requests outstanding)') from exc
          continue
        received = time.perf_counter_ns()
        if kind == PROGRESS:
          continue
        if kind == ERROR:
          try: detail = json.loads(raw)
          except Exception: detail = {'error': raw[:120].hex()}
          raise RuntimeError(f'Jetlink server error during inference: {detail}')
        if kind != INFER_RESP:
          raise RuntimeError(f'Unexpected reply type {kind} during pipelined inference')
        with self.lock:
          entry = self.pending.pop(seq, None)
        if entry is None:
          raise RuntimeError(f'Unmatched Jetlink inference response seq={seq}')
        item, t0, t_sent = entry
        timing = self.peer.decode_infer_reply(raw, item['frame_id'], t0, t_sent, received)
        self.results.put((item, timing, received))
    except BaseException as exc:
      if not self.stop_event.is_set():
        self.error = exc
        self.stop_event.set()

  def drain(self):
    out=[]
    while True:
      try: out.append(self.results.get_nowait())
      except queue.Empty: return out

  def wait_idle(self, deadline_s=4.0):
    until = time.monotonic()+deadline_s
    while self.outstanding() and time.monotonic()<until and self.error is None:
      time.sleep(.01)
    if self.error is not None:
      raise RuntimeError(f'Jetlink receive thread failed: {self.error}') from self.error
    if self.outstanding():
      raise TimeoutError(f'{self.outstanding()} pending Jetlink inference responses at end of benchmark')

  def close(self):
    self.stop_event.set()
    self.peer.close()
    self.worker.join(timeout=3.2)

def model_hw(spec):
  shapes = spec['input_shapes']
  key = 'new_img' if 'new_img' in shapes else 'img'
  if key not in shapes:
    raise ValueError('Server spec has no image input')
  h,w = shapes[key][-2:]
  return int(h),int(w)


def warped_bytes(spec):
  h,w=model_hw(spec)
  return 2*6*h*w


def scalar_layout(spec):
  shapes = spec['input_shapes']
  if 'new_img' in shapes:
    desc={'desire': math.prod(shapes['desire']), 'traffic_convention':math.prod(shapes['traffic_convention']), 'action_t':math.prod(shapes['action_t'])}
  else:
    desc={'desire':shapes['desire_pulse'][2], 'traffic_convention':math.prod(shapes['traffic_convention']), 'action_t':math.prod(shapes['action_t'])}
  return desc


def scalar_count(spec):
  return sum(scalar_layout(spec).values())


def output_reply_count(spec):
  count=math.prod(spec['output_shapes']['outputs'])
  sl=spec.get('output_slices',{}).get('hidden_state')
  if sl:
    start,stop=sl
    a,b,_=slice(start,stop).indices(count)
    if a<b:
      count-=(b-a)
  return count


def packed_scalars(spec, is_rhd=False, action_t=0.):
  """Return full scalar wire buffer in Jetlink's declared ordering."""
  layout=scalar_layout(spec)
  out=np.zeros(sum(layout.values()),dtype=np.float32)
  offset=layout['desire']
  out[offset+int(bool(is_rhd))]=1.0
  offset+=layout['traffic_convention']
  out[offset:offset+layout['action_t']]=float(action_t)
  return out


def _project(matrix: np.ndarray, xx: np.ndarray, yy: np.ndarray, src_w: int, src_h: int):
  """compile_warp.py's nearest-neighbour map, with clipped source coordinates."""
  den=(matrix[2,0]*xx+matrix[2,1]*yy+matrix[2,2])
  den=np.where(np.abs(den)<1e-12,1e-12,den)
  px=(matrix[0,0]*xx+matrix[0,1]*yy+matrix[0,2])/den
  py=(matrix[1,0]*xx+matrix[1,1]*yy+matrix[1,2])/den
  return np.rint(px).clip(0,src_w-1).astype(np.intp),np.rint(py).clip(0,src_h-1).astype(np.intp)


def warp_indices(cam_w:int, cam_h:int, model_h:int, model_w:int, matrix:np.ndarray):
  """Return index map for packed NV12 input -> six LS2 uint8 image channels.
  Input NV12 data layout/stride is from xnor's own get_nv12_info().
  """
  from openpilot.system.camerad.cameras.nv12_info import get_nv12_info
  stride,y_height,_,_ = get_nv12_info(cam_w,cam_h)
  xx,yy=np.meshgrid(np.arange(model_w,dtype=np.float32),np.arange(model_h,dtype=np.float32))
  sx,sy=_project(matrix,xx,yy,cam_w,cam_h)
  offsets=np.empty((6,model_h//2,model_w//2),dtype=np.intp)
  offsets[0]=sy[0::2,0::2]*stride+sx[0::2,0::2]
  offsets[1]=sy[1::2,0::2]*stride+sx[1::2,0::2]
  offsets[2]=sy[0::2,1::2]*stride+sx[0::2,1::2]
  offsets[3]=sy[1::2,1::2]*stride+sx[1::2,1::2]
  # NV12 UV samples are at half resolution; transform S@M@inv(S).
  scale=np.array([[1.,1.,.5],[1.,1.,.5],[2.,2.,1.]],dtype=np.float32)
  uv_matrix=matrix*scale
  ux,uy=np.meshgrid(np.arange(model_w//2,dtype=np.float32),np.arange(model_h//2,dtype=np.float32))
  uxx,uyy=_project(uv_matrix,ux,uy,cam_w//2,cam_h//2)
  uv_offset=stride*y_height+uyy*stride+uxx*2
  offsets[4]=uv_offset
  offsets[5]=uv_offset+1
  return offsets.reshape(-1)


def ns_ms(value:int)->float:
  return value/1e6


def pct(values):
  if not values:
    return None
  a=np.asarray(values,dtype=np.float64)
  return {'mean':round(float(a.mean()),3),'p50':round(float(np.percentile(a,50)),3),
          'p95':round(float(np.percentile(a,95)),3),'p99':round(float(np.percentile(a,99)),3),
          'max':round(float(a.max()),3)}


def check_route(host, port, *, require_usb=True, require_panda=True):
  route=subprocess.run(['ip','route','get',host],capture_output=True,text=True,check=True).stdout.strip()
  if require_usb and ' dev usb0 ' not in f' {route} ':
    raise RuntimeError(f'Stopping: Jetlink route is not usb0: {route}')
  panda=subprocess.run(['lsusb'],capture_output=True,text=True,check=True).stdout
  if require_panda and '3801:ddcc' not in panda.lower():
    raise RuntimeError('Stopping: external xnor panda is not enumerated in lsusb')
  return {'route':route,'xnor_panda_enumerated': '3801:ddcc' in panda.lower()}


def parked(sm):
  """Fail closed if any observed vehicle signal indicates motion or control."""
  if not sm.seen.get('carState',False):
    return False,'No live carState: cannot verify stationary test'
  car=sm['carState']
  if abs(float(car.vEgo)) >= 0.2:
    return False,'Vehicle is moving; shadow benchmark stopped'
  gear=str(car.gearShifter).split('.')[-1].lower()
  if gear not in ('park','p'):
    return False,f'Gear {gear!r} is not Park; shadow benchmark stopped'
  for service in ('carControl','selfdriveState'):
    if sm.seen.get(service,False):
      msg=sm[service]
      if bool(getattr(msg,'enabled',False)) or bool(getattr(msg,'active',False)):
        return False,f'{service} indicates active controls; stopped'
  return True,None


def camera_frames(args,spec):
  """Yield independent frames from camerad, with current calibration.
  Separate VisionIPC subscriber, no modeld process changes or live messages.
  """
  from msgq.visionipc import VisionIpcClient, VisionStreamType
  from cereal.messaging import SubMaster
  from openpilot.common.transformations.camera import DEVICE_CAMERAS
  from openpilot.common.transformations.model import get_warp_matrix

  names=['carState','carControl','selfdriveState','driverMonitoringState','liveCalibration','deviceState','roadCameraState']
  sm=SubMaster(names)
  start=time.monotonic()
  streams=[]
  while time.monotonic()-start<args.camera_timeout and RUNNING:
    streams=VisionIpcClient.available_streams('camerad',block=False)
    if streams: break
    time.sleep(.2)
  if not streams:raise RuntimeError('No camerad stream. Enable camera while parked and retry.')
  has_both=VisionStreamType.VISION_STREAM_ROAD in streams and VisionStreamType.VISION_STREAM_WIDE_ROAD in streams
  main_wide=VisionStreamType.VISION_STREAM_ROAD not in streams
  main=VisionIpcClient('camerad',VisionStreamType.VISION_STREAM_WIDE_ROAD if main_wide else VisionStreamType.VISION_STREAM_ROAD, True)
  extra=VisionIpcClient('camerad',VisionStreamType.VISION_STREAM_WIDE_ROAD,False)
  for client in [main]+([extra] if has_both else []):
    while RUNNING and not client.connect(False):
      if time.monotonic()-start>args.camera_timeout:
        raise RuntimeError('Timed out connecting to camerad')
      time.sleep(.1)
  if not RUNNING:return
  h,w=model_hw(spec)
  if (h,w)!=(128,256):
    raise RuntimeError(f'LS2 camera geometry {h}x{w} differs from validated 128x256')
  out=np.empty((2,6,h,w),dtype=np.uint8)
  last_calib=None
  maps=None
  last_log=0
  while RUNNING:
    buf0=main.recv()
    if buf0 is None:
      raise RuntimeError('Main camera stream ended')
    buf1=extra.recv() if has_both else buf0
    if buf1 is None:
      raise RuntimeError('Wide camera stream ended')
    sm.update(0)
    ok,reason=parked(sm)
    if not ok:
      # A few frames are allowed for the first carState message to arrive.
      if reason.startswith('No live carState') and time.monotonic()-start<args.camera_timeout:
        continue
      raise RuntimeError(reason)
    if not all(sm.seen.get(n,False) for n in ('liveCalibration','deviceState','roadCameraState')):
      if time.monotonic()-start<args.camera_timeout:continue
      raise RuntimeError('No live camera calibration / device / sensor metadata')
    calib=tuple(sm['liveCalibration'].rpyCalib)
    dev=str(sm['deviceState'].deviceType)
    sensor=str(sm['roadCameraState'].sensor)
    state=(calib,dev,sensor,main.width,main.height,extra.width if has_both else main.width,extra.height if has_both else main.height)
    if state!=last_calib:
      dc=DEVICE_CAMERAS[(dev,sensor)]
      f=np.asarray(calib,dtype=np.float32)
      main_mat=get_warp_matrix(f,dc.ecam.intrinsics if main_wide else dc.fcam.intrinsics,False).astype(np.float32)
      wide_mat=get_warp_matrix(f,dc.ecam.intrinsics,True).astype(np.float32)
      maps=[warp_indices(main.width,main.height,h*2,w*2,main_mat),
            warp_indices(extra.width if has_both else main.width,extra.height if has_both else main.height,h*2,w*2,wide_mat)]
      last_calib=state
    timestamp_eof=int(main.timestamp_eof)
    frame_id=int(main.frame_id)
    rgbh=bool(sm['driverMonitoringState'].isRHD) if sm.seen.get('driverMonitoringState',False) else False
    # Precomputation of mapping is deliberately excluded from hot-frame timings.
    t0=time.perf_counter_ns()
    for index,b in enumerate((buf0,buf1)):
      source=np.frombuffer(b.data,dtype=np.uint8)
      np.take(source,maps[index],out=out[index].reshape(-1))
    preparation=ns_ms(time.perf_counter_ns()-t0)
    yield frame_id,timestamp_eof,preparation,out,rgbh



def bench(args):
  output=Path(args.output).expanduser()
  output.mkdir(parents=True,exist_ok=False)
  meta={'version':VERSION,'mode':args.mode,'host':args.host,'port':args.port,'rate_requested_hz':args.rate,
        'seconds_requested':args.seconds,'model_sha256':SHA,'no_control':True,'cwd':os.getcwd(),
        'pipeline_depth':args.pipeline_depth, 'started_utc':datetime.now(timezone.utc).isoformat()}
  records=[]
  errors=[]
  counts={'frames':0,'frames_sent':0,'over_50_ms':0,'capture_to_reply_over_50_ms':0,
          'source_frames_skipped':0,'pipeline_full_skips':0,'preprocess_count':0}
  peer=None
  pipeline=None
  began=None
  try:
    meta['physical']=check_route(args.host,args.port,require_usb=not args.allow_other_network,
                                  require_panda=not args.skip_panda_check)
    peer=JetlinkTCP(args.host,args.port,args.timeout)
    hello=peer.hello()
    meta['server']={k:hello.get(k) for k in ('loaded','backend','device','runtime_version','protocol','engine_state')}
    spec=peer.load_existing()
    meta['spec']={k:spec.get(k) for k in ('sha256','frame_skip','input_shapes','output_shapes','output_slices')}
    meta['wire_payload_bytes']={'upload':INFER_HEADER.size+warped_bytes(spec)+scalar_count(spec)*4,
                                'download':INFER_REPLY.size+output_reply_count(spec)*4}
    print(json.dumps({'connected':True,'mode':args.mode,'server':meta['server'],
                      'pipeline_depth':args.pipeline_depth,
                      'wire_payload_bytes':meta['wire_payload_bytes']},indent=2),flush=True)
    fields=['sample','frame_id','capture_age_ms','preprocess_ms','wire_ms','send_ms','recv_ms',
            'link_overhead_ms','capture_to_reply_ms','server_total_ms','server_npu_ms','server_queue_ms',
            'model_outputs','over_50ms']
    path=output/'frames.csv'
    with path.open('w',newline='') as f:
      writer=csv.DictWriter(f,fieldnames=fields)
      writer.writeheader()
      began=time.monotonic()
      next_due=began
      last_source_id=None
      camera=camera_frames(args,spec) if args.mode=='camera' else None
      rng=np.random.default_rng(0xC0A0)
      scratch=(rng.integers(0,256,(2,6,*model_hw(spec)),dtype=np.uint8) if args.mode=='synthetic' else None)
      n=0
      if args.pipeline_depth:
        pipeline=PipelinedInference(peer,args.pipeline_depth)
        pipeline.start()

      def record_result(item,timing,received):
        pre_ms=item['preprocess_ms']
        capture_ns=item['capture_ns']
        capture_to_reply=(received-capture_ns)/1e6 if capture_ns is not None else None
        total=timing['wire_ms']+pre_ms
        row={'sample':item['sample'],'frame_id':item['frame_id'],'capture_age_ms':item['capture_age_ms'],
             'preprocess_ms':pre_ms,'wire_ms':timing['wire_ms'],'send_ms':timing['send_ms'],
             'recv_ms':timing['recv_ms'],'link_overhead_ms':timing['link_overhead_ms'],
             'capture_to_reply_ms':capture_to_reply,'server_total_ms':timing['server_total_ms'],
             'server_npu_ms':timing['server_npu_ms'],'server_queue_ms':timing['server_queue_ms'],
             'model_outputs':timing['output_floats'],'over_50ms':int(total>50.)}
        writer.writerow(row)
        records.append(row)
        counts['frames']+=1
        counts['over_50_ms']+=int(total>50.)
        counts['capture_to_reply_over_50_ms']+=int(capture_to_reply is not None and capture_to_reply>50.)
        if counts['frames'] % 20 == 0:
          f.flush()
          print(f"{counts['frames']} replies; NPU {timing['server_npu_ms']:.1f}ms, "
                f"round-trip {timing['wire_ms']:.1f}ms, pipeline outstanding "
                f"{pipeline.outstanding() if pipeline else 0}",flush=True)

      while RUNNING and time.monotonic()-began < args.seconds:
        if pipeline:
          for item,timing,received in pipeline.drain():
            record_result(item,timing,received)
          if pipeline.error is not None:
            raise RuntimeError(f'Jetlink receiver failed: {pipeline.error}') from pipeline.error
        if args.rate>0 and time.monotonic()<next_due:
          # V1's min() could become negative between the test and this call.
          time.sleep(max(0.,min(.01,next_due-time.monotonic())))
          continue
        if args.rate>0:
          next_due=max(next_due+1.0/args.rate,time.monotonic())
        if camera is not None:
          frame_id,capture_ns,pre_ms,warped,is_rhd=next(camera)
          counts['preprocess_count']+=1
          if last_source_id is not None and frame_id>last_source_id+1:
            counts['source_frames_skipped']+=frame_id-last_source_id-1
          last_source_id=frame_id
          age_before=max(0.,ns_ms(time.monotonic_ns()-capture_ns))
        else:
          warped=scratch
          warped[0,0,0,0]=n%256
          frame_id=n;pre_ms=0.;is_rhd=False;age_before=None;capture_ns=None
        packed=packed_scalars(spec,is_rhd,0.)
        item={'sample':n,'frame_id':frame_id,'capture_age_ms':age_before,
              'capture_ns':capture_ns,'preprocess_ms':pre_ms}
        if pipeline:
          if not pipeline.send_frame(item,warped,packed,reset=(counts['frames_sent']==0)):
            counts['pipeline_full_skips']+=1
          else:
            counts['frames_sent']+=1
        else:
          timing=peer.infer(warped,packed,frame_id,reset=(counts['frames_sent']==0))
          timing['link_overhead_ms']=timing['wire_ms']-timing['server_total_ms']
          counts['frames_sent']+=1
          record_result(item,timing,time.monotonic_ns())
        n+=1
      if pipeline:
        pipeline.wait_idle()
        for item,timing,received in pipeline.drain():
          record_result(item,timing,received)
      counts['elapsed_s']=round(time.monotonic()-began,3)
      counts['achieved_fps']=round(counts['frames']/max(.001,counts['elapsed_s']),3)
      counts['sent_fps']=round(counts['frames_sent']/max(.001,counts['elapsed_s']),3)
  except (KeyboardInterrupt,StopIteration):
    errors.append('Stopped or camerad finished before benchmark duration')
  except Exception as exc:
    errors.append(f'{type(exc).__name__}: {exc}')
    print('Benchmark error: '+errors[-1],file=sys.stderr,flush=True)
  finally:
    if pipeline:
      # On error the CSV with-block has closed. Preserve completed in-flight
      # replies from the receiver queue rather than silently discarding them.
      late = pipeline.drain()
      if late and 'writer' in locals() and 'record_result' in locals():
        with path.open('a',newline='') as tail:
          writer=csv.DictWriter(tail,fieldnames=fields)
          for item,timing,received in late:
            record_result(item,timing,received)
      pipeline.close()
    elif peer:
      peer.close()
    if began is not None and 'elapsed_s' not in counts:
      counts['elapsed_s']=round(time.monotonic()-began,3)
      counts['achieved_fps']=round(counts['frames']/max(.001,counts['elapsed_s']),3)
      counts['sent_fps']=round(counts['frames_sent']/max(.001,counts['elapsed_s']),3)
    meta['finished_utc']=datetime.now(timezone.utc).isoformat()
    summary={'meta':meta,'counts':counts,'errors':errors,'inference_verified':not errors and counts['frames']>0,
             'controls_affected':False,
             'data_interpretation':'Independent shadow-only probe, no vehicle control. CPU warp not numerical parity validated. Pipelining improves throughput, not necessarily latency.',
             'timing_ms':{k:pct([r[k] for r in records if r[k] is not None])
                          for k in ('preprocess_ms','wire_ms','send_ms','recv_ms','link_overhead_ms',
                                    'capture_to_reply_ms','server_total_ms','server_npu_ms','server_queue_ms')},
             'gate_note':'A successful connected benchmark is not approval for model authority.'}
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({'output':str(output),'counts':counts,'errors':errors,'timing_ms':summary['timing_ms']},indent=2),flush=True)
    return 0 if not errors and counts['frames'] else 1


def parse_args(argv=None):
  p=argparse.ArgumentParser(description=__doc__)
  p.add_argument('--mode',choices=('synthetic','camera'),default='synthetic',help='camera: real NV12 frames, 2 cameras; synthetic: full-size Jetlink LS2 tensors')
  p.add_argument('--host',default='10.169.232.180')
  p.add_argument('--port',type=int,default=5599)
  p.add_argument('--rate',type=float,default=5.0,help='Default 5Hz; --rate 20 for 20Hz connected stress test, stationary only')
  p.add_argument('--seconds',type=float,default=60.)
  p.add_argument('--timeout',type=float,default=5.)
  p.add_argument('--pipeline-depth',type=int,choices=(0,2,3),default=0,
                 help='0=legacy synchronous (A/B baseline); 2=two outstanding requests; 3=experimental. Throughput only, never vehicle control.')
  p.add_argument('--camera-timeout',type=float,default=25.)
  p.add_argument('--output',default=f'/data/jetlink-shadow-{datetime.now().strftime("%Y%m%d-%H%M%S")}')
  p.add_argument('--allow-other-network',action='store_true',help='Disable USB-only route check')
  p.add_argument('--skip-panda-check',action='store_true',help='Disable xnor panda enumeration check')
  args=p.parse_args(argv)
  if not(0<args.rate<=20) or not(0<args.seconds<=900):p.error('rate 0..20 and seconds 0..900 required')
  return args


def main():
  signal.signal(signal.SIGINT,stop_now)
  signal.signal(signal.SIGTERM,stop_now)
  return bench(parse_args())


if __name__=='__main__':
  sys.exit(main())
