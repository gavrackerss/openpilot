#!/usr/bin/env python3
"""XNOR Jetlink connected shadow transport benchmark, v5.6-onroad.

Subscribes to camerad in a separate, non-realtime process; never imports/patches
modeld, publishes messages, accesses CAN, or writes openpilot Params. Synthetic
mode tests the live link and LS2 NPU without camera. Camera mode uses the live
NV12 road/wide frames and the same nearest-neighbour planar YUV layout as
openpilot's compile_warp.py. The CPU implementation is *not* proven numerically
identical to the tinygrad warp; DO NOT use its outputs for driving.

Timer pacing is the safe default after real-world V5 camera-event pacing increased
USB2 TCP response stalls. --reply-guard-ms optionally holds a *bounded* new
upload near the expected previous reply, with detailed overlap instrumentation.
Camera event pacing remains opt-in for A/B diagnostics, not recommended.

V5.3 adds an opt-in Linux sendmmsg() UDP upload batch, retaining the tested
V5.2 sendmsg() path as default. No model, camera, Android or control changes.

V5.5 adds an opt-in post-run phone timing trace download from the instrumented v0.5 Jetlink APK.
V5.6 offers an opt-in Linux TCP_QUICKACK A/B for the *return* path and separately
measures receive-completion intervals (delivery cadence) and age-at-reply. This
is an experiment, not an established speed fix; baseline behaviour is unchanged.
Parked mode remains the default. --onroad-shadow allows a separate, passive
vision-only observer while driving; never publish or control from these outputs.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
import errno
from collections import deque
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
import traceback
from datetime import datetime, timezone

import numpy as np

VERSION = '5.8-prediction-shadow-feed'
# Runtime selection for the read-only transport/parked benchmark only.
# Production modeld/Zoompilot authority remains LS2-only until separately validated.
MODEL_PROFILES = {
  'ls2': ('909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8', 768749892),
  'ctm': ('e8d821733be15ebe9e27498bc27ad8bbbd741980ece37d77f377294010b8ff28', 765950064),
}
SHA, NBYTES = MODEL_PROFILES['ls2']
HEADER = struct.Struct('<IHHIIIQ4x')
INFER_HEADER = struct.Struct('<II')
INFER_REPLY = struct.Struct('<IIIII')
MAGIC = 0x4B4E4C4A
PROTOCOL = 3
HELLO_REQ, HELLO_RESP = 1, 2
ENGINE_REQ, ENGINE_RESP, PROGRESS = 3, 4, 7
INFER_REQ, INFER_RESP, ERROR = 8, 9, 14
STATE_REQ, STATE_RESP = 12, 13
PING, PONG, LEAVE = 15, 16, 19
PADDED = 1 << 7
RESET_QUEUES = 1
MAX_BODY = 16 * 1024 * 1024
DATAGRAM_MAGIC = 0x4D52464A
DATAGRAM_HEADER = struct.Struct('<IIIII')
DATAGRAM_PAYLOAD = 65000
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


def recv_timed(sock: socket.socket, n: int):
  """Read exactly n bytes and record the instant its FIRST byte arrived.

  This is comma-side first-response timing, not an Android clock reading.
  """
  chunks=bytearray(n)
  view=memoryview(chunks)
  got=0
  first_ns=None
  calls=0
  while got<n:
    count=sock.recv_into(view[got:])
    now=time.perf_counter_ns()
    calls+=1
    if count==0:
      raise ConnectionError('Jetlink server disconnected during receive')
    if first_ns is None:
      first_ns=now
    got+=count
  return bytes(chunks),first_ns,time.perf_counter_ns(),calls


def udp_parts(parts, token, seq, payload_bytes=DATAGRAM_PAYLOAD):
  """Exactly the seven-or-more equally sized Jetlink FrameDatagrams chunks.

  Header + frame + padding is treated as one stream of byte views, without
  copying a ~393KB numpy array into a second buffer.
  """
  if not 1024 <= payload_bytes <= DATAGRAM_PAYLOAD:
    raise ValueError('UDP fragment payload must be 1024..65000 bytes')
  total=sum(p.nbytes for p in parts)
  chunks=max(1,(total+payload_bytes-1)//payload_bytes)
  base,extra=divmod(total,chunks)
  offset=0
  for i in range(chunks):
    length=base+(i<extra)
    header=DATAGRAM_HEADER.pack(DATAGRAM_MAGIC, token, seq, offset, total)
    remaining=length
    skip=offset
    slices=[memoryview(header)]
    for part in parts:
      if skip>=part.nbytes:
        skip-=part.nbytes
        continue
      take=min(remaining,part.nbytes-skip)
      slices.append(part[skip:skip+take])
      remaining-=take
      skip=0
      if remaining==0:break
    if remaining:
      raise RuntimeError('Jetlink datagram partition error')
    yield slices
    offset+=length



# Optional Linux UDP sendmmsg() A/B optimisation. Default V5.2 sendmsg() is
# unchanged. This batches the existing Jetlink datagrams into one kernel call;
# neither the datagram header, payload partitioning nor server protocol change.
# Writable numpy image/scalar buffers are referenced without copying ~393 KB.
class _IOVec(ctypes.Structure):
  _fields_ = [('iov_base', ctypes.c_void_p), ('iov_len', ctypes.c_size_t)]


class _MsgHdr(ctypes.Structure):
  _fields_ = [('msg_name', ctypes.c_void_p), ('msg_namelen', ctypes.c_uint),
              ('msg_iov', ctypes.POINTER(_IOVec)), ('msg_iovlen', ctypes.c_size_t),
              ('msg_control', ctypes.c_void_p), ('msg_controllen', ctypes.c_size_t),
              ('msg_flags', ctypes.c_int)]


class _MMsgHdr(ctypes.Structure):
  _fields_ = [('msg_hdr', _MsgHdr), ('msg_len', ctypes.c_uint)]


def udp_sendmmsg(sock: socket.socket, datagrams):
  """Send existing UDP datagrams in one or more Linux sendmmsg syscalls.

  Returns (number of datagrams, syscalls). Do not fall back silently to TCP.
  Payloads must remain alive and unmodified throughout this synchronous call.
  """
  libc = ctypes.CDLL(None, use_errno=True)
  sendmmsg = getattr(libc, 'sendmmsg', None)
  if sendmmsg is None:
    raise RuntimeError('Linux sendmmsg() unavailable; use --udp-send-mode sendmsg')
  sendmmsg.argtypes = [ctypes.c_int, ctypes.POINTER(_MMsgHdr), ctypes.c_uint, ctypes.c_uint]
  sendmmsg.restype = ctypes.c_int
  datagrams = list(datagrams)
  if not datagrams:
    return 0, 0
  if len(datagrams)>1024:
    raise ValueError('Too many datagrams for one sendmmsg batch')
  msg_array = (_MMsgHdr * len(datagrams))()
  keepalive = []
  totals = []
  for i, parts in enumerate(datagrams):
    vec = (_IOVec * len(parts))()
    keepalive.append(vec)
    total = 0
    for j, part in enumerate(parts):
      part = memoryview(part).cast('B')
      # numpy-backed parts are already writable/contiguous; the tiny immutable
      # Jetlink headers need private buffers (<=32 bytes), but not the image.
      try:
        owner = (ctypes.c_char * len(part)).from_buffer(part)
      except (TypeError, BufferError):
        owner = ctypes.create_string_buffer(part.tobytes())
      keepalive.append(owner)
      vec[j].iov_base = ctypes.addressof(owner)
      vec[j].iov_len = len(part)
      total += len(part)
    msg_array[i].msg_hdr.msg_iov = ctypes.cast(vec, ctypes.POINTER(_IOVec))
    msg_array[i].msg_hdr.msg_iovlen = len(parts)
    totals.append(total)
  submitted = 0
  calls = 0
  while submitted < len(datagrams):
    tail = ctypes.cast(ctypes.byref(msg_array, submitted * ctypes.sizeof(_MMsgHdr)),
                       ctypes.POINTER(_MMsgHdr))
    ret = sendmmsg(sock.fileno(), tail, len(datagrams) - submitted, 0)
    calls += 1
    if ret < 0:
      code = ctypes.get_errno()
      if code == errno.EINTR:
        continue
      raise OSError(code, 'sendmmsg USB UDP upload failed')
    if ret == 0:
      raise ConnectionError('Jetlink sendmmsg submitted no datagrams')
    for i in range(submitted, submitted + ret):
      if msg_array[i].msg_len != totals[i]:
        raise ConnectionError(f'Short Jetlink UDP datagram: {msg_array[i].msg_len}/{totals[i]}')
    submitted += ret
  return submitted, calls


def tcp_info(sock):
  """Linux TCP_INFO: only stable early fields; None on unsupported kernels."""
  if not hasattr(socket,'TCP_INFO'):
    return {'tcp_rtt_ms':None,'tcp_retrans_total':None,'tcp_unacked':None}
  try:
    info=sock.getsockopt(socket.IPPROTO_TCP,socket.TCP_INFO,104)
    if len(info)<104:
      raise ValueError('TCP_INFO too short')
    return {
      'tcp_rtt_ms':int.from_bytes(info[68:72],sys.byteorder)/1000.0,
      'tcp_retrans_total':int.from_bytes(info[100:104],sys.byteorder),
      'tcp_unacked':int.from_bytes(info[24:28],sys.byteorder),
    }
  except (OSError,ValueError):
    return {'tcp_rtt_ms':None,'tcp_retrans_total':None,'tcp_unacked':None}


def usb0_stats():
  fields=('rx_dropped','tx_dropped','rx_errors','tx_errors','rx_packets','tx_packets')
  root=Path('/sys/class/net/usb0/statistics')
  values={}
  for field in fields:
    try:values[field]=int((root/field).read_text().strip())
    except (OSError,ValueError):values[field]=None
  return values


def usbstats_diff(before,after):
  return {key:after[key]-before[key] if before.get(key) is not None and after.get(key) is not None else None
          for key in after}


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
    self.udp = None
    self.udp_token = None
    self.datagram_port = None
    self.last_rx = {}
    self.udp_payload_bytes = DATAGRAM_PAYLOAD
    self.udp_datagrams_sent = 0
    self.udp_send_syscalls = 0
    self.udp_send_mode = 'sendmsg'
    self.udp_batches_sent = 0
    self.stage_trace_supported = False
    self.tcp_quickack = False
    self.tcp_quickack_sets = 0

  def configure_tcp_quickack(self, enabled: bool):
    """Opt-in Linux quick ACKs; never silently fall back if unavailable."""
    if enabled and not hasattr(socket, 'TCP_QUICKACK'):
      raise RuntimeError('TCP_QUICKACK not supported on this comma kernel/Python; disable --tcp-quickack')
    self.tcp_quickack = bool(enabled)
    if enabled:
      self.acknowledge_promptly()

  def acknowledge_promptly(self):
    """Request immediate ACK after a complete TCP response (A/B only).

    TCP_QUICKACK is Linux-only and transient. Re-arming for each inference
    response is intentional. It cannot bypass USB or shorten NPU execution.
    No other TCP or UDP socket configuration changes.
    """
    if self.tcp_quickack:
      self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
      self.tcp_quickack_sets += 1

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
    if kind == INFER_REQ and self.udp is not None:
      pieces = udp_parts(pending,self.udp_token,seq,self.udp_payload_bytes)
      if self.udp_send_mode == 'sendmmsg':
        n, calls = udp_sendmmsg(self.udp, pieces)
        self.udp_datagrams_sent += n
        self.udp_send_syscalls += calls
        self.udp_batches_sent += 1
      else:
        for piece in pieces:
          n=self.udp.sendmsg(piece)
          self.udp_send_syscalls += 1
          if n!=sum(p.nbytes for p in piece):
            raise ConnectionError('Short Jetlink frame datagram send')
          self.udp_datagrams_sent += 1
    else:
      while pending:
        # Same TCP sendmsg path as V2; keep it as a comparable baseline.
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
    h,first_ns,header_done,header_reads=recv_timed(self.sock,HEADER.size)
    magic, version, kind, seq, flags, length, _ = HEADER.unpack(h)
    if magic != MAGIC or version != PROTOCOL:
      raise ValueError('Jetlink v3 header mismatch')
    if length > MAX_BODY:
      raise ValueError(f'Refusing Jetlink payload length {length}')
    if length+(1 if flags & PADDED else 0):
      raw,_,body_done,body_reads=recv_timed(self.sock,length+(1 if flags & PADDED else 0))
    else:
      raw=b'';body_done=header_done;body_reads=0
    self.last_rx={'first_ns':first_ns,'header_done_ns':header_done,'body_done_ns':body_done,
                  'header_reads':header_reads,'body_reads':body_reads}
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

  def hello(self,usb_link=False):
    link = {'kind':'usb'} if usb_link else {'kind':'tcp'}  # USB speed unknown: never advertise fabricated USB 2
    client = {'name': 'xnor-transport-shadow-v5', 'nonce': secrets.token_hex(4), 'link': link}
    seq = self.send(HELLO_REQ, json.dumps({'client': client}).encode())
    return json.loads(self.expect(HELLO_RESP, seq, 5.0))


  def enable_datagrams(self,hello):
    """Only accept a UDP frame port explicitly offered by the Jetlink server.

    No fallback to TCP: a successful A/B must genuinely use UDP frames.
    Reply messages always arrive over the original TCP socket.
    """
    port=hello.get('frame_port')
    token=hello.get('frame_token')
    if not isinstance(port,int) or not(1<=port<=65535) or not isinstance(token,int) or token==0:
      raise RuntimeError('This Android Jetlink build did not offer UDP frames over USB. '
                         'TCP profiling still works with --transport tcp; the Android server needs its '
                         'USB-NCM peer classification enabled before a UDP A/B can run.')
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:
      udp.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,4<<20)
      udp.connect((self.host,port))
    except BaseException:
      udp.close()
      raise
    self.udp=udp
    self.udp_token=token
    self.datagram_port=port

  def ping_samples(self,count=10):
    times=[]
    for _ in range(count):
      before=time.perf_counter_ns()
      seq=self.send(PING)
      self.expect(PONG,seq,3.0)
      times.append((time.perf_counter_ns()-before)/1e6)
    return times

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

  def infer(self, warped: np.ndarray, packed: np.ndarray, frame_id: int, *, reset: bool = False, capture_output=False):
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
    self.acknowledge_promptly()
    return self.decode_infer_reply(resp, frame_id, t0, t_sent, t_recv, capture_output=capture_output)

  def decode_infer_reply(self, resp: bytes, frame_id: int, t0: int, t_sent: int, t_recv: int, rx=None, *, capture_output=False):
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
    rx=rx or self.last_rx
    first_ns=rx.get('first_ns')
    header_done_ns=rx.get('header_done_ns')
    body_done_ns=rx.get('body_done_ns')
    tcp=tcp_info(self.sock)
    return {
      'response_first_byte_wait_ms':(first_ns-t_sent)/1e6 if first_ns else None,
      'response_body_read_ms':(body_done_ns-first_ns)/1e6 if first_ns and body_done_ns else None,
      'response_header_read_ms':(header_done_ns-first_ns)/1e6 if first_ns and header_done_ns else None,
      'response_read_syscalls':rx.get('header_reads',0)+rx.get('body_reads',0),
      **tcp,
      'wire_ms': wire_ms, 'send_ms':(t_sent-t0)/1e6, 'recv_ms':(t_recv-t_sent)/1e6,
      'server_total_ms':total_us/1000, 'server_npu_ms':gpu_us/1000, 'server_queue_ms':queue_us/1000,
      'link_overhead_ms': wire_ms-total_us/1000,
      'output_floats':expected,
      # Snapshot only selected frames for offline output-parity work. The JPEG/NV12
      # camera buffers are NOT written and no model output is published.
      '_output':out.copy() if capture_output else None,
      # This is a residual measured on different clocks/measurement boundaries.
      # It includes UDP delivery, Android reassembly/scheduling/output and TCP return.
      'post_upload_unattributed_ms':((first_ns-t_sent)/1e6-total_us/1000) if first_ns else None,
    }

  def close(self):
    if self.udp is not None:
      self.udp.close()
      self.udp=None
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
  def __init__(self, peer: JetlinkTCP, depth: int, *, dump_outputs_every: int = 0, prediction_every: int = 0):
    if depth not in (2, 3):
      raise ValueError('Pipeline depth must be 2 or 3')
    self.peer = peer
    self.depth = depth
    self.dump_outputs_every = dump_outputs_every
    self.prediction_every = prediction_every
    self.lock = threading.Lock()
    self.changed = threading.Condition(self.lock)
    self.pending = {}
    self.processing_results = 0  # receiver decoded but not yet published to result queue
    # Small history of actual UDP upload windows; no disk writes on the hot path.
    self.send_windows = deque(maxlen=128)
    self.results = queue.Queue()
    self.state_response = queue.Queue(maxsize=1)
    self.pending_state_seq = None
    self.error = None
    self.stop_event = threading.Event()
    self.udp_lost = 0
    self.udp_expired = 0
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
    with self.changed:
      if len(self.pending) >= self.depth:
        return False
      # Snapshot outstanding work before submitting: useful for associating
      # 50 ms response stalls with competing upload traffic.
      item['outstanding_at_send'] = len(self.pending)
      oldest = min((t_sent for _item, _t0, t_sent in self.pending.values()), default=None)
      item['oldest_outstanding_age_ms'] = ((time.perf_counter_ns() - oldest) / 1e6
                                           if oldest is not None else None)
      t0 = time.perf_counter_ns()
      item['send_start_ns'] = t0
      seq = self.peer.send(INFER_REQ, (INFER_HEADER.pack(item['frame_id'] & 0xffffffff,
                            RESET_QUEUES if reset else 0), img, scalars))
      sent = time.perf_counter_ns()
      self.pending[seq] = (item, t0, sent)
      self.send_windows.append((seq, t0, sent))
    return True

  def await_reply_window(self, *, max_wait_ms: float, min_age_ms: float):
    """Optional bounded admission guard, never a full serialised pipeline.

    Applied ONLY at a timer tick, before capturing the next camera frame.
    Wait for an imminent previous reply to reduce USB2 TX/RX competition.
    Returning a timeout does not block the next frame indefinitely.
    """
    if max_wait_ms <= 0:
      return 0., False, False
    with self.changed:
      if self.error is not None or not self.pending:
        return 0., False, False
      oldest = min(t_sent for _item, _t0, t_sent in self.pending.values())
      age_ms = (time.perf_counter_ns() - oldest) / 1e6
      if not(min_age_ms <= age_ms <= 150.):
        return 0., False, False
      oldest_seq = min(self.pending, key=lambda seq: self.pending[seq][2])
      started = time.perf_counter_ns()
      deadline = started + int(max_wait_ms * 1e6)
      # Condition.wait releases the lock so the receiver can retire a reply.
      # Recheck after spurious notifications; never wait past the deadline.
      while oldest_seq in self.pending and self.error is None and not self.stop_event.is_set():
        remaining_ns = deadline - time.perf_counter_ns()
        if remaining_ns <= 0:
          break
        self.changed.wait(timeout=remaining_ns / 1e9)
      waited = (time.perf_counter_ns() - started) / 1e6
      return waited, True, oldest_seq not in self.pending

  @staticmethod
  def overlap_diagnostics(seq, sent_ns, first_ns, received_ns, windows):
    """Track *other* uploads during this reply's pre-first-byte wait.

    Purely correlational: USB packets, scheduler latency and TCP buffering
    cannot be separated from send timestamps on the comma alone.
    """
    if first_ns is None or first_ns < sent_ns:
      return {'other_uploads_before_first_byte': None,
              'other_upload_ms_before_first_byte': None,
              'other_upload_overlaps_response_read': None}
    others = [(start, end) for other_seq, start, end in windows if other_seq != seq]
    waits = [max(0, min(end, first_ns) - max(start, sent_ns)) for start, end in others]
    return {'other_uploads_before_first_byte': sum(x > 0 for x in waits),
            'other_upload_ms_before_first_byte': sum(waits) / 1e6,
            'other_upload_overlaps_response_read': int(any(start < received_ns and end > first_ns
                                                            for start, end in others))}

  def outstanding(self):
    with self.lock:
      return len(self.pending)

  def unfinished(self):
    """Account for replies taken from pending but not yet queued for draining."""
    with self.lock:
      return len(self.pending) + self.processing_results

  def request_stage_report(self, timeout=15.0):
    """Request the opt-in timing ring AFTER all image replies have drained.

    Uses the existing receiver; never competes with it for TCP reads.
    """
    if self.error is not None or self.unfinished():
      raise RuntimeError('Stage trace requires a healthy, idle inference pipeline')
    with self.changed:
      if self.pending_state_seq is not None:
        raise RuntimeError('Duplicate stage trace request')
      seq = self.peer.send(STATE_REQ, json.dumps({'ls2_stage_diagnostics':True}).encode())
      self.pending_state_seq = seq
    try:
      result = self.state_response.get(timeout=timeout)
    except queue.Empty as exc:
      raise TimeoutError('Android STATE_RESP stage trace timed out') from exc
    if self.error is not None:
      raise RuntimeError(f'Stage trace receiver failed: {self.error}')
    return result

  def _reader(self):
    try:
      while not self.stop_event.is_set():
        try:
          kind, seq, _flags, raw = self.peer.recv()
        except socket.timeout as exc:
          if self.outstanding() > 0:
            raise TimeoutError('Jetlink inference response timed out (3s with requests outstanding)') from exc
          continue
        received = time.perf_counter_ns()  # capture before optional socket ACK work
        if kind == INFER_RESP:
          self.peer.acknowledge_promptly()
        if kind == PROGRESS:
          continue
        if kind == STATE_RESP:
          with self.changed:
            if self.pending_state_seq != seq:
              raise RuntimeError(f'Unexpected STATE_RESP seq={seq}')
            self.pending_state_seq = None
          self.state_response.put(json.loads(raw))
          continue
        if kind == ERROR:
          try: detail = json.loads(raw)
          except Exception: detail = {'error': raw[:120].hex()}
          raise RuntimeError(f'Jetlink server error during inference: {detail}')
        if kind != INFER_RESP:
          raise RuntimeError(f'Unexpected reply type {kind} during pipelined inference')
        with self.changed:
          entry = self.pending.pop(seq, None)
          if entry is not None and self.peer.udp is not None:
            # Jetlink's UDP reassembler never answers incomplete older frames.
            older=[old for old in self.pending if old<seq]
            for old in older:self.pending.pop(old)
            self.udp_lost+=len(older)
          windows = tuple(self.send_windows)
          if entry is not None:
            self.processing_results += 1
            self.changed.notify_all()
        if entry is None:
          if self.peer.udp is not None:
            continue  # an answer for a UDP request already expired
          raise RuntimeError(f'Unmatched Jetlink inference response seq={seq}')
        try:
          item, t0, t_sent = entry
          rx = dict(self.peer.last_rx)
          timing = self.peer.decode_infer_reply(
              raw, item['frame_id'], t0, t_sent, received, rx=rx,
              capture_output=((self.dump_outputs_every > 0 and item.get('sample',0) % self.dump_outputs_every == 0)
                              or (self.prediction_every > 0 and item.get('sample',0) % self.prediction_every == 0)))
          timing.update(self.overlap_diagnostics(seq, t_sent, rx.get('first_ns'), received, windows))
          self.results.put((item, timing, received))
        finally:
          with self.changed:
            self.processing_results -= 1
            self.changed.notify_all()
    except BaseException as exc:
      if not self.stop_event.is_set():
        self.error = exc
        self.stop_event.set()

  def expire_udp(self,older_than_s=0.35):
    """A lost UDP frame has no reply. Keep its slot bounded and account for it."""
    if self.peer.udp is None:return
    cutoff=time.perf_counter_ns()-int(older_than_s*1e9)
    with self.changed:
      expired=[seq for seq,(_,t0,_) in self.pending.items() if t0<cutoff]
      for seq in expired:self.pending.pop(seq)
      self.udp_expired+=len(expired)
      if expired:
        self.changed.notify_all()

  def drain(self):
    out=[]
    while True:
      try: out.append(self.results.get_nowait())
      except queue.Empty: return out

  def wait_idle(self, deadline_s=4.0):
    until = time.monotonic()+deadline_s
    while self.unfinished() and time.monotonic()<until and self.error is None:
      self.expire_udp()
      time.sleep(.002)
    if self.error is not None:
      raise RuntimeError(f'Jetlink receive thread failed: {self.error}') from self.error
    if self.unfinished():
      raise TimeoutError(f'{self.unfinished()} unfinished Jetlink inference responses at end of benchmark')

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


def onroad_observation_allowed(sm):
  """Observe camera frames on-road only with live state; never touch controls.

  This does NOT assert that remote predictions are safe for driving. Its sole
  purpose is to permit a distinct, non-publishing performance observer to run
  in parallel with the stock authoritative modeld.
  """
  for key in ('deviceState', 'carState'):
    if not sm.seen.get(key, False):
      return False, f'Waiting for live {key} before on-road shadow'
    if not sm.alive.get(key, False):
      return False, f'Stale {key}; on-road shadow stopped'
  if not bool(sm['deviceState'].started):
    return False, 'Off-road transition; on-road shadow stopped'
  return True, None


def camera_frames(args,spec):
  """Yield real VisionIPC camera events, *before* doing the costly NV12 warp.

  A caller that is backed up must still consume events so the independent
  subscriber cannot accumulate stale frames while the Pixel has two jobs.
  This is deliberately separate from production modeld; it publishes nothing.
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
  extra=VisionIpcClient('camerad',VisionStreamType.VISION_STREAM_WIDE_ROAD,False) if has_both else None
  for client in [main]+([extra] if extra is not None else []):
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
  last_frame_id=None
  while RUNNING:
    buf0=main.recv()  # camera event is the pacing signal; no independent sleep
    received_ns=time.monotonic_ns()
    if buf0 is None:
      raise RuntimeError('Main camera stream ended')
    if extra is not None:
      buf1=extra.recv()
      received_ns=time.monotonic_ns()  # frame pair is available at this point
      if buf1 is None:
        raise RuntimeError('Wide camera stream ended')
    else:
      buf1=buf0
    sm.update(0)
    ok,reason=(onroad_observation_allowed(sm) if args.onroad_shadow else parked(sm))
    if not ok:
      # During initial startup wait briefly for live device/car state.
      if (reason.startswith(('No live carState', 'Waiting for live'))
          and time.monotonic()-start<args.camera_timeout):
        continue
      raise RuntimeError(reason)
    if not all(sm.seen.get(n,False) for n in ('liveCalibration','deviceState','roadCameraState')):
      if time.monotonic()-start<args.camera_timeout:continue
      raise RuntimeError('No live camera calibration / device / sensor metadata')
    calib=tuple(sm['liveCalibration'].rpyCalib)
    dev=str(sm['deviceState'].deviceType)
    sensor=str(sm['roadCameraState'].sensor)
    state=(calib,dev,sensor,main.width,main.height,extra.width if extra is not None else main.width,extra.height if extra is not None else main.height)
    if state!=last_calib:
      dc=DEVICE_CAMERAS[(dev,sensor)]
      f=np.asarray(calib,dtype=np.float32)
      main_mat=get_warp_matrix(f,dc.ecam.intrinsics if main_wide else dc.fcam.intrinsics,False).astype(np.float32)
      wide_mat=get_warp_matrix(f,dc.ecam.intrinsics,True).astype(np.float32)
      maps=[warp_indices(main.width,main.height,h*2,w*2,main_mat),
            warp_indices(extra.width if extra is not None else main.width,
                         extra.height if extra is not None else main.height,h*2,w*2,wide_mat)]
      last_calib=state
    timestamp_eof=int(main.timestamp_eof)
    frame_id=int(main.frame_id)
    if last_frame_id is not None and frame_id < last_frame_id:
      raise RuntimeError('camerad frame IDs moved backwards: refusing stale stream')
    last_frame_id=frame_id
    is_rhd=bool(sm['driverMonitoringState'].isRHD) if sm.seen.get('driverMonitoringState',False) else False
    pair_skew_ms=abs(int(main.timestamp_sof)-int(extra.timestamp_sof))/1e6 if extra is not None else 0.
    # No numpy.take here: caller checks capacity and frame age first.
    yield (frame_id,timestamp_eof,received_ns,buf0,buf1,maps,out,is_rhd,pair_skew_ms)


def prepare_camera_event(event):
  """Warp one accepted frame, reusing its preallocated packed-image buffer."""
  _fid,_capture,_received,b0,b1,maps,out,_rhd,_skew=event
  started=time.perf_counter_ns()
  for index,b in enumerate((b0,b1)):
    source=np.frombuffer(b.data,dtype=np.uint8)
    np.take(source,maps[index],out=out[index].reshape(-1))
  return out,ns_ms(time.perf_counter_ns()-started)


def camera_admission(*, frame_id, last_frame_id, capture_ns, now_ns,
                     busy=False, max_age_ms=100.0):
  """Drop stale/busy/repeated camera events without preprocessing them."""
  if last_frame_id is not None and frame_id<=last_frame_id:
    return 'duplicate'
  if busy:
    return 'pipeline_full'
  if max_age_ms>0 and (now_ns-capture_ns)>max_age_ms*1e6:
    return 'stale'
  return 'accept'


def stall_analysis(rows, threshold_ms=80.):
  """Correlate slow first-byte replies with competing uploads, without inferring cause."""
  values = [r for r in rows if r.get('response_first_byte_wait_ms') is not None]
  slow = [r for r in values if r['response_first_byte_wait_ms'] >= threshold_ms]
  fast = [r for r in values if r['response_first_byte_wait_ms'] < threshold_ms]
  def group(group_rows):
    overlaps = [r.get('other_uploads_before_first_byte') for r in group_rows]
    measured = [x for x in overlaps if x is not None]
    npu = [r['server_npu_ms'] for r in group_rows if r.get('server_npu_ms') is not None]
    overhead = [r['link_overhead_ms'] for r in group_rows if r.get('link_overhead_ms') is not None]
    return {'frames':len(group_rows),
            'npu_ms_mean':round(sum(npu)/len(npu),3) if npu else None,
            'link_overhead_ms_mean':round(sum(overhead)/len(overhead),3) if overhead else None,
            'other_uploads_during_wait_mean':(round(sum(measured)/len(measured),3)
                                              if measured else None),
            'any_other_uploads_percent':(round(100*sum(x>0 for x in measured)/len(measured),1)
                                         if measured else None)}
  return {'first_byte_stall_threshold_ms':threshold_ms,
          'stalled_frames':group(slow),'non_stalled_frames':group(fast),
          'note':'Counts overlapping comma send() intervals; does not capture USB/network queuing or Android scheduling.'}


def bench(args):
  global SHA, NBYTES
  SHA, NBYTES = MODEL_PROFILES[args.model]
  output=Path(args.output).expanduser()
  output.mkdir(parents=True,exist_ok=False)
  meta={'version':VERSION,'mode':args.mode,'host':args.host,'port':args.port,'rate_requested_hz':args.rate,
        'seconds_requested':args.seconds,'model_profile':args.model,'model_sha256':SHA,'no_control':True,'onroad_shadow':args.onroad_shadow,'cwd':os.getcwd(),
        'pipeline_depth':args.pipeline_depth,'transport':args.transport,
        'udp_send_mode':args.udp_send_mode,
        'tcp_quickack':args.tcp_quickack,
        'dump_outputs_every':args.dump_outputs_every,
        'compare_local_modelv2':getattr(args,'compare_local',False),
        'shadow_prediction_feed':getattr(args,'shadow_predictions',False),
        'prediction_every':getattr(args,'prediction_every',None) if getattr(args,'shadow_predictions',False) else None,
        'pacing':('camera_event' if args.mode=='camera' and args.pacing!='timer' else 'timer'),
        'max_camera_age_ms':args.max_camera_age_ms,
        'reply_guard_ms':args.reply_guard_ms,'reply_guard_min_age_ms':args.reply_guard_min_age_ms,
        'started_utc':datetime.now(timezone.utc).isoformat()}
  records=[]
  output_snapshots=[]
  errors=[]
  counts={'frames':0,'frames_sent':0,'over_50_ms':0,'capture_to_reply_over_50_ms':0,
          'source_frames_skipped':0,'pipeline_full_skips':0,'preprocess_count':0,
          'camera_events':0,'stale_frames_dropped':0,'repeated_camera_frames':0,
          'camera_pair_skew_over_10ms':0,
          'reply_guard_waits':0,'reply_guard_released':0,'reply_guard_timed_out':0,
          'reply_guard_waited_ms_total':0.,
          'udp_lost_replies':0,'udp_expired_replies':0}
  peer=None
  pipeline=None
  began=None
  stage_report=None
  stage_rows=[]
  joined_stage_rows=[]
  comparator=None
  comparison_summary=None
  prediction_recorder=None
  prediction_summary=None
  try:
    meta['physical']=check_route(args.host,args.port,require_usb=not args.allow_other_network,
                                  require_panda=not args.skip_panda_check)
    peer=JetlinkTCP(args.host,args.port,args.timeout)
    if args.tcp_quickack:
      peer.configure_tcp_quickack(True)
    meta['usb0_before']=usb0_stats()
    hello=peer.hello(usb_link=not args.allow_other_network)
    meta['ls2_stage_diagnostics_supported']=bool(hello.get('ls2_stage_diagnostics'))
    if args.fetch_stage_report and not meta['ls2_stage_diagnostics_supported']:
      raise RuntimeError('Stage tracing unavailable in this Android Jetlink app. Install the signed V0.5 diagnostics APK first; original V5.4 still works without --fetch-stage-report.')
    meta['ping_rtt_ms']=pct(peer.ping_samples(args.pings))
    meta['udp_advertised']=bool(hello.get('frame_port') and hello.get('frame_token'))
    if args.transport=='udp':
      peer.enable_datagrams(hello)
    peer.udp_payload_bytes=args.udp_payload_bytes
    peer.udp_send_mode=args.udp_send_mode
    meta['udp_payload_bytes']=args.udp_payload_bytes if peer.udp else None
    meta['datagram_port']=peer.datagram_port
    meta['server']={k:hello.get(k) for k in ('loaded','backend','device','runtime_version','protocol','engine_state')}
    spec=peer.load_existing()
    meta['spec']={k:spec.get(k) for k in ('sha256','frame_skip','input_shapes','output_shapes','output_slices')}
    if getattr(args,'compare_local',False):
      from openpilot.jetlink_prediction_compare import PredictionObserver
      comparator=PredictionObserver(output,spec)
      comparator.poll()
    if getattr(args,'shadow_predictions',False):
      from openpilot.jetlink_prediction_shadow import ShadowPredictionRecorder
      prediction_recorder=ShadowPredictionRecorder(output,spec)
    meta['wire_payload_bytes']={'upload':INFER_HEADER.size+warped_bytes(spec)+scalar_count(spec)*4,
                                'download':INFER_REPLY.size+output_reply_count(spec)*4}
    print(json.dumps({'connected':True,'mode':args.mode,'server':meta['server'],
                      'pipeline_depth':args.pipeline_depth, 'transport':args.transport,
                      'udp_send_mode':args.udp_send_mode,
                      'udp_advertised':meta['udp_advertised'], 'idle_ping_ms':meta['ping_rtt_ms'],
                      'wire_payload_bytes':meta['wire_payload_bytes']},indent=2),flush=True)
    fields=['sample','frame_id','capture_age_ms','preprocess_ms','wire_ms','send_ms','recv_ms',
            'link_overhead_ms','capture_to_reply_ms','camera_delivery_age_ms','camera_event_to_send_ms',
                                    'camera_pair_skew_ms',
                                    'server_total_ms','server_npu_ms','server_queue_ms',
            'model_outputs','over_50ms','preprocess_plus_wire_ms',
            'reply_rx_interval_ms','send_start_interval_ms',
            'response_first_byte_wait_ms','response_body_read_ms','response_header_read_ms',
            'post_upload_unattributed_ms','preprocess_done_to_upload_begin_ms',
            'capture_to_upload_begin_ms','receiver_result_queue_delay_ms',
            'response_read_syscalls','tcp_rtt_ms','tcp_retrans_total','tcp_unacked',
            'outstanding_at_send','oldest_outstanding_age_ms',
            'other_uploads_before_first_byte','other_upload_ms_before_first_byte',
            'other_upload_overlaps_response_read']
    path=output/'frames.csv'
    # Keep the fast path free of synchronous CSV disk I/O. At most 900 s * 20 Hz
    # gives 18,000 small result dicts; write once in finally, even on exceptions.
    began=time.monotonic()
    next_due=began
    last_source_id=None
    last_accepted_id=None
    event_paced=(args.mode=='camera' and args.pacing!='timer')
    camera=camera_frames(args,spec) if args.mode=='camera' else None
    rng=np.random.default_rng(0xC0A0)
    scratch=(rng.integers(0,256,(2,6,*model_hw(spec)),dtype=np.uint8) if args.mode=='synthetic' else None)
    scalars_lhd=packed_scalars(spec,False,0.)
    scalars_rhd=packed_scalars(spec,True,0.)
    last_progress=time.monotonic()
    n=0
    last_reply_rx_ns=None
    last_send_start_ns=None
    if args.pipeline_depth:
      pipeline=PipelinedInference(peer,args.pipeline_depth,dump_outputs_every=args.dump_outputs_every,
                                  prediction_every=(args.prediction_every if args.shadow_predictions else 0))
      pipeline.start()

    def record_result(item,timing,received):
      nonlocal last_progress,last_reply_rx_ns
      snapshot=timing.pop('_output',None)
      if snapshot is not None:
        sample_id = item['sample']
        capture_ns = item.get('capture_ns')
        if args.dump_outputs_every > 0 and sample_id % args.dump_outputs_every == 0:
          output_snapshots.append((sample_id,item['frame_id'],snapshot))
          if comparator is not None and capture_ns is not None:
            comparator.poll()
            comparator.offer_remote(item['frame_id'],capture_ns,received,snapshot)
        if (prediction_recorder is not None and capture_ns is not None
            and sample_id % args.prediction_every == 0):
          prediction_recorder.offer(item['frame_id'],capture_ns,received,snapshot,
                                    npu_ms=timing.get('server_npu_ms'))
      pre_ms=item['preprocess_ms']
      capture_ns=item['capture_ns']
      capture_to_reply=(received-capture_ns)/1e6 if capture_ns is not None else None
      total=timing['wire_ms']+pre_ms
      # True receive-completion cadence. Distinct from processing latency and
      # dequeue lag; measured on the receiver thread's timestamp, not now().
      reply_rx_interval_ms=(ns_ms(received-last_reply_rx_ns)
                            if last_reply_rx_ns is not None else None)
      last_reply_rx_ns=received
      row={'sample':item['sample'],'frame_id':item['frame_id'],'capture_age_ms':item['capture_age_ms'],
           'preprocess_ms':pre_ms,'wire_ms':timing['wire_ms'],'send_ms':timing['send_ms'],
           'recv_ms':timing['recv_ms'],'link_overhead_ms':timing['link_overhead_ms'],
           'capture_to_reply_ms':capture_to_reply,
           'camera_delivery_age_ms':item.get('camera_delivery_age_ms'),
           'camera_event_to_send_ms':item.get('camera_event_to_send_ms'),
           'camera_pair_skew_ms':item.get('camera_pair_skew_ms'),
           'preprocess_done_to_upload_begin_ms':(
             ns_ms(item['send_start_ns']-item['prep_finished_ns'])
             if item.get('send_start_ns') is not None and item.get('prep_finished_ns') is not None else None),
           'capture_to_upload_begin_ms':(
             ns_ms(item['send_start_ns']-capture_ns)
             if item.get('send_start_ns') is not None and capture_ns is not None else None),
           'receiver_result_queue_delay_ms':ns_ms(time.perf_counter_ns()-received),
           'post_upload_unattributed_ms':timing.get('post_upload_unattributed_ms'),
           'server_total_ms':timing['server_total_ms'],
           'server_npu_ms':timing['server_npu_ms'],'server_queue_ms':timing['server_queue_ms'],
           'model_outputs':timing['output_floats'],'over_50ms':int(total>50.),
           'preprocess_plus_wire_ms':total,
           'reply_rx_interval_ms':reply_rx_interval_ms,
           'send_start_interval_ms':item.get('send_start_interval_ms'),
           'outstanding_at_send':item.get('outstanding_at_send'),
           'oldest_outstanding_age_ms':item.get('oldest_outstanding_age_ms'),
           **{k:timing.get(k) for k in ('response_first_byte_wait_ms','response_body_read_ms',
               'response_header_read_ms','response_read_syscalls','tcp_rtt_ms','tcp_retrans_total','tcp_unacked',
               'other_uploads_before_first_byte','other_upload_ms_before_first_byte',
               'other_upload_overlaps_response_read')}}
      records.append(row)
      counts['frames']+=1
      counts['over_50_ms']+=int(total>50.)
      counts['capture_to_reply_over_50_ms']+=int(capture_to_reply is not None and capture_to_reply>50.)
      if args.progress_seconds and time.monotonic()-last_progress>=args.progress_seconds:
        last_progress=time.monotonic()
        print(f"{counts['frames']} replies; NPU {timing['server_npu_ms']:.1f}ms, "
              f"round-trip {timing['wire_ms']:.1f}ms, pipeline outstanding "
              f"{pipeline.outstanding() if pipeline else 0}",flush=True)

    while RUNNING and time.monotonic()-began < args.seconds:
      if comparator is not None:
        comparator.poll()
      if pipeline:
        pipeline.expire_udp()
        for item,timing,received in pipeline.drain():
          record_result(item,timing,received)
        if pipeline.error is not None:
          raise RuntimeError(f'Jetlink receiver failed: {pipeline.error}') from pipeline.error
      # Synthetic and explicit timer A/B keep their original software cadence.
      # Camera-event mode instead blocks on VisionIPC recv, just like the
      # production modeld pipeline, with NO independent 20-Hz sleep.
      if not event_paced and args.rate>0 and time.monotonic()<next_due:
        time.sleep(max(0.,next_due-time.monotonic()))
        continue
      if not event_paced and args.rate>0:
        # Optional A/B guard: avoid beginning a large UDP upload in the few
        # milliseconds around a pending reply. Keep BOTH slots available and
        # cap the wait. Default zero means timing is V5.1-comparable.
        if pipeline and peer.udp is not None and args.reply_guard_ms > 0:
          waited, did_wait, released = pipeline.await_reply_window(
              max_wait_ms=args.reply_guard_ms, min_age_ms=args.reply_guard_min_age_ms)
          if did_wait:
            counts['reply_guard_waits'] += 1
            counts['reply_guard_waited_ms_total'] += waited
            counts['reply_guard_released' if released else 'reply_guard_timed_out'] += 1
            # The reader may have accumulated results while we waited.
            for item,timing,received in pipeline.drain():
              record_result(item,timing,received)
        next_due=max(next_due+1.0/args.rate,time.monotonic())
      if camera is not None:
        event=next(camera)  # consume EVERY camerad event even when full
        frame_id,capture_ns,received_ns,*_=event
        counts['camera_events']+=1
        counts['camera_pair_skew_over_10ms']+=int(event[8]>10.)
        if last_source_id is not None and frame_id>last_source_id+1:
          counts['source_frames_skipped']+=frame_id-last_source_id-1
        if last_source_id is None or frame_id>last_source_id:
          last_source_id=frame_id
        busy=bool(pipeline and pipeline.outstanding()>=pipeline.depth)
        decision=camera_admission(frame_id=frame_id,last_frame_id=last_accepted_id,
                   capture_ns=capture_ns,now_ns=time.monotonic_ns(),busy=busy,
                   max_age_ms=args.max_camera_age_ms)
        if decision!='accept':
          if decision=='pipeline_full':counts['pipeline_full_skips']+=1
          elif decision=='stale':counts['stale_frames_dropped']+=1
          else:counts['repeated_camera_frames']+=1
          continue
        warped,pre_ms=prepare_camera_event(event)
        prep_finished_ns=time.perf_counter_ns()
        counts['preprocess_count']+=1
        is_rhd=event[7]
        age_before=max(0.,ns_ms(time.monotonic_ns()-capture_ns))
        camera_delivery_age_ms=max(0.,ns_ms(received_ns-capture_ns))
        camera_pair_skew_ms=event[8]
      else:
        if pipeline and pipeline.outstanding()>=pipeline.depth:
          counts['pipeline_full_skips']+=1
          continue
        warped=scratch
        warped[0,0,0,0]=n%256
        frame_id=n;pre_ms=0.;is_rhd=False;age_before=None;capture_ns=None
        received_ns=None;camera_delivery_age_ms=None;camera_pair_skew_ms=None
        prep_finished_ns=None
      packed=scalars_rhd if is_rhd else scalars_lhd
      item={'sample':n,'frame_id':frame_id,'capture_age_ms':age_before,
            'capture_ns':capture_ns,'preprocess_ms':pre_ms,
            'prep_finished_ns':prep_finished_ns,
            'camera_delivery_age_ms':camera_delivery_age_ms,
            'camera_pair_skew_ms':camera_pair_skew_ms,
            'camera_event_to_send_ms':ns_ms(time.monotonic_ns()-received_ns) if received_ns else None}
      if pipeline:
        if not pipeline.send_frame(item,warped,packed,reset=(counts['frames_sent']==0)):
          counts['pipeline_full_skips']+=1
        else:
          item['send_start_interval_ms']=(ns_ms(item['send_start_ns']-last_send_start_ns)
                                           if last_send_start_ns is not None else None)
          last_send_start_ns=item['send_start_ns']
          counts['frames_sent']+=1
          if camera is not None:last_accepted_id=frame_id
      else:
        item['send_start_ns']=time.perf_counter_ns()
        item['send_start_interval_ms']=(ns_ms(item['send_start_ns']-last_send_start_ns)
                                         if last_send_start_ns is not None else None)
        last_send_start_ns=item['send_start_ns']
        timing=peer.infer(warped,packed,frame_id,reset=(counts['frames_sent']==0),
                          capture_output=((args.dump_outputs_every>0 and n%args.dump_outputs_every==0)
                                          or (args.shadow_predictions and n%args.prediction_every==0)))
        timing['link_overhead_ms']=timing['wire_ms']-timing['server_total_ms']
        counts['frames_sent']+=1
        if camera is not None:last_accepted_id=frame_id
        record_result(item,timing,time.monotonic_ns())
      n+=1
    if pipeline:
      pipeline.wait_idle()
      for item,timing,received in pipeline.drain():
        record_result(item,timing,received)
    # Exclude diagnostic transfer from the timed test. Existing INFER_RESP bytes
    # and UDP/TCP hot paths remain unchanged; STATE_REQ is sent after draining.
    timed_end=time.monotonic()
    if args.fetch_stage_report:
      if pipeline:
        stage_report=pipeline.request_stage_report()
      else:
        seq=peer.send(STATE_REQ,json.dumps({'ls2_stage_diagnostics':True}).encode())
        stage_report=json.loads(peer.expect(STATE_RESP,seq,15.0))
      stage_payload=stage_report.get('ls2_stage_diagnostics')
      if not isinstance(stage_payload,dict) or stage_payload.get('schema')!=1:
        raise RuntimeError('Android STATE_RESP lacked LS2 stage-diagnostics schema v1')
      stage_rows=stage_payload.get('frames',[])
      if not stage_rows:
        raise RuntimeError('Android returned no frame stage records')
      counts['stage_trace_records']=len(stage_rows)
    counts['elapsed_s']=round(timed_end-began,3)
    counts['achieved_fps']=round(counts['frames']/max(.001,counts['elapsed_s']),3)
    counts['sent_fps']=round(counts['frames_sent']/max(.001,counts['elapsed_s']),3)
    if pipeline:
      counts['udp_lost_replies']=pipeline.udp_lost
      counts['udp_expired_replies']=pipeline.udp_expired
  except (KeyboardInterrupt,StopIteration):
    errors.append('Stopped or camerad finished before benchmark duration')
  except Exception as exc:
    errors.append(f'{type(exc).__name__}: {exc}')
    (output/'error_traceback.txt').write_text(traceback.format_exc())
    print('Benchmark error: '+errors[-1]+' (see error_traceback.txt)',file=sys.stderr,flush=True)
  finally:
    # These artifacts are generated after benchmark measurement, never during
    # the 20 Hz timed loop. Clock-domain differences preclude direct subtraction
    # of phone and comma timestamps; only within-device durations are compared.
    if stage_report is not None:
      try:
        (output/'phone_stage_report.json').write_text(json.dumps(stage_report,indent=2)+'\n')
        by_frame={int(s['frame_id']):s for s in stage_rows if 'frame_id' in s}
        for row in records:
          phone=by_frame.get(int(row['frame_id']))
          if phone is None:continue
          merged={'frame_id':row['frame_id'],'sample':row['sample'],
                  'capture_to_reply_ms':row.get('capture_to_reply_ms'),
                  'wire_ms':row.get('wire_ms'),
                  'post_upload_unattributed_ms':row.get('post_upload_unattributed_ms'),
                  'server_total_ms':row.get('server_total_ms')}
          merged.update({k.replace('_us','_ms'):round(v/1000.,6) if k.endswith('_us') and isinstance(v,(int,float)) and v>=0 else v
                         for k,v in phone.items()})
          joined_stage_rows.append(merged)
        if joined_stage_rows:
          keys=list(dict.fromkeys(k for row in joined_stage_rows for k in row))
          with (output/'joined_stage_frames.csv').open('w',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=keys)
            writer.writeheader()
            writer.writerows(joined_stage_rows)
      except (OSError,ValueError,TypeError,KeyError) as exc:
        errors.append(f'Phone stage trace export failed: {exc}')
    if pipeline:
      # Preserve completed in-flight replies even on an error.
      late = pipeline.drain()
      if late and 'record_result' in locals():
        for item,timing,received in late:
          record_result(item,timing,received)
      pipeline.close()
    elif peer:
      peer.close()
    if began is not None and 'elapsed_s' not in counts:
      counts['elapsed_s']=round(time.monotonic()-began,3)
      counts['achieved_fps']=round(counts['frames']/max(.001,counts['elapsed_s']),3)
      counts['sent_fps']=round(counts['frames_sent']/max(.001,counts['elapsed_s']),3)
    # Flush results once, outside the camera/network hot path.
    if 'fields' in locals() and 'path' in locals():
      try:
        with path.open('w',newline='') as f:
          writer=csv.DictWriter(f,fieldnames=fields)
          writer.writeheader()
          writer.writerows(records)
      except OSError as exc:
        errors.append(f'CSV write error: {exc}')
    if comparator is not None:
      try:
        comparison_summary=comparator.finish()
      except Exception as exc:
        errors.append(f'Local model agreement report failed: {type(exc).__name__}: {exc}')
    if prediction_recorder is not None:
      try:
        prediction_summary=prediction_recorder.finish()
      except Exception as exc:
        errors.append(f'Shadow prediction report failed: {type(exc).__name__}: {exc}')
    if output_snapshots:
      try:
        np.savez_compressed(output/'output_samples.npz',
                            sample=np.array([x[0] for x in output_snapshots],dtype=np.int64),
                            frame_id=np.array([x[1] for x in output_snapshots],dtype=np.int64),
                            outputs=np.stack([x[2] for x in output_snapshots]))
        meta['captured_output_samples']=len(output_snapshots)
      except (OSError,ValueError) as exc:
        errors.append(f'Output snapshot write failed: {exc}')
    if peer:
      meta['udp_datagrams_sent']=peer.udp_datagrams_sent
      meta['udp_send_syscalls']=peer.udp_send_syscalls
      meta['udp_batches_sent']=getattr(peer,'udp_batches_sent',0)
      meta['udp_syscalls_per_datagram']=round(peer.udp_send_syscalls/max(1,peer.udp_datagrams_sent),3)
      meta['tcp_quickack_sets']=getattr(peer,'tcp_quickack_sets',0)
    meta['finished_utc']=datetime.now(timezone.utc).isoformat()
    if pipeline:
      counts['udp_lost_replies']=pipeline.udp_lost
      counts['udp_expired_replies']=pipeline.udp_expired
    meta['usb0_after']=usb0_stats()
    if 'usb0_before' in meta:
      meta['usb0_counters_delta']=usbstats_diff(meta['usb0_before'],meta['usb0_after'])
    stage_metrics={k.replace('_us','_ms'):pct([r[k] for r in joined_stage_rows if isinstance(r.get(k),(int,float)) and r[k]>=0])
                   for k in ('datagram_assembly_ms','datagram_ready_to_handle_ms','host_lock_wait_ms',
                             'infer_call_ms','before_reply_ms','reply_pack_ms','reply_tcp_write_ms','reply_write_ms','handle_to_reply_ms')
                   if any(isinstance(r.get(k),(int,float)) and r[k]>=0 for r in joined_stage_rows)}
    # Frames above 50ms here mean preprocess+transport roundtrip, NOT an
    # inter-frame/cycle deadline. Report cycle and age-at-reply separately.
    cadence=[r['reply_rx_interval_ms'] for r in records
             if r.get('reply_rx_interval_ms') is not None]
    cycle_p95=pct(cadence).get('p95') if cadence else None
    elapsed=counts.get('elapsed_s',0.)
    gate_qualified=bool(args.mode=='camera' and elapsed >= 180. and counts['frames'] >= 2)
    no_errors=not errors and counts.get('udp_lost_replies',0)==0 and counts.get('udp_expired_replies',0)==0
    gate_pass=bool(gate_qualified and no_errors and counts['achieved_fps']>=20. and
                   cycle_p95 is not None and cycle_p95<=50.)
    summary={'meta':meta,'counts':counts,'errors':errors,
             'prediction_comparison': comparison_summary,
             'shadow_prediction_feed': prediction_summary,
             'cadence_gate':{'qualified':gate_qualified,'pass':gate_pass if gate_qualified else None,
                             'reason':('180s camera run required for 20 FPS / p95 <= 50ms gate'
                                       if not gate_qualified else 'observational shadow-only cadence gate'),
                             'throughput_min_fps':20.,'reply_cycle_p95_max_ms':50.,
                             'reply_cycle_p95_ms':cycle_p95,
                             'capture_to_reply_is_separate':True},
             'phone_stage_trace_available':bool(stage_report),
             'phone_stage_timing_ms':stage_metrics,
             'phone_stage_trace_note':'Phone *_us durations are converted into ms here; phone and comma monotonic clock epochs are not aligned. Reassembly begins on the first accepted UDP packet in userspace, not the kernel receive timestamp. Timings are observational only.', 'inference_verified':not errors and counts['frames']>0,
             'controls_affected':False,
             'data_interpretation':'Independent passive on-road shadow only' if args.onroad_shadow else 'Independent parked-only shadow probe',
             'timing_ms':{k:pct([r[k] for r in records if r[k] is not None])
                          for k in ('preprocess_ms','wire_ms','send_ms','recv_ms','link_overhead_ms',
                                    'capture_to_reply_ms','reply_rx_interval_ms','send_start_interval_ms',
                                    'preprocess_plus_wire_ms','camera_delivery_age_ms','camera_event_to_send_ms',
                                    'camera_pair_skew_ms',
                                    'server_total_ms','server_npu_ms','server_queue_ms',
                                    'response_first_byte_wait_ms','response_body_read_ms','response_header_read_ms',
                                    'post_upload_unattributed_ms','preprocess_done_to_upload_begin_ms',
                                    'capture_to_upload_begin_ms','receiver_result_queue_delay_ms',
                                    'response_read_syscalls','tcp_rtt_ms','tcp_unacked',
                                    'outstanding_at_send','oldest_outstanding_age_ms',
                                    'other_uploads_before_first_byte','other_upload_ms_before_first_byte',
                                    'other_upload_overlaps_response_read')},
             'latency_budget_note':('post_upload_unattributed_ms = response_first_byte_wait_ms - '
               'server_total_ms; it aggregates network, Android processing outside server timer, '
               'scheduling and response propagation, not pure network delay. This is not an Android '
               'per-stage timestamp. Capture-to-reply is computed using comma capture clocks.'),
             'stall_analysis':stall_analysis(records),
             'gate_note':'A successful connected benchmark is not approval for model authority.'}
    if args.model == 'ctm':
      # This is the COMPLETE comma-side preprocessing start -> Jetlink reply
      # interval, distinct from server inference and distinct from camera age.
      # Pipelining can satisfy 20Hz throughput while per-frame latency exceeds
      # 50 ms; never confuse cadence with end-to-end per-frame latency.
      complete = [float(r['preprocess_plus_wire_ms']) for r in records
                  if r.get('preprocess_plus_wire_ms') is not None]
      captured = [float(r['capture_to_reply_ms']) for r in records
                  if r.get('capture_to_reply_ms') is not None]
      summary['ctm_parked_integration'] = {
        'source_sha256': SHA,
        'mode': 'non_publishing_read_only_shadow',
        'complete_path_metric': 'preprocess_plus_wire_ms',
        'complete_path_ms': pct(complete),
        'complete_path_over_50_ms': sum(v > 50 for v in complete),
        'complete_path_all_below_50_ms': bool(complete) and all(v <= 50 for v in complete),
        'capture_to_reply_ms': pct(captured),
        'capture_to_reply_over_50_ms': sum(v > 50 for v in captured),
        'measured_frames': len(complete),
        'authority_qualified': False,
        'authority_enabled': False,
        'full_numerical_parity_verified': False,
        'phone_only_20hz_is_not_connected_20hz': True,
        'note': '50 ms complete path excludes upstream camerad buffering; capture-to-reply includes it. Pipelined throughput is separate from single-frame latency.',
      }
    (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({'output':str(output),'counts':counts,'errors':errors,'timing_ms':summary['timing_ms']},indent=2),flush=True)
    return 0 if not errors and counts['frames'] else 1


def parse_args(argv=None):
  p=argparse.ArgumentParser(description=__doc__)
  p.add_argument('--model', choices=('ls2', 'ctm'), default='ls2',
                 help='Pinned phone source model: LS2 (default) or Cinque Terre Original (2026-09-04)')
  p.add_argument('--mode',choices=('synthetic','camera'),default='synthetic',help='camera: real NV12 frames, 2 cameras; synthetic: full-size Jetlink LS2 tensors')
  p.add_argument('--onroad-shadow',action='store_true',
                 help='PASSIVE observation only: allow real on-road camera frames, never publish predictions or control anything')
  p.add_argument('--compare-local',action='store_true',
                 help='Sample same-frame local modelV2 geometry/probabilities without changing modeld; no parity/authority claims')
  p.add_argument('--shadow-predictions',action='store_true',
                 help='Opt-in read-only LS2 geometry feed (JSONL+atomic latest); never publishes modeld predictions')
  p.add_argument('--prediction-every',type=int,default=2,
                 help='Record one of every N remote predictions to the shadow feed; 2 is ~10Hz at 20FPS')
  p.add_argument('--host',default='10.169.232.180')
  p.add_argument('--port',type=int,default=5599)
  p.add_argument('--rate',type=float,default=5.0,
                 help='Timer-mode/synthetic pacing Hz; ignored in camera-event mode, where camerad sets cadence')
  p.add_argument('--pacing',choices=('camera','timer'),default='timer',
                 help='Camera mode: timer=stable 20 Hz (default), camera=event-paced regression A/B only')
  p.add_argument('--reply-guard-ms',type=float,default=0.,
                 help='Optional 0..12 ms wait for an imminent previous TCP reply before UDP upload (0=off)')
  p.add_argument('--reply-guard-min-age-ms',type=float,default=40.,
                 help='Guard only when oldest in-flight upload finished at least this many ms ago (default 40)')
  p.add_argument('--max-camera-age-ms',type=float,default=100.,
                 help='Reject camerad frames older than this before sending; 0 disables (default 100 ms)')
  p.add_argument('--seconds',type=float,default=60.)
  p.add_argument('--timeout',type=float,default=5.)
  p.add_argument('--transport',choices=('tcp','udp'),default='tcp',
                 help='TCP baseline or UDP frame upload (only when Android server advertises UDP port/token)')
  p.add_argument('--pings',type=int,default=10,help='Idle Jetlink PING roundtrips before model test')
  p.add_argument('--tcp-quickack',action='store_true',
                 help='Opt-in Linux TCP_QUICKACK for reply transport A/B; default OFF (V5.5-equivalent). No phone APK change.')
  p.add_argument('--fetch-stage-report',action='store_true',
                 help='Require V0.5 instrumented Jetlink APK and fetch 5000-frame phone timing ring after test. No extra per-frame transport.')
  p.add_argument('--dump-outputs-every',type=int,default=0,
                 help='Opt-in: save one LS2 output vector per N completed frames to NPZ for offline parity work (0=off)')
  p.add_argument('--udp-send-mode',choices=('sendmsg','sendmmsg'),default='sendmsg',
                 help='UDP upload: V5.2 sendmsg default, or opt-in Linux sendmmsg syscall batching A/B')
  p.add_argument('--udp-payload-bytes',type=int,choices=(8192,16384,32768,65000),default=65000,
                 help='Experimental UDP frame fragment size (8K/16K/32K/65K); 65K is unchanged V3 baseline')
  p.add_argument('--progress-seconds',type=float,default=5.,
                 help='Progress print interval in seconds (0 disables, default 5)')
  p.add_argument('--pipeline-depth',type=int,choices=(0,2,3),default=0,
                 help='0=legacy synchronous (A/B baseline); 2=two outstanding requests; 3=experimental. Throughput only, never vehicle control.')
  p.add_argument('--camera-timeout',type=float,default=25.)
  p.add_argument('--output',default=f'/data/jetlink-shadow-{datetime.now().strftime("%Y%m%d-%H%M%S")}')
  p.add_argument('--allow-other-network',action='store_true',help='Disable USB-only route check')
  p.add_argument('--skip-panda-check',action='store_true',help='Disable xnor panda enumeration check')
  args=p.parse_args(argv)
  if not(0<args.rate<=20) or not(0<args.seconds<=900):p.error('rate 0..20 and seconds 0..900 required')
  if not(0<=args.pings<=100):p.error('pings must be 0..100')
  if not(0<=args.dump_outputs_every<=10000):p.error('dump outputs every must be 0..10000')
  if not(0<=args.progress_seconds<=60):p.error('progress interval must be 0..60 seconds')
  if not(0<=args.max_camera_age_ms<=1000):p.error('max camera age must be 0..1000 ms')
  if not(0<=args.reply_guard_ms<=12):p.error('reply guard must be 0..12 ms')
  if not(0<=args.reply_guard_min_age_ms<=150):p.error('reply guard minimum age must be 0..150 ms')
  if args.reply_guard_ms and (args.pacing != 'timer' or args.transport != 'udp' or args.pipeline_depth == 0):
    p.error('reply guard requires --pacing timer --transport udp --pipeline-depth 2 or 3')
  if args.transport=='udp' and args.pipeline_depth==0:p.error('UDP requires --pipeline-depth 2 for bounded loss tracking')
  if args.transport=='udp' and args.allow_other_network:p.error('UDP frames only permitted with verified usb0 routing')
  if args.compare_local and (not args.onroad_shadow or args.dump_outputs_every==0):
    p.error('--compare-local requires --onroad-shadow and --dump-outputs-every > 0')
  if not 1 <= args.prediction_every <= 100:
    p.error('--prediction-every must be 1..100')
  if args.shadow_predictions and not args.onroad_shadow:
    p.error('--shadow-predictions requires --onroad-shadow')
  if args.onroad_shadow and (args.mode!='camera' or args.transport!='udp' or args.allow_other_network or args.skip_panda_check):
    p.error('--onroad-shadow requires --mode camera --transport udp, verified USB and xnor panda')
  if args.udp_send_mode=='sendmmsg' and args.transport!='udp':p.error('sendmmsg requires --transport udp')
  return args


def main():
  signal.signal(signal.SIGINT,stop_now)
  signal.signal(signal.SIGTERM,stop_now)
  return bench(parse_args())


if __name__=='__main__':
  sys.exit(main())
