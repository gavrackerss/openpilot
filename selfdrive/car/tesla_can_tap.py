"""XNOR V230 passive in-process CAN recorder.

No messaging sockets, no panda access, no actuation, no safety changes.
The existing card CAN subscription and existing sendcan producer feed this observer.
"""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import threading
import time

ARM_FILE = '/data/xnor_enable_v230_can_capture'
ROOT = '/data/xnor_v230_captures'
CAN_HEADER = ('mono_ns', 'direction', 'src', 'bus', 'tx_echo', 'address', 'data')
STATE_HEADER = ('mono_ns', 'pcm_cruise', 'op_enabled', 'long_active', 'requested_accel',
                'v_ego', 'a_ego', 'gas_pressed', 'brake_pressed', 'raw_di_state',
                'can_valid', 'panda_states')
MAX_CAN_BYTES = 150 * 1024 * 1024


def _frame(frame):
  if isinstance(frame, (tuple, list)):
    return int(frame[0]), bytes(frame[1]), int(frame[2])
  return int(frame.address), bytes(frame.dat), int(frame.src)


class TeslaCanTap:
  def __init__(self, root: str = ROOT, session: str | None = None,
               max_can_bytes: int = MAX_CAN_BYTES) -> None:
    self.root = Path(root)
    self.session = session or (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ') + '-' + str(os.getpid()))
    self.path = self.root / self.session
    self.max_can_bytes = max_can_bytes
    self.q: queue.Queue[tuple[str, list[tuple]]] = queue.Queue(maxsize=2048)
    self.drop_batches = 0
    self.drop_frames = 0
    self.invalid_frames = 0
    self.stopped_by_size = False
    self._failed = False
    self._state_tick = 0
    self._end = threading.Event()
    self._thread = threading.Thread(target=self._write, name='v230-can-tap', daemon=True)
    self._thread.start()

  def _offer(self, kind: str, rows: list[tuple]):
    if not rows or self._failed or self.stopped_by_size:
      return
    try:
      self.q.put_nowait((kind, rows))
    except queue.Full:
      self.drop_batches += 1
      self.drop_frames += len(rows)

  def rx(self, can_list):
    # can_capnp_to_list already ran in card: [(logMonoTime, [(addr, dat, src), ...]), ...]
    rows = []
    for nanos, frames in can_list:
      for frame in frames:
        try:
          addr, dat, src = _frame(frame)
          rows.append((int(nanos), 'can', src, src & 0x7f, int(src >= 128), hex(addr), dat.hex()))
        except (TypeError, ValueError, AttributeError, IndexError):
          self.invalid_frames += 1
    self._offer('can', rows)

  def tx(self, frames, nanos: int | None = None):
    # Local *requested* sendcan, never independent proof of ECU delivery.
    if not frames:
      return
    timestamp = int(time.monotonic_ns() if nanos is None else nanos)
    rows = []
    for frame in frames:
      try:
        addr, dat, src = _frame(frame)
        rows.append((timestamp, 'sendcan_request', src, src & 0x7f, 0, hex(addr), dat.hex()))
      except (TypeError, ValueError, AttributeError, IndexError):
        self.invalid_frames += 1
    self._offer('can', rows)

  def state(self, CP, CS, CC, pandas, raw_di: str = "UNKNOWN", nanos: int | None = None):
    self._state_tick += 1
    if self._state_tick % 10:
      return  # about 10Hz: the raw CAN recorder remains full-rate
    timestamp = int(time.monotonic_ns() if nanos is None else nanos)
    states = []
    for p in pandas:
      try:
        states.append({'controlsAllowed': bool(p.controlsAllowed),
                       'safetyTxBlocked': int(p.safetyTxBlocked),
                       'safetyRxChecksInvalid': int(getattr(p, 'safetyRxChecksInvalid', 0))})
      except (TypeError, ValueError, AttributeError):
        states.append({'unavailable': True})
    out = CS
    self._offer('state', [(timestamp, int(bool(CP.pcmCruise)), int(bool(CC.enabled)),
                           int(bool(CC.longActive)), float(CC.actuators.accel), float(out.vEgo),
                           float(out.aEgo), int(bool(out.gasPressed)), int(bool(out.brakePressed)),
                           str(raw_di),
                           int(bool(out.canValid)), json.dumps(states, separators=(',', ':')))])

  def _write(self):
    try:
      self.path.mkdir(parents=True, exist_ok=False)
      with (self.path / 'manifest.json').open('w') as f:
        json.dump({'build': 'V230', 'capture': 'card existing CAN subscription; all published CAN buses',
                   'rx_source': 'card can_capnp_to_list before CI and 0x659 processing',
                   'tx_source': 'card pre-sendcan requests (not ECU acknowledgement)',
                   'src_ge_128': 'panda transmit echo; not independent bus-side reception',
                   'physical_receive_point': 'unknown; confirm vehicle wiring for independent ECU-side proof',
                   'root': str(self.path), 'start_unix_ns': time.time_ns(),
                   'max_can_bytes': self.max_can_bytes}, f, indent=2)
      with (self.path / 'can.csv').open('w', newline='', buffering=1024 * 1024) as cf, \
           (self.path / 'state.csv').open('w', newline='', buffering=65536) as sf:
        can_writer = csv.writer(cf)
        state_writer = csv.writer(sf)
        can_writer.writerow(CAN_HEADER)
        state_writer.writerow(STATE_HEADER)
        frame_count = 0
        while not self._end.is_set() or not self.q.empty():
          try:
            kind, rows = self.q.get(timeout=0.25)
          except queue.Empty:
            cf.flush(); sf.flush()
            continue
          if kind == 'can':
            if cf.tell() >= self.max_can_bytes:
              self.stopped_by_size = True
            else:
              can_writer.writerows(rows)
              frame_count += len(rows)
          elif kind == 'state':
            state_writer.writerows(rows)
          self.q.task_done()
        cf.flush(); sf.flush()
      with (self.path / 'capture_status.json').open('w') as f:
        json.dump({'frames_written': frame_count, 'drop_batches': self.drop_batches,
                   'drop_frames': self.drop_frames, 'invalid_frames': self.invalid_frames,
                   'stopped_by_size': self.stopped_by_size, 'end_unix_ns': time.time_ns()}, f, indent=2)
    except Exception as exc:
      self._failed = True
      try:
        with (self.path / 'CAPTURE_ERROR.txt').open('w') as f:
          f.write(repr(exc) + '\n')
      except Exception:
        pass

  def stop(self, timeout: float = 2.0):
    self._end.set()
    self._thread.join(timeout=timeout)
