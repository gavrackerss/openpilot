"""V220 observational Hybrid longitudinal ownership probe (NO CAN actuation).

All message observations are raw bus-source counts. An OP sendcan request, a
panda safety latch, and ECU acceptance are different facts; none are inferred
from the others. Never use this observer to authorize actuator messages.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from typing import Iterable

LONG_ADDRS = (0x2B9, 0x2BF)


def _can_tuple(msg):
  """Support the CanData received from card and ordinary sendcan tuples."""
  if isinstance(msg, tuple):
    addr, data, src = msg[:3]
  else:
    addr, data, src = msg.address, msg.dat, msg.src
  return int(addr), bytes(data), int(src)


@dataclass(frozen=True)
class OwnerAssessment:
  label: str
  reason: str
  cutover_permitted: bool = False  # V220 never implements a physical cutover.


def assess_owner(*, cc_enabled: bool, long_active: bool,
                 op_requests: int, ap_frames: int, aeb_frames: int,
                 panda_allowed: bool | None, tx_block_delta: int) -> OwnerAssessment:
  """Shadow-only candidate classification, deliberately never grants CAN authority."""
  if aeb_frames:
    return OwnerAssessment('AEB_PRESENT', 'preserve genuine AEB; no cutover')
  if tx_block_delta > 0:
    return OwnerAssessment('SAFETY_TX_BLOCKED', 'panda blocked a TX; exact address unknown')
  if not cc_enabled or not long_active:
    return OwnerAssessment('OP_LONG_INACTIVE', 'OP longitudinal not engaged or overridden')
  if panda_allowed is False:
    return OwnerAssessment('PANDA_NOT_ALLOWED', 'reported controlsAllowed=0')
  if not op_requests:
    return OwnerAssessment('OP_REQUEST_MISSING', 'no OP 0x2BF request in observation window')
  if ap_frames:
    return OwnerAssessment('OVERLAP_CANDIDATE', 'OP requested 0x2BF while raw bus 2/6 also carried 0x2BF; no ECU acceptance proof')
  return OwnerAssessment('SINGLE_SOURCE_CANDIDATE', 'OP requested 0x2BF; cannot prove delivered or accepted from this trace')


class LongitudinalShadow:
  """Read-only 1-second observation windows, no sends or safety mutations."""
  def __init__(self):
    self.rx = Counter()
    self.tx = Counter()
    self.rx_data = {}
    self.last_tx_hex = ''
    self.last_requested_accel = 0.0
    self.last_a_ego = 0.0
    self.last_v_ego = 0.0
    self.cc_enabled = False
    self.long_active = False
    self.native_cruise = 'UNKNOWN'
    self._last_panda_tx = {}
    self._panda_deltas = {}
    self._panda_allowed = {}
    self._last_aeb_count = 0
    self._window_start_ns = None

  def observe_rx(self, packets: Iterable):
    # card.py receives can_capnp_to_list(), whose real shape is:
    # [(logMonoTime, [(address, data, src), ...]), ...]. The original V220
    # mistakenly treated each OUTER (timestamp, frames) as a CAN triple,
    # crashing card as soon as shadow mode was enabled. Also retain support
    # for flat CAN-frame lists used by replay and existing QA.
    for packet in packets:
      if isinstance(packet, tuple) and len(packet) == 2 and isinstance(packet[1], (tuple, list)):
        messages = packet[1]
      else:
        messages = (packet,)
      for m in messages:
        try:
          addr, data, src = _can_tuple(m)
        except (AttributeError, TypeError, ValueError, IndexError):
          # An observer must never take down the vehicle-state process.
          continue
        if addr not in LONG_ADDRS or len(data) != 8:
          continue
        self.rx[(addr, src)] += 1
        self.rx_data[(addr, src)] = data.hex()
        if data[2] & 0x03:
          self.rx[('aeb', addr, src)] += 1

  def observe_pandas(self, pandas):
    for index, p in enumerate(pandas):
      try:
        blocked = int(p.safetyTxBlocked)
        self._panda_allowed[index] = bool(p.controlsAllowed)
      except (AttributeError, TypeError, ValueError):
        continue
      prev = self._last_panda_tx.get(index)
      if prev is not None:
        # Counter reset should not be interpreted as a huge burst of rejects.
        self._panda_deltas[index] = self._panda_deltas.get(index, 0) + max(0, blocked - prev)
      self._last_panda_tx[index] = blocked

  def observe_tx(self, messages: Iterable, *, enabled: bool, long_active: bool,
                 requested_accel: float, native_cruise: str, a_ego: float, v_ego: float):
    self.cc_enabled = bool(enabled)
    self.long_active = bool(long_active)
    self.last_requested_accel = float(requested_accel)
    self.last_a_ego = float(a_ego)
    self.last_v_ego = float(v_ego)
    self.native_cruise = str(native_cruise)
    for msg in messages:
      addr, data, bus = _can_tuple(msg)
      if addr in LONG_ADDRS and len(data) == 8:
        self.tx[(addr, bus)] += 1
        if addr == 0x2BF:
          self.last_tx_hex = data.hex()

  def summary(self):
    aeb = sum(n for key, n in self.rx.items() if len(key) == 3 and key[0] == 'aeb')
    # The raw bus map must be validated on the specific external panda; do not
    # report an arbitrary src as a physical ECU acknowledgement.
    ap_frames = sum(n for key, n in self.rx.items() if len(key) == 2 and key[0] == 0x2BF and key[1] in (2, 6))
    op_requested = sum(n for (addr, _), n in self.tx.items() if addr == 0x2BF)
    tx_blocked = sum(self._panda_deltas.values())
    external_allowed = self._panda_allowed.get(1)
    assessment = assess_owner(cc_enabled=self.cc_enabled, long_active=self.long_active,
                              op_requests=op_requested, ap_frames=ap_frames,
                              aeb_frames=aeb, panda_allowed=external_allowed,
                              tx_block_delta=tx_blocked)
    rx_repr = ','.join(f'{key[0]:03X}@{key[1]}:{n}' for key, n in sorted(self.rx.items(), key=lambda kv: str(kv[0])) if len(key) == 2) or 'none'
    tx_repr = ','.join(f'{addr:03X}@{bus}:{n}' for (addr, bus), n in sorted(self.tx.items())) or 'none'
    p_repr = ','.join(f'p{i}:allowed={int(self._panda_allowed.get(i, False))}:txBlockedDelta={self._panda_deltas.get(i, 0)}'
                      for i in sorted(self._panda_allowed)) or 'pandaUnavailable'
    return (f'[XNOR_V220_OWNER_SHADOW] owner={assessment.label} reason={assessment.reason} '
            f'cutover=0 ccEnabled={int(self.cc_enabled)} longActive={int(self.long_active)} '
            f'nativeTacc={self.native_cruise} rx={rx_repr} sendcan={tx_repr} '
            f'aebRaw={aeb} {p_repr} reqAccel={self.last_requested_accel:.3f} '
            f'aEgo={self.last_a_ego:.3f} vEgo={self.last_v_ego:.3f} '
            f'lastOp2BF={self.last_tx_hex or "none"} acceptance=UNPROVEN')

  def flush_if_due(self, now_ns: int) -> str | None:
    if self._window_start_ns is None:
      self._window_start_ns = int(now_ns)
      return None
    if int(now_ns) - self._window_start_ns < 1_000_000_000:
      return None
    result = self.summary()
    self.rx.clear()
    self.tx.clear()
    self._panda_deltas.clear()
    self._window_start_ns = int(now_ns)
    return result
