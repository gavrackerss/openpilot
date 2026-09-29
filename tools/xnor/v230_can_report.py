#!/usr/bin/env python3
"""V230 offline CAN observation: existing rlog -> CSV, and CAN CSV -> evidence report.

Never subscribes to messaging, never opens a panda or a SocketCAN device. No CAN TX.
Run while parked/offline; rlog conversion uses the checkout's standard LogReader.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict, deque
import csv
import json
from pathlib import Path
import sys

CAN_HEADER = ('mono_ns', 'direction', 'src', 'bus', 'tx_echo', 'address', 'data')
STATE_HEADER = ('mono_ns', 'pcm_cruise', 'op_enabled', 'long_active', 'requested_accel',
                'v_ego', 'a_ego', 'gas_pressed', 'brake_pressed', 'raw_di_state',
                'can_valid', 'panda_states')
DI_STATES = {0: 'OFF', 1: 'STANDBY', 2: 'ENABLED', 3: 'STANDSTILL', 4: 'OVERRIDE',
             5: 'FAULT', 6: 'PRE_FAULT', 7: 'PRE_CANCEL'}
ACC_TYPES = {0: 'none', 6: 'stop_sign_target', 7: 'traffic_light_target', 10: 'csa', 24: 'behavior_report'}
WARN_TYPES = {4: 'stop_sign_stopline', 5: 'traffic_light_stopline', 15: 'not_available'}
RELEVANT = {0x2BF, 0x2B9, 0x238, 0x389, 0x309, 0x368, 0x256, 0x399, 0x2F8, 0x659, 0x25D}


def raw(data: bytes, start: int, width: int) -> int:
  """DBC little-endian (@1+) bitfield only; these legacy fields are @1+."""
  return (int.from_bytes(data, 'little') >> start) & ((1 << width) - 1)


def export_rlog(logfile: str, output: Path, relevant_only: bool = False):
  # Import only for offline rlog mode so analyze() needs only the Python stdlib.
  from openpilot.tools.lib.logreader import LogReader
  output.mkdir(parents=True, exist_ok=True)
  current = {'pcm_cruise': '', 'op_enabled': '', 'long_active': '', 'requested_accel': '',
             'v_ego': '', 'a_ego': '', 'gas_pressed': '', 'brake_pressed': '',
             'raw_di_state': 'UNKNOWN', 'can_valid': '', 'panda_states': '[]'}
  counts = Counter()
  last_state = 0
  with (output / 'can.csv').open('w', newline='') as cf, (output / 'state.csv').open('w', newline='') as sf:
    cw, sw = csv.writer(cf), csv.writer(sf)
    cw.writerow(CAN_HEADER); sw.writerow(STATE_HEADER)
    for event in LogReader(logfile):
      try:
        which = event.which()
      except Exception:
        continue
      ns = int(event.logMonoTime)
      if which in ('can', 'sendcan'):
        for f in getattr(event, which):
          addr, src = int(f.address), int(f.src)
          if relevant_only and addr not in RELEVANT:
            continue
          cw.writerow((ns, 'can' if which == 'can' else 'sendcan_request', src,
                       src & 0x7f, int(which == 'can' and src >= 128), hex(addr), bytes(f.dat).hex()))
          counts[which] += 1
      elif which == 'carParams':
        current['pcm_cruise'] = int(bool(event.carParams.pcmCruise))
      elif which == 'carControl':
        cc = event.carControl
        current.update(op_enabled=int(bool(cc.enabled)), long_active=int(bool(cc.longActive)),
                       requested_accel=float(cc.actuators.accel))
      elif which == 'pandaStates':
        current['panda_states'] = json.dumps([{'controlsAllowed': bool(p.controlsAllowed),
                'safetyTxBlocked': int(p.safetyTxBlocked)} for p in event.pandaStates], separators=(',', ':'))
      elif which == 'carState':
        cs = event.carState
        current.update(v_ego=float(cs.vEgo), a_ego=float(cs.aEgo),
                       gas_pressed=int(bool(cs.gasPressed)), brake_pressed=int(bool(cs.brakePressed)),
                       can_valid=int(bool(cs.canValid)))
        if ns - last_state >= 100_000_000:
          sw.writerow([ns] + [current[k] for k in STATE_HEADER[1:]])
          last_state = ns
  (output / 'export_status.json').write_text(json.dumps({'rlog': logfile, 'counts': counts,
    'note': 'Offline from existing recorded streams only. No live CAN or panda subscription.'}, indent=2))
  return dict(counts)


def decode_row(ns, src, addr, data):
  if len(data) < 8:
    return None
  if addr in (0x368, 0x256):
    v = raw(data, 12, 4)
    return ('di_cruise', {'cruise_code': v, 'cruise_state': DI_STATES.get(v, 'UNKNOWN'),
                          'address': hex(addr)})
  if addr == 0x389:
    report, warning = raw(data, 26, 5), raw(data, 48, 4)
    return ('tesla_acc', {'acc_report_code': report, 'acc_report': ACC_TYPES.get(report, str(report)),
                          'long_warning_code': warning, 'long_warning': WARN_TYPES.get(warning, str(warning))})
  if addr == 0x238:
    mux = raw(data, 0, 8)
    if mux in (1, 2):
      dist, conf = raw(data, 8, 10) * 0.25 - 8, raw(data, 18, 7)
      return ('stop_line', {'kind': 'stop_sign' if mux == 1 else 'traffic_light',
                             'distance_m': dist, 'confidence': conf, 'mux': mux})
    return ('road_sign_other_mux', {'mux': mux})
  if addr == 0x309:
    return ('das_object_raw', {'payload': data.hex()})
  return None


def analyze(capture: Path, output: Path):
  output.mkdir(parents=True, exist_ok=True)
  counts = Counter(); buses = defaultdict(Counter); events = []
  # Payload-sensitive one-to-one matching is a TX echo observation, never ECU ACK.
  outstanding = defaultdict(deque)
  matches = 0; requests = 0; echoes = 0; non_echo_2bf = 0
  raw_di = Counter(); acc_report = Counter(); stoplines = Counter(); warning = Counter()
  interesting = []
  with (capture / 'can.csv').open(newline='') as f:
    for row in csv.DictReader(f):
      try:
        ns = int(row['mono_ns']); src = int(row['src']); addr = int(row['address'], 0)
        data = bytes.fromhex(row['data']); direction = row['direction']
      except (ValueError, KeyError):
        counts['invalid_rows'] += 1; continue
      bus = src & 0x7f
      counts[direction] += 1
      buses[direction][str(src)] += 1
      if addr == 0x2BF:
        if direction == 'sendcan_request':
          requests += 1
          outstanding[(bus, data)].append(ns)
        elif direction == 'can' and src >= 128:
          echoes += 1
          q = outstanding.get((bus, data))
          if q:
            while q and ns - q[0] > 500_000_000:
              q.popleft()
            if q and -30_000_000 <= ns - q[0] <= 500_000_000:
              q.popleft(); matches += 1
        elif direction == 'can':
          non_echo_2bf += 1
      if direction != 'can' or src >= 128:
        continue # vehicle-origin candidate only; no presumed physical side
      decoded = decode_row(ns, src, addr, data)
      if decoded is None:
        continue
      typ, info = decoded
      if typ == 'di_cruise': raw_di[info['cruise_state']] += 1
      elif typ == 'tesla_acc':
        acc_report[info['acc_report']] += 1
        warning[info['long_warning']] += 1
      elif typ == 'stop_line':
        stoplines[info['kind']] += 1
      if typ in ('di_cruise','tesla_acc','stop_line') or (typ == 'das_object_raw' and counts['das_object_samples'] % 100 == 0):
        events.append((ns, src, hex(addr), typ, json.dumps(info, separators=(',', ':'))))
      if typ == 'das_object_raw': counts['das_object_samples'] += 1
  with (output / 'decoded_events.csv').open('w', newline='') as f:
    writer=csv.writer(f);writer.writerow(('mono_ns','src','address','event','details_json'));writer.writerows(events)
  summary = {'can_counts': dict(counts), 'src_counts': {k: dict(v) for k,v in buses.items()},
             'di_states': dict(raw_di), 'das_acc_reports': dict(acc_report),
             'das_long_warnings': dict(warning), 'stopline_mux_observations': dict(stoplines),
             '0x2bf': {'sendcan_requests': requests, 'panda_tx_echoes': echoes,
                       'payload_matched_echoes': matches, 'non_echo_can_observations': non_echo_2bf},
             'interpretation': [
               'A CAN src>=128 is a panda transmit echo, NOT an independent vehicle-side capture or ECU acknowledgement.',
               'A matching non-echo frame on a comma-observed bus is still not proof of DI application-layer acceptance.',
               'Traffic-light stop-line distance/confidence does NOT indicate signal colour.',
               'No direct DI torque/actuation conclusion can be drawn from these counters alone.',
               'CAN src/bus is the published panda mapping; only verified wiring establishes physical segment.']}
  (output / 'report.json').write_text(json.dumps(summary, indent=2) + '\n')
  lines = ['XNOR V230 — passive observational report', '',
           'Source: existing card/logged CAN streams; no new subscription or panda access.',
           f"2BF: requested {requests}, panda echoes {echoes}, exact payload-matched echoes {matches}, non-echo CAN {non_echo_2bf}",
           'Raw DI states: '+repr(dict(raw_di)), 'ACC targets: '+repr(dict(acc_report)),
           'Stopline mux: '+repr(dict(stoplines)), '',
           'Caution: echoes are NOT ECU acknowledgement. Non-echo != proven ECU acceptance.',
           'If there is no independent RX on the vehicle side, wiring-level delivery remains unresolved.']
  (output / 'report.txt').write_text('\n'.join(lines)+'\n')
  return summary


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__)
  sub = ap.add_subparsers(dest='mode', required=True)
  r = sub.add_parser('export-rlog', help='OFFLINE only: use existing rlog, never subscribe live')
  r.add_argument('rlog');r.add_argument('--out', required=True);r.add_argument('--relevant-only',action='store_true')
  a = sub.add_parser('analyze', help='Analyze an exported or in-process V230 capture directory')
  a.add_argument('capture');a.add_argument('--out', required=True)
  args = ap.parse_args(argv)
  if args.mode == 'export-rlog':
    print(export_rlog(args.rlog, Path(args.out), args.relevant_only))
  else:
    summary = analyze(Path(args.capture), Path(args.out));print((Path(args.out)/'report.txt').read_text())

if __name__ == '__main__':
  main()
