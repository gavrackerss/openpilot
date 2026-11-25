"""XNOR Jetlink protocol-v3 TCP connectivity/health probe.

This is a NON-CONTROLLING integration foundation. It does not take over USB,
change panda/CAN, start a USB gadget, load a model, or replace modeld.

Transport: comma 3X as USB host -> USB hub -> Android USB tethering ->
Jetlink Android developer TCP listener, port 5599.

No third-party dependencies; safe for manager to import. MIT-compatible,
implemented from the public Jetlink protocol v3 wire specification.
"""
from __future__ import annotations

import argparse
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import socket
import struct
import subprocess
import time
from typing import Any

CONFIG = Path('/data/jetlink_xnor')
ENABLED = CONFIG / 'ENABLED'
STATUS = CONFIG / 'status.json'
HOST_FILE = CONFIG / 'HOST'
PORT_FILE = CONFIG / 'PORT'

# Jetlink v3 header: magic, version, type, sequence, flags, payload len, reserved.
HEADER = struct.Struct('<IHHIIIQ4x')
MAGIC = 0x4B4E4C4A
VERSION = 3
PADDED = 0x80
HELLO_REQ, HELLO_RESP, STATE_REQ, STATE_RESP = 1, 2, 12, 13
ERROR, PING, PONG = 14, 15, 16
MAX_PAYLOAD = 16 << 20
CYCLE_S = 5.0


class LinkError(Exception):
  pass


def send_message(sock: socket.socket, msg_type: int, seq: int, obj: dict | None = None) -> None:
  body = b'' if obj is None else json.dumps(obj, separators=(',', ':')).encode('utf-8')
  flags = PADDED if (HEADER.size + len(body)) % 512 == 0 else 0
  sock.sendall(HEADER.pack(MAGIC, VERSION, msg_type, seq, flags, len(body), 0) + body + (b'\x00' if flags else b''))


def recv_exact(sock: socket.socket, n: int) -> bytes:
  chunks = []
  while n:
    chunk = sock.recv(n)
    if not chunk:
      raise LinkError('Jetlink peer closed TCP connection')
    chunks.append(chunk)
    n -= len(chunk)
  return b''.join(chunks)


def receive_message(sock: socket.socket) -> tuple[int, int, bytes]:
  magic, version, msg_type, seq, flags, length, _ = HEADER.unpack(recv_exact(sock, HEADER.size))
  if magic != MAGIC:
    raise LinkError(f'Bad Jetlink magic: {magic:#x}')
  if version != VERSION:
    raise LinkError(f'Protocol version mismatch: peer={version}, required=3')
  if length > MAX_PAYLOAD:
    raise LinkError(f'Unreasonably large payload: {length}')
  payload = recv_exact(sock, length)
  if flags & PADDED:
    recv_exact(sock, 1)
  return msg_type, seq, payload


def exchange(sock: socket.socket, request: int, response: int, seq: int, obj: dict | None = None) -> dict:
  send_message(sock, request, seq, obj)
  for _ in range(10):
    got_type, got_seq, payload = receive_message(sock)
    if got_type == ERROR:
      try:
        detail = json.loads(payload)
      except (UnicodeDecodeError, ValueError):
        detail = payload[:128].hex()
      raise LinkError(f'Jetlink ERROR: {detail}')
    # The Jetlink server can deliver async PROGRESS on the same link.
    if got_type == 7:
      continue
    if got_type != response or got_seq != seq:
      raise LinkError(f'Unexpected response type={got_type} seq={got_seq} (want {response}/{seq})')
    if not payload:
      return {}
    try:
      return json.loads(payload.decode('utf-8'))
    except (ValueError, UnicodeDecodeError) as e:
      raise LinkError(f'Invalid JSON response: {e}') from e
  raise LinkError('Too many unsolicited Jetlink messages')


def _command(*cmd: str) -> str:
  try:
    return subprocess.check_output(cmd, text=True, timeout=1.5, stderr=subprocess.DEVNULL)
  except (OSError, subprocess.SubprocessError):
    return ''


def usb_tether_candidates() -> list[str]:
  """Find Android phone's gateway on the USB Ethernet link; not a Wi-Fi gateway.

  No network management changes: reads only `ip` output and neighbor cache.
  Override by placing the known phone address in /data/jetlink_xnor/HOST.
  """
  override = HOST_FILE.read_text().strip() if HOST_FILE.is_file() else os.environ.get('XNOR_JETLINK_HOST', '')
  if override:
    return [str(ipaddress.ip_address(override.strip()))]
  links = _command('ip', '-o', 'link', 'show')
  ifnames = []
  for line in links.splitlines():
    m = re.match(r'\d+:\s+([^:@]+)(?:@[^:]+)?:', line)
    if not m:
      continue
    name = m.group(1)
    # Ignore wlan and CAN interfaces; typical Linux tether interfaces are
    # usb0, rndis0, enx..., eth0, or enp...u...
    if re.match(r'^(usb\d+|rndis\d*|enx[0-9a-f]+|eth\d+|enp\S*u\S*)$', name):
      ifnames.append(name)
  candidates: list[str] = []
  routes = _command('ip', '-4', 'route', 'show').splitlines()
  for line in routes:
    for dev in ifnames:
      if not re.search(rf'\bdev\s+{re.escape(dev)}(?:\s|$)', line):
        continue
      via = re.search(r'\bvia\s+(\d+\.\d+\.\d+\.\d+)', line)
      if via:
        candidates.append(via.group(1))
  neigh = _command('ip', '-4', 'neigh', 'show').splitlines()
  for line in neigh:
    for dev in ifnames:
      if re.search(rf'\bdev\s+{re.escape(dev)}(?:\s|$)', line):
        match = re.match(r'(\d+\.\d+\.\d+\.\d+)', line)
        if match:
          candidates.append(match.group(1))
  return list(dict.fromkeys(candidates))[:8]


def probe(host: str, port: int = 5599, timeout: float = 3.0) -> dict[str, Any]:
  t0 = time.monotonic()
  with socket.create_connection((host, port), timeout=timeout) as sock:
    sock.settimeout(timeout)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    hello = exchange(sock, HELLO_REQ, HELLO_RESP, 1, {
      'client': {'nonce': secrets.token_hex(4), 'name': 'xnor-dev-jetlink-tcp', 'link': {'kind': 'tcp'}}
    })
    if not isinstance(hello, dict):
      raise LinkError('Invalid HELLO response')
    # Do not ask the server to build/load a model here. Only connectivity.
    pong_at = time.perf_counter()
    exchange(sock, PING, PONG, 2)
    rtt_ms = (time.perf_counter() - pong_at) * 1000
    return {
      'connected': True, 'host': host, 'port': port, 'protocol': 3,
      'connect_and_hello_ms': round((time.monotonic() - t0) * 1000, 2),
      'ping_rtt_ms': round(rtt_ms, 2),
      'server': {key: hello.get(key) for key in ('version', 'engine', 'loaded', 'state', 'frame_port', 'frame_token', 'pick') if key in hello},
      'timestamp_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
      'mode': 'diagnostics-only',
    }


def _status(record: dict) -> None:
  try:
    STATUS.parent.mkdir(parents=True, exist_ok=True)
    tmp = STATUS.with_name('.status.tmp')
    tmp.write_text(json.dumps(record, indent=2) + '\n')
    os.replace(tmp, STATUS)
  except OSError:
    pass


def run_once(verbose: bool = True, forced_host: str | None = None, forced_port: int | None = None) -> bool:
  try:
    port = forced_port if forced_port is not None else (int(PORT_FILE.read_text().strip()) if PORT_FILE.is_file() else 5599)
    if not 1 <= port <= 65535:
      raise ValueError(f'Invalid TCP port {port}')
    candidates = [forced_host] if forced_host else usb_tether_candidates()
    if not candidates:
      state = {
        'connected': False,
        'reason': 'No USB-tether gateway found; set /data/jetlink_xnor/HOST',
        'timestamp_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
      }
      _status(state)
      if verbose: print(json.dumps(state, indent=2))
      return False
    errors = []
    for host in candidates:
      try:
        state = probe(host, port)
        _status(state)
        if verbose: print(json.dumps(state, indent=2))
        return True
      except (LinkError, OSError, TimeoutError, ValueError) as e:
        errors.append(f'{host}:{port}: {e}')
    state = {
      'connected': False,
      'errors': errors,
      'candidates': candidates,
      'timestamp_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
      'mode': 'diagnostics-only',
    }
    _status(state)
    if verbose: print(json.dumps(state, indent=2))
    return False
  except Exception as e:
    state = {'connected': False, 'reason': f'{type(e).__name__}: {e}', 'mode': 'diagnostics-only'}
    _status(state)
    if verbose: print(json.dumps(state, indent=2))
    return False


def main() -> None:
  """Manager-managed offroad process. Runs only when ENABLED exists."""
  while True:
    run_once(verbose=False)
    time.sleep(CYCLE_S)


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('--once', action='store_true', help='One connection/HELLO/PING check')
  parser.add_argument('--host', help='Phone tether IP; skips gateway discovery')
  parser.add_argument('--port', type=int, help='Jetlink TCP port; default 5599')
  args = parser.parse_args()
  if args.once or args.host:
    raise SystemExit(0 if run_once(forced_host=args.host, forced_port=args.port) else 1)
  main()
