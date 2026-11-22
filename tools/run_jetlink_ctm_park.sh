#!/usr/bin/env bash
# Read-only 180s comma -> Pixel CTM latency/throughput parked qualification.
# Never starts the stock/LS2 JoiningModelState or transmits vehicle controls.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MARKER_DIR=/data/jetlink_xnor
SOURCE=e8d821733be15ebe9e27498bc27ad8bbbd741980ece37d77f377294010b8ff28
OUTPUT_BASE=/data/jetlink_ctm_park
cd "$ROOT"

if [[ ! -f "$MARKER_DIR/CTM_INTEGRATION_STAGED" || ! -f "$MARKER_DIR/CTM_PARKED_TEST_ENABLED" || ! -f "$MARKER_DIR/CTM_AUTHORITY_NOT_QUALIFIED" ]]; then
  echo 'STOP: first run bash tools/jetlink_ctm_markers.sh stage' >&2
  exit 2
fi
if [[ "$(cat "$MARKER_DIR/CTM_INTEGRATION_STAGED")" != "$SOURCE" ]]; then
  echo 'STOP: staged source hash is not original Cinque Terre' >&2
  exit 2
fi
for forbidden in AUTHORITY_ENABLED LS2_AUTHORITY_QUALIFIED CTM_AUTHORITY_QUALIFIED; do
  if [[ -e "$MARKER_DIR/$forbidden" ]]; then
    echo "STOP: authority marker $forbidden present; passive test refused" >&2
    exit 2
  fi
done
if [[ ! -f openpilot/jetlink_shadow_bench.py ]]; then
  echo 'STOP: Jetlink shadow benchmark is missing from the checkout' >&2
  exit 2
fi

# Tether address changes across phone reconnects. The old fixed
# 10.169.232.180 address need not be reachable in a new USB subnet.
# Discover the USB gateway using the route to the historic address.
# Explicit JETLINK_CTM_HOST overrides discovery for non-standard tether setups.
HOST="$(python3 - <<'PY'
import ipaddress
import os
import socket
import subprocess
import sys

legacy = '10.169.232.180'
try:
    route = subprocess.check_output(['ip', '-4', 'route', 'get', legacy], text=True, timeout=3)
except (OSError, subprocess.SubprocessError) as exc:
    raise SystemExit(f'STOP: cannot inspect USB tether route: {exc}')
parts = route.split()
if 'dev' not in parts or parts[parts.index('dev') + 1] != 'usb0':
    raise SystemExit(f'STOP: configured route is not usb0: {route.strip()}')
candidate = os.environ.get('JETLINK_CTM_HOST')
if not candidate:
    if 'via' not in parts:
        raise SystemExit('STOP: no usb0 gateway discovered. Specify JETLINK_CTM_HOST explicitly.')
    candidate = parts[parts.index('via') + 1]
try:
    host = str(ipaddress.IPv4Address(candidate))
    host_route = subprocess.check_output(['ip', '-4', 'route', 'get', host], text=True, timeout=3)
    host_parts = host_route.split()
    if 'dev' not in host_parts or host_parts[host_parts.index('dev') + 1] != 'usb0':
        raise ValueError(f'target is not routed over usb0: {host_route.strip()}')
except (ValueError, OSError, subprocess.SubprocessError) as exc:
    raise SystemExit(f'STOP: invalid Jetlink USB endpoint: {exc}')
print(f'USB gateway candidate: {host}; testing Jetlink port 5599 ...', file=sys.stderr)
try:
    with socket.create_connection((host, 5599), timeout=3):
        pass
except (OSError, TimeoutError) as exc:
    raise SystemExit(
        f'STOP: Jetlink TCP unreachable at {host}:5599 ({exc}). '
        'On Pixel, open Jetlink Settings and enable "Enable Jetlink TCP", '
        'make sure the Jetlink server is running, and reconnect USB tethering. '
        'For a non-standard phone IP, export JETLINK_CTM_HOST=<phone_usb_ip>.'
    )
print(f'Jetlink TCP connection established at {host}:5599', file=sys.stderr)
print(host)
PY
)" || exit 2

mkdir -p "$OUTPUT_BASE"
stamp="$(date -u +%Y%m%dT%H%M%SZ)"
dir="$OUTPUT_BASE/park-${stamp}-$$"
log="$OUTPUT_BASE/park-${stamp}-$$.log"
echo "Original Cinque Terre parked test. 180 seconds, 20 Hz, 2-frame USB pipeline."
echo 'Phone must have CTM selected/Prepared; connected USB tether and external XNOR Panda required.'
echo "Output: $dir"
echo "Log: $log"
echo 'NO authority or modeld publication; vehicle must remain in Park.'
set +e
PYTHONPATH="$ROOT:${PYTHONPATH:-}" python3 -u -m openpilot.jetlink_shadow_bench \
  --model ctm --mode camera --pacing timer \
  --seconds 180 --rate 20 --pipeline-depth 2 \
  --transport udp --udp-payload-bytes 65000 \
  --udp-send-mode sendmmsg --reply-guard-ms 0 \
  --fetch-stage-report --dump-outputs-every 0 \
  --host "$HOST" --port 5599 \
  --output "$dir" 2>&1 | tee "$log"
ret=${PIPESTATUS[0]}
set -e
if [[ -s "$dir/summary.json" ]]; then
  python3 - "$dir/summary.json" <<'PY'
import json,sys
o=json.load(open(sys.argv[1]))
gate=o.get('ctm_parked_integration') or {}
print('\nCTM_PARKED_RESULT',json.dumps({
  'model_sha256':(o.get('meta') or {}).get('model_sha256'),
  'frames':(o.get('counts') or {}).get('frames'),
  'achieved_fps':(o.get('counts') or {}).get('achieved_fps'),
  'complete_path_ms':gate.get('complete_path_ms'),
  'complete_path_over_50_ms':gate.get('complete_path_over_50_ms'),
  'capture_to_reply_ms':gate.get('capture_to_reply_ms'),
  'capture_to_reply_over_50_ms':gate.get('capture_to_reply_over_50_ms'),
  'errors':o.get('errors'),
  'authority_qualified':False,
},indent=2),flush=True)
PY
else
  echo "No summary.json generated; see diagnostic log: $log" >&2
fi
echo "Parked test exit: $ret; outputs: $dir"
echo 'To disarm staged CTM markers: bash tools/jetlink_ctm_markers.sh disarm'
exit "$ret"
