#!/usr/bin/env bash
# End-to-end native CTM -> Zoompilot parked join, no production modeld changes.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
MARKERS=/data/jetlink_xnor
EXPECTED=e8d821733be15ebe9e27498bc27ad8bbbd741980ece37d77f377294010b8ff28

if [[ ! -r "$MARKERS/CTM_INTEGRATION_STAGED" ]] || [[ "$(cat "$MARKERS/CTM_INTEGRATION_STAGED")" != "$EXPECTED" ]]; then
  echo 'STOP: run bash tools/jetlink_ctm_markers.sh stage first' >&2
  exit 2
fi
for required in CTM_PARKED_TEST_ENABLED CTM_AUTHORITY_NOT_QUALIFIED; do
  if [[ ! -f "$MARKERS/$required" ]]; then
    echo "STOP: missing non-authority marker $required" >&2
    exit 2
  fi
done
for forbidden in AUTHORITY_ENABLED LS2_AUTHORITY_QUALIFIED CTM_AUTHORITY_QUALIFIED CTM_AUTHORITY_ENABLED ONROAD_SHADOW_ENABLED; do
  if [[ -e "$MARKERS/$forbidden" ]]; then
    echo "STOP: $forbidden is present; disarm authoritative/other Jetlink sessions before parked test." >&2
    exit 2
  fi
done
if [[ ! -f openpilot/jetlink_ctm_park_handover.py ]]; then
  echo 'STOP: CTM native adapter files not installed in this checkout' >&2
  exit 2
fi

directory=/data/jetlink_ctm_park/handover
mkdir -p "$directory"
stamp="$(date -u +%Y%m%dT%H%M%SZ)-$$"
log="$directory/ctm-handover-$stamp.log"
echo 'CTM real Zoompilot parked handover test, non-publishing, stock runner.'
echo 'Vehicle must remain stationary in Park, cameras active, CTM Prepared on Pixel.'
echo 'Remote join -> a few valid replies -> simulated socket loss -> local fallback.'
echo 'The real production modeld remains local and controls are not modified.'
echo "Log: $log"

export JETLINK_CTM_ISOLATED_PARKED_TEST=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
set +e
PYTHONPATH="$ROOT:${PYTHONPATH:-}" python3 -u -m openpilot.jetlink_ctm_park_handover \
  --runner stock --seconds "${CTM_PARK_SECONDS:-100}" \
  --state-timeout 20 --remote-frames 3 2>&1 | tee "$log"
code=${PIPESTATUS[0]}
set -e
echo "CTM parked handover exit $code; see $log"
echo 'Authority remains unqualified. No flag grants CTM steering.'
exit "$code"
