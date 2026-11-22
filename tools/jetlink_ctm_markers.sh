#!/usr/bin/env bash
# Stage the ORIGINAL Cinque Terre Model for passive parked integration only.
# No steering/actuation or modeld handover is enabled by these markers.
set -euo pipefail

ROOT=/data/jetlink_xnor
SOURCE=e8d821733be15ebe9e27498bc27ad8bbbd741980ece37d77f377294010b8ff28
MODE="${1:-status}"

show() {
  printf 'Cinque Terre Original: %s\n' "$SOURCE"
  for label in CTM_INTEGRATION_STAGED CTM_PARKED_TEST_ENABLED CTM_AUTHORITY_NOT_QUALIFIED; do
    if test -f "$ROOT/$label"; then printf '  %-34s PRESENT\n' "$label"
    else printf '  %-34s absent\n' "$label"; fi
  done
  for label in AUTHORITY_ENABLED LS2_AUTHORITY_QUALIFIED CTM_AUTHORITY_QUALIFIED CTM_AUTHORITY_ENABLED; do
    if test -e "$ROOT/$label"; then printf '  %-34s PRESENT (must be absent for parked test)\n' "$label"; fi
  done
}

case "$MODE" in
  stage)
    # Never silently delete an existing actuation/handover qualification.
    # Refuse to run alongside anything that can engage remote authority.
    for marker in AUTHORITY_ENABLED LS2_AUTHORITY_QUALIFIED CTM_AUTHORITY_QUALIFIED CTM_AUTHORITY_ENABLED ONROAD_SHADOW_ENABLED; do
      if test -e "$ROOT/$marker"; then
        echo "STOP: $ROOT/$marker exists. Disarm remote authority before parked-only testing." >&2
        exit 2
      fi
    done
    mkdir -p "$ROOT"
    printf '%s\n' "$SOURCE" > "$ROOT/CTM_INTEGRATION_STAGED"
    : > "$ROOT/CTM_PARKED_TEST_ENABLED"
    : > "$ROOT/CTM_AUTHORITY_NOT_QUALIFIED"
    echo 'CTM_STAGED_FOR_PARKED_SHADOW_ONLY; no authority or control changes'
    show
    ;;
  disarm)
    rm -f "$ROOT/CTM_INTEGRATION_STAGED" "$ROOT/CTM_PARKED_TEST_ENABLED" "$ROOT/CTM_AUTHORITY_NOT_QUALIFIED"
    echo 'CTM parked test markers removed. No other authority markers changed.'
    show
    ;;
  status)
    show
    ;;
  *)
    echo "Usage: bash tools/jetlink_ctm_markers.sh {stage|status|disarm}" >&2
    exit 2
    ;;
esac
