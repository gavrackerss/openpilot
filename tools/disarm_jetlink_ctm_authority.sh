#!/usr/bin/env bash
# Emergency CTM authority revocation. Safe to run again if already disarmed.
# A running guarded CTM joiner notices STOP on its next model frame, demotes
# locally and stops reconnecting. Does not stop local modeld or alter Panda.
set -euo pipefail
ROOT=/data/jetlink_xnor
mkdir -p "$ROOT"
# Write STOP first: this is the per-frame lease kill switch.
: > "$ROOT/CTM_AUTHORITY_STOP"
rm -f "$ROOT/CTM_AUTHORITY_ENABLED"
echo "CTM authority STOP marker set and CTM arm removed."
echo "The guarded joiner must return to local stock at its next model frame."
echo "This script does not validate that a physical steering fallback has occurred."
echo "Review modeld CTM publisher logs and vehicle state after disarming."
