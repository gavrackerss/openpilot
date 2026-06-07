#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="17.2"
fi

export STAGING_ROOT="/data/safe_staging"

# --- XNOR force-fingerprint / skip FW query (Unity-style early teslaLegacy) ----------------
# Cold-boot AEB-flash fix. Root cause (confirmed in boot logs): the stock AP asserts a boot-time
# fcw=3 on the chassis bus at ~t=24s, while the panda is still in SILENT/ELM327 (intercept relay
# CLOSED) running the FW (UDS/OBD) query. teslaLegacy -- which opens the relay and brings the
# forward-scrub live -- doesn't come up until ~t=28s, so that one fcw=3 reaches the IC and latches
# the AEB warning for the whole ignition cycle (only an IC power cycle clears it). After teslaLegacy
# is up the scrub quarantines every fcw=3 correctly; the leak is purely the ~4s pre-teslaLegacy gap.
#
# Skipping the FW query collapses that gap: with FINGERPRINT + SKIP_FW_QUERY set, get_car() returns
# almost immediately, so FirmwareQueryDone + ControlsReady fire in single-digit seconds and pandad
# applies teslaLegacy BEFORE the stock fcw=3 -- the IC never sees it. This is exactly how the Unity
# fork stays flash-free on the same stock-AP-passthrough hardware (launch_chffrplus.sh:
# TinklaAPForceFingerprint -> FINGERPRINT + SKIP_FW_QUERY).
#
# NOTE: this is NOT the previous early-safety attempt. It adds no second setSafetyMode call and does
# not touch engaged-time behavior -- it only makes the existing single safety-mode application happen
# sooner via the normal FirmwareQueryDone/ControlsReady gates. There is no path here that can reset
# controls_allowed mid-drive.
#
# Setup (run once on the device; use your car's exact platform):
#   echo -n "TESLA_MODEL_S_HW2" > /data/params/d/XnorForceFingerprint   # Model S HW2
#   echo -n "TESLA_MODEL_X_HW2" > /data/params/d/XnorForceFingerprint   # Model X HW2
# To restore the full FW query, clear the file or set it to NONE.
__xnor_ffp_file="/data/params/d/XnorForceFingerprint"
if [ -f "$__xnor_ffp_file" ]; then
  __xnor_ffp="$(cat "$__xnor_ffp_file" 2>/dev/null)"
  __xnor_ffp="${__xnor_ffp//[$'\n\r']/}"   # strip newlines
  __xnor_ffp="${__xnor_ffp//NONE/}"        # treat NONE as unset
  if [ -n "$__xnor_ffp" ]; then
    export FINGERPRINT="$__xnor_ffp"
    export SKIP_FW_QUERY=1
  fi
  unset __xnor_ffp
fi
unset __xnor_ffp_file
