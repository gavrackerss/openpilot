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

# --- XNOR forced fingerprint / skip FW query --------------------------------------------------
# Translates the XnorForceFingerprint param into FINGERPRINT + SKIP_FW_QUERY so get_car() returns
# immediately without running the OBD/UDS firmware query. Skipping the query avoids the ELM327
# CAN_MODE_OBD_CAN2 window that storms CAN2 (interruptRateCan2 -> faultTemp -> noOutput).
#   Set:   echo -n "TESLA_MODEL_S_HW2" > /data/params/d/XnorForceFingerprint
#   Clear: write "NONE" or remove the file to run the full FW query again.
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
