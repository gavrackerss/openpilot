#!/usr/bin/env python3
"""Read-only migration audit against the *installed* 24d304b Tesla baseline.

The remote xnor-dev branch differs from the user's last verified small-model
baseline: its older longitudinal planner contains DEC/boost arbitration. Do
not unintentionally reintroduce DEC while porting to XNOR-native 0.11.2.

This audit is a source inventory, not proof of proper lead braking or safe
vehicle control. It performs no vehicle I/O.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / "migration/installed_24d304b"
NATIVE = ROOT / "openpilot/selfdrive"
EXPECTED_BLOBS = {
  "longitudinal_planner.py": "ad9777c65a88ad26be547d636e312feb85131be1",
  "long_mpc.py": "be326d00e01eb801f801d7e25e3b1dfcf3c1442a",
  "controlsd.py": "0003abda91e4dcacb4cad6e7a4df1a44b9bc9db5",
  "modeld.py": "e3646ba7ba94f4df15071e8ac2b11a720209fb89",
  "radard.py": "98fce1cb26e7b5fb80ae3fe9e166bcd82ad66c8f",
  "drive_helpers.py": "bf6dd04f603a2f16b3c067206c2eb8f969f3bb67",
  "cruise.py": "3dc3c56b9a238b0fac98f5fea06623dd17850bd3",
}


def git_blob_sha(data: bytes) -> str:
  return hashlib.sha1(f"blob {len(data)}\0".encode() + data).hexdigest()


def audit() -> dict:
  missing, changed = [], []
  for name, sha in EXPECTED_BLOBS.items():
    file = OLD / name
    if not file.is_file():
      missing.append(name)
    elif git_blob_sha(file.read_bytes()) != sha:
      changed.append(name)

  if missing or changed:
    raise RuntimeError(f"Installed baseline integrity failed: missing={missing}, altered={changed}")

  old_planner = (OLD / "longitudinal_planner.py").read_text()
  new_planner = (NATIVE / "controls/lib/longitudinal_planner.py").read_text()
  old_controls = (OLD / "controlsd.py").read_text()
  new_controls = (NATIVE / "controls/controlsd.py").read_text()
  old_mpc = (OLD / "long_mpc.py").read_text()
  new_mpc = (NATIVE / "controls/lib/longitudinal_mpc_lib/long_mpc.py").read_text()
  new_modeld = (NATIVE / "modeld/modeld.py").read_text()

  if "XnorDynamicExperimentalControl" in new_planner or "XnorDynamicExperimentalControl" in old_planner:
    raise RuntimeError("DEC arbitration unexpectedly present in native/installed baseline")
  old_owner = "_xnor_hybrid_brake_long_cancelled" in old_controls
  native_owner = "_xnor_hybrid_brake_long_cancelled" in new_controls
  old_follow = "get_t_follow_from_carstate" in old_mpc
  native_follow = "get_t_follow_from_carstate" in new_mpc
  native_direct_action = "model_output['action'][0,1]" in new_modeld
  if not (old_owner and old_follow and native_direct_action):
    raise RuntimeError("Unexpected installed Tesla control or native model action contract")

  return {
    "installed_reference": "24d304bb06b6beef6b0a0633bf966dbbc23e9d39",
    "installed_baseline_sha256_status": "git blob SHA-1 exact match for 7 files",
    "native_reference": "xnor-tech/openpilot xnor-dev 44be035782288ed94d1a15116907cc08c0a7c83d",
    "installed_dec_present": False,
    "native_dec_present": False,
    "custom_hybrid_brake_owner_original": old_owner,
    "custom_hybrid_brake_owner_ported": native_owner,
    "custom_follow_distance_original": old_follow,
    "custom_follow_distance_ported": native_follow,
    "native_modeld_direct_action_head_present": native_direct_action,
    "port_ready_for_vehicle_control": False,
    "next": "Reconcile Tesla carState/controlsd/long MPC ownership with Panda and run locked replay/negative tests",
  }


if __name__ == "__main__":
  print(json.dumps(audit(), indent=2, sort_keys=True))
