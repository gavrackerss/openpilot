#!/usr/bin/env python3
"""Tesla HW1/HW2/HW3 Panda/opendbc migration preflight (READ ONLY).

Run with the exact XNOR opendbc submodule initialised:
  git submodule update --init opendbc_repo
  python3 tools/xnor_v0112_tesla_panda_preflight.py

This checks source identity and explicit safety differences. It is not a Panda
safety-test executable; no CAN messages are transmitted. The modified
legacy Tesla safety implementation is NOT eligible for production activation.
"""
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
PINNED = ROOT / "opendbc_repo/opendbc"
LEGACY = ROOT / "migration/tesla_opendbc_legacy/opendbc"
OVERLAY = ROOT / "migration/tesla_v0112/opendbc_overlay"
NATIVE_BLOBS = {
  "car/tesla/values.py": "c9b2025e9d35f04fbee2dee470f9bce80f28264c",
  "car/tesla/interface.py": "30a9c2c9f79682e11a765447065df207b21e6946",
  "car/tesla/radar_interface.py": "c379f2163b0d3d59a28b5073c75590017d0c3736",
  "dbc/tesla_can.dbc": "447231d5655961e0ee69e89d2cdf43e77c262819",
  "safety/modes/tesla_legacy.h": "ba23ea1459999923e6b7d715eb7938224d5a3bba",
}
EXPECTED_SAFE_FLAGS = {
  "LONG_CONTROL": 1,
  "FSD_14": 2,
  "FLAG_EXTERNAL_PANDA": 4,
  "FLAG_HW1": 8,
  "FLAG_HW2": 16,
  "FLAG_HW3": 32,
}
OLD_EXTRA_FLAGS = {
  "OP_STALK_ENABLE": 64,
  "IGNORE_STOCK_AEB": 128,
}


def git_blob(data: bytes) -> str:
  return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()


def read(path: Path) -> str:
  if not path.is_file():
    raise FileNotFoundError(f"Required source missing: {path}. Initialise opendbc_repo.")
  return path.read_text(encoding="utf-8")


def flags(path: Path) -> dict[str, int]:
  tree = ast.parse(read(path))
  cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "TeslaSafetyFlags")
  return {
    node.targets[0].id: int(ast.literal_eval(node.value))
    for node in cls.body
    if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name)
  }


def message_names(dbc: str) -> dict[int, str]:
  out = {}
  for address, name in re.findall(r"^BO_\s+(\d+)\s+(\w+):", dbc, re.M):
    key = int(address)
    if key in out:
      raise ValueError(f"Duplicate DBC message address: {key}")
    out[key] = name
  return out


def signal(msg: str, dbc: str) -> str:
  m = re.search(rf"^ SG_\s+{re.escape(msg)}\s*:\s*([^\n]+)$", dbc, re.M)
  if m is None:
    raise ValueError(f"Missing DBC signal: {msg}")
  return m.group(1).strip()


def audit() -> dict:
  for name, sha in NATIVE_BLOBS.items():
    actual = git_blob((PINNED / name).read_bytes())
    if actual != sha:
      raise RuntimeError(f"Pinned opendbc drifted: {name}: {actual}, expected {sha}")

  native_safety = read(PINNED / "safety/modes/tesla_legacy.h")
  stage_safety = read(OVERLAY / "safety/modes/tesla_legacy.h")
  old_safety = read(LEGACY / "safety/modes/tesla_legacy.h")
  if stage_safety != native_safety:
    raise RuntimeError("Staged safety header differs from pinned XNOR safety; fail closed")
  if not ("if (aeb_event != 0)" in native_safety and "if (tesla_legacy_stock_aeb)" in native_safety):
    raise RuntimeError("Native AEB block conditions missing")
  if "TESLA_LEGACY_FLAG_IGNORE_STOCK_AEB" in stage_safety or "0x659" in stage_safety:
    raise RuntimeError("Unqualified carrier or AEB suppression present in staged safety")

  nf = flags(PINNED / "car/tesla/values.py")
  sf = flags(OVERLAY / "car/tesla/values.py")
  lf = flags(LEGACY / "car/tesla/values.py")
  if nf != EXPECTED_SAFE_FLAGS or sf != nf:
    raise RuntimeError(f"Native Tesla safety flag mapping changed: {nf!r}, overlay {sf!r}")
  if any(lf.get(key) != expected for key, expected in OLD_EXTRA_FLAGS.items()):
    raise RuntimeError("Unexpected historical safety flag values")
  native_values = read(PINNED / "car/tesla/values.py")
  values = read(OVERLAY / "car/tesla/values.py")
  if "ANGLE_LIMITS: AngleSteeringLimitsVM" not in values:
    raise RuntimeError("Overlay lost native VM steering-limit class")
  if "class CruiseButtons:" not in values or "CURVE_ASSIST_MAX_DELTA_V" not in values:
    raise RuntimeError("Missing passive Tesla constants")
  # Ensure the baseline is byte-for-byte present except inserted declarations.
  if any(line not in values for line in native_values.splitlines()):
    raise RuntimeError("Overlay removed/changed an existing native Tesla values line")

  native_dbc = read(PINNED / "dbc/tesla_can.dbc")
  stage_dbc = read(OVERLAY / "dbc/tesla_can.dbc")
  original_dbc = read(LEGACY / "dbc/tesla_can.dbc")
  add = "BO_ 937 DAS_telemetry: 8 XXX"
  if add not in stage_dbc or add in native_dbc:
    raise RuntimeError("Additive lane telemetry DBC 0x3A9 absent/duplicated")
  # The additive BO_937 belongs among BO_ declarations, before the VAL_
  # tables. Removing *only* that block must reproduce the pinned DBC byte for
  # byte, not just preserve its set of signal names.
  insertion_start = stage_dbc.index(add)
  insertion_end = stage_dbc.find("\n\n", insertion_start)
  if insertion_end < 0 or stage_dbc[:insertion_start] + stage_dbc[insertion_end + 2:] != native_dbc:
    raise RuntimeError("Staged DBC modified existing XNOR messages/signals")
  names = message_names(stage_dbc)
  if names.get(937) != "DAS_telemetry":
    raise RuntimeError("Telemetry address mismatch")
  for name in ("DAS_telemetryMultiplexer", "DAS_telLeftLaneType",
               "DAS_telRightLaneType", "DAS_telLeftMarkerQuality",
               "DAS_telRightMarkerQuality", "DAS_telLeftMarkerColor",
               "DAS_telRightMarkerColor", "DAS_telLeftLaneCrossing",
               "DAS_telRightLaneCrossing"):
    signal(name, stage_dbc)
  native_cruise = signal("DI_cruiseSet", native_dbc)
  original_cruise = signal("DI_cruiseSet", original_dbc)
  native_speed = signal("DI_digitalSpeed", native_dbc)
  original_speed = signal("DI_digitalSpeed", original_dbc)
  if native_cruise == original_cruise or native_speed == original_speed:
    raise RuntimeError("DI_state signal-layout conflict unexpectedly disappeared")
  native_status2 = native_dbc.split("BO_ 905 DAS_status2:", 1)[1].split("\nBO_", 1)[0]
  legacy_status2 = original_dbc.split("BO_ 905 DAS_status2:", 1)[1].split("\nBO_", 1)[0]
  if "31|3@1+" not in signal("DAS_lssState", native_status2):
    raise RuntimeError("Pinned DAS_status2 no longer defines native LSS occupancy")
  if "31|1@1+" not in signal("DAS_relaxCruiseLimits", legacy_status2):
    raise RuntimeError("Legacy DAS_status2 conflict unexpectedly changed")
  if "32|2@1+" not in signal("DAS_csaState", legacy_status2):
    raise RuntimeError("Legacy CSA state location unexpectedly changed")

  # Native Bosch radar repeats its most recent valid 8 Hz sample to the
  # faster 20 Hz consumer. The installed fork discards that on missing
  # updates. Retain native caching and port only three point-data fields.
  native_radar = read(PINNED / "car/tesla/radar_interface.py")
  overlay_radar = read(OVERLAY / "car/tesla/radar_interface.py")
  radar_fields = (
    "      self.pts[i].aRel = msg_a['LongAccel']\\n"
    "      self.pts[i].yvRel = msg_b['LatSpeed']\\n"
    "      self.pts[i].measured = bool(msg_a['Meas'])\\n"
  )
  if overlay_radar.replace(radar_fields, "", 1) != native_radar:
    raise RuntimeError("Radar overlay changed native behavior beyond 3 point fields")
  if not ("return self.last_radar_data" in overlay_radar and
          "self.last_radar_data = rr" in overlay_radar):
    raise RuntimeError("Native Bosch radar update caching was lost")

  native_interface = read(PINNED / "car/tesla/interface.py")
  old_interface = read(LEGACY / "car/tesla/interface.py")
  if "ret.safetyConfigs[0].safetyParam |= TeslaSafetyFlags.LONG_CONTROL.value" not in native_interface:
    raise RuntimeError("Unexpected native Tesla LONG_CONTROL config")
  if "for cfg in ret.safetyConfigs:" not in old_interface:
    raise RuntimeError("Unexpected legacy Tesla dual-Panda safety flags")

  return {
    "pinned_opendbc_sha": "3f86206a13c3181f0224e8f1424b836592e2b2d2",
    "verified_native_blob_count": len(NATIVE_BLOBS),
    "safety_header": "unchanged from XNOR; stock-AEB TX inhibitions retained",
    "tesla_platform_flags": sf,
    "legacy_unsupported_safety_flags": OLD_EXTRA_FLAGS,
    "additive_telemetry_dbc_address": "0x3A9 / 937",
    "radar_overlay": "3 additional read-only point fields; native Bosch 8 Hz update cache retained",
    "native_DI_cruiseSet": native_cruise,
    "legacy_DI_cruiseSet": original_cruise,
    "native_DI_digitalSpeed": native_speed,
    "legacy_DI_digitalSpeed": original_speed,
    "blocked_on": [
      "physical CAN replay: incompatible DI_state speed signal positions",
      "DAS_status2: native DAS_lssState overlaps legacy relaxCruiseLimits/csaState",
      "Tesla HW2 dual-Panda LONG_CONTROL and safety parameter ownership",
      "old 0x659 cooperative Panda carrier does not exist in pinned safety",
      "old 0x2B9/0x2BF substitute-on-native-cadence hybrid mode not ported",
      "legacy AEB suppression is unqualified; stock AEB inhibition is retained",
      "native VM steering limits differ from legacy windup/unwind tuning",
      "legacy Tesla CarState/CarController, radar and CAN sender not ported",
    ],
    "ready_to_flash_panda": False,
    "ready_to_enable_tesla_longitudinal": False,
    "note": "source audit only; does not build or run Panda safety tests",
  }


if __name__ == "__main__":
  print(json.dumps(audit(), indent=2, sort_keys=True))
