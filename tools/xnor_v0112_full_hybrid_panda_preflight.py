#!/usr/bin/env python3
"""Source-only preflight for full XNOR 0.11.2 Tesla Hybrid Panda candidate.

Checks the exact pinned sources and properties of the *staged* replacement
files. This does NOT compile Panda or establish safety/performance. Active
gitlinks and installed Panda firmware remain unchanged.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "migration/tesla_v0112/hybrid_panda_candidate"
SOURCES = {
  "opendbc": ROOT / "opendbc_repo",
  "panda": ROOT / "panda",
}
NATIVE_BLOBS = {
  "opendbc": {
    "opendbc/safety/declarations.h": "b877ae3d5c6f45520bdccb1b3bd5b08ecffec24d",
    "opendbc/safety/safety.h": "10a0c4db515e1abf35356bdb7c6f9019accf0b99",
    "opendbc/safety/modes/tesla_legacy.h": "ba23ea1459999923e6b7d715eb7938224d5a3bba",
    "opendbc/car/tesla/values.py": "c9b2025e9d35f04fbee2dee470f9bce80f28264c",
    "opendbc/car/tesla/interface.py": "30a9c2c9f79682e11a765447065df207b21e6946",
    "opendbc/car/tesla/radar_interface.py": "c379f2163b0d3d59a28b5073c75590017d0c3736",
    "opendbc/dbc/tesla_can.dbc": "447231d5655961e0ee69e89d2cdf43e77c262819",
    "opendbc/safety/tests/libsafety/libsafety_py.py": "23c9dccc26032d9147f54666dd54d5094e7f49e2",
    "opendbc/car/car.capnp": "464743e33f4bc275fbed4744231e0f991c829543",
    "opendbc/car/tesla/carcontroller.py": "6d5e40a1e74b0f10c43ecdffd626ae82337cb745",
    "opendbc/car/tesla/carstate.py": "eb44677e0a85a5b47dfdc60f3d4faff657476152",
    "opendbc/car/tesla/teslacan.py": "9760f926615ef90b2c0897ef217c59330dd1bbc8",
    "opendbc/car/tesla/teslacan_legacy.py": "3d905b7754d0021b427d079c42166c29b95db2",
  },
  "panda": {
    "board/drivers/bxcan.h": "52872e3c2ae9fab162ed9a3074622167bd08c592",
    "board/drivers/fdcan.h": "867029dffbbaa3f52c75d03be247b07c5826e982",
  },
}
NEW_TESTS = ["opendbc/safety/tests/test_tesla_hybrid_packet.py"]
NEW_VEHICLE_FILES = (
  "opendbc/car/tesla/carcontroller_native_0112.py",
  "opendbc/car/tesla/carstate_native_0112.py",
  "opendbc/car/tesla/legacy_modules/__init__.py",
  "opendbc/car/tesla/legacy_modules/ALC_module.py",
  "opendbc/car/tesla/legacy_modules/ACC_module.py",
  "opendbc/car/tesla/legacy_modules/BLNK_module.py",
  "opendbc/car/tesla/legacy_modules/CFG_module.py",
  "opendbc/car/tesla/legacy_modules/HSO_module.py",
  "opendbc/car/tesla/legacy_modules/LONG_module.py",
)


def git_blob(path: Path) -> str:
  data = path.read_bytes()
  return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()


def text(relative: str, component: str) -> str:
  return (BASE / component / relative).read_text(encoding="utf-8")


def require(yes: bool, why: str) -> None:
  if not yes:
    raise RuntimeError(why)


def audit() -> dict:
  count = 0
  for component, files in NATIVE_BLOBS.items():
    for rel, sha in files.items():
      native = SOURCES[component] / rel
      require(native.is_file(), f"Missing pinned {component} source {native}")
      require(git_blob(native) == sha,
              f"Pinned {component} has drifted: {rel}; do not export/apply candidate")
      staged = BASE / component / rel
      require(staged.is_file(), f"Missing candidate source {staged}")
      count += 1

  declarations = text("opendbc/safety/declarations.h", "opendbc")
  safety = text("opendbc/safety/safety.h", "opendbc")
  mode = text("opendbc/safety/modes/tesla_legacy.h", "opendbc")
  values = text("opendbc/car/tesla/values.py", "opendbc")
  interface = text("opendbc/car/tesla/interface.py", "opendbc")
  dbc = text("opendbc/dbc/tesla_can.dbc", "opendbc")
  radar = text("opendbc/car/tesla/radar_interface.py", "opendbc")
  cffi = text("opendbc/safety/tests/libsafety/libsafety_py.py", "opendbc")
  bxcan = text("board/drivers/bxcan.h", "panda")
  fdcan = text("board/drivers/fdcan.h", "panda")

  require("typedef bool (*fwd_msg_hook)" in declarations and
          "fwd_msg_hook fwd_msg;" in declarations,
          "Packet callback missing from safety_hooks definition")
  require("int safety_fwd_hook(int bus_num, int addr)" in declarations and
          "int safety_fwd_hook_pkt(int bus_num, CANPacket_t *to_fwd)" in declarations,
          "Address-only and packet-aware safety dispatch ABIs must coexist")
  require("int safety_fwd_hook_pkt(int bus_num, CANPacket_t *to_fwd) {" in safety,
          "Packet-forward dispatcher implementation missing")
  require("const CANPacket_t before = *to_fwd;" in safety and
          "if (block || (to_fwd->addr != before.addr)" in safety,
          "Forward callback must fail closed on message identity mutation")
  require("const bool real_aeb = current_safety_mode == SAFETY_TESLA_LEGACY" in safety and
          "if (real_aeb) {" in safety and
          "return destination_bus;" in safety,
          "Real stock AEB must bypass all mutation hooks")
  require("bool frequency_invalid = frequency < 10U;" in safety and
          "current_safety_config.disable_forwarding" in safety,
          "XNOR's modern RX checks and forwarding disable must survive")
  require("tesla_hybrid_brake_lateral_only" in declarations and
          "tesla_hybrid_brake_lateral_only = false;" in safety and
          "controls_allowed && !tesla_legacy_stock_aeb" in safety,
          "Brake-only lateral exception has lost its stock-AEB guard")
  require("static bool tesla_legacy_fwd_msg_hook(" in mode and
          ".fwd_msg = tesla_legacy_fwd_msg_hook" in mode,
          "Full Tesla Hybrid fwd_msg receiver not registered")
  for token in ("0x659", "tesla_legacy_v229_capture_das_control",
                "tesla_legacy_v229_apply_das_control", "tesla_legacy_stock_aeb",
                "tesla_legacy_v229_observe_di_state", "TESLA_LEGACY_TX_MSGS_LONG",
                "TESLA_LEGACY_TX_MSGS_LATERAL"):
    require(token in mode, f"Legacy Hybrid safety feature missing: {token}")
  require("#define TESLA_LEGACY_MITM_BLOCK_STOCK_GTW 0" in mode and
          "#define TESLA_LEGACY_OVERRIDE_GTW_AUTOPILOT 0" in mode,
          "Dangerous bench-only GTW gateway override must default OFF")
  require("if (aeb_event != 1 && tesla_legacy_should_scrub_aeb_event(aeb_event))" in mode and
          "tesla_legacy_stock_aeb = real_aeb;" in mode,
          "Genuine AEB arbitration must remain preserved")
  require("OP_STALK_ENABLE = 64" in values and "IGNORE_STOCK_AEB = 128" in values and
          "AngleSteeringLimitsVM" in values,
          "Ported safety parameter ABI or native Tesla steering limits absent")
  require("for cfg in ret.safetyConfigs:" in interface and
          "cfg.safetyParam |= int(TeslaSafetyFlags.LONG_CONTROL)" in interface,
          "Dual-Panda LONG_CONTROL not configured on both devices")
  require("ret.pcmCruise = hybrid_native_ap" in interface,
          "Legacy native-AP Hybrid cruise ownership not configured")
  require("BO_ 937 DAS_telemetry: 8 XXX" in dbc and
          "DAS_lssState : 31|3@1+" in dbc,
          "Native DBC layout or extra Tesla HUD lane frame missing")
  require("return self.last_radar_data" in radar and
          "self.pts[i].aRel = msg_a['LongAccel']" in radar,
          "Native Bosch radar sample hold or legacy acceleration point missing")
  require("int safety_fwd_hook_pkt(int bus_num, CANPacket_t *to_fwd);" in cffi,
          "Compiled safety test binding missing")
  for driver, content in (("bxcan", bxcan), ("fdcan", fdcan)):
    require("CANPacket_t to_send = to_push;" in content and
            "safety_fwd_hook_pkt(bus_number, &to_send)" in content and
            "can_set_checksum(&to_send)" in content and
            "safety_rx_hook(&to_push)" in content,
            f"{driver} no longer forwards only a private, checksummed RX copy")
  require("current_safety_mode != SAFETY_TESLA_LEGACY" in fdcan and
          "#ifdef PANDA_BODY" in fdcan,
          "Tesla safety block must not fall back to an unsafe bus, or lose native body support")
  for rel in NEW_TESTS:
    require((BASE / "opendbc" / rel).is_file(),
            f"Missing compiled-safety negative tests: {rel}")

  for rel in (
      "opendbc/car/tesla/values.py",
      "opendbc/car/tesla/interface.py",
      "opendbc/car/tesla/radar_interface.py",
      "opendbc/safety/tests/libsafety/libsafety_py.py",
      *NEW_TESTS,
  ):
    file = BASE / "opendbc" / rel
    compile(file.read_text(encoding="utf-8"), str(file), "exec")

  return {
    "baseline": {
      "opendbc": "3f86206a13c3181f0224e8f1424b836592e2b2d2",
      "panda": "fa83a40ad799c827ab9cbda4560cf3426531ceff",
    },
    "native_blob_sources_verified": count,
    "full_hybrid_staged": True,
    "packet_aware_fwd_present": True,
    "both_panda_drivers_ported": True,
    "genuine_stock_AEB_unmodified": True,
    "GTW_bench_MITM_default": "OFF",
    "compiled_safety_tests_executed": False,
    "panda_firmware_built_or_flashed": False,
    "suitable_for_vehicle_control": False,
  }


if __name__ == "__main__":
  print(json.dumps(audit(), indent=2, sort_keys=True))
