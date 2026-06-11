#!/usr/bin/env python3
"""
XNOR AEB boot watcher v3.

Captures the boot / panda safety / Tesla AEB handover without dumping every raw CAN
frame by default. The previous watcher could produce 100MB+ JSONL captures because it
logged every CAN frame. This version records decoded AEB-related transitions and state
snapshots by default, and only logs raw CAN if --raw-can is explicitly passed.

Live use on device:
  cd /data/openpilot
  tools/xnor_aeb_boot_watch.py --duration 240 --output-dir /data/openpilot/aeb_boot_watch

Offline re-decode of an old raw JSONL:
  tools/xnor_aeb_boot_watch.py --replay-jsonl /data/openpilot/aeb_boot_watch/old.jsonl --output-dir /data/openpilot/aeb_boot_watch
"""
from __future__ import annotations

import argparse
import collections
import datetime as _dt
import gzip
import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any

# --- Tesla signal decode ----------------------------------------------------
# Lightweight DBC signal extraction for the handful of signals needed here.
# All signals below are little-endian (@1) in the Tesla DBCs.

AEB_STATE = {
  0: "UNAVAILABLE",
  1: "STANDBY",
  2: "ENABLED",
  3: "STANDSTILL",
  4: "FAULT",
  7: "SNA",
}

DAS_AEB_EVENT = {
  0: "AEB_NOT_ACTIVE",
  1: "AEB_ACTIVE",
  2: "AEB_FAULT",
  3: "AEB_SNA",
}

DAS_ACC_STATE = {
  0: "ACC_CANCEL_GENERIC",
  1: "ACC_CANCEL_CAMERA_BLIND",
  2: "ACC_CANCEL_RADAR_BLIND",
  3: "ACC_HOLD",
  4: "ACC_ON",
  5: "APC_BACKWARD",
  6: "APC_FORWARD",
  7: "APC_COMPLETE",
  8: "APC_ABORT",
  9: "APC_PAUSE",
  10: "APC_UNPARK_COMPLETE",
  11: "APC_SELFPARK_START",
  12: "ACC_CANCEL_PATH_NOT_CLEAR",
  13: "ACC_CANCEL_GENERIC_SILENT",
  14: "ACC_CANCEL_OUT_OF_CALIBRATION",
  15: "FAULT_SNA",
}

ENABLE_2BIT = {
  0: "OFF/UNSET",
  1: "ON/ENABLED",
  2: "UNAVAILABLE/FAULT?",
  3: "SNA",
}

CRUISE_STATE = {
  0: "STANDBY",
  1: "ENABLED",
  2: "STANDSTILL",
  3: "OVERRIDE",
  4: "PRE_FAULT",
  5: "PRE_CANCEL",
  6: "CANCELLED",
  7: "FAULT",
  8: "SNA",
}

# name, addr, start_bit, length, enum, message_name
SIGNALS = [
  # Legacy S/X powertrain DI_state
  ("DI_driveReady", 0x256, 7, 1, {0: "NOT_READY", 1: "READY"}, "DI_state/tesla_powertrain"),
  ("DI_cruiseState", 0x256, 12, 4, CRUISE_STATE, "DI_state/tesla_powertrain"),
  ("DI_aebState", 0x256, 41, 3, AEB_STATE, "DI_state/tesla_powertrain"),

  # Legacy S/X party/chassis DI_state
  ("DI_driveReady", 0x368, 7, 1, {0: "NOT_READY", 1: "READY"}, "DI_state/tesla_can"),
  ("DI_cruiseState", 0x368, 12, 4, CRUISE_STATE, "DI_state/tesla_can"),
  ("DI_aebState", 0x368, 41, 3, AEB_STATE, "DI_state/tesla_can"),

  # Model 3/Y party DI_state
  ("DI_aebState", 0x286, 37, 3, AEB_STATE, "DI_state/model3_party"),

  # DAS_control on party and powertrain variants
  ("DAS_accState", 0x2B9, 12, 4, DAS_ACC_STATE, "DAS_control/party"),
  ("DAS_aebEvent", 0x2B9, 16, 2, DAS_AEB_EVENT, "DAS_control/party"),
  ("DAS_accState", 0x2BF, 12, 4, DAS_ACC_STATE, "DAS_control/powertrain"),
  ("DAS_aebEvent", 0x2BF, 16, 2, DAS_AEB_EVENT, "DAS_control/powertrain"),

  # MCU_chassisControl settings; useful to prove whether AEB is enabled at settings level
  ("MCU_fcwEnable", 0x218, 6, 2, ENABLE_2BIT, "MCU_chassisControl"),
  ("MCU_latControlEnable", 0x218, 8, 2, ENABLE_2BIT, "MCU_chassisControl"),
  ("MCU_ldwEnable", 0x218, 12, 2, ENABLE_2BIT, "MCU_chassisControl"),
  ("MCU_aebEnable", 0x218, 14, 2, ENABLE_2BIT, "MCU_chassisControl"),
  ("MCU_pedalSafetyEnable", 0x218, 22, 2, ENABLE_2BIT, "MCU_chassisControl"),
]

SIGNALS_BY_ADDR: dict[int, list[tuple[str, int, int, int, dict[int, str], str]]] = collections.defaultdict(list)
for spec in SIGNALS:
  SIGNALS_BY_ADDR[spec[1]].append(spec)

SUSPICIOUS_VALUES = {
  "DI_aebState": {0, 4, 7},
  "DAS_aebEvent": {1, 2, 3},
  "DAS_accState": {1, 2, 12, 13, 14, 15},
}


def iso_now() -> str:
  return _dt.datetime.now(_dt.timezone.utc).isoformat(timespec="milliseconds")


def safe_enum(v: Any) -> Any:
  if v is None:
    return None
  try:
    s = str(v)
    # capnp enum values often stringify cleanly already, but trim noisy prefixes if present
    return s.split(".")[-1]
  except Exception:
    return v


def safe_float(v: Any, default: float = 0.0) -> float:
  try:
    return float(v)
  except Exception:
    return default


def mph_from_ms(v: Any) -> float:
  return safe_float(v) * 2.2369362921


def get_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def extract_le(dat: bytes, start: int, length: int) -> int:
  raw = int.from_bytes(dat, "little", signed=False)
  return (raw >> start) & ((1 << length) - 1)


def decode_can_frame(address: int, dat: bytes, src: int, mono_ms: int, wall_time: str) -> list[dict[str, Any]]:
  out = []
  for signal, addr, start, length, enum_map, msg_name in SIGNALS_BY_ADDR.get(address, []):
    val = extract_le(dat, start, length)
    out.append({
      "kind": "decoded_can",
      "monoMs": mono_ms,
      "wallTime": wall_time,
      "src": src,
      "address": address,
      "addressHex": f"0x{address:X}",
      "message": msg_name,
      "signal": signal,
      "value": val,
      "text": enum_map.get(val, f"UNKNOWN_{val}"),
      "dat": dat.hex(),
    })
  return out


def summarise_car_state(cs: Any) -> dict[str, Any]:
  cruise = get_attr(cs, "cruiseState")
  return {
    "vEgo": safe_float(get_attr(cs, "vEgo")),
    "vEgoMph": round(mph_from_ms(get_attr(cs, "vEgo")), 1),
    "gasPressed": bool(get_attr(cs, "gasPressed", False)),
    "brakePressed": bool(get_attr(cs, "brakePressed", False)),
    "steeringPressed": bool(get_attr(cs, "steeringPressed", False)),
    "gearShifter": safe_enum(get_attr(cs, "gearShifter")),
    "standstill": bool(get_attr(cs, "standstill", False)),
    "stockAeb": bool(get_attr(cs, "stockAeb", False)),
    "accFaulted": bool(get_attr(cs, "accFaulted", False)),
    "canValid": bool(get_attr(cs, "canValid", False)),
    "canTimeout": bool(get_attr(cs, "canTimeout", False)),
    "cruiseState.enabled": bool(get_attr(cruise, "enabled", False)),
    "cruiseState.available": bool(get_attr(cruise, "available", False)),
    "cruiseState.speed": safe_float(get_attr(cruise, "speed")),
    "cruiseState.speedMph": round(mph_from_ms(get_attr(cruise, "speed")), 1),
  }


def summarise_controls_state(ctrl: Any) -> dict[str, Any]:
  return {
    "enabled": get_attr(ctrl, "enabled"),
    "active": get_attr(ctrl, "active"),
    "state": safe_enum(get_attr(ctrl, "state")),
    "longControlState": safe_enum(get_attr(ctrl, "longControlState")),
    "alertText1": get_attr(ctrl, "alertText1"),
    "alertText2": get_attr(ctrl, "alertText2"),
    "alertType": get_attr(ctrl, "alertType"),
    "forceDecel": bool(get_attr(ctrl, "forceDecel", False)),
  }


def summarise_device_state(ds: Any) -> dict[str, Any]:
  return {
    "started": bool(get_attr(ds, "started", False)),
    "startedMonoTime": int(get_attr(ds, "startedMonoTime", 0) or 0),
    "thermalStatus": safe_enum(get_attr(ds, "thermalStatus")),
    "freeSpacePercent": safe_float(get_attr(ds, "freeSpacePercent")),
    "networkType": safe_enum(get_attr(ds, "networkType")),
    "networkStrength": safe_enum(get_attr(ds, "networkStrength")),
  }


def summarise_panda_states(panda_states: Any) -> list[dict[str, Any]]:
  out = []
  try:
    iterable = list(panda_states)
  except Exception:
    iterable = []
  for idx, ps in enumerate(iterable):
    faults = []
    try:
      faults = [safe_enum(x) for x in list(get_attr(ps, "faults", []))]
    except Exception:
      pass
    out.append({
      "idx": idx,
      "pandaType": safe_enum(get_attr(ps, "pandaType")),
      "safetyModel": safe_enum(get_attr(ps, "safetyModel")),
      "safetyParam": int(get_attr(ps, "safetyParam", 0) or 0),
      "controlsAllowed": bool(get_attr(ps, "controlsAllowed", False)),
      "faultStatus": safe_enum(get_attr(ps, "faultStatus")),
      "faults": faults,
      "ignitionLine": bool(get_attr(ps, "ignitionLine", False)),
      "ignitionCan": bool(get_attr(ps, "ignitionCan", False)),
      "heartbeatLost": bool(get_attr(ps, "heartbeatLost", False)),
      "safetyTxBlocked": int(get_attr(ps, "safetyTxBlocked", 0) or 0),
      "safetyRxInvalid": int(get_attr(ps, "safetyRxInvalid", 0) or 0),
      "safetyRxChecksInvalid": bool(get_attr(ps, "safetyRxChecksInvalid", False)),
    })
  return out


def summarise_manager_state(ms: Any) -> list[dict[str, Any]]:
  out = []
  try:
    processes = list(get_attr(ms, "processes", []))
  except Exception:
    processes = []
  for p in processes:
    name = get_attr(p, "name")
    if name in ("controlsd", "card", "boardd", "pandad", "modeld", "plannerd", "radard"):
      out.append({
        "name": name,
        "running": bool(get_attr(p, "running", False)),
        "shouldBeRunning": bool(get_attr(p, "shouldBeRunning", False)),
        "exitCode": int(get_attr(p, "exitCode", 0) or 0),
      })
  return out


def context_line(ctx: dict[str, Any]) -> str:
  cs = ctx.get("carState", {}) or {}
  if cs.get("vEgoMph") is None and cs.get("vEgo") is not None:
    try:
      cs["vEgoMph"] = round(float(cs.get("vEgo")) * 2.2369362921, 1)
    except Exception:
      pass
  ctr = ctx.get("controlsState", {}) or {}
  ds = ctx.get("deviceState", {}) or {}
  pandas = ctx.get("pandaStates", []) or []
  panda_s = ";".join(
    f"p{(p.get('idx') if p.get('idx') is not None else i)}:{p.get('safetyModel')}/allowed={int(bool(p.get('controlsAllowed')))}"
    f"/fault={p.get('faultStatus')}/txBlk={p.get('safetyTxBlocked')}"
    for i, p in enumerate(pandas)
  ) or "no-panda"
  return (
    f"v={cs.get('vEgoMph')}mph gear={cs.get('gearShifter')} gas={int(bool(cs.get('gasPressed')))} "
    f"brake={int(bool(cs.get('brakePressed')))} cruiseEn={int(bool(cs.get('cruiseState.enabled')))} "
    f"cruiseAvail={int(bool(cs.get('cruiseState.available')))} long={ctr.get('longControlState')} "
    f"op_started={int(bool(ds.get('started')))} {panda_s}"
  )


class WatchWriter:
  def __init__(self, out_dir: Path, prefix: str, quiet: bool = False, gzip_jsonl: bool = False) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    self.jsonl_path = out_dir / f"{prefix}_{stamp}.jsonl"
    self.txt_path = out_dir / f"{prefix}_{stamp}.txt"
    self.quiet = quiet
    self._txt = self.txt_path.open("w", encoding="utf-8")
    self._jsonl_raw = self.jsonl_path.open("wb") if gzip_jsonl else self.jsonl_path.open("w", encoding="utf-8")
    self._gzip = gzip.GzipFile(fileobj=self._jsonl_raw, mode="wb") if gzip_jsonl else None

  def write_json(self, obj: dict[str, Any]) -> None:
    line = json.dumps(obj, sort_keys=True, separators=(",", ":")) + "\n"
    if self._gzip is not None:
      self._gzip.write(line.encode("utf-8"))
    else:
      self._jsonl_raw.write(line)

  def write_txt(self, line: str = "") -> None:
    self._txt.write(line + "\n")
    self._txt.flush()
    if not self.quiet:
      print(line)

  def close(self) -> None:
    if self._gzip is not None:
      self._gzip.close()
    self._jsonl_raw.close()
    self._txt.close()


def open_replay(path: Path):
  if str(path).endswith(".gz"):
    return gzip.open(path, "rt", encoding="utf-8", errors="replace")
  return path.open("r", encoding="utf-8", errors="replace")


def process_stream(args: argparse.Namespace, replay_path: Path | None = None) -> tuple[Path, Path]:
  prefix = "xnor_aeb_boot_watch_v3"
  writer = WatchWriter(Path(args.output_dir), prefix, quiet=args.quiet, gzip_jsonl=args.gzip_jsonl)

  decoded_last: dict[tuple[int, int, str], int] = {}
  state_last: dict[str, Any] | None = None
  panda_last: list[dict[str, Any]] | None = None
  counts_by_addr_src: collections.Counter[tuple[int, int]] = collections.Counter()
  changes_by_addr_src: collections.Counter[tuple[int, int]] = collections.Counter()
  last_payload_by_addr_src: dict[tuple[int, int], str] = {}
  decoded_events = 0
  raw_can_logged = 0
  stopped = False
  t0_wall = iso_now()
  t0_mono = int(time.monotonic() * 1000)
  rel_base_mono: int | None = None
  ctx: dict[str, Any] = {}
  last_state_emit_ms = -10**9
  last_suspicious_repeat: dict[tuple[int, int, str, int], int] = {}

  def rel_s(mono_ms: int) -> float:
    base = rel_base_mono if rel_base_mono is not None else t0_mono
    return (mono_ms - base) / 1000.0

  def handle_sig(_signum, _frame):
    nonlocal stopped
    stopped = True

  signal.signal(signal.SIGINT, handle_sig)
  signal.signal(signal.SIGTERM, handle_sig)

  meta = {
    "kind": "meta",
    "tool": "xnor_aeb_boot_watch_v3",
    "started": t0_wall,
    "monoMs": t0_mono,
    "duration": args.duration,
    "rawCan": bool(args.raw_can),
    "bootAuto": bool(getattr(args, "boot_auto", False)),
    "note": "Decoded AEB/panda boot capture. Raw CAN is off by default to avoid huge git-blocking JSONL files. v3 adds launch-hook auto-start support.",
    "replayJsonl": str(replay_path) if replay_path else None,
  }
  writer.write_json(meta)
  writer.write_txt("# xnor AEB boot watch v2")
  writer.write_txt(f"jsonl: {writer.jsonl_path}")
  writer.write_txt(f"txt:   {writer.txt_path}")
  writer.write_txt(f"raw_can_logging: {bool(args.raw_can)}")
  writer.write_txt("")

  def update_state_from_obj(mono_ms: int, wall_time: str, car_state: Any = None, controls_state: Any = None,
                            device_state: Any = None, panda_states: Any = None, manager_state: Any = None,
                            replay_state_dict: dict[str, Any] | None = None) -> None:
    nonlocal state_last, panda_last, last_state_emit_ms, ctx
    if replay_state_dict is not None:
      # Old JSONL replay already has flattened state dictionaries.
      ctx = {
        "carState": replay_state_dict.get("carState", {}),
        "controlsState": replay_state_dict.get("controlsState", {}),
        "deviceState": replay_state_dict.get("deviceState", {}),
        "pandaStates": replay_state_dict.get("pandaStates", []),
        "managerState": replay_state_dict.get("managerState", []),
      }
    else:
      if car_state is not None:
        ctx["carState"] = summarise_car_state(car_state)
      if controls_state is not None:
        ctx["controlsState"] = summarise_controls_state(controls_state)
      if device_state is not None:
        ctx["deviceState"] = summarise_device_state(device_state)
      if panda_states is not None:
        ctx["pandaStates"] = summarise_panda_states(panda_states)
      if manager_state is not None:
        ctx["managerState"] = summarise_manager_state(manager_state)

    # Emit state snapshots at a low fixed rate, plus immediately when panda safety/fault state changes.
    pandas_now = ctx.get("pandaStates", []) or []
    panda_changed = pandas_now != (panda_last or [])
    should_emit = (mono_ms - last_state_emit_ms) >= int(args.state_interval * 1000) or panda_changed
    if should_emit:
      rec = {
        "kind": "state",
        "monoMs": mono_ms,
        "wallTime": wall_time,
        **ctx,
      }
      writer.write_json(rec)
      last_state_emit_ms = mono_ms
      if panda_changed and panda_last is not None:
        writer.write_txt(f"STATE/PANDA t={rel_s(mono_ms):.3f}s {context_line(ctx)}")
      state_last = rec
      panda_last = list(pandas_now)

  def process_can(address: int, src: int, dat_hex: str, mono_ms: int, wall_time: str, bus_time: int = 0) -> None:
    nonlocal decoded_events, raw_can_logged
    dat = bytes.fromhex(dat_hex)
    key = (src, address)
    counts_by_addr_src[key] += 1
    if last_payload_by_addr_src.get(key) != dat_hex:
      changes_by_addr_src[key] += 1
      last_payload_by_addr_src[key] = dat_hex

    if args.raw_can:
      raw_can_logged += 1
      if args.max_raw_can and raw_can_logged > args.max_raw_can:
        pass
      else:
        writer.write_json({
          "kind": "can",
          "monoMs": mono_ms,
          "wallTime": wall_time,
          "src": src,
          "address": address,
          "addressHex": f"0x{address:X}",
          "busTime": bus_time,
          "dat": dat_hex,
          "dlc": len(dat),
        })

    for dec in decode_can_frame(address, dat, src, mono_ms, wall_time):
      sig_key = (src, address, dec["signal"])
      old_val = decoded_last.get(sig_key)
      val = int(dec["value"])
      suspicious = val in SUSPICIOUS_VALUES.get(dec["signal"], set())
      repeat_key = (src, address, dec["signal"], val)
      repeat_due = suspicious and ((mono_ms - last_suspicious_repeat.get(repeat_key, -10**9)) >= int(args.repeat_suspicious_sec * 1000))
      if old_val != val or repeat_due:
        dec["kind"] = "decoded_event"
        dec["oldValue"] = old_val
        dec["oldText"] = None if old_val is None else next((s[4].get(old_val) for s in SIGNALS if s[1] == address and s[0] == dec["signal"]), str(old_val))
        dec["suspicious"] = suspicious
        dec["context"] = ctx
        writer.write_json(dec)
        decoded_events += 1
        decoded_last[sig_key] = val
        if suspicious:
          last_suspicious_repeat[repeat_key] = mono_ms
        old_s = "init" if old_val is None else dec["oldText"]
        marker = "AEB!" if suspicious else "CAN "
        writer.write_txt(
          f"{marker} t={rel_s(mono_ms):.3f}s src={src} {dec['addressHex']} "
          f"{dec['message']}.{dec['signal']} {old_s}->{dec['text']} raw={val} {context_line(ctx)}"
        )

  try:
    if replay_path is not None:
      nonlocal_rel_note = None
      with open_replay(replay_path) as f:
        for line in f:
          if stopped:
            break
          try:
            obj = json.loads(line)
          except Exception:
            continue
          kind = obj.get("kind")
          mono_ms = int(obj.get("monoMs", t0_mono) or t0_mono)
          if rel_base_mono is None:
            rel_base_mono = mono_ms
          wall_time = obj.get("wallTime") or iso_now()
          if kind == "state":
            update_state_from_obj(mono_ms, wall_time, replay_state_dict=obj)
          elif kind == "can":
            process_can(int(obj.get("address", 0)), int(obj.get("src", 0)), obj.get("dat", ""), mono_ms, wall_time, int(obj.get("busTime", 0) or 0))
    else:
      try:
        from cereal import messaging  # pylint: disable=import-error
      except Exception as e:
        raise RuntimeError("Live capture must be run from an openpilot environment where cereal.messaging is available") from e

      services = ["can", "carState", "controlsState", "deviceState", "pandaStates", "managerState"]
      sm = messaging.SubMaster(services, poll="can", ignore_alive=services, ignore_avg_freq=services)
      deadline = time.monotonic() + float(args.duration)
      while not stopped and time.monotonic() < deadline:
        sm.update(1000)
        mono_ms = int(time.monotonic() * 1000)
        wall_time = iso_now()
        update_state_from_obj(
          mono_ms, wall_time,
          car_state=sm["carState"] if sm.updated.get("carState") else None,
          controls_state=sm["controlsState"] if sm.updated.get("controlsState") else None,
          device_state=sm["deviceState"] if sm.updated.get("deviceState") else None,
          panda_states=sm["pandaStates"] if sm.updated.get("pandaStates") else None,
          manager_state=sm["managerState"] if sm.updated.get("managerState") else None,
        )
        if sm.updated.get("can"):
          for c in sm["can"]:
            try:
              process_can(int(c.address), int(c.src), bytes(c.dat).hex(), mono_ms, wall_time, int(c.busTime))
            except Exception as e:
              writer.write_json({"kind": "decode_error", "monoMs": mono_ms, "wallTime": wall_time, "error": repr(e)})

  finally:
    top = counts_by_addr_src.most_common(40)
    writer.write_txt("")
    writer.write_txt("Top CAN addresses by count:")
    for (src, addr), count in top:
      writer.write_txt(f"  src={src} addr=0x{addr:X} count={count} changes={changes_by_addr_src[(src, addr)]} last={last_payload_by_addr_src.get((src, addr), '')}")
    final = {
      "kind": "final_summary",
      "monoMs": int(time.monotonic() * 1000),
      "wallTime": iso_now(),
      "jsonlPath": str(writer.jsonl_path),
      "txtPath": str(writer.txt_path),
      "rawCanLogged": raw_can_logged if args.raw_can else 0,
      "decodedEvents": decoded_events,
      "totalCanFramesSeen": sum(counts_by_addr_src.values()),
      "uniqueAddrSrc": len(counts_by_addr_src),
      "rawCanEnabled": bool(args.raw_can),
    }
    writer.write_json(final)
    writer.write_txt("")
    writer.write_txt(f"decoded_events: {decoded_events}")
    writer.write_txt(f"total_can_frames_seen: {sum(counts_by_addr_src.values())}")
    writer.write_txt(f"raw_can_logged: {raw_can_logged if args.raw_can else 0}")
    writer.write_txt(f"jsonl_size_bytes: {writer.jsonl_path.stat().st_size if writer.jsonl_path.exists() else 0}")
    writer.close()

  return writer.jsonl_path, writer.txt_path


def parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description="XNOR AEB boot watcher v2")
  p.add_argument("--duration", type=float, default=240.0, help="Live capture duration in seconds")
  p.add_argument("--output-dir", default="/data/openpilot/aeb_boot_watch", help="Output directory")
  p.add_argument("--state-interval", type=float, default=1.0, help="State snapshot interval in seconds")
  p.add_argument("--repeat-suspicious-sec", type=float, default=2.0, help="Repeat suspicious AEB values this often even if unchanged")
  p.add_argument("--raw-can", action="store_true", help="Log every raw CAN frame. WARNING: produces huge JSONL files.")
  p.add_argument("--max-raw-can", type=int, default=0, help="Optional max raw CAN records when --raw-can is used; 0 means unlimited")
  p.add_argument("--gzip-jsonl", action="store_true", help="Gzip the JSONL output")
  p.add_argument("--quiet", action="store_true", help="Do not print event lines to console")
  p.add_argument("--replay-jsonl", help="Offline re-decode an existing watcher JSONL instead of live capture")
  p.add_argument("--boot-auto", action="store_true", help="Marker used when launched automatically from launch_openpilot.sh")
  return p.parse_args()


def main() -> int:
  args = parse_args()
  replay = Path(args.replay_jsonl) if args.replay_jsonl else None
  if replay is not None and not replay.exists():
    print(f"Replay JSONL not found: {replay}", file=sys.stderr)
    return 2
  process_stream(args, replay)
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
