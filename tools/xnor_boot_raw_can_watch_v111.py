#!/usr/bin/env python3
# /data/openpilot/tools/xnor_boot_raw_can_watch_v111.py
"""
Bounded boot CAN/sendcan watcher for Tesla AEB HUD warning diagnosis.

Runs early at boot, captures the first boot window, then exits.
Default capture policy is intentionally bounded:
  - all outgoing sendcan attempts
  - targeted incoming CAN frames for Tesla AEB/HUD/EPAS/ACC addresses
  - panda safety txBlk deltas with recent outgoing sendcan correlation
  - car/controls state snapshots
"""

from __future__ import annotations

import argparse
import gzip
import json
import os
import signal
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path("/data/openpilot")
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))
opendbc_repo = REPO_ROOT / "opendbc_repo"
if opendbc_repo.exists() and str(opendbc_repo) not in sys.path:
  sys.path.insert(0, str(opendbc_repo))

try:
  import cereal.messaging as messaging
except Exception as exc:
  print(f"IMPORT_FAIL cereal.messaging: {exc!r}", file=sys.stderr)
  raise

DEFAULT_TARGETS: dict[int, str] = {
  0x02BF: "DAS_control/AEB/longitudinal",
  0x0488: "DAS_steeringControl",
  0x027D: "APS_eacMonitor",
  0x0659: "XNOR fake/internal DAS carrier",
  0x0045: "STW_ACTN_RQ stalk",
  0x0399: "AutopilotStatus/FCW HUD",
  0x0389: "DAS_status2/longCollision HUD",
  0x0370: "EPAS_sysStatus/EAC",
  0x0219: "AP/EAC/autopark state",
  0x0309: "warning matrix",
  0x0329: "warning matrix",
  0x0349: "warning matrix",
  0x0369: "warning matrix",
  0x0239: "DAS_lanes",
  0x03A9: "DAS_telemetry",
  0x0106: "DI_torque1 PT",
  0x0108: "DI_torque1 chassis",
  0x0116: "DI_torque2 PT",
  0x0118: "DI_torque2 chassis",
  0x0155: "ESP_vehicleSpeed",
  0x020A: "BrakeMessage",
  0x0368: "DI_state/cruise",
}

WARNING_KEYS = {
  "das_aeb_event",
  "fcw",
  "long_collision_warning",
  "pmm_obstacle_severity",
  "pmm_radar_fault",
  "pmm_sys_fault",
  "pmm_camera_fault",
  "activation_failure",
  "side_collision_avoid",
  "side_collision_warning",
  "stock_aeb",
  "steer_temp",
  "steer_perm",
}


@dataclass(frozen=True)
class OutputFiles:
  txt: Path
  jsonl_gz: Path
  summary: Path


def utc_stamp() -> str:
  return datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")


def safe_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except Exception:
    return default


def safe_float(value: Any, default: float = 0.0) -> float:
  try:
    return float(value)
  except Exception:
    return default


def enum_str(value: Any) -> str:
  try:
    return str(value).split(".")[-1]
  except Exception:
    return "-"


def hex_bytes(dat: bytes) -> str:
  return dat.hex().upper()


def parse_addr_list(raw: str | None) -> set[int]:
  out: set[int] = set()
  if not raw:
    return out
  for part in raw.replace(";", ",").split(","):
    item = part.strip()
    if item:
      out.add(int(item, 0))
  return out


def decode_frame(addr: int, dat: bytes) -> dict[str, Any]:
  d: dict[str, Any] = {}

  if addr == 0x2BF and len(dat) >= 8:
    raw_set_speed = int(dat[0]) | ((int(dat[1]) & 0x0F) << 8)
    raw_accel_min = ((int(dat[5]) & 0x0F) << 5) | (int(dat[4]) >> 3)
    raw_accel_max = ((int(dat[6]) & 0x1F) << 4) | (int(dat[5]) >> 4)
    d.update({
      "das_set_speed_kph": round(raw_set_speed * 0.1, 1),
      "das_acc_state": (int(dat[1]) >> 4) & 0x0F,
      "das_aeb_event": int(dat[2]) & 0x03,
      "das_accel_min_raw": raw_accel_min,
      "das_accel_max_raw": raw_accel_max,
      "das_accel_min_ms2": round((raw_accel_min * 0.04) - 15.0, 3),
      "das_accel_max_ms2": round((raw_accel_max * 0.04) - 15.0, 3),
      "das_counter": (int(dat[6]) >> 5) & 0x07,
      "checksum": int(dat[7]),
    })

  elif addr == 0x488 and len(dat) >= 4:
    raw_angle = (((int(dat[0]) & 0x7F) << 8) | int(dat[1])) - 16384
    d.update({
      "steer_angle_req_deg": round(raw_angle * 0.1, 2),
      "steer_haptic": int(dat[0]) >> 7,
      "steer_control_type": (int(dat[2]) >> 6) & 0x03,
      "raw_b2": int(dat[2]),
      "checksum": int(dat[3]),
    })

  elif addr == 0x27D and len(dat) >= 3:
    d.update({
      "aps_eac_allow": int(dat[0]) & 0x03,
      "aps_counter": int(dat[1]) & 0x0F,
      "checksum": int(dat[2]),
    })

  elif addr == 0x659 and len(dat) >= 6:
    b5 = int(dat[5])
    d.update({
      "pedal_enabled_bit": int(bool(b5 & 0x20)),
      "autopilot_disabled_bit": int(bool(b5 & 0x80)),
      "stalk_main_edge_bit": int(bool(b5 & 0x02)),
      "stalk_cancel_edge_bit": int(bool(b5 & 0x01)),
      "byte5": b5,
    })

  elif addr == 0x45 and len(dat) >= 3:
    d.update({
      "stalk_button": int(dat[0]) & 0x3F,
      "dtr_distance_request": int(dat[1]),
      "turn_indicator_stalk": int(dat[2]) & 0x03,
      "raw_b0": int(dat[0]),
      "raw_b1": int(dat[1]),
      "raw_b2": int(dat[2]),
    })

  elif addr == 0x399 and len(dat) >= 5:
    d.update({
      "autopilot_status": int(dat[0]) & 0x0F,
      "fcw": (int(dat[2]) >> 6) & 0x03,
      "side_collision_avoid": (int(dat[3]) >> 6) & 0x03,
      "side_collision_warning": int(dat[4]) & 0x03,
      "raw_b2": int(dat[2]),
      "raw_b3": int(dat[3]),
      "raw_b4": int(dat[4]),
    })

  elif addr == 0x389 and len(dat) >= 8:
    d.update({
      "pmm_obstacle_severity": (int(dat[1]) >> 2) & 0x07,
      "activation_failure": (int(dat[1]) >> 6) & 0x03,
      "pmm_ultrasonics_fault": int(dat[2]) & 0x07,
      "pmm_radar_fault": (int(dat[2]) >> 3) & 0x03,
      "pmm_sys_fault": (int(dat[2]) >> 5) & 0x07,
      "pmm_camera_fault": int(dat[3]) & 0x03,
      "acc_report": (int(dat[3]) >> 2) & 0x1F,
      "long_collision_warning": int(dat[6]) & 0x0F,
      "counter": (int(dat[6]) >> 4) & 0x0F,
      "checksum": int(dat[7]),
    })

  elif addr == 0x370 and len(dat) >= 8:
    angle_raw = (((int(dat[4]) & 0x3F) << 8) | int(dat[5])) - 8192
    d.update({
      "epas_internal_sas_deg": round(angle_raw * 0.1, 2),
      "hands_on_level": int(dat[4]) >> 6,
      "eac_error_code": int(dat[2]) >> 4,
      "eac_status": int(dat[6]) >> 5,
      "counter": int(dat[6]) & 0x0F,
      "checksum": int(dat[7]),
    })

  return d


def is_warning_like(decoded: dict[str, Any]) -> bool:
  for key in WARNING_KEYS:
    if key not in decoded:
      continue
    value = decoded[key]
    if key == "long_collision_warning":
      if value not in (0, 15):
        return True
    elif value not in (0, False, None):
      return True
  return False


def can_item(kind: str, rel_t: float, msg: Any, targets: dict[int, str]) -> dict[str, Any]:
  addr = safe_int(getattr(msg, "address", 0))
  dat = bytes(getattr(msg, "dat", b""))
  decoded = decode_frame(addr, dat)
  item = {
    "type": kind,
    "t": round(rel_t, 6),
    "addr": addr,
    "addr_hex": f"0x{addr:X}",
    "src": safe_int(getattr(msg, "src", -1), -1),
    "len": len(dat),
    "dat": hex_bytes(dat),
    "name": targets.get(addr, ""),
  }
  if decoded:
    item["decoded"] = decoded
    if is_warning_like(decoded):
      item["warning_like"] = True
  return item


def panda_item(idx: int, rel_t: float, ps: Any) -> dict[str, Any]:
  return {
    "type": "pandaState",
    "t": round(rel_t, 6),
    "idx": idx,
    "safety_model": enum_str(getattr(ps, "safetyModel", "-")),
    "controls_allowed": int(bool(getattr(ps, "controlsAllowed", False))),
    "fault_status": enum_str(getattr(ps, "faultStatus", "-")),
    "safety_tx_blocked": safe_int(getattr(ps, "safetyTxBlocked", 0)),
    "safety_rx_invalid": safe_int(getattr(ps, "safetyRxInvalid", 0)),
    "safety_rx_checks_invalid": int(bool(getattr(ps, "safetyRxChecksInvalid", False))),
    "safety_param": safe_int(getattr(ps, "safetyParam", 0)),
    "faults": [enum_str(f) for f in list(getattr(ps, "faults", []))],
  }


def car_state_item(rel_t: float, cs: Any) -> dict[str, Any]:
  cruise = getattr(cs, "cruiseState", None)
  return {
    "type": "carState",
    "t": round(rel_t, 6),
    "v_ego": round(safe_float(getattr(cs, "vEgo", 0.0), 0.0), 3),
    "gear": enum_str(getattr(cs, "gearShifter", "-")),
    "gas": int(bool(getattr(cs, "gasPressed", False))),
    "brake": int(bool(getattr(cs, "brakePressed", False))),
    "cruise_enabled": int(bool(getattr(cruise, "enabled", False))) if cruise is not None else 0,
    "cruise_available": int(bool(getattr(cruise, "available", False))) if cruise is not None else 0,
    "stock_aeb": int(bool(getattr(cs, "stockAeb", False))),
    "steer_temp": int(bool(getattr(cs, "steerFaultTemporary", False))),
    "steer_perm": int(bool(getattr(cs, "steerFaultPermanent", False))),
  }


def controls_item(rel_t: float, ctrl: Any) -> dict[str, Any]:
  out = {
    "type": "controlsState",
    "t": round(rel_t, 6),
    "enabled": int(bool(getattr(ctrl, "enabled", False))),
    "active": int(bool(getattr(ctrl, "active", False))),
  }
  if hasattr(ctrl, "longControlState"):
    out["long_control_state"] = enum_str(getattr(ctrl, "longControlState"))
  if hasattr(ctrl, "experimentalMode"):
    out["experimental_mode"] = int(bool(getattr(ctrl, "experimentalMode")))
  return out


def state_line(rel_t: float, car: dict[str, Any], ctrl: dict[str, Any], pandas: dict[int, dict[str, Any]]) -> str:
  ptxt = ";".join(
    f"p{idx}:{p.get('safety_model')}/allowed={p.get('controls_allowed')}/fault={p.get('fault_status')}/txBlk={p.get('safety_tx_blocked')}"
    for idx, p in sorted(pandas.items())
  ) or "-"
  return (
    f"STATE t={rel_t:.3f} v={car.get('v_ego','-')} gear={car.get('gear','-')} "
    f"gas={car.get('gas','-')} brake={car.get('brake','-')} "
    f"cruiseEn={car.get('cruise_enabled','-')} cruiseAvail={car.get('cruise_available','-')} "
    f"stockAeb={car.get('stock_aeb','-')} steerTmp={car.get('steer_temp','-')} "
    f"ctrlEn={ctrl.get('enabled','-')} ctrlAct={ctrl.get('active','-')} "
    f"long={ctrl.get('long_control_state','-')} {ptxt}"
  )


class BoundedText:
  def __init__(self, path: Path, max_bytes: int, quiet: bool) -> None:
    self.path = path
    self.max_bytes = max_bytes
    self.quiet = quiet
    self.written = 0
    self.dropped = 0
    self.file = path.open("w", encoding="utf-8")

  def write(self, line: str) -> None:
    raw = (line + "\n").encode("utf-8", errors="replace")
    if self.written + len(raw) <= self.max_bytes:
      self.file.write(line + "\n")
      self.file.flush()
      self.written += len(raw)
    else:
      self.dropped += 1
    if not self.quiet:
      print(line, flush=True)

  def close(self) -> None:
    self.file.close()


class BoundedJsonlGzip:
  def __init__(self, path: Path, max_uncompressed_bytes: int) -> None:
    self.path = path
    self.max_uncompressed_bytes = max_uncompressed_bytes
    self.written_uncompressed = 0
    self.records = 0
    self.dropped_records = 0
    self.file = gzip.open(path, "wt", encoding="utf-8", compresslevel=3)

  def write(self, obj: dict[str, Any]) -> None:
    line = json.dumps(obj, sort_keys=True, separators=(",", ":")) + "\n"
    raw_len = len(line.encode("utf-8", errors="replace"))
    if self.written_uncompressed + raw_len <= self.max_uncompressed_bytes:
      self.file.write(line)
      self.records += 1
      self.written_uncompressed += raw_len
      if self.records % 100 == 0:
        self.file.flush()
    else:
      self.dropped_records += 1

  def close(self) -> None:
    self.file.flush()
    self.file.close()


def make_outputs(out_dir: Path, prefix: str) -> OutputFiles:
  out_dir.mkdir(parents=True, exist_ok=True)
  stamp = utc_stamp()
  return OutputFiles(
    txt=out_dir / f"{prefix}_{stamp}.txt",
    jsonl_gz=out_dir / f"{prefix}_{stamp}.jsonl.gz",
    summary=out_dir / f"{prefix}_{stamp}.summary.json",
  )


def prune_old_runs(out_dir: Path, prefix: str, keep_runs: int) -> list[str]:
  removed: list[str] = []
  if keep_runs <= 0 or not out_dir.exists():
    return removed

  groups: dict[str, list[Path]] = {}
  for p in out_dir.glob(f"{prefix}_*"):
    stem = p.name
    for suffix in (".summary.json", ".jsonl.gz", ".txt"):
      if stem.endswith(suffix):
        stamp = stem.removeprefix(prefix + "_").removesuffix(suffix)
        groups.setdefault(stamp, []).append(p)
        break

  stamps = sorted(groups.keys())
  for stamp in stamps[:-keep_runs]:
    for p in groups[stamp]:
      try:
        removed.append(str(p))
        p.unlink()
      except FileNotFoundError:
        pass
  return removed


def recent_sendcan_summary(recent: deque[dict[str, Any]], now_t: float, window_sec: float) -> dict[str, Any]:
  cutoff = now_t - window_sec
  rows = [r for r in recent if safe_float(r.get("t"), 0.0) >= cutoff]
  counts = Counter(f"{r.get('addr_hex')}@{r.get('src')}" for r in rows)
  last: dict[str, Any] = {}
  for r in rows:
    key = f"{r.get('addr_hex')}@{r.get('src')}"
    last[key] = {
      "dat": r.get("dat"),
      "name": r.get("name"),
      "decoded": r.get("decoded", {}),
    }
  return {
    "window_sec": window_sec,
    "count": len(rows),
    "counts": dict(counts),
    "last": last,
  }


def compact_counts(counter: Counter[str], limit: int = 12) -> str:
  return ";".join(f"{k}:{v}" for k, v in counter.most_common(limit)) or "-"


def should_log_can(addr: int, args: argparse.Namespace, targets: dict[int, str]) -> bool:
  if args.raw_can_mode == "all":
    return True
  return addr in targets


def setup_sockets(addr: str, text: BoundedText) -> dict[str, Any]:
  socks: dict[str, Any] = {}
  services = ["can", "sendcan", "pandaStates", "carState", "controlsState", "selfdriveState"]
  for service in services:
    try:
      socks[service] = messaging.sub_sock(service, conflate=False, timeout=0, addr=addr)
    except Exception as exc:
      text.write(f"SOCK_FAIL service={service} err={exc!r}")
  return socks


def run(args: argparse.Namespace) -> int:
  if args.addr != "127.0.0.1":
    os.environ["ZMQ"] = "1"
    try:
      messaging.reset_context()
    except Exception:
      pass

  targets = dict(DEFAULT_TARGETS)
  for addr in parse_addr_list(args.extra_addr):
    targets[addr] = targets.get(addr, "extra target")

  out_dir = Path(args.out_dir)
  removed = prune_old_runs(out_dir, args.prefix, int(args.keep_runs))
  files = make_outputs(out_dir, args.prefix)

  text = BoundedText(files.txt, max_bytes=int(args.max_txt_kb * 1024), quiet=bool(args.quiet))
  jsonl = BoundedJsonlGzip(files.jsonl_gz, max_uncompressed_bytes=int(args.max_jsonl_mb * 1024 * 1024))

  stop = {"value": False}

  def _stop(_signum: int, _frame: Any) -> None:
    stop["value"] = True

  signal.signal(signal.SIGINT, _stop)
  signal.signal(signal.SIGTERM, _stop)

  start = time.monotonic()
  deadline = start + float(args.duration) if args.duration > 0 else None
  last_state = start
  last_summary = start

  counts: Counter[str] = Counter()
  warning_counts: Counter[str] = Counter()
  dropped_summary: Counter[str] = Counter()
  recent_sendcan: deque[dict[str, Any]] = deque(maxlen=int(args.recent_sendcan_max))
  last_txblk: dict[int, int] = {}
  last_pandas: dict[int, dict[str, Any]] = {}
  last_car: dict[str, Any] = {}
  last_ctrl: dict[str, Any] = {}
  txblk_events = 0
  warning_events = 0
  first_fault_temp_t: float | None = None

  text.write("# xnor boot raw CAN watcher v111")
  text.write(f"# started={datetime.now(timezone.utc).isoformat()}")
  text.write(f"# duration={args.duration}s raw_can_mode={args.raw_can_mode} all_sendcan=True")
  text.write(f"# caps=jsonl_uncompressed<{args.max_jsonl_mb}MiB txt<{args.max_txt_kb}KiB keep_runs={args.keep_runs}")
  text.write(f"# txt={files.txt}")
  text.write(f"# jsonl_gz={files.jsonl_gz}")
  text.write(f"# summary={files.summary}")
  if removed:
    text.write(f"# pruned_old_runs={len(removed)}")
  text.write("# target_addrs=" + ",".join(f"0x{k:X}:{v}" for k, v in sorted(targets.items())))

  jsonl.write({
    "type": "header",
    "version": "v111",
    "started": datetime.now(timezone.utc).isoformat(),
    "args": vars(args),
    "targets": {f"0x{k:X}": v for k, v in sorted(targets.items())},
    "pruned": removed,
  })

  socks = setup_sockets(args.addr, text)

  while not stop["value"]:
    now = time.monotonic()
    rel_t = now - start
    if deadline is not None and now >= deadline:
      break

    had_event = False

    for service, sock in list(socks.items()):
      try:
        events = messaging.drain_sock(sock, wait_for_one=False)
      except Exception as exc:
        text.write(f"DRAIN_FAIL service={service} err={exc!r}")
        continue

      if events:
        had_event = True

      for evt in events:
        rel_t = time.monotonic() - start

        if service in ("can", "sendcan"):
          for msg in list(getattr(evt, service, [])):
            addr = safe_int(getattr(msg, "address", 0))
            item = can_item(service, rel_t, msg, targets)

            if service == "sendcan":
              recent_sendcan.append(item)
              counts[f"sendcan:{item['addr_hex']}@{item['src']}"] += 1
              jsonl.write(item)
            elif should_log_can(addr, args, targets):
              counts[f"can:{item['addr_hex']}@{item['src']}"] += 1
              jsonl.write(item)
            else:
              dropped_summary["can_not_target"] += 1
              continue

            if item.get("warning_like"):
              warning_events += 1
              warning_counts[f"{service}:{item['addr_hex']}@{item['src']}"] += 1
              text.write(
                f"{service.upper()}_WARNING t={rel_t:.3f} src={item['src']} addr={item['addr_hex']} "
                f"dat={item['dat']} name={item.get('name','')} decoded={item.get('decoded',{})}"
              )

        elif service == "pandaStates":
          for idx, ps in enumerate(list(getattr(evt, "pandaStates", []))):
            pd = panda_item(idx, rel_t, ps)
            last_pandas[idx] = pd
            txblk = safe_int(pd.get("safety_tx_blocked", 0))
            prev = last_txblk.get(idx)
            last_txblk[idx] = txblk
            delta = txblk - prev if prev is not None else 0

            if pd.get("fault_status") == "faultTemp" and first_fault_temp_t is None:
              first_fault_temp_t = rel_t
              text.write(
                f"FIRST_FAULT_TEMP t={rel_t:.3f} panda={idx} "
                f"model={pd['safety_model']} allowed={pd['controls_allowed']} txBlk={txblk}"
              )

            if delta > 0:
              txblk_events += 1
              recent = recent_sendcan_summary(recent_sendcan, rel_t, float(args.txblk_window_sec))
              event = {
                "type": "txblk_delta",
                "t": round(rel_t, 6),
                "panda": idx,
                "delta": delta,
                "panda_state": pd,
                "recent_sendcan": recent,
                "carState": last_car,
                "controlsState": last_ctrl,
              }
              jsonl.write(event)
              counts[f"txblk:p{idx}"] += delta
              text.write(
                f"TXBLK_DELTA t={rel_t:.3f} panda={idx} delta={delta} "
                f"model={pd['safety_model']} allowed={pd['controls_allowed']} fault={pd['fault_status']} "
                f"recent={compact_counts(Counter(recent['counts']))}"
              )

            if delta > 0 or args.log_all_panda:
              jsonl.write(pd)

        elif service == "carState":
          cs = getattr(evt, "carState", None)
          if cs is not None:
            last_car = car_state_item(rel_t, cs)
            if is_warning_like(last_car):
              warning_events += 1
              warning_counts["carState"] += 1
              text.write(f"CARSTATE_WARNING t={rel_t:.3f} data={last_car}")
              jsonl.write(last_car)

        elif service == "controlsState":
          ctrl = getattr(evt, "controlsState", None)
          if ctrl is not None:
            last_ctrl = controls_item(rel_t, ctrl)

    now = time.monotonic()
    rel_t = now - start

    if now - last_state >= float(args.state_interval):
      last_state = now
      line = state_line(rel_t, last_car, last_ctrl, last_pandas)
      text.write(line)
      jsonl.write({
        "type": "state",
        "t": round(rel_t, 6),
        "carState": last_car,
        "controlsState": last_ctrl,
        "pandaStates": last_pandas,
      })

    if now - last_summary >= float(args.summary_interval):
      last_summary = now
      text.write(
        f"SUMMARY t={rel_t:.1f} txblk_events={txblk_events} warnings={warning_events} "
        f"top={compact_counts(counts, 10)} jsonl_drop={jsonl.dropped_records}"
      )

    if not had_event:
      time.sleep(float(args.idle_sleep))

  summary = {
    "version": "v111",
    "finished": datetime.now(timezone.utc).isoformat(),
    "duration_sec": round(time.monotonic() - start, 3),
    "txt": str(files.txt),
    "jsonl_gz": str(files.jsonl_gz),
    "summary": str(files.summary),
    "caps": {
      "max_jsonl_mb_uncompressed": args.max_jsonl_mb,
      "max_txt_kb": args.max_txt_kb,
      "jsonl_records": jsonl.records,
      "jsonl_uncompressed_bytes": jsonl.written_uncompressed,
      "jsonl_dropped_records": jsonl.dropped_records,
      "txt_bytes": text.written,
      "txt_dropped_lines": text.dropped,
    },
    "first_fault_temp_t": first_fault_temp_t,
    "txblk_events": txblk_events,
    "warning_events": warning_events,
    "counts": dict(counts),
    "warning_counts": dict(warning_counts),
    "dropped_summary": dict(dropped_summary),
    "last_panda_state": last_pandas,
    "last_car_state": last_car,
    "last_controls_state": last_ctrl,
  }

  jsonl.write({"type": "final_summary", **summary})
  files.summary.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
  text.write(f"# finished summary={files.summary} jsonl_dropped={jsonl.dropped_records} txt_dropped={text.dropped}")
  jsonl.close()
  text.close()
  return 0


def build_arg_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
    description="Bounded boot raw CAN/sendcan watcher for XNOR Tesla AEB HUD diagnosis.",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
  )
  parser.add_argument("--duration", type=float, default=150.0)
  parser.add_argument("--out-dir", default="/data/openpilot/xnor_aeb_boot_capture")
  parser.add_argument("--prefix", default="xnor_boot_raw_can_v111")
  parser.add_argument("--addr", default="127.0.0.1")
  parser.add_argument("--raw-can-mode", choices=("targeted", "all"), default="targeted",
                      help="targeted logs only AEB/HUD/EPAS incoming CAN; all logs every incoming CAN frame.")
  parser.add_argument("--extra-addr", default="", help="Comma-separated extra incoming CAN addresses, e.g. 0x123,0x456.")
  parser.add_argument("--max-jsonl-mb", type=float, default=32.0,
                      help="Maximum uncompressed JSONL payload before dropping additional raw records.")
  parser.add_argument("--max-txt-kb", type=float, default=768.0)
  parser.add_argument("--keep-runs", type=int, default=6)
  parser.add_argument("--txblk-window-sec", type=float, default=0.45)
  parser.add_argument("--recent-sendcan-max", type=int, default=10000)
  parser.add_argument("--summary-interval", type=float, default=1.0)
  parser.add_argument("--state-interval", type=float, default=0.5)
  parser.add_argument("--idle-sleep", type=float, default=0.01)
  parser.add_argument("--log-all-panda", action="store_true")
  parser.add_argument("--quiet", action="store_true")
  return parser


def main() -> int:
  return run(build_arg_parser().parse_args())


if __name__ == "__main__":
  raise SystemExit(main())
