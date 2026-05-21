#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SERVICES = ["pandaStates", "carState", "controlsState", "deviceState", "managerState"]

EXPECTED_SAFETY = "teslaLegacy"
BAD_TRANSIENT_SAFETIES = {"elm327", "noOutput", "silent"}


def utc_now() -> str:
  return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def mono_ms() -> int:
  return int(time.monotonic() * 1000)


def get(obj: Any, path: str, default: Any = None) -> Any:
  cur = obj
  for part in path.split("."):
    try:
      cur = cur.get(part, default) if isinstance(cur, dict) else getattr(cur, part)
    except Exception:
      return default
    if cur is None:
      return default
  if isinstance(cur, bytes):
    return cur.hex()
  if isinstance(cur, (str, int, float, bool)):
    return cur
  return str(cur)


def safety_name(value: Any) -> str:
  text = str(value)
  if "." in text:
    text = text.rsplit(".", 1)[-1]
  return text


def bool_any(values: list[Any]) -> bool:
  return any(bool(v) for v in values)


@dataclass
class GuardState:
  started_at: float
  last_transition: float
  last_good: float = 0.0
  bad_since: float = 0.0
  ready_since: float = 0.0
  recover_count: int = 0
  last_recover: float = 0.0
  last_reason: str = "starting"
  last_safety_key: str = ""


class AebBootGuard:
  def __init__(self, args: argparse.Namespace) -> None:
    self.args = args
    self.stop = False
    self.state = GuardState(started_at=time.monotonic(), last_transition=time.monotonic())
    self.out_dir = Path(args.output_dir)
    self.out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    self.jsonl_path = self.out_dir / f"xnor_aeb_boot_guard_{stamp}.jsonl"
    self.txt_path = self.out_dir / f"xnor_aeb_boot_guard_{stamp}.txt"
    self.jsonl = self.jsonl_path.open("w", encoding="utf-8")
    self.txt = self.txt_path.open("w", encoding="utf-8")
    self.params = None

  def close(self) -> None:
    try:
      self.put_param("XnorAebBootGuardActive", "0")
      self.put_param("XnorAebBootGuardReady", "0")
    except Exception:
      pass
    self.jsonl.close()
    self.txt.close()

  def on_signal(self, *_: Any) -> None:
    self.stop = True

  def emit(self, rec: dict[str, Any]) -> None:
    rec.setdefault("wallTime", utc_now())
    rec.setdefault("monoMs", mono_ms())
    self.jsonl.write(json.dumps(rec, sort_keys=True, separators=(",", ":")) + "\n")
    self.jsonl.flush()

  def line(self, text: str) -> None:
    self.txt.write(text + "\n")
    self.txt.flush()
    if self.args.stdout:
      print(text, flush=True)

  def put_param(self, key: str, value: str) -> None:
    if self.args.no_params:
      return
    if self.params is None:
      from openpilot.common.params import Params
      self.params = Params()
    self.params.put_nonblocking(key, value)

  @staticmethod
  def panda_summary(panda_states: Any) -> list[dict[str, Any]]:
    out = []
    try:
      for idx, panda in enumerate(panda_states):
        out.append({
          "idx": idx,
          "safetyModel": safety_name(get(panda, "safetyModel", "-")),
          "safetyParam": get(panda, "safetyParam", 0),
          "controlsAllowed": bool(get(panda, "controlsAllowed", False)),
          "ignitionCan": bool(get(panda, "ignitionCan", False)),
          "ignitionLine": bool(get(panda, "ignitionLine", False)),
          "faultStatus": safety_name(get(panda, "faultStatus", "-")),
          "heartbeatLost": bool(get(panda, "heartbeatLost", False)),
        })
    except Exception as exc:
      out.append({"error": repr(exc), "raw": str(panda_states)})
    return out

  @staticmethod
  def manager_processes(manager_state: Any) -> list[dict[str, Any]]:
    procs = []
    try:
      for proc in manager_state.processes:
        name = str(get(proc, "name", ""))
        if any(token in name.lower() for token in ("boardd", "pandad", "controlsd", "card")):
          procs.append({
            "name": name,
            "running": bool(get(proc, "running", False)),
            "shouldBeRunning": bool(get(proc, "shouldBeRunning", False)),
            "pid": get(proc, "pid", 0),
            "exitCode": get(proc, "exitCode", 0),
          })
    except Exception:
      pass
    return procs

  def recover_boardd(self, reason: str, snapshot: dict[str, Any]) -> bool:
    now = time.monotonic()
    if self.args.recover_after <= 0:
      return False
    if self.state.recover_count >= self.args.max_recovers:
      return False
    if now - self.state.last_recover < self.args.recover_cooldown:
      return False

    car_state = snapshot.get("carState", {})
    controls_state = snapshot.get("controlsState", {})
    v_ego = float(car_state.get("vEgo") or 0.0)
    gas = bool(car_state.get("gasPressed"))
    brake = bool(car_state.get("brakePressed"))
    controls_enabled = bool(controls_state.get("enabled"))
    controls_active = bool(controls_state.get("active"))

    safe_to_recover = (
      v_ego <= self.args.max_recover_speed and
      not gas and
      not brake and
      not controls_enabled and
      not controls_active
    )

    if not safe_to_recover:
      self.emit({"kind": "recover_skipped", "reason": reason, "snapshot": snapshot,
                 "maxRecoverSpeed": self.args.max_recover_speed})
      return False

    if self.args.dry_run:
      self.emit({"kind": "recover_dry_run", "reason": reason, "snapshot": snapshot})
      self.line(f"{utc_now()} recover_dry_run reason={reason}")
      self.state.recover_count += 1
      self.state.last_recover = now
      return True

    # At boot/standstill only, killing boardd lets manager restart it and reapply Tesla safety.
    cmd = ["pkill", "-f", "boardd"]
    try:
      res = subprocess.run(cmd, timeout=3.0, capture_output=True, text=True)
      self.emit({"kind": "recover_boardd", "reason": reason, "cmd": cmd, "returncode": res.returncode,
                 "stdout": res.stdout, "stderr": res.stderr, "snapshot": snapshot})
      self.line(f"{utc_now()} recover_boardd reason={reason} returncode={res.returncode}")
      self.state.recover_count += 1
      self.state.last_recover = now
      return True
    except Exception as exc:
      self.emit({"kind": "recover_error", "reason": reason, "error": repr(exc), "snapshot": snapshot})
      self.line(f"{utc_now()} recover_error reason={reason} error={exc}")
      return False

  def classify(self, snapshot: dict[str, Any]) -> tuple[bool, bool, str]:
    pandas = snapshot.get("pandaStates", [])
    safety_models = [p.get("safetyModel", "-") for p in pandas]
    ignition = bool_any([p.get("ignitionCan") or p.get("ignitionLine") for p in pandas])
    device_started = bool(snapshot.get("deviceState", {}).get("started"))
    cruise_available = bool(snapshot.get("carState", {}).get("cruiseState.available"))
    car_awake = ignition or device_started or cruise_available

    if not pandas:
      return False, False, "no_panda_states"

    all_expected = all(model == self.args.expected_safety for model in safety_models)
    any_bad = any(model in BAD_TRANSIENT_SAFETIES for model in safety_models)
    any_fault = any(str(p.get("faultStatus", "none")) not in ("none", "-", "0") for p in pandas)
    heartbeat_lost = any(bool(p.get("heartbeatLost")) for p in pandas)

    if all_expected and not any_fault and not heartbeat_lost:
      return True, False, "panda_safety_ready"

    if car_awake and any_bad:
      return False, True, "panda_safety_transient"

    if any_fault or heartbeat_lost:
      return False, True, "panda_fault_or_heartbeat"

    return False, False, "waiting_for_expected_safety"

  def update_params(self, ready: bool, blocked: bool, reason: str, snapshot: dict[str, Any]) -> None:
    active = blocked or not ready
    payload = {
      "ready": ready,
      "active": active,
      "blocked": blocked,
      "reason": reason,
      "recoverCount": self.state.recover_count,
      "pandaStates": snapshot.get("pandaStates", []),
      "carState": snapshot.get("carState", {}),
      "controlsState": snapshot.get("controlsState", {}),
      "deviceState": snapshot.get("deviceState", {}),
      "wallTime": utc_now(),
    }
    self.put_param("XnorAebBootGuardActive", "1" if active else "0")
    self.put_param("XnorAebBootGuardReady", "1" if ready else "0")
    self.put_param("XnorAebBootGuardState", json.dumps(payload, sort_keys=True, separators=(",", ":")))

  def run(self) -> int:
    signal.signal(signal.SIGINT, self.on_signal)
    signal.signal(signal.SIGTERM, self.on_signal)

    self.emit({"kind": "meta", "argv": sys.argv, "duration": self.args.duration,
               "expectedSafety": self.args.expected_safety,
               "recoverAfter": self.args.recover_after,
               "dryRun": self.args.dry_run,
               "note": "Panda safety boot guard for transient Tesla AEB unavailable warning."})
    self.line(f"Writing {self.jsonl_path}")
    self.line(f"Writing {self.txt_path}")

    try:
      import cereal.messaging as messaging
    except Exception as exc:
      self.emit({"kind": "error", "where": "import", "error": repr(exc)})
      self.line(f"failed to import cereal.messaging: {exc}")
      return 2

    sm = messaging.SubMaster(SERVICES, ignore_alive=SERVICES, ignore_avg_freq=SERVICES)
    end = time.monotonic() + self.args.duration
    last_emit = 0.0
    last_transition_key = ""
    ready_stable_since = 0.0

    while not self.stop and time.monotonic() < end:
      sm.update(0)
      now_mono = time.monotonic()

      pandas = self.panda_summary(sm["pandaStates"])
      car_state = {
        "vEgo": get(sm["carState"], "vEgo", 0.0),
        "standstill": get(sm["carState"], "standstill", False),
        "gasPressed": get(sm["carState"], "gasPressed", False),
        "brakePressed": get(sm["carState"], "brakePressed", False),
        "gearShifter": safety_name(get(sm["carState"], "gearShifter", "-")),
        "cruiseState.enabled": get(sm["carState"], "cruiseState.enabled", False),
        "cruiseState.available": get(sm["carState"], "cruiseState.available", False),
      }
      controls_state = {
        "enabled": get(sm["controlsState"], "enabled", False),
        "active": get(sm["controlsState"], "active", False),
        "state": safety_name(get(sm["controlsState"], "state", "-")),
        "alertText1": get(sm["controlsState"], "alertText1", ""),
        "alertText2": get(sm["controlsState"], "alertText2", ""),
        "alertType": get(sm["controlsState"], "alertType", ""),
      }
      device_state = {
        "started": get(sm["deviceState"], "started", False),
        "startedMonoTime": get(sm["deviceState"], "startedMonoTime", 0),
      }
      snapshot = {
        "pandaStates": pandas,
        "carState": car_state,
        "controlsState": controls_state,
        "deviceState": device_state,
        "managerState": self.manager_processes(sm["managerState"]),
      }

      ready_raw, blocked, reason = self.classify(snapshot)
      if ready_raw:
        if ready_stable_since <= 0.0:
          ready_stable_since = now_mono
        ready = (now_mono - ready_stable_since) >= self.args.ready_stable_seconds
      else:
        ready_stable_since = 0.0
        ready = False

      safety_key = "|".join(f"{p.get('idx')}:{p.get('safetyModel')}:{p.get('safetyParam')}:{p.get('faultStatus')}:ca={int(bool(p.get('controlsAllowed')))}" for p in pandas)
      transition_key = f"ready={ready}:blocked={blocked}:reason={reason}:safety={safety_key}"
      if transition_key != last_transition_key:
        self.state.last_transition = now_mono
        last_transition_key = transition_key
        self.emit({"kind": "transition", "ready": ready, "readyRaw": ready_raw, "blocked": blocked,
                   "reason": reason, "snapshot": snapshot})
        self.line(f"{utc_now()} ready={int(ready)} blocked={int(blocked)} reason={reason} safety={safety_key}")

      if blocked:
        if self.state.bad_since <= 0.0:
          self.state.bad_since = now_mono
        bad_for = now_mono - self.state.bad_since
        if self.args.recover_after > 0 and bad_for >= self.args.recover_after:
          self.recover_boardd(f"{reason}_for_{bad_for:.1f}s", snapshot)
          self.state.bad_since = now_mono
      else:
        self.state.bad_since = 0.0

      self.update_params(ready=ready, blocked=blocked, reason=reason, snapshot=snapshot)

      if now_mono - last_emit >= self.args.state_interval:
        last_emit = now_mono
        self.emit({"kind": "state", "ready": ready, "readyRaw": ready_raw, "blocked": blocked,
                   "reason": reason, "badForSeconds": round(now_mono - self.state.bad_since, 3) if self.state.bad_since else 0.0,
                   "readyStableForSeconds": round(now_mono - ready_stable_since, 3) if ready_stable_since else 0.0,
                   "snapshot": snapshot})

      time.sleep(self.args.sleep)

    self.put_param("XnorAebBootGuardActive", "0")
    self.emit({"kind": "final", "recoverCount": self.state.recover_count,
               "jsonlPath": str(self.jsonl_path), "txtPath": str(self.txt_path)})
    self.line(f"{utc_now()} finished recoverCount={self.state.recover_count}")
    return 0


def parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description="Boot guard for transient Panda safety state / Tesla AEB unavailable warning.")
  p.add_argument("--duration", type=float, default=300)
  p.add_argument("--output-dir", default="/data/openpilot/aeb_boot_guard")
  p.add_argument("--expected-safety", default=EXPECTED_SAFETY)
  p.add_argument("--ready-stable-seconds", type=float, default=3.0)
  p.add_argument("--recover-after", type=float, default=0.0,
                 help="If >0, pkill boardd after this many seconds of bad safety state, but only at standstill with controls inactive.")
  p.add_argument("--max-recovers", type=int, default=1)
  p.add_argument("--recover-cooldown", type=float, default=30.0)
  p.add_argument("--max-recover-speed", type=float, default=0.5)
  p.add_argument("--dry-run", action="store_true")
  p.add_argument("--no-params", action="store_true")
  p.add_argument("--state-interval", type=float, default=0.25)
  p.add_argument("--sleep", type=float, default=0.05)
  p.add_argument("--stdout", action="store_true")
  return p.parse_args()


def main() -> int:
  guard = AebBootGuard(parse_args())
  try:
    return guard.run()
  finally:
    guard.close()


if __name__ == "__main__":
  raise SystemExit(main())
