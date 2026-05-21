#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, signal, sys, time
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

SERVICES = ["carState", "controlsState", "pandaStates", "managerState", "deviceState"]

def now() -> str:
  return datetime.now(timezone.utc).isoformat(timespec="milliseconds")

def ms() -> int:
  return int(time.monotonic() * 1000)

def get(obj: Any, path: str, default: Any = None) -> Any:
  cur = obj
  for p in path.split("."):
    try:
      cur = cur.get(p, default) if isinstance(cur, dict) else getattr(cur, p)
    except Exception:
      return default
    if cur is None:
      return default
  if isinstance(cur, bytes):
    return cur.hex()
  if isinstance(cur, (str, int, float, bool)):
    return cur
  return str(cur)

def pick(obj: Any, fields: list[str]) -> dict[str, Any]:
  return {f: get(obj, f) for f in fields}

@dataclass
class Stat:
  count: int = 0
  changes: int = 0
  first_ms: int = 0
  last_ms: int = 0
  last: str = ""
  unique: set[str] = field(default_factory=set)
  srcs: Counter = field(default_factory=Counter)

  def update(self, dat: str, src: int, t: int) -> bool:
    changed = self.count == 0 or dat != self.last
    if self.count == 0:
      self.first_ms = t
    self.count += 1
    self.last_ms = t
    self.srcs[src] += 1
    self.unique.add(dat)
    if changed:
      self.changes += 1
      self.last = dat
    return changed

class Watch:
  def __init__(self, args: argparse.Namespace) -> None:
    self.args = args
    self.stop = False
    self.start_mono = time.monotonic()
    self.stats: dict[tuple[int, int], Stat] = defaultdict(Stat)
    self.can_count = 0
    self.last_can = None
    self.last_state = 0.0
    self.last_summary = 0.0
    self.addrs = self.parse_addrs(args.addr)
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    self.jsonl_path = out / f"xnor_aeb_boot_watch_{stamp}.jsonl"
    self.txt_path = out / f"xnor_aeb_boot_watch_{stamp}.txt"
    self.j = self.jsonl_path.open("w", encoding="utf-8")
    self.t = self.txt_path.open("w", encoding="utf-8")

  @staticmethod
  def parse_addrs(values: list[str] | None) -> set[int]:
    out = set()
    for value in values or []:
      for item in value.split(","):
        item = item.strip()
        if item:
          out.add(int(item, 16 if item.lower().startswith("0x") else 10))
    return out

  def close(self) -> None:
    self.j.close()
    self.t.close()

  def on_stop(self, *_: Any) -> None:
    self.stop = True

  def emit(self, rec: dict[str, Any]) -> None:
    rec.setdefault("wallTime", now())
    rec.setdefault("monoMs", ms())
    self.j.write(json.dumps(rec, sort_keys=True, separators=(",", ":")) + "\n")
    self.j.flush()

  def txt(self, line: str = "") -> None:
    self.t.write(line + "\n")
    self.t.flush()
    if self.args.stdout:
      print(line, flush=True)

  def log_frame(self, frame: Any) -> None:
    t = ms()
    addr = int(getattr(frame, "address", 0))
    src = int(getattr(frame, "src", 0))
    dat = bytes(getattr(frame, "dat", b"")).hex()
    bus_time = int(getattr(frame, "busTime", 0))
    self.can_count += 1

    mono = time.monotonic()
    if self.last_can is not None and mono - self.last_can >= self.args.can_gap_seconds:
      self.emit({"kind": "can_gap", "gapSeconds": round(mono - self.last_can, 3), "canCount": self.can_count})
    self.last_can = mono

    stat = self.stats[(src, addr)]
    changed = stat.update(dat, src, t)
    if self.addrs and addr not in self.addrs:
      return
    if self.args.log_all or changed:
      self.emit({"kind":"can","src":src,"address":addr,"addressHex":f"0x{addr:X}","busTime":bus_time,
                 "dat":dat,"dlc":len(dat)//2,"changed":changed,"countForAddr":stat.count,"changesForAddr":stat.changes})

  def log_state(self, sm: Any) -> None:
    if time.monotonic() - self.last_state < self.args.state_interval:
      return
    self.last_state = time.monotonic()
    rec: dict[str, Any] = {"kind": "state"}

    if "carState" in sm.updated:
      rec["carState"] = pick(sm["carState"], [
        "vEgo","standstill","gasPressed","brakePressed","steeringPressed","steeringAngleDeg",
        "steeringRateDeg","gearShifter","cruiseState.enabled","cruiseState.available",
        "cruiseState.speed","cruiseState.standstill","cruiseState.nonAdaptive","cruiseState.speedCluster"])
    if "controlsState" in sm.updated:
      rec["controlsState"] = pick(sm["controlsState"], [
        "enabled","active","state","longControlState","forceDecel","alertText1","alertText2",
        "alertType","alertStatus","alertSize","experimentalMode"])
    if "pandaStates" in sm.updated:
      pandas = []
      try:
        for p in sm["pandaStates"]:
          pandas.append(pick(p, ["ignitionLine","ignitionCan","controlsAllowed","safetyModel","safetyParam",
                                 "faultStatus","pandaType","heartbeatLost","powerSaveEnabled"]))
      except Exception:
        pandas = [str(sm["pandaStates"])]
      rec["pandaStates"] = pandas
    if "managerState" in sm.updated:
      procs = []
      try:
        for p in sm["managerState"].processes:
          name = str(getattr(p, "name", ""))
          if any(x in name.lower() for x in ("boardd","control","model","location","map","panda")):
            procs.append({"name":name,"running":get(p,"running"),"shouldBeRunning":get(p,"shouldBeRunning"),
                          "pid":get(p,"pid"),"exitCode":get(p,"exitCode")})
      except Exception:
        pass
      rec["managerState"] = procs
    if "deviceState" in sm.updated:
      rec["deviceState"] = pick(sm["deviceState"], ["started","startedMonoTime","freeSpacePercent","networkType",
                                                     "networkStrength","thermalStatus"])
    self.emit(rec)

  def summary(self, force: bool = False) -> None:
    if not force and time.monotonic() - self.last_summary < self.args.summary_interval:
      return
    self.last_summary = time.monotonic()
    top = sorted(self.stats.items(), key=lambda x: (x[1].changes, x[1].count), reverse=True)[:self.args.summary_top_n]
    self.emit({"kind":"can_summary","totalFrames":self.can_count,"uniqueAddrSrc":len(self.stats),
               "topChanging":[{"src":s,"address":a,"addressHex":f"0x{a:X}","count":st.count,
                               "changes":st.changes,"uniquePayloads":len(st.unique),"lastDat":st.last}
                              for (s,a), st in top]})

  def final(self) -> None:
    by_count = sorted(self.stats.items(), key=lambda x: x[1].count, reverse=True)[:self.args.summary_top_n]
    by_change = sorted(self.stats.items(), key=lambda x: x[1].changes, reverse=True)[:self.args.summary_top_n]
    self.txt("# xnor AEB boot watch")
    self.txt(f"jsonl: {self.jsonl_path}")
    self.txt(f"duration_s: {round(time.monotonic()-self.start_mono, 1)}")
    self.txt(f"total_can_frames: {self.can_count}")
    self.txt(f"unique_src_addr: {len(self.stats)}")
    self.txt("")
    self.txt("Top CAN addresses by count:")
    for (src, addr), st in by_count:
      self.txt(f"  src={src} addr=0x{addr:X} count={st.count} changes={st.changes} unique={len(st.unique)} last={st.last}")
    self.txt("")
    self.txt("Top CAN addresses by payload changes:")
    for (src, addr), st in by_change:
      self.txt(f"  src={src} addr=0x{addr:X} count={st.count} changes={st.changes} unique={len(st.unique)} last={st.last}")
    self.summary(force=True)
    self.emit({"kind":"final_summary","jsonlPath":str(self.jsonl_path),"txtPath":str(self.txt_path),
               "totalCanFrames":self.can_count,"uniqueAddrSrc":len(self.stats)})

  def run(self) -> int:
    signal.signal(signal.SIGINT, self.on_stop)
    signal.signal(signal.SIGTERM, self.on_stop)
    self.emit({"kind":"meta","started":now(),"argv":sys.argv,"duration":self.args.duration,
               "note":"Boot capture for transient HUD warnings such as AEB unavailable."})
    self.txt(f"Writing {self.jsonl_path}")
    self.txt(f"Writing {self.txt_path}")
    try:
      import cereal.messaging as messaging
    except Exception as exc:
      self.emit({"kind":"error","where":"import","error":repr(exc)})
      self.txt(f"failed to import cereal.messaging: {exc}")
      return 2

    can_sock = messaging.sub_sock("can", timeout=100)
    sm = messaging.SubMaster(SERVICES, ignore_alive=SERVICES, ignore_avg_freq=SERVICES)
    end = self.start_mono + self.args.duration
    while not self.stop and time.monotonic() < end:
      try:
        sm.update(0)
        self.log_state(sm)
        for msg in messaging.drain_sock(can_sock, wait_for_one=False):
          if msg.which() == "can":
            for frame in msg.can:
              self.log_frame(frame)
        self.summary()
        time.sleep(self.args.sleep)
      except Exception as exc:
        self.emit({"kind":"error","where":"loop","error":repr(exc)})
        time.sleep(0.25)
    self.final()
    return 0

def parse_args() -> argparse.Namespace:
  p = argparse.ArgumentParser(description="Boot CAN/state watcher for transient Tesla AEB unavailable warnings.")
  p.add_argument("--duration", type=float, default=180)
  p.add_argument("--output-dir", default="/data/openpilot/aeb_boot_watch")
  p.add_argument("--addr", action="append")
  p.add_argument("--log-all", action="store_true")
  p.add_argument("--state-interval", type=float, default=0.2)
  p.add_argument("--summary-interval", type=float, default=2.0)
  p.add_argument("--summary-top-n", type=int, default=40)
  p.add_argument("--can-gap-seconds", type=float, default=0.5)
  p.add_argument("--sleep", type=float, default=0.01)
  p.add_argument("--stdout", action="store_true")
  return p.parse_args()

def main() -> int:
  w = Watch(parse_args())
  try:
    return w.run()
  finally:
    w.close()

if __name__ == "__main__":
  raise SystemExit(main())
