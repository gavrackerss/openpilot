#!/usr/bin/env python3
"""Compare installed XNOR planner with staged comma 0.11.2 planner on rlogs.

Use offroad or on a workstation with XNOR native Acados solver and Cereal:
  cd /data/openpilot
  python3 tools/xnor_v0112_compare_planners.py /path/to/rlogls2.zip \
    --out /data/xnor_v0112_planner_replay

This reconstructs two planner instances against identical RECORDED events.
It is open-loop process-level analysis; NOT a vehicle dynamics simulation,
not a stock process_replay replacement, and not a safety qualification.
It never publishes to Cereal, runs modeld, communicates with Pixel, or
issues vehicle commands. It uses the installed native MPC binary only in
the local Python process. Run on the comma only when PARKED and OFFROAD.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import shutil
import tempfile
import zipfile

from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner as OldPlanner
from openpilot.selfdrive.controls.lib.v0112_migration.longitudinal_planner import LongitudinalPlanner as V0112Planner
from openpilot.selfdrive.controls.lib.drive_helpers import radar_lead_present


NEEDED = ("carControl", "carState", "controlsState", "liveParameters",
          "radarState", "modelV2", "selfdriveState")
RELEVANT = set(NEEDED) | {"carParams"}


def source(v):
  try:
    return str(v)
  except Exception:
    return "<unprintable>"


def flt(v):
  try:
    x = float(v)
    return round(x, 6) if math.isfinite(x) else None
  except (TypeError, ValueError, OverflowError):
    return None


def zip_log_paths(path, tmpdir):
  """Extract ONLY rlogs into a private temporary directory."""
  if path.is_dir():
    return sorted(p for p in path.rglob("*") if p.is_file() and
                  (p.name == "rlog" or p.name.startswith("rlog.")))
  if path.suffix.lower() != ".zip":
    if not path.exists():
      raise FileNotFoundError(path)
    return [path]
  out = []
  with zipfile.ZipFile(path) as z:
    names = sorted(i for i in z.infolist() if not i.is_dir() and
                   (Path(i.filename).name == "rlog" or
                    Path(i.filename).name.startswith("rlog.")))
    for idx, info in enumerate(names):
      fn = Path(info.filename).name
      suffix = ".zst" if fn.endswith(".zst") else ".bz2" if fn.endswith(".bz2") else ""
      dst = Path(tmpdir) / f"seg_{idx:04d}_rlog{suffix}"
      with z.open(info, "r") as inp, dst.open("wb") as fp:
        shutil.copyfileobj(inp, fp)
      out.append(dst)
  return out


def main():
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument("input", type=Path)
  ap.add_argument("--out", type=Path, default=Path("/data/xnor_v0112_planner_replay"))
  args = ap.parse_args()
  args.out.mkdir(parents=True, exist_ok=True)
  rows = []
  counters = {
    "model_frames": 0,
    "replayed_frames": 0,
    "missing_startup_inputs": 0,
    "big_model_frames": 0,
    "old_vs_new_difference_over_0_25": 0,
    "new_requests_stronger_braking_over_0_25": 0,
    "old_requests_stronger_braking_over_0_25": 0,
    "both_lead_status_true_frames": 0,
  }
  latest = {}
  old_planner = new_planner = None
  t0 = None

  with tempfile.TemporaryDirectory(prefix="xn_v0112_replay_") as work:
    paths = zip_log_paths(args.input, work)
    if not paths:
      raise SystemExit("No rlog files found.")
    for k, path in enumerate(paths):
      print(f"Reading segment {k+1}/{len(paths)}: {path.name}", flush=True)
      # LogReader sorts events within a segment. Replay states persist between
      # adjacent segments; the real drive is not reconstructed closed-loop.
      for msg in LogReader(str(path), sort_by_time=True):
        try:
          kind = msg.which()
        except Exception:
          continue
        if kind not in RELEVANT:
          continue
        timestamp = int(msg.logMonoTime)
        if t0 is None:
          t0 = timestamp
        if kind == "carParams":
          if old_planner is None:
            cp = msg.carParams
            old_planner = OldPlanner(cp)
            new_planner = V0112Planner(cp)
            print(f"Planner instances created, car={cp.brand}", flush=True)
          continue

        latest[kind] = getattr(msg, kind)
        if kind != "modelV2":
          continue

        counters["model_frames"] += 1
        if old_planner is None or any(k not in latest for k in NEEDED):
          counters["missing_startup_inputs"] += 1
          continue

        try:
          old_planner.update(latest)
          new_planner.update(latest)
        except Exception as exc:
          detail = {
            "error": f"{type(exc).__name__}: {exc}",
            "segment": str(path),
            "time_s": round((timestamp - t0)/1e9, 3),
            "model_big": bool(latest["modelV2"].big),
          }
          (args.out / "replay_error.json").write_text(json.dumps(detail, indent=2))
          raise RuntimeError(f"Planner replay stopped. Details: {detail}") from exc

        car = latest["carState"]
        radar = latest["radarState"]
        big = bool(latest["modelV2"].big)
        lead = radar.leadOne
        old_a = flt(old_planner.output_a_target)
        new_a = flt(new_planner.output_a_target)
        if old_a is None or new_a is None:
          raise RuntimeError("Non-finite acceleration in replay")
        delta = new_a - old_a

        row = {
          "time_s": round((timestamp - t0)/1e9, 3),
          "model_big": big,
          "experimental_mode": bool(latest["selfdriveState"].experimentalMode),
          "long_active": bool(latest["carControl"].longActive),
          "brake_pressed": bool(car.brakePressed),
          "v_ego": flt(car.vEgo),
          "v_cruise_kph": flt(car.vCruise),
          "lead_present": radar_lead_present(lead),
          "lead_dRel": flt(lead.dRel),
          "lead_vRel": flt(lead.vRel),
          "lead_model_prob": flt(lead.modelProb),
          "model_a": flt(latest["modelV2"].action.desiredAcceleration),
          "old_a": old_a,
          "new_a": new_a,
          "new_minus_old_a": round(delta, 6),
          "old_source": source(old_planner.mpc.source),
          "new_source": source(new_planner.mpc.source),
          "old_stop": bool(old_planner.output_should_stop),
          "new_stop": bool(new_planner.output_should_stop),
          "new_a_cruise": flt(new_planner.a_cruise),
          "new_accel_boost": flt(new_planner.accel_boost.total_boost),
        }
        rows.append(row)
        counters["replayed_frames"] += 1
        counters["big_model_frames"] += int(big)
        counters["old_vs_new_difference_over_0_25"] += int(abs(delta) >= 0.25)
        counters["new_requests_stronger_braking_over_0_25"] += int(delta <= -0.25)
        counters["old_requests_stronger_braking_over_0_25"] += int(delta >= 0.25)
        counters["both_lead_status_true_frames"] += int(row["lead_present"])

  out = args.out / "old_vs_0112_planner.csv"
  if rows:
    with out.open("w", newline="", encoding="utf-8") as f:
      writer = csv.DictWriter(f, fieldnames=list(rows[0]))
      writer.writeheader()
      writer.writerows(rows)
  summary = {
    "description": "Open-loop planner comparison with recorded messages, not vehicle dynamics",
    "source": str(args.input),
    "counters": counters,
    "limitations": [
      "The same recorded radarState and modelV2 are fed to both planners, not a closed-loop recalculation.",
      "Official 0.11.2 radard filtering is not included in this run; use radard process replay separately.",
      "Both MPC instances use the old fork's generated solver ABI. Rebuild/native parity is unverified.",
      "The new accel boost is only evaluated in this isolated test; it is not enabled on the car.",
      "Model braking strength, camera warp, Pixel recurrent state and actual Tesla control are not validated.",
      "Do not interpret reduced acceleration as a demonstrated safe stop.",
    ],
  }
  (args.out/"planner_summary.json").write_text(json.dumps(summary, indent=2))
  print(json.dumps(summary, indent=2))
  print(f"Saved {out} and {args.out / 'planner_summary.json'}")


if __name__ == "__main__":
  main()
