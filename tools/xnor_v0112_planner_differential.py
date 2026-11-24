#!/usr/bin/env python3
"""Offline differential test of actual 0.11.1/0.11.2 arbitration source.

No runtime imports, messaging, CAN, hardware, Params or control commands.
Inputs are artificial planner candidates, not recorded driving behavior.
The old DEC state machine is evaluated only to characterize migration gaps.
"""
from pathlib import Path
import ast
import json
import subprocess
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
BASE = "c62e6423874f37f0afb27773027fd15464489c1f"
REL = "selfdrive/controls/lib/longitudinal_planner.py"


def method_body(source):
  cls = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef)
             and n.name == "LongitudinalPlanner")
  return next(n.body for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")


def target_name(stmt, name):
  return isinstance(stmt, ast.Assign) and any(
    isinstance(t, ast.Name) and t.id == name for t in stmt.targets)


def compiled_block(source, legacy):
  body = method_body(source)
  start = next(i for i, stmt in enumerate(body) if target_name(stmt, "experimental_mode" if legacy else "candidates"))
  if legacy:
    end = next(i for i in range(start + 1, len(body)) if isinstance(body[i], ast.If)) + 1
  else:
    end = next(i for i in range(start + 1, len(body))
               if isinstance(body[i], ast.Assign) and any(
                 isinstance(t, ast.Attribute) and t.attr == "output_should_stop" for t in body[i].targets)) + 1
  return compile(ast.fix_missing_locations(ast.Module(body=body[start:end], type_ignores=[])),
                 "offline-arbitration", "exec")


def evaluate(block, *, experimental, dec_mode, mpc, cruise, e2e, mpc_stop=False, cruise_stop=False, e2e_stop=False):
  planner = NS(mpc=NS(source="lead"), a_cruise=cruise, fcw=False,
               _xnor_dec=NS(update=lambda *args: dec_mode, enabled=lambda: True))
  ns = dict(self=planner, sm={"selfdriveState": NS(experimentalMode=experimental)},
            output_a_target_mpc=mpc, output_a_target_e2e=e2e,
            output_should_stop_mpc=mpc_stop, output_should_stop_e2e=e2e_stop,
            cruise_should_stop=cruise_stop, LongitudinalPlanSource=NS(cruise="cruise", e2e="e2e"))
  exec(block, ns)
  return {"acceleration_candidate": float(ns["output_a_target"]),
          "stop_requested": bool(planner.output_should_stop), "source": planner.mpc.source}


def main():
  old = subprocess.check_output(["git", "-C", str(ROOT), "show", BASE + ":" + REL], text=True)
  new = (ROOT / "openpilot" / REL).read_text()
  old_block, new_block = compiled_block(old, True), compiled_block(new, False)
  scenarios = [
    ("normal ACC lead deceleration", dict(experimental=False, dec_mode="acc", mpc=-1.2, cruise=0.5, e2e=0.1)),
    ("experimental blended explicit model stop", dict(experimental=True, dec_mode="blended", mpc=0.2, cruise=0.2, e2e=-1.5, e2e_stop=True)),
    ("DEC ACC cruise recovery", dict(experimental=True, dec_mode="acc", mpc=0.8, cruise=0.8, e2e=0.0)),
    ("native separate cruise constraint", dict(experimental=False, dec_mode="acc", mpc=0.8, cruise=-0.8, e2e=0.2)),
    ("blended radar lead stop", dict(experimental=True, dec_mode="blended", mpc=-1.0, cruise=0.3, e2e=0.4, mpc_stop=True)),
  ]
  rows = []
  for name, inputs in scenarios:
    before, after = evaluate(old_block, **inputs), evaluate(new_block, **inputs)
    rows.append({"scenario": name, "inputs": inputs, "baseline_arbitration": before,
                 "candidate_arbitration": after, "same_selected_acceleration": before["acceleration_candidate"] == after["acceleration_candidate"],
                 "same_stop_request": before["stop_requested"] == after["stop_requested"]})
  # These assertions describe expected existing differences; they do not approve
  # replacing the new native cruise constraint with the older implementation.
  assert rows[0]["same_selected_acceleration"]
  assert rows[1]["same_selected_acceleration"] and rows[1]["candidate_arbitration"]["stop_requested"]
  assert not rows[2]["same_selected_acceleration"]
  assert not rows[3]["same_selected_acceleration"]
  assert rows[4]["same_selected_acceleration"] and rows[4]["candidate_arbitration"]["stop_requested"]
  report = {"baseline": BASE, "synthetic_arbitration_only": True,
            "dec_mode_supplied_as_fixture": True, "vehicle_qualified": False, "scenarios": rows}
  print(json.dumps(report, indent=2))
  (ROOT / "tesla-planner-differential.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
  main()
