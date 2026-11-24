#!/usr/bin/env python3
"""Compare actual brake/regen event generation, offline only."""
import ast
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
BASE = "c62e6423874f37f0afb27773027fd15464489c1f"
REL = "selfdrive/selfdrived/selfdrived.py"

def extract(source, legacy):
  tree = ast.parse(source)
  if legacy:
    for node in ast.walk(tree):
      for _, children in ast.iter_fields(node):
        if not isinstance(children, list):
          continue
        for i, stmt in enumerate(children):
          if isinstance(stmt, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "keep_lat_on_brake" for t in stmt.targets):
            return compile(ast.fix_missing_locations(ast.Module(body=children[i:i+3], type_ignores=[])), "legacy_events", "exec")
  else:
    for node in ast.walk(tree):
      if isinstance(node, ast.If):
        condition = ast.unparse(node.test)
        if all(s in condition for s in ("CS.gasPressed", "CS.brakePressed", "CS.regenBraking", "self.CS_prev")):
          return compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), "native_events", "exec")
  raise AssertionError("Brake event source not found")

def evaluate(block, brand="tesla", brake=False, regen=False, gas=False,
             previous_brake=False, previous_regen=False, previous_gas=False, standstill=False):
  events = []
  self = NS(CP=NS(brand=brand), CS_prev=NS(brakePressed=previous_brake, regenBraking=previous_regen,
            gasPressed=previous_gas), disengage_on_accelerator=True, events=NS(add=events.append))
  CS = NS(brakePressed=brake, regenBraking=regen, gasPressed=gas, standstill=standstill)
  exec(block, dict(self=self, CS=CS, EventName=NS(pedalPressed="pedalPressed", gasPressedOverride="gasPressedOverride")))
  return events

def main():
  old = subprocess.check_output(["git","-C",str(ROOT),"show",BASE+":"+REL],text=True)
  new = (ROOT/"openpilot"/REL).read_text()
  a,b = extract(old,True),extract(new,False)
  cases = [
    ("no pedal", {}, [], []),
    ("Tesla brake rising edge", {"brake":True}, ["gasPressedOverride"], ["pedalPressed"]),
    ("Tesla regen rising edge", {"regen":True}, ["gasPressedOverride"], ["pedalPressed"]),
    ("Tesla brake held moving", {"brake":True,"previous_brake":True}, ["gasPressedOverride"], ["pedalPressed"]),
    ("Tesla brake held stationary", {"brake":True,"previous_brake":True,"standstill":True}, ["gasPressedOverride"], []),
    ("Tesla gas rising edge", {"gas":True}, ["pedalPressed"], ["pedalPressed"]),
    ("non-Tesla brake rising edge", {"brand":"other","brake":True}, ["pedalPressed"], ["pedalPressed"]),
    ("pedal released", {"previous_brake":True}, [], []),
  ]
  rows=[]
  for name, args, old_expected, new_expected in cases:
    before,after=evaluate(a,**args),evaluate(b,**args)
    assert before==old_expected,(name,before,old_expected)
    assert after==new_expected,(name,after,new_expected)
    rows.append(dict(case=name,baseline_events=before,candidate_events=after))
  report=dict(baseline=BASE,scope="synthetic event generation only; no state transition or CAN execution",
              vehicle_qualified=False, cases=rows)
  print(json.dumps(report,indent=2))
  (ROOT/"tesla-pedal-event-differential.json").write_text(json.dumps(report,indent=2)+"\n")

if __name__=="__main__":
  main()
