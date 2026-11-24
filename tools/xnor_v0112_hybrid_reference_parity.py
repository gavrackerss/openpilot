#!/usr/bin/env python3
"""Exercise the actual baseline controlsd latch against the offline port.

Synthetic inputs only. No full controls process, hardware, CAN, messages or
actuator commands are used. The isolated AST is executed against plain
Python objects; no live runtime is imported.
"""
import ast
import json
from pathlib import Path
import random
import subprocess
import sys
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from migration.tesla_v0112.hybrid_control_review_port import ReviewFrame, ReviewState, observe_frame
from migration.tesla_v0112.hybrid_long_owner import HybridLongState
from tools.xnor_v0112_pedal_event_differential import extract, evaluate

BASE = "c62e6423874f37f0afb27773027fd15464489c1f"


def baseline(path):
  return subprocess.check_output(["git", "-C", str(ROOT), "show", BASE + ":" + path], text=True)


def load_legacy_latch():
  tree = ast.parse(baseline("selfdrive/controls/controlsd.py"))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Controls")
  body = next(n.body for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "state_control")
  start = next(i for i,n in enumerate(body) if isinstance(n,ast.Assign) and
               any(isinstance(t,ast.Name) and t.id=="hybrid_brake_latch" for t in n.targets))
  end = next(i for i,n in enumerate(body) if isinstance(n,ast.Assign) and
             any(isinstance(t,ast.Attribute) and t.attr=="longActive" for t in n.targets))
  return compile(ast.fix_missing_locations(ast.Module(body=body[start:end+1],type_ignores=[])),
                 "isolated_legacy_latch","exec")


def compare_sequence(block, event_block, frames):
  first = frames[0]
  self = NS(_xnor_hybrid_brake_long_cancelled=False, _xnor_hybrid_brake_last_press_ms=0,
            _xnor_param_refresh_frame=0, _xnor_auto_resume_acc=first.auto_resume_preference,
            _xnor_experimental_mode=first.experimental)
  review = ReviewState()
  transitions = {}
  for f in frames:
    expected_events = evaluate(event_block, brand=f.brand, brake=f.brake, regen=f.regen,
        gas=f.gas, previous_brake=review.previous_brake, previous_regen=review.previous_regen,
        previous_gas=review.previous_gas, standstill=f.standstill)
    got = observe_frame(review, f)
    assert tuple(expected_events) == got.legacy_reference_pedal_events, (f, expected_events, got)
    self.CP = NS(brand=f.brand, pcmCruise=f.pcm_cruise, openpilotLongitudinalControl=f.op_long_available)
    self.params = NS(get_bool=lambda key: {"TinklaAutoResumeACC": f.auto_resume_preference,
                                          "ExperimentalMode": f.experimental}[key])
    self.sm = {"onroadEvents":[NS(overrideLongitudinal=True)] if
               f.external_long_override or "gasPressedOverride" in expected_events else []}
    CC = NS(enabled=f.enabled)
    CS = NS(brakePressed=f.brake, regenBraking=f.regen, gasPressed=f.gas, accFaulted=f.acc_fault,
            buttonEvents=[NS(pressed=True,type="resume")] if f.physical_main_res else [])
    exec(block, dict(self=self, CC=CC, CS=CS, time=NS(monotonic_ns=lambda:f.timestamp_ms*1_000_000),
                     cloudlog=NS(info=lambda *_:None),
                     car=NS(CarState=NS(ButtonEvent=NS(Type=NS(resumeCruise="resume"))))))
    assert bool(CC.longActive)==got.reference_latch.would_allow_long, (f,CC,got)
    assert HybridLongState(self._xnor_hybrid_brake_long_cancelled,self._xnor_hybrid_brake_last_press_ms)==got.state.latch, (f,self,got)
    assert not got.runtime_activated and not got.vehicle_qualified
    transitions[got.reference_latch.transition] = transitions.get(got.reference_latch.transition,0)+1
    review=got.state
  return transitions


def main():
  block=load_legacy_latch()
  event_block=extract(baseline("selfdrive/selfdrived/selfdrived.py"),True)
  rng=random.Random(112)
  sequences=[]
  sequences.append([ReviewFrame(100),ReviewFrame(110,brake=True),
                    ReviewFrame(200,regen=True),ReviewFrame(1199),ReviewFrame(1200),
                    ReviewFrame(1300,brake=True),ReviewFrame(1310,physical_main_res=True)])
  for brand in ("tesla","other"):
    for auto in (False,True):
      for experimental in (False,True):
        for pcm in (False,True):
          for op_long in (False,True):
            now=100
            frames=[]
            for _ in range(250):
              now+=rng.choice((10,20,100,500,1000,1500))
              frames.append(ReviewFrame(now,brand=brand,pcm_cruise=pcm,op_long_available=op_long,
                  auto_resume_preference=auto,experimental=experimental,
                  enabled=rng.random()>.10,brake=rng.random()<.15,regen=rng.random()<.12,
                  gas=rng.random()<.12,acc_fault=rng.random()<.05,standstill=rng.random()<.10,
                  physical_main_res=rng.random()<.08,external_long_override=rng.random()<.10))
            sequences.append(frames)
  transitions={}
  for frames in sequences:
    for name,count in compare_sequence(block,event_block,frames).items():
      transitions[name]=transitions.get(name,0)+count
  assert transitions.get("manual_resume",0)>0 and transitions.get("auto_resume",0)>0
  report={"baseline":BASE,"sequences":len(sequences),"frames":sum(map(len,sequences)),
          "mismatches":0,"transitions":transitions,"scope":"exact legacy AST differential on synthetic fixtures",
          "runtime_activated":False,"vehicle_qualified":False}
  print(json.dumps(report,indent=2))
  (ROOT/"tesla-hybrid-reference-parity.json").write_text(json.dumps(report,indent=2)+"\n")

if __name__=="__main__":
  main()
