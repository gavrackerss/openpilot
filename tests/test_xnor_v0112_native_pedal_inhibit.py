#!/usr/bin/env python3
"""Verify the actual candidate's brake/regen inhibition using detached ASTs.

No runtime imports, hardware, CAN or live CarControl. A gate may only deny
native eligibility; no case may create eligibility the native gate denied.
"""
import ast
import itertools
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / "openpilot/selfdrive/controls/controlsd.py"


def candidate_block():
  cls = next(n for n in ast.parse(CONTROL.read_text()).body
             if isinstance(n, ast.ClassDef) and n.name == "Controls")
  body = next(n.body for n in cls.body if isinstance(n, ast.FunctionDef)
              and n.name == "state_control")
  start = next(i for i, n in enumerate(body)
               if isinstance(n, ast.Assign) and any(isinstance(t, ast.Attribute)
               and t.attr == "longActive" for t in n.targets))
  gate, inhibit = body[start:start + 2]
  assert isinstance(inhibit, ast.If)
  return compile(ast.Module(body=[gate], type_ignores=[]), "native", "exec"), compile(
      ast.Module(body=[gate, inhibit], type_ignores=[]), "candidate", "exec")


class TestPedalInhibit(unittest.TestCase):
  def test_never_widens_native_eligibility(self):
    native, candidate = candidate_block()
    for brand, pcm, enabled, available, override, brake, regen in itertools.product(
        ("tesla", "other"), *[(False, True)] * 6):
      owner = NS(CP=NS(brand=brand, pcmCruise=pcm, openpilotLongitudinalControl=available),
                 sm={"onroadEvents": [NS(overrideLongitudinal=override)]},
                 _xnor_hybrid_pedal_inhibited=False)
      cs = NS(brakePressed=brake, regenBraking=regen)
      def evaluate(code):
        cc = NS(enabled=enabled)
        exec(code, dict(self=owner, CS=cs, CC=cc))
        return bool(cc.longActive)
      before, after = evaluate(native), evaluate(candidate)
      expected = before and not (brand == "tesla" and pcm and (brake or regen))
      self.assertEqual(after, expected)
      self.assertFalse(after and not before)

  def test_latch_persists_until_native_disengagement(self):
    native, candidate = candidate_block()
    owner = NS(CP=NS(brand="tesla", pcmCruise=True, openpilotLongitudinalControl=True),
               sm={"onroadEvents": []}, _xnor_hybrid_pedal_inhibited=False)
    # Release and physical-input transitions cannot clear a held inhibition
    # while native engagement remains active.
    for enabled, brake, regen, expected, latched in (
        (True, False, False, True, False),
        (True, True, False, False, True),
        (True, False, True, False, True),
        (True, False, False, False, True),
        (False, False, False, False, False),
        (True, False, False, True, False),
    ):
      cc = NS(enabled=enabled)
      cs = NS(brakePressed=brake, regenBraking=regen)
      exec(candidate, dict(self=owner, CS=cs, CC=cc))
      self.assertEqual(bool(cc.longActive), expected)
      self.assertEqual(owner._xnor_hybrid_pedal_inhibited, latched)

  def test_regen_alone_does_not_latch(self):
    _, candidate = candidate_block()
    owner = NS(CP=NS(brand="tesla", pcmCruise=True, openpilotLongitudinalControl=True),
               sm={"onroadEvents": []}, _xnor_hybrid_pedal_inhibited=False)
    for regen in (True, False):
      cc = NS(enabled=True)
      exec(candidate, dict(self=owner, CS=NS(brakePressed=False, regenBraking=regen), CC=cc))
      self.assertEqual(bool(cc.longActive), not regen)
      self.assertFalse(owner._xnor_hybrid_pedal_inhibited)

  def test_native_pedal_disengagement_is_unchanged(self):
    import subprocess
    path = "openpilot/selfdrive/selfdrived/selfdrived.py"
    original = subprocess.check_output(["git", "show",
        "19696747b72ad48ace8c0ecf9a3a000d1e52dc50:" + path], cwd=ROOT)
    self.assertEqual((ROOT / path).read_bytes(), original)


if __name__ == "__main__":
  unittest.main(verbosity=2)
