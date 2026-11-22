#!/usr/bin/env python3
"""Regression checks for XNOR's DEC removal and LS2 smoothing.

Run from repo root: python3 tests/test_xnor_dec_removal.py
No vehicle hardware, cereal, tinygrad, or model assets are needed.
"""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
PLANNER = ROOT / 'selfdrive/controls/lib/longitudinal_planner.py'
MODELD = ROOT / 'selfdrive/modeld/modeld.py'


class TestNoDEC(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.planner_src = PLANNER.read_text(encoding='utf-8')
    cls.modeld_src = MODELD.read_text(encoding='utf-8')
    planner_ast = ast.parse(cls.planner_src, filename=str(PLANNER))
    modeld_ast = ast.parse(cls.modeld_src, filename=str(MODELD))
    planner = next(c for c in planner_ast.body if isinstance(c, ast.ClassDef) and c.name == 'LongitudinalPlanner')
    planner_update = next(n for n in planner.body if isinstance(n, ast.FunctionDef) and n.name == 'update')
    candidate = [n for n in planner_update.body if isinstance(n, ast.If)
                 and isinstance(n.test, ast.Name) and n.test.id == 'experimental_mode']
    if len(candidate) != 1:
      raise AssertionError('Expected exactly one Experimental Mode arbitration block')
    cls.arbitration = compile(ast.fix_missing_locations(ast.Module(body=[candidate[0]], type_ignores=[])), str(PLANNER), 'exec')
    cls.modeld_fn = next(n for n in modeld_ast.body if isinstance(n, ast.FunctionDef)
                         and n.name == 'get_action_from_model')
    cls.modeld_ast = modeld_ast

  def arbitrate(self, experimental, model_a, mpc_a, model_stop=False, mpc_stop=False):
    mpc = SimpleNamespace(source='cruise')
    scope = {
      'self': SimpleNamespace(mpc=mpc),
      'LongitudinalPlanSource': SimpleNamespace(e2e='e2e'),
      'experimental_mode': experimental,
      'output_a_target_mpc': mpc_a,
      'output_a_target_e2e': model_a,
      'output_should_stop_mpc': mpc_stop,
      'output_should_stop_e2e': model_stop,
    }
    exec(self.arbitration, scope, scope)
    return scope['output_a_target'], scope['self'].output_should_stop, mpc.source

  def test_no_dec_or_release_assist(self):
    for marker in ('XnorDynamicExperimentalControl', '_xnor_dec', '_xnor_release', 'xnor_disable_dec', 'XNOR_V232_RELEASE'):
      self.assertNotIn(marker, self.planner_src)

  def test_model_braking_wins_when_experimental(self):
    a, stop, source = self.arbitrate(True, model_a=-0.35, mpc_a=0.50)
    self.assertEqual(a, -0.35)
    self.assertEqual(source, 'e2e')
    self.assertFalse(stop)

  def test_more_conservative_mpc_wins(self):
    a, _, source = self.arbitrate(True, model_a=0.4, mpc_a=-0.7)
    self.assertEqual(a, -0.7)
    self.assertEqual(source, 'cruise')

  def test_model_stop_always_considered_in_experimental(self):
    a, stop, _ = self.arbitrate(True, model_a=0.1, mpc_a=0.2, model_stop=True)
    self.assertTrue(stop)
    self.assertEqual(a, 0.1)

  def test_non_experimental_still_uses_mpc(self):
    a, stop, source = self.arbitrate(False, model_a=-1.0, mpc_a=0.25, model_stop=True)
    self.assertEqual(a, 0.25)
    self.assertFalse(stop)
    self.assertEqual(source, 'cruise')

  def test_ls2_longitudinal_smoothing_enabled(self):
    tau = next(n.value.value for n in self.modeld_ast.body
               if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id == 'LONG_SMOOTH_SECONDS' for t in n.targets))
    self.assertGreater(tau, 0)
    remote_action = next(n for n in ast.walk(self.modeld_fn)
                         if isinstance(n, ast.If) and ast.unparse(n.test) == "'action' in model_output")
    subtree = ast.unparse(remote_action)
    self.assertIn('smooth_value(float(remote_action[1]), prev_action.desiredAcceleration, LONG_SMOOTH_SECONDS)', subtree)
    self.assertIn('shouldStop=bool(should_stop)', subtree)


if __name__ == '__main__':
  unittest.main(verbosity=2)
