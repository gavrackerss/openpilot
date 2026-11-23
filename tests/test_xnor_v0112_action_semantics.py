#!/usr/bin/env python3
"""Unconnected tests of the actual XNOR stock modeld action extraction code.

Extracts the function using AST because importing modeld would initialize
Tinygrad/QCOM runtime. Does not run modeld, Jetlink, or any vehicle process.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

from openpilot.selfdrive.controls.lib.drive_helpers import should_stop as should_stop_from_accel


MODEL_PATH = Path(__file__).resolve().parents[1] / "selfdrive/modeld/modeld.py"


class Action:
  def __init__(self, desiredCurvature=0.0, desiredAcceleration=0.0, shouldStop=False):
    self.desiredCurvature = desiredCurvature
    self.desiredAcceleration = desiredAcceleration
    self.shouldStop = shouldStop


def extracted_action_function():
  module = ast.parse(MODEL_PATH.read_text())
  funcs = [node for node in module.body if isinstance(node, ast.FunctionDef)
           and node.name == "get_action_from_model"]
  if len(funcs) != 1:
    raise RuntimeError("Expected exactly one get_action_from_model")
  node = funcs[0]
  env = {
    "np": np,
    "log": SimpleNamespace(ModelDataV2=SimpleNamespace(Action=Action)),
    "smooth_value": lambda v, prev, tau: (1-math.exp(-0.05/tau))*v+math.exp(-0.05/tau)*prev if tau>0 else v,
    "should_stop_from_accel": should_stop_from_accel,
    "LONG_SMOOTH_SECONDS": 0.3,
    "LAT_SMOOTH_SECONDS": 0.0,
    "MIN_LAT_CONTROL_SPEED": 0.3,
    "ModelConstants": SimpleNamespace(T_IDXS=[0]*33),
    "Plan": SimpleNamespace(VELOCITY=slice(0,3), ACCELERATION=slice(3,6),
                            T_FROM_CURRENT_EULER=slice(6,9), ORIENTATION_RATE=slice(9,12)),
    "get_accel_from_plan": lambda *args, **kwargs: (0.2, True),
    "get_curvature_from_plan": lambda *args, **kwargs: 0.02,
  }
  isolated = ast.Module(body=[node], type_ignores=[])
  exec(compile(ast.fix_missing_locations(isolated), str(MODEL_PATH), "exec"), env)
  return env["get_action_from_model"]


class V0112ActionStopTest(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.action_fn = staticmethod(extracted_action_function())

  def act(self, accel, v_ego, prev=0.0):
    prediction = {"plan": np.zeros((1, 33, 15), dtype=np.float32),
                  "action": np.array([[0.2, accel]], dtype=np.float32)}
    return self.action_fn(prediction, Action(desiredAcceleration=prev), 0.4, 0.4, v_ego)

  def test_big_model_at_speed_does_not_assert_standstill(self):
    out = self.act(-2.0, 15.0)
    self.assertFalse(out.shouldStop)

  def test_big_model_stop_at_low_speed(self):
    out = self.act(-0.3, 0.2)
    self.assertTrue(out.shouldStop)

  def test_big_model_accelerating_at_low_speed_does_not_stop(self):
    out = self.act(0.3, 0.2)
    self.assertFalse(out.shouldStop)

  def test_direct_action_is_smoothed(self):
    out = self.act(-2.0, 15.0)
    expected = -2.0*(1-math.exp(-0.05/0.3))
    self.assertAlmostEqual(out.desiredAcceleration, expected, places=5)

  def test_nonfinite_direct_action_rejected(self):
    with self.assertRaises(ValueError):
      self.act(float("nan"), 15.0)

  def test_local_small_model_retains_trajectory_stop(self):
    # No 'action' head invokes legacy get_accel_from_plan => returns (0.2, True)
    prediction = {"plan": np.zeros((1, 33, 15), dtype=np.float32)}
    result = self.action_fn(prediction, Action(), 0.4, 0.4, 15.0)
    self.assertTrue(result.shouldStop)
    self.assertGreater(result.desiredAcceleration, 0.0)


if __name__ == "__main__":
  unittest.main(verbosity=2)
