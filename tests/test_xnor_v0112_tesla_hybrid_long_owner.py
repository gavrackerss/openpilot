#!/usr/bin/env python3
"""Pure offline tests: staged Tesla LONG latch, no live CarControl."""
import ast
from dataclasses import replace
import importlib.util
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
FILE = ROOT / "migration/tesla_v0112/hybrid_long_owner.py"
SPEC = importlib.util.spec_from_file_location("staged_tesla_long_owner", FILE)
m = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = m
SPEC.loader.exec_module(m)


def sample(**kwargs):
  initial = m.HybridLongInputs(
    now_ms=100, is_tesla=True, pcm_cruise=True, openpilot_long_available=True,
    enabled=True, brake_pressed=False, regen_braking=False, gas_pressed=False,
    acc_faulted=False, physical_main_res_pressed=False,
    auto_resume_acc_enabled=True, experimental_mode=True,
    override_longitudinal=False,
  )
  return replace(initial, **kwargs)


class TeslaHybridLongOwnerContract(unittest.TestCase):
  def test_brake_latches_and_prevents_long_but_preserves_no_lateral_logic(self):
    start = m.HybridLongState()
    active = m.step_hybrid_long_owner(start, sample())
    self.assertTrue(active.would_allow_long)
    pressed = m.step_hybrid_long_owner(active.state, sample(brake_pressed=True))
    self.assertTrue(pressed.state.cancelled_by_brake)
    self.assertFalse(pressed.would_allow_long)
    self.assertEqual(pressed.transition, "brake_cancel")
    self.assertFalse(pressed.ecu_acceptance_verified)
    self.assertFalse(pressed.vehicle_authority_qualified)

  def test_auto_resume_requires_full_1000ms_release_and_experimental(self):
    latched = m.step_hybrid_long_owner(m.HybridLongState(), sample(brake_pressed=True))
    early = m.step_hybrid_long_owner(latched.state, sample(now_ms=1099))
    self.assertFalse(early.would_allow_long)
    self.assertEqual(early.transition, "latched")
    late = m.step_hybrid_long_owner(early.state, sample(now_ms=1100))
    self.assertTrue(late.would_allow_long)
    self.assertEqual(late.transition, "auto_resume")

    for blockers in ({"experimental_mode": False}, {"auto_resume_acc_enabled": False},
                     {"gas_pressed": True}, {"acc_faulted": True}):
      with self.subTest(blockers=blockers):
        denied = m.step_hybrid_long_owner(latched.state, sample(now_ms=2000, **blockers))
        self.assertTrue(denied.state.cancelled_by_brake)
        self.assertFalse(denied.would_allow_long)

  def test_regen_hold_resets_release_clock(self):
    pressed = m.step_hybrid_long_owner(m.HybridLongState(), sample(brake_pressed=True))
    regen = m.step_hybrid_long_owner(pressed.state, sample(now_ms=800, regen_braking=True))
    self.assertFalse(regen.would_allow_long)
    self.assertEqual(regen.state.last_brake_or_regen_ms, 800)
    early = m.step_hybrid_long_owner(regen.state, sample(now_ms=1799))
    self.assertFalse(early.would_allow_long)
    late = m.step_hybrid_long_owner(early.state, sample(now_ms=1800))
    self.assertTrue(late.would_allow_long)

  def test_physical_stalk_manual_rearm_independent_of_experimental(self):
    latched = m.HybridLongState(True, 100)
    done = m.step_hybrid_long_owner(latched, sample(
      now_ms=250, physical_main_res_pressed=True, experimental_mode=False,
      auto_resume_acc_enabled=False,
    ))
    self.assertFalse(done.state.cancelled_by_brake)
    self.assertTrue(done.would_allow_long)
    self.assertEqual(done.transition, "manual_resume")

  def test_no_manual_rearm_during_regen_or_brake(self):
    latched = m.HybridLongState(True, 100)
    for kw in ({"brake_pressed": True}, {"regen_braking": True}):
      with self.subTest(kw=kw):
        r = m.step_hybrid_long_owner(latched, sample(now_ms=2000, physical_main_res_pressed=True, **kw))
        self.assertFalse(r.would_allow_long)
        self.assertTrue(r.state.cancelled_by_brake)

  def test_override_and_disengage(self):
    engaged = m.step_hybrid_long_owner(m.HybridLongState(), sample(override_longitudinal=True))
    self.assertFalse(engaged.would_allow_long)
    disengage = m.step_hybrid_long_owner(m.HybridLongState(True, 100), sample(enabled=False, now_ms=1000))
    self.assertEqual(disengage.state, m.HybridLongState())
    self.assertFalse(disengage.would_allow_long)
    self.assertEqual(disengage.transition, "disengaged")

  def test_non_hybrid_resets_latch_without_gaining_unsafe_rights(self):
    r = m.step_hybrid_long_owner(m.HybridLongState(True, 100),
                                sample(now_ms=2000, is_tesla=False))
    self.assertEqual(r.state, m.HybridLongState())
    # This mirrors the legacy non-Hybrid branch; not an actuator command.
    self.assertTrue(r.would_allow_long)

  def test_never_active_or_can_connected(self):
    s = FILE.read_text()
    imports = set()
    for n in ast.walk(ast.parse(s)):
      if isinstance(n, ast.Import):
        imports.update(a.name for a in n.names)
      if isinstance(n, ast.ImportFrom):
        imports.add(n.module)
    self.assertEqual(imports, {"__future__", "dataclasses"})
    runtime = (ROOT / "openpilot/selfdrive/controls/controlsd.py").read_text()
    self.assertNotIn("migration.tesla_v0112.hybrid_long_owner", runtime)
    self.assertNotIn("step_hybrid_long_owner", runtime)
    with self.assertRaises(ValueError):
      m.step_hybrid_long_owner(m.HybridLongState(), sample(now_ms=-1))


if __name__ == "__main__":
  unittest.main(verbosity=2)
