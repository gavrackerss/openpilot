#!/usr/bin/env python3
"""Zero-dependency structural checks for staged comma v0.11.2 migration.

This is intentionally a Python unittest rather than importing modeld, the
Acados solver or Cereal. It can run on a workstation without comma hardware:
  python3 tests/test_xnor_v0112_port_contract.py -v

WARNING: structural parity != verified native solver binary/safe driving.
"""
import ast
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
LEGACY_MPC = ROOT/"selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"
STAGED_MPC = ROOT/"selfdrive/controls/lib/v0112_migration/long_mpc.py"
STAGED_PLANNER = ROOT/"selfdrive/controls/lib/v0112_migration/longitudinal_planner.py"
ACTIVE_PLANNER = ROOT/"selfdrive/controls/lib/longitudinal_planner.py"
PLANNER_DAEMON = ROOT/"selfdrive/controls/plannerd.py"
STAGED_CRUISE = ROOT/"selfdrive/controls/lib/xnor_v0112_candidates.py"


def parse(path):
  return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def function(module, name):
  ret = [n for n in module.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
         and n.name == name]
  if len(ret) != 1:
    raise AssertionError(f"{name}: expected 1 definition, got {len(ret)}")
  return ret[0]


class PortContractTest(unittest.TestCase):
  def test_native_ocp_symbolic_problem_matches_original(self):
    # Both files intentionally generate the same (uncompiled) ocp symbolic
    # problem. The new lead-only policy changes obstacle inputs at runtime.
    # A passing AST check is NOT proof compiled solver binaries match.
    old, new = parse(LEGACY_MPC), parse(STAGED_MPC)
    for name in ("gen_long_model", "gen_long_ocp"):
      with self.subTest(name=name):
        self.assertEqual(
          ast.dump(function(old,name), include_attributes=False),
          ast.dump(function(new,name), include_attributes=False),
        )

  def test_staged_mpc_is_lead_only(self):
    text = STAGED_MPC.read_text()
    self.assertIn("x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])", text)
    self.assertNotIn("cruise_obstacle", text)
    self.assertIn("get_t_follow_from_carstate(carstate, personality)", text)

  def test_staged_planner_imports_staged_mpc_and_cruise(self):
    src = STAGED_PLANNER.read_text()
    self.assertIn("v0112_migration.long_mpc import LongitudinalMpc", src)
    self.assertIn("get_cruise_accel_0112(", src)
    self.assertIn("self.accel_boost.update", src)
    self.assertIn("radar_lead_present(", src)

  def test_live_plannerd_does_not_import_staged_code(self):
    src = PLANNER_DAEMON.read_text()
    self.assertIn("from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner", src)
    self.assertNotIn("v0112_migration", src)
    self.assertNotIn("v0112_migration", ACTIVE_PLANNER.read_text())

  def test_no_bump_to_0112_until_full_migration(self):
    self.assertIn('COMMA_VERSION "0.11.1"', (ROOT/"common/version.h").read_text())


if __name__ == "__main__":
  unittest.main(verbosity=2)
