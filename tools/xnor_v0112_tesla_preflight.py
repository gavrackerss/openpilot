#!/usr/bin/env python3
"""Read-only Tesla HW1/HW2/HW3 native 0.11.2 port preflight.

Compares the preserved old XNOR Tesla source against the actual pinned
opendbc submodule. Prints objective API/safety gaps; NEVER produces a
working safety patch, modifies CAN, or declares this vehicle qualified.

Usage: python3 tools/xnor_v0112_tesla_preflight.py [--pinned opendbc_repo]
"""
from __future__ import annotations

import argparse
import ast
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LEGACY = ROOT / "migration/tesla_opendbc_legacy/opendbc"
PINNED = ROOT / "opendbc_repo"
HW = ("TESLA_MODEL_S_HW1", "TESLA_MODEL_S_HW2", "TESLA_MODEL_S_HW3",
      "TESLA_MODEL_X_HW1", "TESLA_MODEL_X_HW2")
CUSTOM_SAFETY_FLAGS = {"OP_STALK_ENABLE": 64, "IGNORE_STOCK_AEB": 128}


def _flags(path: Path, name: str) -> dict[str, int]:
  tree = ast.parse(path.read_text())
  for node in tree.body:
    if isinstance(node, ast.ClassDef) and node.name == name:
      return {
        child.targets[0].id: int(ast.literal_eval(child.value))
        for child in node.body if isinstance(child, ast.Assign)
        and len(child.targets) == 1 and isinstance(child.targets[0], ast.Name)
      }
  raise ValueError(f"Missing {name} in {path}")


def _source(path: Path) -> str:
  if not path.is_file():
    raise FileNotFoundError(f"Missing pinned source: {path}; run git submodule update --init opendbc_repo")
  return path.read_text()


def report(pinned: Path = PINNED) -> dict:
  legacy_values = _source(LEGACY / "car/tesla/values.py")
  pinned_values = _source(pinned / "opendbc/car/tesla/values.py")
  legacy_flags = _flags(LEGACY / "car/tesla/values.py", "TeslaSafetyFlags")
  native_flags = _flags(pinned / "opendbc/car/tesla/values.py", "TeslaSafetyFlags")
  legacy_safety = _source(LEGACY / "safety/modes/tesla_legacy.h")
  native_safety = _source(pinned / "opendbc/safety/modes/tesla_legacy.h")
  native_car_interfaces = _source(pinned / "opendbc/car/interfaces.py")
  native_card = _source(ROOT / "openpilot/selfdrive/car/card.py")
  legacy_interface = _source(LEGACY / "car/tesla/interface.py")
  native_tesla_interface = _source(pinned / "opendbc/car/tesla/interface.py")

  model_platforms = {name: (name in legacy_values and name in pinned_values) for name in HW}
  custom_bits = {name: {"legacy": legacy_flags.get(name), "native": native_flags.get(name)}
                 for name in CUSTOM_SAFETY_FLAGS}

  # Confirm Python and safety C agree on each custom bit *before* considering
  # any future migration. Do not infer that the pinned safety code accepts it.
  for name, expected in CUSTOM_SAFETY_FLAGS.items():
    if legacy_flags.get(name) != expected:
      raise ValueError(f"Legacy safety flag changed: {name}")
    if f"TESLA_LEGACY_FLAG_{name}" not in legacy_safety:
      raise ValueError(f"Legacy Panda safety flag not defined: {name}")

  native_hook_removed = (
      "def post_update(" in legacy_interface
      and "def post_update(" not in native_car_interfaces
      and ".post_update(" not in native_card)
  needs_lateral_rebase = (
      "AngleSteeringLimits(" in legacy_values
      and "AngleSteeringLimitsVM(" in pinned_values)
  native_no_carrier = "0x659" not in native_safety
  old_carrier = "0x659" in legacy_safety
  foreign_params = ("from openpilot.common.params import Params" in legacy_interface
                    and "from openpilot.common.params import Params" not in native_tesla_interface)
  missing = [name for name, p in model_platforms.items() if not p]
  missing_custom_bits = [name for name, b in custom_bits.items()
                         if b["native"] != b["legacy"]]
  blockers = []
  if missing:
    blockers.append("platforms_missing_from_pinned_opendbc")
  if missing_custom_bits:
    blockers.append("custom_safety_flags_not_in_native_python_and_C")
  if native_no_carrier and old_carrier:
    blockers.append("native_panda_lacks_legacy_internal_0x659_ownership")
  if native_hook_removed:
    blockers.append("legacy_post_update_not_called_by_native_car_lifecycle")
  if needs_lateral_rebase:
    blockers.append("legacy_angle_rate_policy_requires_native_VM_reconciliation")
  if foreign_params:
    blockers.append("legacy_interface_imports_openpilot_from_opendbc_dependency")
  if _source(LEGACY / "car/tesla/carcontroller.py") != _source(pinned / "opendbc/car/tesla/carcontroller.py"):
    blockers.append("custom_Tesla_CAN_actuator_controller_not_rebased")
  if _source(LEGACY / "car/tesla/carstate.py") != _source(pinned / "opendbc/car/tesla/carstate.py"):
    blockers.append("custom_Tesla_CAN_state_decoder_not_rebased")

  return {
    "pinned_submodule": str(pinned),
    "supported_platforms_present_in_both": model_platforms,
    "custom_safety_bits": custom_bits,
    "legacy_custom_0x659_carrier": old_carrier,
    "pinned_custom_0x659_carrier": not native_no_carrier,
    "legacy_post_update_removed": native_hook_removed,
    "angle_limits_VM_migration_needed": needs_lateral_rebase,
    "opendbc_import_boundary_migration_needed": foreign_params,
    "blockers": blockers,
    "ready_to_merge_or_drive": False,
    "method": "static source inspection only; no safety, vehicle or CAN execution",
  }


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--pinned", type=Path, default=PINNED)
  args = parser.parse_args()
  print(json.dumps(report(args.pinned), indent=2, sort_keys=True))


if __name__ == "__main__":
  main()
