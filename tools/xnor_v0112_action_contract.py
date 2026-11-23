#!/usr/bin/env python3
"""Read-only comma 0.11.2 vs Pixel LS2 action-output source audit.

No model inference and no car/USB access. This flags outstanding semantic
validation; numerical widths alone do not establish compatible model actions.
Run: python3 tools/xnor_v0112_action_contract.py
"""
from __future__ import annotations

import ast
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]
PIXEL = ROOT / "openpilot/jetlink_authority.py"
CONSTANTS = ROOT / "openpilot/selfdrive/modeld/constants.py"
MODELD = ROOT / "openpilot/selfdrive/modeld/modeld.py"
BRIDGE = ROOT / "openpilot/jetlink_v0112_native.py"


def native_action_width() -> int:
  module = ast.parse(CONSTANTS.read_text())
  for node in module.body:
    if isinstance(node, ast.ClassDef) and node.name == "ModelConstants":
      for child in node.body:
        if isinstance(child, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == "ACTION_WIDTH" for t in child.targets):
          return int(ast.literal_eval(child.value))
  raise RuntimeError("Native ModelConstants.ACTION_WIDTH was not found")


def pixel_action_span() -> tuple[int, int]:
  # Validated Pixel/LS2 engine spec, not the filtered legacy OUTPUT_SLICES
  # (the latter deliberately omitted action when deriving accel from plan).
  m = re.search(
    r"tuple\(slices\.get\('action', \(\)\)\)\s*!=\s*\((\d+),\s*(\d+)\)",
    PIXEL.read_text(),
  )
  if not m:
    raise RuntimeError("Pixel action slice contract was not found")
  start, stop = map(int, m.groups())
  if start < 0 or stop <= start:
    raise RuntimeError("Pixel action slice is invalid")
  return start, stop


def report() -> dict:
  start, stop = pixel_action_span()
  native = native_action_width()
  modeld = MODELD.read_text()
  bridge = BRIDGE.read_text()
  if "model_output['action'][0,1]" not in modeld:
    raise RuntimeError("0.11.2 direct acceleration extraction has changed")
  if "in_control=lambda: True" not in bridge:
    raise RuntimeError("The parked-only model-swap guard has been removed")
  if "require_offroad_probe()" not in bridge:
    raise RuntimeError("The offroad gate has been removed")

  return {
    "base": "comma v0.11.2 source (not hardware validated)",
    "pixel_raw_action_offset": [start, stop],
    "pixel_raw_action_float_count": stop - start,
    "native_model_action_width": native,
    "native_uses_raw_action_when_present": True,
    "raw_head_semantics_equivalent": "not established",
    "pixel_authority_qualified": False,
    "vehicle_control_connected": False,
    "required_next_test": "Pixel/local synchronized frame, recurrent and action-head numerical comparison",
  }


if __name__ == "__main__":
  print(json.dumps(report(), indent=2, sort_keys=True))
