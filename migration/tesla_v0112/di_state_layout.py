"""Read-only, layout-tagged Tesla legacy DI_state field inspection.

XNOR pinned opendbc 0.11.2 and the installed custom Tesla DBC disagree
on two signals of DI_state (CAN ID 0x368 / 872).

Do NOT automatically choose a layout based on apparent plausible speed:
two different bytes can both produce plausible values. Exact CAN-log replay
and per-vehicle validation are required before changing CarState.

No CAN bus access, controller writes, cruise ownership or speed selection.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

DI_STATE_ADDRESS = 0x368
DI_STATE_LEN = 8


@dataclass(frozen=True)
class DIStateDualLayout:
  """Both interpretations of the same raw eight-byte CAN payload."""
  native_cruise_set_units: float
  native_digital_speed_units: float
  installed_cruise_set_units: float
  installed_digital_speed_units: float
  layout_qualified: bool = False
  source_address: int = DI_STATE_ADDRESS


def inspect_di_state(frame: bytes) -> DIStateDualLayout:
  if not isinstance(frame, bytes) or len(frame) != DI_STATE_LEN:
    raise ValueError("Expected exactly eight bytes from CAN 0x368")
  data = int.from_bytes(frame, "little")
  # XNOR pinned DBC: DI_cruiseSet @ 32|9 (factor .5), DI_digitalSpeed @48|8.
  native_cruise = ((data >> 32) & 0x1FF) * 0.5
  native_digital = float((data >> 48) & 0xFF)
  # Installed custom DBC: DI_digitalSpeed @32|8, DI_cruiseSet @48|8.
  installed_digital = float((data >> 32) & 0xFF)
  installed_cruise = float((data >> 48) & 0xFF)
  result = DIStateDualLayout(native_cruise, native_digital,
                             installed_cruise, installed_digital)
  assert all(math.isfinite(v) for v in (
    result.native_cruise_set_units, result.native_digital_speed_units,
    result.installed_cruise_set_units, result.installed_digital_speed_units))
  return result
