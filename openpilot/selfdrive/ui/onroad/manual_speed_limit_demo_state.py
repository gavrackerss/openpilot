"""Non-actuating V279 interaction state for the offroad UI demonstration.

No Params, CAN, vehicle state, or control commands. The preview uses the
configured offset solely for display; it is not a Tesla cruise SET request.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

PRESETS = (40, 50, 60, 70)
CONFIRM_SECONDS = 3.0


def preview_target_mph(posted: int, offset: float, relative: bool, metric: bool) -> float | None:
  if posted not in PRESETS or not math.isfinite(offset):
    return None
  target = posted * (1.0 + offset / 100.0) if relative else posted + offset * (0.621371 if metric else 1.0)
  return target if math.isfinite(target) and 0 < target <= 85 else None


@dataclass
class SpeedSignDemo:
  selected: int = 0
  pending: int = 0
  deadline: float = 0.0

  def reset(self):
    self.selected = self.pending = 0
    self.deadline = 0.0

  def tick(self, now: float, allowed: bool):
    if not allowed or not math.isfinite(now):
      self.reset()
    elif now >= self.deadline:
      self.pending = 0
      self.deadline = 0.0

  def tap(self, value: int, now: float, allowed: bool) -> bool:
    self.tick(now, allowed)
    if not allowed or not math.isfinite(now):
      return False
    if value == 0:
      self.reset()
      return True
    if value not in PRESETS:
      return False
    if self.pending == value and now < self.deadline:
      self.selected = value
      self.pending = 0
      self.deadline = 0.0
      return True
    self.pending = value
    self.deadline = now + CONFIRM_SECONDS
    return False
