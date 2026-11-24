"""Read-only Tesla instrument-cluster model snapshot adapter for native 0.11.2.

Extracted from legacy CarInterface.post_update. This module has no imports
from opendbc, cereal, modeld, CAN, panda or car control, and is NOT connected
to the 0.11.2 card/manager lifecycle.

Do not copy legacy post_update wholesale: native CarInterfaceBase lacks that
callback. HSO, Hybrid and ALC need independent contracts and tests before any
runtime integration. This module only preserves modelV2 visual information.
"""
from __future__ import annotations

import math
from typing import Any


def extract_ic_model_data(model_v2: Any, last_good: dict | None = None) -> dict | None:
  """Parse the familiar modelV2 lane probabilities and path, or retain last good.

  The original Tesla code retained the most recent valid sample when there
  was no new model message. Reject malformed/nonfinite data as well; never
  synthesize a lane/path command from missing or corrupt model data.

  Return values are ordinary tuples, never Cap'n Proto builders or buffers.
  """
  try:
    probs = tuple(float(value) for value in model_v2.laneLineProbs)
    x = tuple(float(value) for value in model_v2.position.x)
    y = tuple(float(value) for value in model_v2.position.y)
    if len(probs) != 4 or not (2 <= len(x) == len(y) <= 64):
      return last_good
    if not all(math.isfinite(v) for v in (*probs, *x, *y)):
      return last_good
    if any(p < 0.0 or p > 1.0 for p in probs):
      return last_good
    return {'lane_probs': probs, 'position_x': x, 'position_y': y}
  except (AttributeError, TypeError, ValueError, OverflowError):
    return last_good


def snapshot_from_message(message: Any, last_good: dict | None = None) -> dict | None:
  """Optional modelV2 Cereal message -> immutable Python snapshot, read-only."""
  if message is None:
    return last_good
  try:
    return extract_ic_model_data(message.modelV2, last_good)
  except (AttributeError, TypeError, ValueError):
    return last_good
