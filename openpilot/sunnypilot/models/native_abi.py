"""Portable description of native model metadata, for desktop comparison only.

An exact ABI match is necessary but not sufficient for safe model behavior.
No pickle loading, model selection, or vehicle messaging occurs here.
"""
from __future__ import annotations

from collections.abc import Mapping
from typing import Any


def describe_metadata(metadata: Mapping[str, Any]) -> dict:
  """Convert metadata from a locally compiled trusted bundle into plain JSON."""
  def shapes(name):
    return {k: list(v) for k, v in sorted(metadata[name].items())}
  return {
    "input_shapes": shapes("input_shapes"),
    "output_shapes": shapes("output_shapes"),
    "output_slices": {k: [v.start, v.stop, v.step] for k, v in sorted(metadata["output_slices"].items())},
  }


def compare_native_abi(reference: dict, candidate: dict) -> list[str]:
  """Require the same complete input/output ABI as the compiled stock model."""
  if not isinstance(reference, dict) or not isinstance(candidate, dict):
    return ["Invalid reference or candidate ABI"]
  issues = []
  for key in ("input_shapes", "output_shapes", "output_slices"):
    expected, actual = reference.get(key), candidate.get(key)
    if not isinstance(expected, dict) or not expected or not isinstance(actual, dict):
      issues.append(f"{key}: missing ABI section")
      continue
    for name in sorted(expected.keys() - actual.keys()):
      issues.append(f"{key}: missing {name}")
    for name in sorted(actual.keys() - expected.keys()):
      issues.append(f"{key}: unexpected {name}")
    for name in sorted(expected.keys() & actual.keys()):
      if expected[name] != actual[name]:
        issues.append(f"{key}: {name} differs from stock")
  return issues
