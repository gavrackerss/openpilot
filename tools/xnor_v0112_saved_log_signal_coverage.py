#!/usr/bin/env python3
"""Read-only coverage of pedal and physical-resume observations in saved logs.

Run against the same source/schema revision that recorded the log, using its
LogReader. Accepts existing local files only, not route IDs or live messaging.
Outputs aggregate coverage only: no GPS, identifiers, raw CAN or actuator data.
This is not a safety qualification and does not request new driving tests.
"""
import argparse
from collections import Counter
import json
from pathlib import Path


def count_carstates(events):
  counts = Counter()
  previous = {}
  for event in events:
    if event.which() != "carState":
      continue
    counts["carstate_frames"] += 1
    cs = event.carState
    if not event.valid or not cs.canValid or cs.canTimeout:
      counts["invalid_carstate_frames"] += 1
      previous = {}
      continue
    counts["valid_carstate_frames"] += 1
    for name in ("brakePressed", "regenBraking", "gasPressed", "stockAeb", "accFaulted"):
      value = bool(getattr(cs, name))
      counts[name + "_true_frames"] += int(value)
      if name in previous:
        counts[name + "_rising_edges"] += int(value and not previous[name])
        counts[name + "_falling_edges"] += int(not value and previous[name])
      previous[name] = value
    for be in cs.buttonEvents:
      if str(be.type) == "resumeCruise":
        counts["resumeCruise_pressed" if be.pressed else "resumeCruise_released"] += 1
  return counts


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("logs", nargs="+", type=Path)
  args = parser.parse_args()
  paths = [p.resolve(strict=True) for p in args.logs]
  if not all(p.is_file() for p in paths):
    parser.error("Provide only existing local rlog or qlog files")
  try:
    from openpilot.tools.lib.logreader import _LogFileReader
  except ImportError:
    from tools.lib.logreader import _LogFileReader
  counts = Counter()
  for path in paths:
    counts.update(count_carstates(_LogFileReader(str(path), only_union_types=True, sort_by_time=True)))
  for name in ("brakePressed", "regenBraking", "gasPressed", "stockAeb", "accFaulted"):
    for suffix in ("true_frames", "rising_edges", "falling_edges"):
      counts.setdefault(name + "_" + suffix, 0)
  for name in ("resumeCruise_pressed", "resumeCruise_released", "carstate_frames",
               "invalid_carstate_frames", "valid_carstate_frames"):
    counts.setdefault(name, 0)
  print(json.dumps({"scope": "saved-log signal coverage only",
                    "files": len(paths), "counts": dict(sorted(counts.items())),
                    "physical_stalk_origin_verified": False,
                    "vehicle_qualified": False}, indent=2))


if __name__ == "__main__":
  main()
