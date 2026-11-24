#!/usr/bin/env python3
"""XNOR V275 automatic on-road shadow observation.

This process is opt-in, logs observations only, and does not publish speed
limits, send CAN, or change vehicle settings.
"""
import os
import sys
from pathlib import Path

BASE = Path("/data/V275_Shadow")
LOGS = Path("/data/V275_Shadow_logs")


def main() -> int:
  if not (BASE / "ENABLED").is_file():
    return 0

  try:
    os.nice(15)
  except OSError:
    pass

  sys.path.insert(0, str(BASE))
  os.environ.update({
    "V275_MANAGED": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "OMP_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
  })

  try:
    from shadow import main as shadow_main
    sys.argv = [
      "v275_shadow_manager", "--live", "--mode", "auto",
      "--sample-seconds", "5", "--max-infer-seconds", "12",
      "--max-seconds", "0", "--max-samples", "0",
      "--max-candidates", "5", "--log-file", str(LOGS / "observations.jsonl"),
    ]
    return int(shadow_main() or 0)
  except SystemExit as exc:
    if exc.code in (0, None):
      return 0
    raise
  except Exception:
    try:
      (BASE / "ENABLED").unlink(missing_ok=True)
      (BASE / "AUTO_FAULT.json").write_text('{"reason":"manager_exception; check process logs"}\n')
    except OSError:
      pass
    raise


if __name__ == "__main__":
  raise SystemExit(main())
