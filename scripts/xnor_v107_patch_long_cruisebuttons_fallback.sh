#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LONG_PATH="$ROOT/openpilot/selfdrive/car/modules/LONG_module.py"
BACKUP_ROOT="$ROOT/.xnor_backups"
BACKUP_DIR="$BACKUP_ROOT/v107_long_cruisebuttons_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$BACKUP_DIR"

if [ ! -f "$LONG_PATH" ]; then
  echo "BAD missing $LONG_PATH"
  exit 1
fi

cp -a "$LONG_PATH" "$BACKUP_DIR/LONG_module.py"

python - "$LONG_PATH" <<'PY'
from __future__ import annotations

import pathlib
import sys

path = pathlib.Path(sys.argv[1])
text = path.read_text()

old = "from opendbc.car.tesla.values import CruiseButtons"
new = """try:
  from opendbc.car.tesla.values import CruiseButtons
except ImportError:
  from enum import IntEnum

  class CruiseButtons(IntEnum):
    IDLE = 0
    CANCEL = 1
"""

if new in text:
  print("OK LONG_module.py already has CruiseButtons fallback")
elif old in text:
  path.write_text(text.replace(old, new, 1))
  print("OK patched LONG_module.py CruiseButtons fallback")
else:
  print("BAD could not find CruiseButtons import line in LONG_module.py")
  sys.exit(1)
PY

find "$(dirname "$LONG_PATH")" -type d -name "__pycache__" -prune -exec rm -rf {} +

echo "Backup: $BACKUP_DIR/LONG_module.py"
echo "Run: bash scripts/xnor_v107_audit_state.sh"
