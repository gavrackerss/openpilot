#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LONG_PATH="$ROOT/openpilot/selfdrive/car/modules/LONG_module.py"
SRC_PATH="$ROOT/xnor_v106_payload/openpilot/selfdrive/car/modules/LONG_module.py"
BACKUP_ROOT="$ROOT/.xnor_backups"
BACKUP_DIR="$BACKUP_ROOT/v106_long_v99_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$BACKUP_DIR"
if [ -f "$LONG_PATH" ]; then
  cp -a "$LONG_PATH" "$BACKUP_DIR/LONG_module.py"
fi

cp -f "$SRC_PATH" "$LONG_PATH"
find "$(dirname "$LONG_PATH")" -type d -name "__pycache__" -prune -exec rm -rf {} +

echo "v106 restored LONG_module.py to v99."
echo "Backup: $BACKUP_DIR"
echo "Run: bash scripts/xnor_v106_audit_state.sh"
