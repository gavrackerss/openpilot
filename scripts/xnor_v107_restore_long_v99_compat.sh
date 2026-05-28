#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LONG_PATH="$ROOT/openpilot/selfdrive/car/modules/LONG_module.py"
SRC_PATH="$ROOT/xnor_v107_payload/openpilot/selfdrive/car/modules/LONG_module_v99_compat.py"
BACKUP_ROOT="$ROOT/.xnor_backups"
BACKUP_DIR="$BACKUP_ROOT/v107_restore_long_v99_compat_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$BACKUP_DIR"
if [ -f "$LONG_PATH" ]; then
  cp -a "$LONG_PATH" "$BACKUP_DIR/LONG_module.py"
fi

cp -f "$SRC_PATH" "$LONG_PATH"
find "$(dirname "$LONG_PATH")" -type d -name "__pycache__" -prune -exec rm -rf {} +

echo "OK restored LONG_module.py to v99 + CruiseButtons fallback"
echo "Backup: $BACKUP_DIR/LONG_module.py"
echo "Run: bash scripts/xnor_v107_audit_state.sh"
