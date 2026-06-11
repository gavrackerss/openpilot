#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TESLA_DIR="$ROOT/opendbc_repo/opendbc/car/tesla"
MODULES_DIR="$ROOT/openpilot/selfdrive/car/modules"
BACKUP_ROOT="$ROOT/.xnor_backups"
BACKUP_DIR="$BACKUP_ROOT/v104_tesla_restore_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$BACKUP_ROOT"
if [ -d "$TESLA_DIR" ]; then
  mkdir -p "$BACKUP_DIR"
  cp -a "$TESLA_DIR" "$BACKUP_DIR/tesla"
fi

# v103 added these upstream Tesla interface files; this repo originally had only
# carcontroller.py, teslacan.py, teslacan_legacy.py, and values.py here.
rm -f "$TESLA_DIR/__init__.py"
rm -f "$TESLA_DIR/carstate.py"
rm -f "$TESLA_DIR/fingerprints.py"
rm -f "$TESLA_DIR/interface.py"
rm -f "$TESLA_DIR/radar_interface.py"
rm -rf "$TESLA_DIR/tests"

find "$TESLA_DIR" -type d -name "__pycache__" -prune -exec rm -rf {} +
find "$MODULES_DIR" -type d -name "__pycache__" -prune -exec rm -rf {} +

chmod 0644 "$TESLA_DIR/carcontroller.py" "$TESLA_DIR/teslacan.py" "$TESLA_DIR/teslacan_legacy.py" "$TESLA_DIR/values.py"
chmod 0644 "$MODULES_DIR/LONG_module.py"

echo "v104 Tesla minimal restore complete."
echo "Backup: $BACKUP_DIR"
echo "Run: bash scripts/xnor_v104_verify_tesla_minimal.sh"
