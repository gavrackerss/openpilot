#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TESLA_DIR="$ROOT/opendbc_repo/opendbc/car/tesla"
BACKUP_ROOT="$ROOT/.xnor_backups"
BACKUP_DIR="$BACKUP_ROOT/v105_tesla_import_restore_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$BACKUP_DIR"
for f in __init__.py carstate.py fingerprints.py interface.py radar_interface.py; do
  if [ -e "$TESLA_DIR/$f" ]; then
    cp -a "$TESLA_DIR/$f" "$BACKUP_DIR/$f"
  fi
done

mkdir -p "$TESLA_DIR"
cp -f "$ROOT/opendbc_repo/opendbc/car/tesla/__init__.py" "$TESLA_DIR/__init__.py"
cp -f "$ROOT/opendbc_repo/opendbc/car/tesla/carstate.py" "$TESLA_DIR/carstate.py"
cp -f "$ROOT/opendbc_repo/opendbc/car/tesla/fingerprints.py" "$TESLA_DIR/fingerprints.py"
cp -f "$ROOT/opendbc_repo/opendbc/car/tesla/interface.py" "$TESLA_DIR/interface.py"
cp -f "$ROOT/opendbc_repo/opendbc/car/tesla/radar_interface.py" "$TESLA_DIR/radar_interface.py"

chmod 0644 "$TESLA_DIR/__init__.py" "$TESLA_DIR/carstate.py" "$TESLA_DIR/fingerprints.py" "$TESLA_DIR/interface.py" "$TESLA_DIR/radar_interface.py"
find "$TESLA_DIR" -type d -name "__pycache__" -prune -exec rm -rf {} +

if [ ! -e "$ROOT/opendbc" ]; then
  ln -s opendbc_repo/opendbc "$ROOT/opendbc"
fi

echo "v105 restored Tesla import files only."
echo "Backup: $BACKUP_DIR"
echo "Run: bash scripts/xnor_v105_verify_manager_import.sh"
