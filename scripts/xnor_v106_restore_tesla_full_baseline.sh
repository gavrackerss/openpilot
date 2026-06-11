#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TESLA_DIR="$ROOT/opendbc_repo/opendbc/car/tesla"
SRC_DIR="$ROOT/xnor_v106_payload/opendbc_repo/opendbc/car/tesla"
BACKUP_ROOT="$ROOT/.xnor_backups"
BACKUP_DIR="$BACKUP_ROOT/v106_tesla_full_baseline_$(date +%Y%m%d_%H%M%S)"

mkdir -p "$BACKUP_ROOT"
if [ -d "$TESLA_DIR" ]; then
  mkdir -p "$BACKUP_DIR"
  cp -a "$TESLA_DIR" "$BACKUP_DIR/tesla"
fi

mkdir -p "$TESLA_DIR"
cp -a "$SRC_DIR/." "$TESLA_DIR/"

find "$TESLA_DIR" -type d -name "__pycache__" -prune -exec rm -rf {} +

if [ ! -e "$ROOT/opendbc" ]; then
  ln -s opendbc_repo/opendbc "$ROOT/opendbc"
fi

echo "v106 restored full original Tesla opendbc baseline."
echo "Backup: $BACKUP_DIR"
echo "Run: bash scripts/xnor_v106_audit_state.sh"
