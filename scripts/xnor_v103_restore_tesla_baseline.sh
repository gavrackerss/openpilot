#!/usr/bin/env bash
set -euo pipefail

REPO="${1:-/data/openpilot}"
cd "$REPO"

echo "[v103] stopping manager..."
sudo pkill -f "system/manager/manager.py" || true

echo "[v103] backing up current Tesla opendbc files..."
BACKUP="/data/xnor_tesla_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP"
cp -a opendbc_repo/opendbc/car/tesla "$BACKUP/tesla" || true

echo "[v103] clearing Python caches for opendbc/openpilot Tesla modules..."
find "$REPO" -type d -name "__pycache__" \( -path "*/opendbc/*" -o -path "*/openpilot/*" -o -path "*/selfdrive/*" \) -prune -exec rm -rf {} +

echo "[v103] verifying imported Tesla carcontroller..."
python - <<'PY'
import hashlib
import inspect
import opendbc.car.tesla.carcontroller as carcontroller

path = inspect.getsourcefile(carcontroller)
digest = hashlib.sha256(open(path, "rb").read()).hexdigest()
print(f"imported={path}")
print(f"sha256={digest}")
if digest != "92c010676563b37b96104e434ee89bf2d86fbf366ae2cebeaa94f7da48fd0587":
  raise SystemExit("ERROR: imported carcontroller is not the restored Tesla baseline")
PY

echo "[v103] done. Reboot now."
