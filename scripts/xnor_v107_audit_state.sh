#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

fail=0
warn=0

ok() { echo "OK      $1"; }
bad() { echo "BAD     $1"; fail=1; }
warnf() { echo "WARN    $1"; warn=1; }
info() { echo "INFO    $1"; }

check_file() {
  local rel="$1"
  if [ -f "$ROOT/$rel" ]; then
    ok "$rel exists"
  else
    bad "$rel missing"
  fi
}

echo "== root =="
info "$ROOT"

echo
echo "== required Tesla imports =="
check_file "opendbc_repo/opendbc/car/tesla/interface.py"
check_file "opendbc_repo/opendbc/car/tesla/carstate.py"
check_file "opendbc_repo/opendbc/car/tesla/radar_interface.py"
check_file "opendbc_repo/opendbc/car/tesla/values.py"
check_file "opendbc_repo/opendbc/car/tesla/carcontroller.py"

echo
echo "== LONG CruiseButtons compatibility =="
LONG_PATH="$ROOT/openpilot/selfdrive/car/modules/LONG_module.py"
if [ ! -f "$LONG_PATH" ]; then
  bad "openpilot/selfdrive/car/modules/LONG_module.py missing"
elif grep -q "class CruiseButtons" "$LONG_PATH"; then
  ok "LONG_module.py has local CruiseButtons fallback"
elif grep -q "from opendbc.car.tesla.values import CruiseButtons" "$LONG_PATH"; then
  bad "LONG_module.py still directly imports missing opendbc.car.tesla.values.CruiseButtons"
else
  warnf "LONG_module.py has no recognizable CruiseButtons import/fallback"
fi

echo
echo "== Python imports =="
python - <<'PY'
import importlib

mods = [
  "opendbc.car.tesla.interface",
  "opendbc.car.tesla.carstate",
  "opendbc.car.tesla.radar_interface",
  "opendbc.car.car_helpers",
  "openpilot.selfdrive.car.modules.LONG_module",
]

for mod in mods:
  importlib.import_module(mod)
  print(f"IMPORT OK {mod}")
PY

echo
echo "== Python compile =="
python -m py_compile \
  "$ROOT/opendbc_repo/opendbc/car/tesla/carcontroller.py" \
  "$ROOT/opendbc_repo/opendbc/car/tesla/interface.py" \
  "$ROOT/opendbc_repo/opendbc/car/tesla/carstate.py" \
  "$ROOT/opendbc_repo/opendbc/car/tesla/radar_interface.py" \
  "$ROOT/openpilot/selfdrive/car/modules/LONG_module.py"

echo "COMPILE OK"

echo
if [ "$fail" -eq 0 ]; then
  if [ "$warn" -eq 0 ]; then
    echo "v107 audit PASS"
  else
    echo "v107 audit PASS with warnings"
  fi
else
  echo "v107 audit FAIL"
fi

exit "$fail"
