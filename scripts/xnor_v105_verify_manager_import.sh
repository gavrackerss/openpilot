#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

fail=0

check_hash() {
  local rel="$1"
  local expected="$2"
  local path="$ROOT/$rel"
  if [ ! -f "$path" ]; then
    echo "MISSING $rel"
    fail=1
    return
  fi
  local got
  got="$(sha256sum "$path" | awk '{print $1}')"
  if [ "$got" = "$expected" ]; then
    echo "OK      $rel"
  else
    echo "BAD     $rel"
    echo "        expected $expected"
    echo "        got      $got"
    fail=1
  fi
}

check_hash "opendbc_repo/opendbc/car/tesla/__init__.py" "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
check_hash "opendbc_repo/opendbc/car/tesla/carstate.py" "a9e331a973c8ad4cccad51f7dc5b90502c2d78c6b08dfc59b138dfcd3f5f66e2"
check_hash "opendbc_repo/opendbc/car/tesla/fingerprints.py" "fe30c3a22eebb0994fc4815e4351fbc5e2d2ec787a2796715780c81deb9dc073"
check_hash "opendbc_repo/opendbc/car/tesla/interface.py" "b1c7ee1d8f160c70dd6bee0c7d18f4f7735097b145f9e263c782e86469e8a707"
check_hash "opendbc_repo/opendbc/car/tesla/radar_interface.py" "a2e132b28a4fd671438699f08b382b71e6fa687dc59bfc9f4c7cbf659e568b52"

python - <<'PY'
import importlib
mods = [
  "opendbc.car.tesla.interface",
  "opendbc.car.tesla.carstate",
  "opendbc.car.tesla.radar_interface",
]
for mod in mods:
  importlib.import_module(mod)
print("IMPORT OK opendbc.car.tesla.interface")
PY

if [ "$fail" -eq 0 ]; then
  echo "v105 Tesla manager import restore OK"
else
  echo "v105 Tesla manager import restore FAILED"
fi

exit "$fail"
