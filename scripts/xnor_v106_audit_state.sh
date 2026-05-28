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

check_hash() {
  local rel="$1"
  local expected="$2"
  local path="$ROOT/$rel"
  if [ ! -f "$path" ]; then
    bad "MISSING $rel"
    return
  fi
  local got
  got="$(sha256sum "$path" | awk '{print $1}')"
  if [ "$got" = "$expected" ]; then
    ok "$rel"
  else
    bad "$rel"
    echo "        expected $expected"
    echo "        got      $got"
  fi
}

check_hash_any_long() {
  local rel="openpilot/selfdrive/car/modules/LONG_module.py"
  local path="$ROOT/$rel"
  if [ ! -f "$path" ]; then
    bad "MISSING $rel"
    return
  fi
  local got
  got="$(sha256sum "$path" | awk '{print $1}')"
  case "$got" in
    "ddfd15f9d2b00d0c17706a841c2926611f6e64e69cfaa6f3d3e4faa85f8ab366") ok "$rel hash=v99";;
    "5b8cf892236b344af06eeea4b94fd8586939f11a568d2dbf90d1b9207d347801") ok "$rel hash=v100/v102/v104 LONG-only patch";;
    *) warnf "$rel unknown hash $got";;
  esac
}

echo "== root =="
info "$ROOT"

echo
echo "== opendbc path =="
if [ -L "$ROOT/opendbc" ]; then
  target="$(readlink "$ROOT/opendbc")"
  info "opendbc symlink -> $target"
  if [ "$target" = "opendbc_repo/opendbc" ] || [ "$target" = "$ROOT/opendbc_repo/opendbc" ]; then
    ok "opendbc symlink target"
  else
    warnf "opendbc symlink target is unexpected"
  fi
elif [ -d "$ROOT/opendbc" ]; then
  info "opendbc is a directory"
else
  bad "opendbc path missing"
fi

echo
echo "== Tesla file hashes =="
check_hash "opendbc_repo/opendbc/car/tesla/__init__.py" "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
check_hash "opendbc_repo/opendbc/car/tesla/carcontroller.py" "92c010676563b37b96104e434ee89bf2d86fbf366ae2cebeaa94f7da48fd0587"
check_hash "opendbc_repo/opendbc/car/tesla/carstate.py" "a9e331a973c8ad4cccad51f7dc5b90502c2d78c6b08dfc59b138dfcd3f5f66e2"
check_hash "opendbc_repo/opendbc/car/tesla/fingerprints.py" "fe30c3a22eebb0994fc4815e4351fbc5e2d2ec787a2796715780c81deb9dc073"
check_hash "opendbc_repo/opendbc/car/tesla/interface.py" "b1c7ee1d8f160c70dd6bee0c7d18f4f7735097b145f9e263c782e86469e8a707"
check_hash "opendbc_repo/opendbc/car/tesla/radar_interface.py" "a2e132b28a4fd671438699f08b382b71e6fa687dc59bfc9f4c7cbf659e568b52"
check_hash "opendbc_repo/opendbc/car/tesla/teslacan.py" "8386e86a182fbec5d6a50bef087aff4d06754483294230373caa03fbbdf74932"
check_hash "opendbc_repo/opendbc/car/tesla/teslacan_legacy.py" "18365eecc2351aebceba9cf574cea48c384e8c6fd6cdec757280fc03fe478c0f"
check_hash "opendbc_repo/opendbc/car/tesla/values.py" "7c94ed0f0694859f4f656ce4c6ae74d4d05d365456c017a04a70fb18312a1c20"

echo
echo "== LONG file hash =="
check_hash_any_long

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
    echo "v106 audit PASS"
  else
    echo "v106 audit PASS with warnings"
  fi
else
  echo "v106 audit FAIL"
fi

exit "$fail"
