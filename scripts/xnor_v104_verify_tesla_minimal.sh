#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TESLA_DIR="$ROOT/opendbc_repo/opendbc/car/tesla"
MODULES_DIR="$ROOT/openpilot/selfdrive/car/modules"

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

check_absent() {
  local rel="$1"
  if [ -e "$ROOT/$rel" ]; then
    echo "EXTRA   $rel"
    fail=1
  else
    echo "ABSENT  $rel"
  fi
}

check_hash "opendbc_repo/opendbc/car/tesla/carcontroller.py" "92c010676563b37b96104e434ee89bf2d86fbf366ae2cebeaa94f7da48fd0587"
check_hash "opendbc_repo/opendbc/car/tesla/teslacan.py" "8386e86a182fbec5d6a50bef087aff4d06754483294230373caa03fbbdf74932"
check_hash "opendbc_repo/opendbc/car/tesla/teslacan_legacy.py" "18365eecc2351aebceba9cf574cea48c384e8c6fd6cdec757280fc03fe478c0f"
check_hash "opendbc_repo/opendbc/car/tesla/values.py" "7c94ed0f0694859f4f656ce4c6ae74d4d05d365456c017a04a70fb18312a1c20"
check_hash "openpilot/selfdrive/car/modules/LONG_module.py" "5b8cf892236b344af06eeea4b94fd8586939f11a568d2dbf90d1b9207d347801"

check_absent "opendbc_repo/opendbc/car/tesla/__init__.py"
check_absent "opendbc_repo/opendbc/car/tesla/carstate.py"
check_absent "opendbc_repo/opendbc/car/tesla/fingerprints.py"
check_absent "opendbc_repo/opendbc/car/tesla/interface.py"
check_absent "opendbc_repo/opendbc/car/tesla/radar_interface.py"
check_absent "opendbc_repo/opendbc/car/tesla/tests"

if find "$TESLA_DIR" "$MODULES_DIR" -type d -name "__pycache__" | grep -q .; then
  echo "EXTRA   __pycache__ under Tesla/modules"
  fail=1
else
  echo "ABSENT  __pycache__ under Tesla/modules"
fi

if [ "$fail" -eq 0 ]; then
  echo "v104 Tesla minimal restore OK"
else
  echo "v104 Tesla minimal restore FAILED"
fi

exit "$fail"
