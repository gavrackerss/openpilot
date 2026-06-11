#!/usr/bin/env bash
set -euo pipefail

VERSION="v108"
DEFAULT_REPO="/data/openpilot"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
if [[ -n "${OPENPILOT_DIR:-}" ]]; then
  REPO="${OPENPILOT_DIR}"
elif [[ -d "${SCRIPT_DIR}/../selfdrive" || -d "${SCRIPT_DIR}/../opendbc_repo" ]]; then
  REPO="$(cd "${SCRIPT_DIR}/.." >/dev/null 2>&1 && pwd)"
else
  REPO="${DEFAULT_REPO}"
fi

PAYLOAD="${REPO}/xnor_recovery_payload_v108"
TARGET_MANIFEST="${PAYLOAD}/expected_targets.sha256"
BACKUP_BASE="${XNOR_BACKUP_BASE:-/data/xnor_recovery_backups}"
BACKUP_DIR=""

usage() {
  cat <<USAGE
Usage:
  bash scripts/xnor_recover_v108_from_correct_repo.sh audit
  bash scripts/xnor_recover_v108_from_correct_repo.sh apply
  bash scripts/xnor_recover_v108_from_correct_repo.sh verify

Environment:
  OPENPILOT_DIR=/data/openpilot       override repo path
  XNOR_BACKUP_BASE=/data/...          override backup path
USAGE
}

require_payload() {
  if [[ ! -d "${REPO}" ]]; then
    echo "FAIL: repo not found: ${REPO}" >&2
    exit 2
  fi
  if [[ ! -d "${PAYLOAD}" || ! -f "${TARGET_MANIFEST}" ]]; then
    echo "FAIL: payload missing: ${PAYLOAD}" >&2
    echo "Run this from the repo where the recovery zip was extracted." >&2
    exit 2
  fi
}

init_backup() {
  local stamp
  stamp="$(date -u +%Y%m%d_%H%M%S)"
  BACKUP_DIR="${BACKUP_BASE}/${VERSION}_${stamp}"
  mkdir -p "${BACKUP_DIR}"
  echo "Backup: ${BACKUP_DIR}"
}

backup_path() {
  local rel="$1"
  local src="${REPO}/${rel}"
  local dst="${BACKUP_DIR}/${rel}"

  if [[ -e "${src}" || -L "${src}" ]]; then
    mkdir -p "$(dirname "${dst}")"
    cp -a "${src}" "${dst}"
  fi
}

replace_path() {
  local rel="$1"
  local src="${PAYLOAD}/${rel}"
  local dst="${REPO}/${rel}"

  if [[ ! -e "${src}" && ! -L "${src}" ]]; then
    echo "FAIL: payload path missing: ${src}" >&2
    exit 2
  fi

  backup_path "${rel}"
  rm -rf "${dst}"
  mkdir -p "$(dirname "${dst}")"
  cp -a "${src}" "${dst}"
}

ensure_symlink() {
  local rel="$1"
  local target="$2"
  local link="${REPO}/${rel}"
  local current=""

  if [[ -L "${link}" ]]; then
    current="$(readlink "${link}")"
    if [[ "${current}" == "${target}" ]]; then
      return 0
    fi
  fi

  backup_path "${rel}"
  rm -rf "${link}"
  mkdir -p "$(dirname "${link}")"
  ln -s "${target}" "${link}"
}

purge_pycache() {
  find "${REPO}/opendbc_repo/opendbc/car/tesla" \
       "${REPO}/selfdrive/car/modules" \
       "${REPO}/openpilot" \
       -type d -name "__pycache__" -prune -exec rm -rf {} + 2>/dev/null || true
}

print_symlink_state() {
  echo "Repo: ${REPO}"

  if [[ -L "${REPO}/opendbc" ]]; then
    echo "opendbc -> $(readlink "${REPO}/opendbc")"
  elif [[ -e "${REPO}/opendbc" ]]; then
    echo "opendbc is NOT a symlink"
  else
    echo "opendbc is missing"
  fi

  for name in common selfdrive system third_party tools; do
    local p="${REPO}/openpilot/${name}"
    if [[ -L "${p}" ]]; then
      echo "openpilot/${name} -> $(readlink "${p}")"
    elif [[ -e "${p}" ]]; then
      echo "openpilot/${name} is NOT a symlink"
    else
      echo "openpilot/${name} is missing"
    fi
  done
}

hash_check() {
  (
    cd "${REPO}"
    sha256sum -c "${TARGET_MANIFEST}"
  )
}

compile_check() {
  (
    cd "${REPO}"
    PYTHONPATH="${REPO}${PYTHONPATH:+:${PYTHONPATH}}" python3 - <<'PY'
from __future__ import annotations

import pathlib
import py_compile
import sys

repo = pathlib.Path.cwd()
files = []
files.extend(sorted((repo / "opendbc_repo/opendbc/car/tesla").rglob("*.py")))
files.extend(sorted((repo / "selfdrive/car/modules").glob("*.py")))
files.append(repo / "opendbc_repo/opendbc/car/car_helpers.py")
files.append(repo / "openpilot/__init__.py")

seen = set()
failed = False

for path in files:
  if path in seen or not path.exists():
    continue
  seen.add(path)
  try:
    py_compile.compile(str(path), doraise=True)
  except Exception as exc:
    failed = True
    print(f"COMPILE FAIL {path.relative_to(repo)}: {exc}")

if failed:
  sys.exit(1)

print(f"COMPILE OK {len(seen)} files")
PY
  )
}

import_check() {
  (
    cd "${REPO}"
    PYTHONPATH="${REPO}${PYTHONPATH:+:${PYTHONPATH}}" python3 - <<'PY'
from __future__ import annotations

import importlib
import sys

modules = [
  "opendbc.car.tesla.values",
  "opendbc.car.tesla.interface",
  "opendbc.car.tesla.carstate",
  "opendbc.car.tesla.carcontroller",
  "opendbc.car.car_helpers",
  "openpilot.selfdrive.car.modules.LONG_module",
]

for name in modules:
  importlib.import_module(name)
  print(f"IMPORT OK {name}")

from opendbc.car.tesla.values import CruiseButtons

if int(CruiseButtons.IDLE) != 0:
  raise RuntimeError(f"Unexpected CruiseButtons.IDLE={int(CruiseButtons.IDLE)}")
if int(CruiseButtons.CANCEL) != 1:
  raise RuntimeError(f"Unexpected CruiseButtons.CANCEL={int(CruiseButtons.CANCEL)}")

print("CruiseButtons OK IDLE=0 CANCEL=1")
PY
  )
}

verify() {
  require_payload
  print_symlink_state

  echo
  echo "Checking restored file hashes..."
  hash_check

  echo
  echo "Compiling restored Python files..."
  compile_check

  echo
  echo "Checking manager/import path..."
  import_check

  echo
  echo "${VERSION} verify PASS"
}

audit() {
  require_payload
  print_symlink_state

  local status=0

  echo
  echo "Checking current file hashes against the attached correct repo..."
  if ! hash_check; then
    status=1
  fi

  echo
  echo "Compiling current affected Python files..."
  if ! compile_check; then
    status=1
  fi

  echo
  echo "Checking current manager/import path..."
  if ! import_check; then
    status=1
  fi

  if [[ "${status}" -eq 0 ]]; then
    echo
    echo "${VERSION} audit PASS"
  else
    echo
    echo "${VERSION} audit FAIL: run apply to restore from the attached correct repo payload."
  fi

  return "${status}"
}

apply() {
  require_payload
  init_backup

  echo "Restoring affected files from attached correct repo payload..."

  mkdir -p "${REPO}/opendbc_repo/opendbc/car"
  mkdir -p "${REPO}/selfdrive/car"
  mkdir -p "${REPO}/openpilot"

  replace_path "opendbc_repo/opendbc/car/tesla"
  replace_path "opendbc_repo/opendbc/car/car_helpers.py"
  replace_path "selfdrive/car/modules"
  replace_path "openpilot/__init__.py"

  ensure_symlink "opendbc" "opendbc_repo/opendbc"
  ensure_symlink "openpilot/common" "../common"
  ensure_symlink "openpilot/selfdrive" "../selfdrive/"
  ensure_symlink "openpilot/system" "../system/"
  ensure_symlink "openpilot/third_party" "../third_party"
  ensure_symlink "openpilot/tools" "../tools"

  purge_pycache

  echo
  verify

  echo
  echo "${VERSION} apply PASS"
  echo "Backup saved at: ${BACKUP_DIR}"
  echo "Reboot after this PASS line:"
  echo "  sudo reboot"
}

main() {
  local mode="${1:-audit}"

  case "${mode}" in
    audit)
      audit
      ;;
    apply)
      apply
      ;;
    verify)
      verify
      ;;
    -h|--help|help)
      usage
      ;;
    *)
      echo "Unknown mode: ${mode}" >&2
      usage >&2
      exit 2
      ;;
  esac
}

main "$@"
