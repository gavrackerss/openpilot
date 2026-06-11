#!/usr/bin/env bash
# /data/openpilot/scripts/xnor_v112_emergency_uninstall_v113.sh
set -euo pipefail

ROOT="${ROOT:-/data/openpilot}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
PAYLOAD_DIR="${SCRIPT_DIR}/../recovery/launch"
TS="$(date -u +%Y%m%d_%H%M%S)"
BACKUP="/data/xnor_recovery_backups/v113_uninstall_v112_${TS}"

HOOK_ID="XNOR_BOOT_RAW_CAN_V112"
OLD_PATTERN="xnor_aeb_boot_watch|aeb_boot_watch"
NEW_PATTERN="xnor_boot_raw_can_watch_v112.py|xnor_boot_raw_can_watch_v112.sh"

echo "== v113 emergency uninstall for v112 boot hook =="
echo "root=${ROOT}"
echo "backup=${BACKUP}"

mkdir -p "${BACKUP}"

echo
echo "== stop old/new watcher processes =="
pgrep -af "${OLD_PATTERN}|${NEW_PATTERN}" || true
pkill -f "${OLD_PATTERN}|${NEW_PATTERN}" || true
sleep 0.2
pkill -9 -f "${OLD_PATTERN}|${NEW_PATTERN}" || true

echo
echo "== backup current launch files =="
for f in launch_openpilot.sh launch_chffrplus.sh launch_env.sh; do
  if [ -e "${ROOT}/${f}" ]; then
    cp -a "${ROOT}/${f}" "${BACKUP}/${f}.current.bak"
    echo "backed up ${ROOT}/${f}"
  fi
done

echo
echo "== restore known-good launch files from the correct xnor-dev repo =="
for f in launch_openpilot.sh launch_chffrplus.sh launch_env.sh; do
  src="${PAYLOAD_DIR}/${f}"
  dst="${ROOT}/${f}"
  if [ ! -f "${src}" ]; then
    echo "Missing payload file: ${src}" >&2
    exit 1
  fi
  cp -a "${src}" "${dst}"
  chmod 755 "${dst}" || true
  echo "restored ${dst}"
done

echo
echo "== remove v112 tmp boot markers =="
rm -f \
  /tmp/xnor_boot_raw_can_watch_v112.started \
  /tmp/xnor_boot_raw_can_watch_v112.pid \
  /tmp/xnor_boot_raw_can_watch_v112_launch.log \
  /tmp/xnor_boot_raw_can_watch_v112_boot_start.log || true
rm -rf /tmp/xnor_boot_raw_can_watch_v112.lockdir || true

echo
echo "== move v112 watcher files out of active path =="
mkdir -p "${BACKUP}/removed_v112"
for f in \
  "${ROOT}/scripts/xnor_boot_raw_can_watch_v112.sh" \
  "${ROOT}/tools/xnor_boot_raw_can_watch_v112.py"; do
  if [ -e "${f}" ]; then
    mv "${f}" "${BACKUP}/removed_v112/"
    echo "moved ${f}"
  fi
done

echo
echo "== syntax check launch files =="
bash -n "${ROOT}/launch_openpilot.sh"
bash -n "${ROOT}/launch_chffrplus.sh"
bash -n "${ROOT}/launch_env.sh"
echo "launch shell syntax OK"

echo
echo "== verify no v112 launch hook remains =="
if grep -RIn "${HOOK_ID}" \
  "${ROOT}/launch_openpilot.sh" \
  "${ROOT}/launch_chffrplus.sh" \
  "${ROOT}/launch_env.sh" 2>/dev/null; then
  echo "ERROR: v112 hook still present" >&2
  exit 1
fi
echo "v112 hook absent"

echo
echo "== current watcher process check =="
pgrep -af "${OLD_PATTERN}|${NEW_PATTERN}" || true

echo
echo "v113 emergency uninstall PASS"
echo "Now reboot:"
echo "  sudo reboot"
echo
echo "If openpilot still does not start after reboot, collect:"
echo "  cat /tmp/launch_log 2>/dev/null || true"
echo "  tail -200 /tmp/launch_log 2>/dev/null || true"
echo "  ps -ef | grep -E 'manager.py|launch|xnor' | grep -v grep || true"
