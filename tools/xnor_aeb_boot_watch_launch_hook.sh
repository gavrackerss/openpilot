#!/usr/bin/env bash
set -euo pipefail

OP_ROOT="${OP_ROOT:-/data/openpilot}"
WATCH="${OP_ROOT}/tools/xnor_aeb_boot_watch.py"
OUT_DIR="${OP_ROOT}/aeb_boot_watch"
LAUNCH="${OP_ROOT}/launch_openpilot.sh"
BACKUP="${OP_ROOT}/launch_openpilot.sh.xnor_aeb_watch.bak"
START_MARK="# XNOR_AEB_BOOT_WATCH_START"
END_MARK="# XNOR_AEB_BOOT_WATCH_END"
DEFAULT_DURATION="${XNOR_AEB_BOOT_DURATION:-360}"

need_launch() {
  if [[ ! -f "${LAUNCH}" ]]; then
    echo "Could not find ${LAUNCH}" >&2
    echo "Set OP_ROOT=/path/to/openpilot if your tree is elsewhere." >&2
    exit 1
  fi
}

need_watch() {
  if [[ ! -f "${WATCH}" ]]; then
    echo "Could not find ${WATCH}" >&2
    echo "Unzip the bundle into /data first, or set OP_ROOT correctly." >&2
    exit 1
  fi
}

install_hook() {
  need_launch
  need_watch
  chmod +x "${WATCH}"
  mkdir -p "${OUT_DIR}"

  if [[ ! -f "${BACKUP}" ]]; then
    cp "${LAUNCH}" "${BACKUP}"
  fi

  python3 - "${LAUNCH}" "${DEFAULT_DURATION}" <<'PY'
from __future__ import annotations
import sys
from pathlib import Path

path = Path(sys.argv[1])
duration = sys.argv[2]
text = path.read_text()
start = "# XNOR_AEB_BOOT_WATCH_START"
end = "# XNOR_AEB_BOOT_WATCH_END"

# Remove any previous version of the hook first so install is idempotent.
while start in text and end in text:
  a = text.index(start)
  b = text.index(end) + len(end)
  text = text[:a].rstrip() + "\n" + text[b:].lstrip("\n")

block = f"""# XNOR_AEB_BOOT_WATCH_START
# Starts the decoded AEB/Panda boot watcher immediately when launch_openpilot.sh starts.
# Raw CAN logging is OFF by default, so this should not create 100MB+ files.
(
  export PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo:${{PYTHONPATH:-}}
  cd /data/openpilot || exit 0
  mkdir -p /data/openpilot/aeb_boot_watch

  # Keep the capture folder tidy. Do not delete unrelated user files.
  find /data/openpilot/aeb_boot_watch -maxdepth 1 -type f \\
    \( -name 'xnor_aeb_boot_watch_v3_*.jsonl' -o -name 'xnor_aeb_boot_watch_v3_*.txt' -o -name 'aeb_boot_watch_auto_launcher.log' \) \\
    -mtime +7 -delete 2>/dev/null || true

  if ! pgrep -f 'xnor_aeb_boot_watch.py.*--boot-auto' >/dev/null 2>&1; then
    nohup python3 /data/openpilot/tools/xnor_aeb_boot_watch.py \\
      --boot-auto \\
      --duration {duration} \\
      --output-dir /data/openpilot/aeb_boot_watch \\
      --state-interval 0.5 \\
      --repeat-suspicious-sec 1.0 \\
      --quiet \\
      >/data/openpilot/aeb_boot_watch/aeb_boot_watch_auto_launcher.log 2>&1 &
  fi
) &
# XNOR_AEB_BOOT_WATCH_END
"""

lines = text.splitlines(True)
if lines and lines[0].startswith("#!"):
  text = lines[0] + "\n" + block + "\n" + "".join(lines[1:])
else:
  text = block + "\n" + text

path.write_text(text)
PY

  chmod +x "${LAUNCH}"
  echo "Installed AEB boot watcher launch hook into ${LAUNCH}"
  echo "Backup: ${BACKUP}"
  echo "Duration: ${DEFAULT_DURATION}s"
  echo "Outputs: ${OUT_DIR}/xnor_aeb_boot_watch_v3_*.txt and .jsonl"
}

uninstall_hook() {
  need_launch
  python3 - "${LAUNCH}" <<'PY'
from __future__ import annotations
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()
start = "# XNOR_AEB_BOOT_WATCH_START"
end = "# XNOR_AEB_BOOT_WATCH_END"
changed = False
while start in text and end in text:
  a = text.index(start)
  b = text.index(end) + len(end)
  text = text[:a].rstrip() + "\n" + text[b:].lstrip("\n")
  changed = True
if changed:
  path.write_text(text)
PY
  echo "Removed AEB boot watcher launch hook from ${LAUNCH}"
}

restore_backup() {
  if [[ -f "${BACKUP}" ]]; then
    cp "${BACKUP}" "${LAUNCH}"
    chmod +x "${LAUNCH}"
    echo "Restored ${LAUNCH} from ${BACKUP}"
  else
    echo "No backup found at ${BACKUP}" >&2
    exit 1
  fi
}

start_bg() {
  need_watch
  chmod +x "${WATCH}"
  mkdir -p "${OUT_DIR}"
  if pgrep -f 'xnor_aeb_boot_watch.py.*--boot-auto' >/dev/null 2>&1; then
    echo "Watcher already running"
    exit 0
  fi
  (
    export PYTHONPATH="${OP_ROOT}:${OP_ROOT}/opendbc_repo:${PYTHONPATH:-}"
    cd "${OP_ROOT}"
    nohup python3 tools/xnor_aeb_boot_watch.py \
      --boot-auto \
      --duration "${DEFAULT_DURATION}" \
      --output-dir "${OUT_DIR}" \
      --state-interval 0.5 \
      --repeat-suspicious-sec 1.0 \
      --quiet \
      >"${OUT_DIR}/aeb_boot_watch_manual_launcher.log" 2>&1 &
  )
  echo "Started watcher in background"
}

status() {
  echo "Processes:"
  pgrep -af 'xnor_aeb_boot_watch.py' || true
  echo
  echo "Hook in launch_openpilot.sh:"
  if [[ -f "${LAUNCH}" ]]; then
    grep -n "XNOR_AEB_BOOT_WATCH" "${LAUNCH}" || true
  fi
  echo
  echo "Latest outputs:"
  ls -lt "${OUT_DIR}" 2>/dev/null | head -20 || true
}

logs() {
  tail -f "${OUT_DIR}"/aeb_boot_watch_*launcher.log "${OUT_DIR}"/xnor_aeb_boot_watch_v3_*.txt 2>/dev/null || true
}

case "${1:-}" in
  install) install_hook ;;
  uninstall) uninstall_hook ;;
  restore-backup) restore_backup ;;
  start-bg) start_bg ;;
  status) status ;;
  logs) logs ;;
  *)
    echo "Usage: $0 {install|uninstall|restore-backup|start-bg|status|logs}" >&2
    exit 1
    ;;
esac
