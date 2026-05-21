#!/usr/bin/env bash
set -euo pipefail

OP_ROOT="${OP_ROOT:-/data/openpilot}"
WATCH="${OP_ROOT}/tools/xnor_aeb_boot_watch.py"
OUT_DIR="${OP_ROOT}/aeb_boot_watch"
LAUNCH="${OP_ROOT}/launch_openpilot.sh"
BACKUP="${OP_ROOT}/launch_openpilot.sh.xnor_aeb_watch.bak"
START_MARK="# XNOR_AEB_BOOT_WATCH_START"
END_MARK="# XNOR_AEB_BOOT_WATCH_END"

block() {
  cat <<'EOF'
# XNOR_AEB_BOOT_WATCH_START
if [ -x /data/openpilot/tools/xnor_aeb_boot_watch.py ]; then
  mkdir -p /data/openpilot/aeb_boot_watch
  if ! pgrep -f "xnor_aeb_boot_watch.py --duration 240" >/dev/null 2>&1; then
    (
      cd /data/openpilot
      nohup python3 tools/xnor_aeb_boot_watch.py --duration 240 --output-dir /data/openpilot/aeb_boot_watch \
        >/data/openpilot/aeb_boot_watch/boot_hook_launcher.log 2>&1 &
    )
  fi
fi
# XNOR_AEB_BOOT_WATCH_END
EOF
}

need_launch() {
  if [[ ! -f "${LAUNCH}" ]]; then
    echo "Could not find ${LAUNCH}" >&2
    echo "Set OP_ROOT=/path/to/openpilot if your tree is elsewhere." >&2
    exit 1
  fi
}

install_hook() {
  need_launch
  chmod +x "${WATCH}"
  mkdir -p "${OUT_DIR}"

  if [[ ! -f "${BACKUP}" ]]; then
    cp "${LAUNCH}" "${BACKUP}"
  fi

  python3 - "$LAUNCH" <<'PY'
from __future__ import annotations
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()
start = "# XNOR_AEB_BOOT_WATCH_START"
end = "# XNOR_AEB_BOOT_WATCH_END"

while start in text and end in text:
  a = text.index(start)
  b = text.index(end) + len(end)
  text = text[:a].rstrip() + "\n" + text[b:].lstrip("\n")

block = """# XNOR_AEB_BOOT_WATCH_START
if [ -x /data/openpilot/tools/xnor_aeb_boot_watch.py ]; then
  mkdir -p /data/openpilot/aeb_boot_watch
  if ! pgrep -f "xnor_aeb_boot_watch.py --duration 240" >/dev/null 2>&1; then
    (
      cd /data/openpilot
      nohup python3 tools/xnor_aeb_boot_watch.py --duration 240 --output-dir /data/openpilot/aeb_boot_watch \\
        >/data/openpilot/aeb_boot_watch/boot_hook_launcher.log 2>&1 &
    )
  fi
fi
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
  echo "Installed launch hook into ${LAUNCH}"
  echo "Backup: ${BACKUP}"
}

uninstall_hook() {
  need_launch
  python3 - "$LAUNCH" <<'PY'
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
  echo "Removed launch hook from ${LAUNCH}"
}

run_now() {
  chmod +x "${WATCH}"
  mkdir -p "${OUT_DIR}"
  cd "${OP_ROOT}"
  python3 tools/xnor_aeb_boot_watch.py --duration 240 --stdout --output-dir "${OUT_DIR}"
}

start_bg() {
  chmod +x "${WATCH}"
  mkdir -p "${OUT_DIR}"
  if pgrep -f "xnor_aeb_boot_watch.py --duration 240" >/dev/null 2>&1; then
    echo "Watcher already running"
    exit 0
  fi
  cd "${OP_ROOT}"
  nohup python3 tools/xnor_aeb_boot_watch.py --duration 240 --output-dir "${OUT_DIR}" \
    >"${OUT_DIR}/manual_launcher.log" 2>&1 &
  echo "Started watcher in background"
}

status() {
  pgrep -af "xnor_aeb_boot_watch.py" || true
  ls -lt "${OUT_DIR}" 2>/dev/null | head || true
}

logs() {
  tail -f "${OUT_DIR}"/*.txt "${OUT_DIR}"/*launcher.log 2>/dev/null || true
}

case "${1:-}" in
  install) install_hook ;;
  uninstall) uninstall_hook ;;
  run-now) run_now ;;
  start-bg) start_bg ;;
  status) status ;;
  logs) logs ;;
  *) echo "Usage: $0 {install|uninstall|run-now|start-bg|status|logs}" >&2; exit 1 ;;
esac
