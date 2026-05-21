#!/usr/bin/env bash
set -euo pipefail

OP_ROOT="${OP_ROOT:-/data/openpilot}"
GUARD="${OP_ROOT}/tools/xnor_aeb_boot_guard.py"
OUT_DIR="${OP_ROOT}/aeb_boot_guard"
LAUNCH="${OP_ROOT}/launch_openpilot.sh"
BACKUP="${OP_ROOT}/launch_openpilot.sh.xnor_aeb_guard.bak"
START_MARK="# XNOR_AEB_BOOT_GUARD_START"
END_MARK="# XNOR_AEB_BOOT_GUARD_END"

need_launch() {
  if [[ ! -f "${LAUNCH}" ]]; then
    echo "Could not find ${LAUNCH}" >&2
    echo "Set OP_ROOT=/path/to/openpilot if your tree is elsewhere." >&2
    exit 1
  fi
}

install_hook() {
  need_launch
  chmod +x "${GUARD}"
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
start = "# XNOR_AEB_BOOT_GUARD_START"
end = "# XNOR_AEB_BOOT_GUARD_END"

while start in text and end in text:
  a = text.index(start)
  b = text.index(end) + len(end)
  text = text[:a].rstrip() + "\n" + text[b:].lstrip("\n")

block = """# XNOR_AEB_BOOT_GUARD_START
if [ -x /data/openpilot/tools/xnor_aeb_boot_guard.py ]; then
  mkdir -p /data/openpilot/aeb_boot_guard
  if ! pgrep -f "xnor_aeb_boot_guard.py --duration 300" >/dev/null 2>&1; then
    (
      cd /data/openpilot
      nohup python3 tools/xnor_aeb_boot_guard.py --duration 300 --output-dir /data/openpilot/aeb_boot_guard \\
        >/data/openpilot/aeb_boot_guard/boot_guard_launcher.log 2>&1 &
    )
  fi
fi
# XNOR_AEB_BOOT_GUARD_END
"""

lines = text.splitlines(True)
if lines and lines[0].startswith("#!"):
  text = lines[0] + "\n" + block + "\n" + "".join(lines[1:])
else:
  text = block + "\n" + text

path.write_text(text)
PY

  chmod +x "${LAUNCH}"
  echo "Installed AEB boot guard launch hook into ${LAUNCH}"
  echo "Backup: ${BACKUP}"
  echo "Default mode is log/param guard only. It does not restart boardd."
}

uninstall_hook() {
  need_launch
  python3 - "$LAUNCH" <<'PY'
from __future__ import annotations
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()
start = "# XNOR_AEB_BOOT_GUARD_START"
end = "# XNOR_AEB_BOOT_GUARD_END"
changed = False

while start in text and end in text:
  a = text.index(start)
  b = text.index(end) + len(end)
  text = text[:a].rstrip() + "\n" + text[b:].lstrip("\n")
  changed = True

if changed:
  path.write_text(text)
PY
  echo "Removed AEB boot guard launch hook from ${LAUNCH}"
}

run_now() {
  chmod +x "${GUARD}"
  mkdir -p "${OUT_DIR}"
  cd "${OP_ROOT}"
  python3 tools/xnor_aeb_boot_guard.py --duration 300 --stdout --output-dir "${OUT_DIR}"
}

start_bg() {
  chmod +x "${GUARD}"
  mkdir -p "${OUT_DIR}"
  if pgrep -f "xnor_aeb_boot_guard.py --duration 300" >/dev/null 2>&1; then
    echo "AEB boot guard already running"
    exit 0
  fi
  cd "${OP_ROOT}"
  nohup python3 tools/xnor_aeb_boot_guard.py --duration 300 --output-dir "${OUT_DIR}" \
    >"${OUT_DIR}/manual_guard_launcher.log" 2>&1 &
  echo "Started AEB boot guard in background"
}

start_bg_recover() {
  chmod +x "${GUARD}"
  mkdir -p "${OUT_DIR}"
  if pgrep -f "xnor_aeb_boot_guard.py --duration 300" >/dev/null 2>&1; then
    echo "AEB boot guard already running"
    exit 0
  fi
  cd "${OP_ROOT}"
  nohup python3 tools/xnor_aeb_boot_guard.py --duration 300 --recover-after 12 --output-dir "${OUT_DIR}" \
    >"${OUT_DIR}/manual_guard_recover_launcher.log" 2>&1 &
  echo "Started AEB boot guard with boardd recovery enabled"
  echo "Recovery is gated to standstill, no gas/brake, and controls inactive."
}

status() {
  pgrep -af "xnor_aeb_boot_guard.py" || true
  ls -lt "${OUT_DIR}" 2>/dev/null | head || true
  if command -v python3 >/dev/null 2>&1; then
    cd "${OP_ROOT}" 2>/dev/null || true
    python3 - <<'PY' 2>/dev/null || true
from openpilot.common.params import Params
p = Params()
for k in ("XnorAebBootGuardActive", "XnorAebBootGuardReady", "XnorAebBootGuardState"):
  v = p.get(k, encoding="utf-8")
  print(f"{k}={v}")
PY
  fi
}

logs() {
  tail -f "${OUT_DIR}"/*.txt "${OUT_DIR}"/*launcher.log 2>/dev/null || true
}

case "${1:-}" in
  install) install_hook ;;
  uninstall) uninstall_hook ;;
  run-now) run_now ;;
  start-bg) start_bg ;;
  start-bg-recover) start_bg_recover ;;
  status) status ;;
  logs) logs ;;
  *) echo "Usage: $0 {install|uninstall|run-now|start-bg|start-bg-recover|status|logs}" >&2; exit 1 ;;
esac
