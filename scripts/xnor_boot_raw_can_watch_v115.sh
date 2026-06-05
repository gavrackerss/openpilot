#!/usr/bin/env bash
# /data/openpilot/scripts/xnor_boot_raw_can_watch_v115.sh
set -u

ROOT="${ROOT:-/data/openpilot}"
LAUNCH="${ROOT}/launch_chffrplus.sh"
CAPTURE_DIR="${ROOT}/xnor_aeb_boot_capture"
TOOL="${ROOT}/tools/xnor_boot_raw_can_watch_v115.py"
BACKUP_BASE="/data/xnor_recovery_backups"
HOOK_START="# XNOR_BOOT_RAW_CAN_WATCH_V115_START"
HOOK_END="# XNOR_BOOT_RAW_CAN_WATCH_V115_END"
OLD_PATTERN="xnor_aeb_boot_watch|aeb_boot_watch|xnor_boot_raw_can_watch_v111|xnor_boot_raw_can_watch_v112|xnor_boot_raw_can_watch_v114"

msg() {
  printf '%s\n' "$*"
}

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

timestamp_utc() {
  date -u +%Y%m%d_%H%M%S
}

make_backup_dir() {
  local ts
  ts="$(timestamp_utc)"
  BACKUP_DIR="${BACKUP_BASE}/xnor_boot_raw_can_watch_v115_${ts}"
  mkdir -p "${BACKUP_DIR}" || die "could not create ${BACKUP_DIR}"
  msg "${BACKUP_DIR}"
}

backup_file() {
  local src dst
  src="$1"
  test -f "${src}" || return 0
  dst="${BACKUP_DIR}/$(echo "${src}" | sed 's#^/##; s#/#__#g')"
  cp -a "${src}" "${dst}" || die "backup failed for ${src}"
}

kill_watchers() {
  msg "Stopping old/new XNOR watcher processes..."
  pkill -9 -f 'tools/xnor_aeb_boot_watch.py|xnor_aeb_boot_watch|aeb_boot_watch|tools/xnor_boot_raw_can_watch_v111.py|tools/xnor_boot_raw_can_watch_v112.py|tools/xnor_boot_raw_can_watch_v114.py|tools/xnor_boot_raw_can_watch_v115.py' 2>/dev/null || true
}

strip_boot_hooks_from_file() {
  local path
  path="$1"
  test -f "${path}" || return 0

  python3 - "${path}" <<'PY'
from __future__ import annotations
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text(errors="replace")
original = text

for version in ("111", "112", "114", "115"):
  pattern = re.compile(
    rf"\n?# XNOR_BOOT_RAW_CAN_WATCH_V{version}_START\n.*?# XNOR_BOOT_RAW_CAN_WATCH_V{version}_END\n?",
    re.DOTALL,
  )
  text = pattern.sub("\n", text)

out_lines = []
old_re = re.compile(r"xnor_aeb_boot_watch|aeb_boot_watch|xnor_boot_raw_can_watch_v111|xnor_boot_raw_can_watch_v112|xnor_boot_raw_can_watch_v114")
for line in text.splitlines():
  if old_re.search(line) and not line.lstrip().startswith("#"):
    out_lines.append("# XNOR_DISABLED_BY_V115 " + line)
  else:
    out_lines.append(line)

text = "\n".join(out_lines).rstrip() + "\n"
if text != original:
  path.write_text(text)
PY
}

strip_all_known_hooks() {
  msg "Removing old boot hooks from writable launch files..."
  for path in "${ROOT}/launch_openpilot.sh" "${ROOT}/launch_chffrplus.sh" "${ROOT}/launch_env.sh"; do
    test -f "${path}" || continue
    test -w "${path}" || continue
    backup_file "${path}"
    strip_boot_hooks_from_file "${path}"
  done
}

write_v115_hook() {
  test -f "${LAUNCH}" || die "missing ${LAUNCH}"
  test -w "${LAUNCH}" || die "${LAUNCH} is not writable"

  backup_file "${LAUNCH}"

  python3 - "${LAUNCH}" <<'PY'
from __future__ import annotations
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(errors="replace")

hook = """# XNOR_BOOT_RAW_CAN_WATCH_V115_START
if test -z "${XNOR_BOOT_RAW_CAN_WATCH_V115_DISABLE:-}" && test ! -e /tmp/xnor_boot_raw_can_watch_v115.started; then
  touch /tmp/xnor_boot_raw_can_watch_v115.started 2>/dev/null || true
  (
    cd /data/openpilot 2>/dev/null || exit 0
    nohup /usr/bin/env python3 tools/xnor_boot_raw_can_watch_v115.py --duration 210 --out-dir /data/openpilot/xnor_aeb_boot_capture --prefix xnor_boot_raw_can_v115 --max-jsonl-mb 8 --max-txt-kb 256 --quiet >/tmp/xnor_boot_raw_can_watch_v115.log 2>&1 &
  ) >/dev/null 2>&1 || true
fi
# XNOR_BOOT_RAW_CAN_WATCH_V115_END
"""

if "# XNOR_BOOT_RAW_CAN_WATCH_V115_START" in text:
  raise SystemExit("v115 hook already present after strip; refusing duplicate insert")

lines = text.splitlines()
insert_at = 0
if lines and lines[0].startswith("#!"):
  insert_at = 1

lines[insert_at:insert_at] = hook.rstrip("\n").splitlines()
path.write_text("\n".join(lines).rstrip() + "\n")
PY

  bash -n "${LAUNCH}" || die "${LAUNCH} syntax check failed after hook insert"
}

compile_watcher() {
  test -f "${TOOL}" || die "missing ${TOOL}"
  chmod +x "${TOOL}" || true
  python3 -m py_compile "${TOOL}" || die "watcher Python compile failed"
}

install_v115() {
  cd "${ROOT}" || die "missing ${ROOT}"
  make_backup_dir >/dev/null
  msg "Backup: ${BACKUP_DIR}"

  kill_watchers
  mkdir -p "${CAPTURE_DIR}" || die "could not create ${CAPTURE_DIR}"
  strip_all_known_hooks
  compile_watcher
  write_v115_hook

  msg "v115 install PASS"
  msg "Reboot, then run:"
  msg "  cd /data/openpilot && bash scripts/xnor_boot_raw_can_watch_v115.sh status"
}

uninstall_v115() {
  cd "${ROOT}" || die "missing ${ROOT}"
  make_backup_dir >/dev/null
  msg "Backup: ${BACKUP_DIR}"

  kill_watchers
  strip_all_known_hooks

  for path in "${ROOT}/launch_openpilot.sh" "${ROOT}/launch_chffrplus.sh" "${ROOT}/launch_env.sh"; do
    test -f "${path}" || continue
    bash -n "${path}" 2>/dev/null || true
  done

  rm -f /tmp/xnor_boot_raw_can_watch_v115.started /tmp/xnor_boot_raw_can_watch_v115.log 2>/dev/null || true
  msg "v115 uninstall PASS"
}

status_v115() {
  msg "== script syntax =="
  bash -n "${ROOT}/scripts/xnor_boot_raw_can_watch_v115.sh" && msg "script syntax OK"

  msg "== hook status =="
  for path in "${ROOT}/launch_openpilot.sh" "${ROOT}/launch_chffrplus.sh" "${ROOT}/launch_env.sh"; do
    test -f "${path}" || continue
    if grep -q "XNOR_BOOT_RAW_CAN_WATCH_V115" "${path}"; then
      msg "HOOK PRESENT: ${path}"
    else
      msg "no v115 hook: ${path}"
    fi
    if grep -Eq "${OLD_PATTERN}" "${path}"; then
      msg "OLD WATCHER TEXT PRESENT: ${path}"
      grep -nE "${OLD_PATTERN}" "${path}" || true
    fi
  done

  msg "== process status =="
  pgrep -af 'xnor_boot_raw_can_watch|xnor_aeb_boot_watch|aeb_boot_watch' || true

  msg "== launch syntax =="
  test -f "${LAUNCH}" && bash -n "${LAUNCH}" && msg "launch_chffrplus syntax OK"

  msg "== capture dir =="
  mkdir -p "${CAPTURE_DIR}" 2>/dev/null || true
  du -sh "${CAPTURE_DIR}" 2>/dev/null || true
  ls -lhtr "${CAPTURE_DIR}" 2>/dev/null | tail -20 || true

  msg "== tmp watcher log =="
  tail -80 /tmp/xnor_boot_raw_can_watch_v115.log 2>/dev/null || true
}

case "${1:-status}" in
  install)
    install_v115
    ;;
  uninstall)
    uninstall_v115
    ;;
  status)
    status_v115
    ;;
  *)
    die "usage: $0 {install|uninstall|status}"
    ;;
esac
