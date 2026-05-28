#!/usr/bin/env bash
# /data/openpilot/scripts/xnor_boot_raw_can_watch_v111.sh
set -euo pipefail

ROOT="${ROOT:-/data/openpilot}"
SERVICE="xnor-boot-raw-can-watch-v111.service"
UNIT_PATH="/etc/systemd/system/${SERVICE}"
WATCHER="${ROOT}/tools/xnor_boot_raw_can_watch_v111.py"
OUT_DIR="${ROOT}/xnor_aeb_boot_capture"
PYTHON="${PYTHON:-/usr/bin/python3}"

usage() {
  cat <<USAGE
Usage:
  bash scripts/xnor_boot_raw_can_watch_v111.sh install
  bash scripts/xnor_boot_raw_can_watch_v111.sh uninstall
  bash scripts/xnor_boot_raw_can_watch_v111.sh status
  bash scripts/xnor_boot_raw_can_watch_v111.sh run-once
  bash scripts/xnor_boot_raw_can_watch_v111.sh clean
  bash scripts/xnor_boot_raw_can_watch_v111.sh uninstall-old

Commands:
  install       Install bounded boot service. Captures first boot window automatically.
  uninstall     Disable/remove v111 boot service only. Keeps logs.
  status        Show service status, process, and latest captures.
  run-once      Run watcher now for a short manual validation.
  clean         Remove v111 capture logs.
  uninstall-old Disable old xnor_aeb_boot_watch/aeb_boot_watch hooks/services.
USAGE
}

require_root_or_sudo() {
  if [ "$(id -u)" -ne 0 ] && ! command -v sudo >/dev/null 2>&1; then
    echo "Need root or sudo for service install/uninstall." >&2
    exit 1
  fi
}

as_root() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

write_unit() {
  local tmp
  tmp="$(mktemp)"
  cat > "${tmp}" <<UNIT
[Unit]
Description=XNOR bounded boot raw CAN/sendcan watcher v111
Documentation=file://${ROOT}/tools/xnor_boot_raw_can_watch_v111.py
DefaultDependencies=no
After=local-fs.target
Before=openpilot.service manager.service comma.service
Wants=local-fs.target

[Service]
Type=simple
WorkingDirectory=${ROOT}
Environment=PYTHONPATH=${ROOT}:${ROOT}/opendbc_repo:${ROOT}/openpilot
ExecStart=${PYTHON} ${WATCHER} --duration 150 --out-dir ${OUT_DIR} --prefix xnor_boot_raw_can_v111 --raw-can-mode targeted --max-jsonl-mb 32 --max-txt-kb 768 --keep-runs 6 --quiet
Restart=no
Nice=-5
IOSchedulingClass=best-effort
IOSchedulingPriority=7

[Install]
WantedBy=multi-user.target
UNIT
  as_root cp "${tmp}" "${UNIT_PATH}"
  rm -f "${tmp}"
}

install_service() {
  require_root_or_sudo
  cd "${ROOT}"

  if [ ! -f "${WATCHER}" ]; then
    echo "Missing watcher: ${WATCHER}" >&2
    exit 1
  fi

  "${PYTHON}" -m py_compile "${WATCHER}"
  mkdir -p "${OUT_DIR}"

  echo "== disabling old huge boot watcher hooks first =="
  uninstall_old || true

  echo "== installing ${SERVICE} =="
  write_unit
  as_root systemctl daemon-reload
  as_root systemctl enable "${SERVICE}"

  echo "Installed. It will run automatically on next boot for 150s."
  echo "Output directory: ${OUT_DIR}"
  echo
  echo "To test without reboot:"
  echo "  bash scripts/xnor_boot_raw_can_watch_v111.sh run-once"
}

uninstall_service() {
  require_root_or_sudo
  echo "== uninstalling ${SERVICE} =="
  if command -v systemctl >/dev/null 2>&1; then
    as_root systemctl disable --now "${SERVICE}" || true
    as_root rm -f "${UNIT_PATH}"
    as_root systemctl daemon-reload || true
  fi
  pkill -f "xnor_boot_raw_can_watch_v111.py" || true
  echo "Uninstalled v111 service. Logs kept in ${OUT_DIR}."
}

status_service() {
  echo "== service =="
  if command -v systemctl >/dev/null 2>&1; then
    systemctl status "${SERVICE}" --no-pager || true
  else
    echo "systemctl not available"
  fi

  echo
  echo "== process =="
  pgrep -af "xnor_boot_raw_can_watch_v111.py" || true

  echo
  echo "== latest captures =="
  if [ -d "${OUT_DIR}" ]; then
    ls -lhtr "${OUT_DIR}" | tail -30 || true
    echo
    du -sh "${OUT_DIR}" || true
  else
    echo "No output directory: ${OUT_DIR}"
  fi
}

run_once() {
  cd "${ROOT}"
  "${PYTHON}" -m py_compile "${WATCHER}"
  mkdir -p "${OUT_DIR}"
  echo "Running one 45s validation capture now..."
  "${PYTHON}" "${WATCHER}" \
    --duration 45 \
    --out-dir "${OUT_DIR}" \
    --prefix xnor_boot_raw_can_v111_manual \
    --raw-can-mode targeted \
    --max-jsonl-mb 12 \
    --max-txt-kb 384 \
    --keep-runs 3
}

clean_logs() {
  echo "Removing v111 capture logs from ${OUT_DIR}"
  rm -rf "${OUT_DIR}"
  mkdir -p "${OUT_DIR}"
}

uninstall_old() {
  local ts backup pattern
  ts="$(date -u +%Y%m%d_%H%M%S)"
  backup="/data/xnor_recovery_backups/uninstall_old_aeb_boot_watch_${ts}"
  pattern="xnor_aeb_boot_watch|aeb_boot_watch"

  mkdir -p "${backup}"

  echo "Stopping old AEB watcher processes, if any..."
  pkill -f "${pattern}" || true

  if command -v systemctl >/dev/null 2>&1; then
    mapfile -t units < <(
      systemctl list-unit-files --all --no-legend 2>/dev/null \
        | awk '{print $1}' \
        | grep -Ei "${pattern}" || true
    )

    for unit in "${units[@]:-}"; do
      [ -n "${unit}" ] || continue
      echo "disabling old unit ${unit}"
      as_root systemctl disable --now "${unit}" || true
    done

    shopt -s nullglob
    for f in /etc/systemd/system/*aeb*watch*.service /etc/systemd/system/*xnor*aeb*.service; do
      [ "$(basename "${f}")" != "${SERVICE}" ] || continue
      echo "removing old unit ${f}"
      as_root cp -a "${f}" "${backup}/"
      as_root rm -f "${f}"
    done
    shopt -u nullglob

    as_root systemctl daemon-reload || true
  fi

  echo "Commenting old launch/cron hooks, if found..."
  shopt -s nullglob
  for f in \
    "${ROOT}/launch_openpilot.sh" \
    "${ROOT}/launch_chffrplus.sh" \
    "${ROOT}/launch_env.sh" \
    /etc/crontab \
    /etc/cron.d/*; do
    [ -f "${f}" ] || continue
    if grep -Eq "${pattern}" "${f}"; then
      echo "commenting ${f}"
      cp -a "${f}" "${backup}/$(basename "${f}").bak"
      as_root sed -i -E "/${pattern}/s/^/# XNOR_DISABLED_${ts} /" "${f}"
    fi
  done
  shopt -u nullglob

  echo "Moving old watcher scripts out of active path, if present..."
  mkdir -p "${backup}/removed_scripts"
  for f in \
    "${ROOT}/tools/xnor_aeb_boot_watch.py" \
    "${ROOT}/tools/xnor_aeb_boot_watch_v2.py" \
    "${ROOT}/tools/xnor_aeb_boot_watch_v3.py" \
    "${ROOT}/scripts/xnor_aeb_boot_watch.sh" \
    "${ROOT}/scripts/xnor_install_aeb_boot_watch.sh"; do
    [ -e "${f}" ] || continue
    echo "moving ${f}"
    mv "${f}" "${backup}/removed_scripts/"
  done

  echo "Old watcher backup: ${backup}"
}

cmd="${1:-}"
case "${cmd}" in
  install) install_service ;;
  uninstall) uninstall_service ;;
  status) status_service ;;
  run-once) run_once ;;
  clean) clean_logs ;;
  uninstall-old) uninstall_old ;;
  ""|-h|--help|help) usage ;;
  *) echo "Unknown command: ${cmd}" >&2; usage; exit 2 ;;
esac
