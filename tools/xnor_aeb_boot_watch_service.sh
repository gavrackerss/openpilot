#!/usr/bin/env bash
set -euo pipefail
UNIT="xnor-aeb-boot-watch.service"
SRC="/data/openpilot/systemd/${UNIT}"
DST="/etc/systemd/system/${UNIT}"

run_root() {
  if [[ "$(id -u)" -eq 0 ]]; then "$@"; else sudo "$@"; fi
}

case "${1:-}" in
  install)
    chmod +x /data/openpilot/tools/xnor_aeb_boot_watch.py
    run_root cp "${SRC}" "${DST}"
    run_root systemctl daemon-reload
    run_root systemctl enable "${UNIT}"
    run_root systemctl restart "${UNIT}" || true
    ;;
  start) run_root systemctl start "${UNIT}" ;;
  stop) run_root systemctl stop "${UNIT}" || true ;;
  restart) run_root systemctl restart "${UNIT}" ;;
  status) systemctl status "${UNIT}" --no-pager || true ;;
  logs) journalctl -u "${UNIT}" -f ;;
  uninstall)
    run_root systemctl stop "${UNIT}" || true
    run_root systemctl disable "${UNIT}" || true
    run_root rm -f "${DST}"
    run_root systemctl daemon-reload
    run_root systemctl reset-failed "${UNIT}" || true
    ;;
  *) echo "Usage: $0 {install|start|stop|restart|status|logs|uninstall}" >&2; exit 1 ;;
esac
