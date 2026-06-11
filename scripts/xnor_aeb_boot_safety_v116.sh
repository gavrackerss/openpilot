#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/data/openpilot}"
SAFETY_FILE="$ROOT/opendbc_repo/opendbc/safety/modes/tesla_legacy.h"
PATCH_MARKER="XNOR_V116_AEB_BOOT_SCRUB"
BACKUP_ROOT="/data/xnor_recovery_backups"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/xnor_aeb_boot_safety_v116.sh audit
  bash scripts/xnor_aeb_boot_safety_v116.sh apply
  bash scripts/xnor_aeb_boot_safety_v116.sh restore
  bash scripts/xnor_aeb_boot_safety_v116.sh status

v116 only patches:
  opendbc_repo/opendbc/safety/modes/tesla_legacy.h

It does not touch:
  carcontroller.py
  LONG_module.py
  Tesla values/interface/carstate
  launch scripts
EOF
}

require_file() {
  if [ ! -f "$SAFETY_FILE" ]; then
    echo "ERROR: missing $SAFETY_FILE" >&2
    exit 2
  fi
}

audit() {
  require_file
  echo "== v116 safety audit =="
  echo "file: $SAFETY_FILE"
  if grep -q "$PATCH_MARKER" "$SAFETY_FILE"; then
    echo "PATCH PRESENT: $PATCH_MARKER"
  else
    echo "PATCH NOT PRESENT: $PATCH_MARKER"
  fi

  echo
  echo "key snippets:"
  grep -nE "$PATCH_MARKER|stale_boot_warning|boot_warning_scrub|return !real_stock_aeb|vehicle_stopped_or_unknown" "$SAFETY_FILE" || true

  echo
  echo "syntax-sensitive brace count:"
  python3 - "$SAFETY_FILE" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
s = p.read_text(errors="replace")
print("open_braces", s.count("{"), "close_braces", s.count("}"))
if s.count("{") != s.count("}"):
  raise SystemExit("ERROR: brace count mismatch")
PY
}

apply_patch() {
  require_file
  if grep -q "$PATCH_MARKER" "$SAFETY_FILE"; then
    echo "v116 patch already present"
    audit
    return 0
  fi

  local ts
  ts="$(date -u +%Y%m%d_%H%M%S)"
  local bk="$BACKUP_ROOT/v116_aeb_${ts}"
  mkdir -p "$bk"
  cp -a "$SAFETY_FILE" "$bk/tesla_legacy.h"
  echo "backup: $bk/tesla_legacy.h"

  python3 - "$SAFETY_FILE" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text()

insert_after = """static void tesla_legacy_clear_warning_matrix(CANPacket_t *msg) {
  for (int i = 0; i < GET_LEN(msg); i++) {
    msg->data[i] = 0U;
  }
  tesla_legacy_set_last_byte_checksum(msg);
}
"""

helper = insert_after + """

// XNOR_V116_AEB_BOOT_SCRUB: only hide stale boot/standstill AP warning frames.
// Preserve moving stock AEB/FCW visibility when OP is not in control.
static bool tesla_legacy_vehicle_stopped_or_unknown(void) {
  return vehicle_speed.max < 500;  // 0.5 m/s in VEHICLE_SPEED_FACTOR units
}

static bool tesla_legacy_boot_warning_scrub_active(void) {
  return !controls_allowed && tesla_legacy_vehicle_stopped_or_unknown();
}
"""

old_ext = """  if (tesla_legacy_external_panda) {
    if ((bus_num == 2) && (addr == 0x2BF)) {
      if (!controls_allowed) {
        return false;
      }
      return !tesla_legacy_stock_aeb;
    }
    return true;
  }
"""

new_ext = """  if (tesla_legacy_external_panda) {
    if ((bus_num == 2) && (addr == 0x2BF)) {
      const int aeb_event = (int)(to_fwd->data[2] & 0x03U);
      const bool real_stock_aeb = (aeb_event == 1);
      const bool stale_boot_warning = (aeb_event != 0) && !real_stock_aeb;

      if (!controls_allowed) {
        return stale_boot_warning && tesla_legacy_vehicle_stopped_or_unknown();
      }
      return !real_stock_aeb;
    }
    return true;
  }
"""

old_bus2 = """  // bus2 -> bus0: mutate HUD/AP status only while OP is actively controlling.
  // Outside controls_allowed, pass the stock status stream through unchanged so AP/AEB state can recover on standby/reboot.
  if (bus_num == 2) {
    const bool op_hud_owner = tesla_legacy_op_autopilot_disabled &&
                              !tesla_legacy_autopilot_enabled &&
                              !tesla_legacy_eac_enabled &&
                              !tesla_legacy_autopark_enabled;
    if (controls_allowed && op_hud_owner) {
      if (addr == 0x389) {
        tesla_legacy_scrub_status2_warnings(to_fwd);
      } else if (addr == 0x399) {
        tesla_legacy_scrub_status_warnings(to_fwd, 0x05U);
      } else if ((addr == 0x309) || (addr == 0x329) || (addr == 0x349) || (addr == 0x369)) {
        tesla_legacy_clear_warning_matrix(to_fwd);
      } else {
      }
    }

    if (!controls_allowed) {
      tesla_legacy_hide_errors_armed = false;
    }
    return false;
  }
"""

new_bus2 = """  // bus2 -> bus0: mutate HUD/AP status while OP owns the path.
  // XNOR_V116_AEB_BOOT_SCRUB also scrubs stale boot/standstill warnings before OP is active.
  if (bus_num == 2) {
    const bool op_hud_owner = tesla_legacy_op_autopilot_disabled &&
                              !tesla_legacy_autopilot_enabled &&
                              !tesla_legacy_eac_enabled &&
                              !tesla_legacy_autopark_enabled;
    const bool boot_warning_scrub = tesla_legacy_boot_warning_scrub_active();

    if ((controls_allowed && op_hud_owner) || boot_warning_scrub) {
      if (addr == 0x389) {
        tesla_legacy_scrub_status2_warnings(to_fwd);
      } else if (addr == 0x399) {
        const uint8_t status = controls_allowed ? 0x05U : (to_fwd->data[0] & 0x0FU);
        tesla_legacy_scrub_status_warnings(to_fwd, status);
      } else if ((addr == 0x309) || (addr == 0x329) || (addr == 0x349) || (addr == 0x369)) {
        tesla_legacy_clear_warning_matrix(to_fwd);
      } else {
      }
    }

    if (!controls_allowed) {
      tesla_legacy_hide_errors_armed = false;
    }
    return false;
  }
"""

if "XNOR_V116_AEB_BOOT_SCRUB" in text:
  print("already patched")
  raise SystemExit(0)

checks = [
  ("warning matrix helper", insert_after),
  ("external panda 0x2BF forwarding block", old_ext),
  ("bus2 status forwarding block", old_bus2),
]
missing = [name for name, needle in checks if needle not in text]
if missing:
  print("ERROR: expected current xnor-dev safety text was not found:", ", ".join(missing), file=sys.stderr)
  print("Stop here and send this output; no file was changed.", file=sys.stderr)
  raise SystemExit(3)

text = text.replace(insert_after, helper, 1)
text = text.replace(old_ext, new_ext, 1)
text = text.replace(old_bus2, new_bus2, 1)

if text.count("{") != text.count("}"):
  print("ERROR: brace count mismatch after patch; no file was changed", file=sys.stderr)
  raise SystemExit(4)

tmp = path.with_suffix(path.suffix + ".v116_tmp")
tmp.write_text(text)
tmp.replace(path)
print("patched", path)
PY

  audit
  echo
  echo "v116 apply PASS"
}

restore_latest() {
  require_file
  local latest
  latest="$(ls -td "$BACKUP_ROOT"/v116_aeb_* 2>/dev/null | head -1 || true)"
  if [ -z "$latest" ] || [ ! -f "$latest/tesla_legacy.h" ]; then
    echo "ERROR: no v116 backup found under $BACKUP_ROOT/v116_aeb_*" >&2
    exit 2
  fi
  cp -a "$SAFETY_FILE" "$latest/tesla_legacy.h.before_restore.$(date -u +%Y%m%d_%H%M%S)"
  cp -a "$latest/tesla_legacy.h" "$SAFETY_FILE"
  echo "restored from: $latest/tesla_legacy.h"
  audit
}

status() {
  audit
  echo
  echo "watcher processes:"
  pgrep -af 'xnor_boot_raw_can_watch|xnor_aeb_boot_watch|aeb_boot_watch' || true
  echo
  echo "latest boot capture files:"
  ls -lhtr "$ROOT/xnor_aeb_boot_capture" 2>/dev/null | tail -12 || true
}

cmd="${1:-}"
case "$cmd" in
  audit) audit ;;
  apply) apply_patch ;;
  restore) restore_latest ;;
  status) status ;;
  -h|--help|help|"") usage ;;
  *) echo "ERROR: unknown command: $cmd" >&2; usage; exit 2 ;;
esac
