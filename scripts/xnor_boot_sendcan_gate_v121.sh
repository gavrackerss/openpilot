#!/usr/bin/env bash
set -euo pipefail

ROOT="${OPENPILOT_ROOT:-/data/openpilot}"
CC_FILE="$ROOT/opendbc_repo/opendbc/car/tesla/carcontroller.py"
BACKUP_ROOT="${XNOR_BACKUP_ROOT:-/data/xnor_recovery_backups}"
MARKER="XNOR_V121_BOOT_SENDCAN_GATE"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

need_file() {
  [ -f "$CC_FILE" ] || die "missing $CC_FILE"
}

audit() {
  need_file
  echo "== v121 boot sendcan gate audit =="
  echo "root: $ROOT"
  echo "file: $CC_FILE"

  if grep -q "$MARKER" "$CC_FILE"; then
    echo "PATCH PRESENT: $MARKER"
  else
    echo "PATCH NOT PRESENT: $MARKER"
  fi

  echo "marker lines:"
  grep -nE 'XNOR_V121_BOOT_SENDCAN_GATE|XNOR_BOOT_SENDCAN_GUARD_FRAMES|_boot_sendcan_guard_active|_reset_boot_gated_outputs' "$CC_FILE" || true

  echo "syntax check:"
  python3 -m py_compile "$CC_FILE"
  echo "audit PASS"
}

apply_patch() {
  need_file

  TS="$(date -u +%Y%m%d_%H%M%S)"
  BK="$BACKUP_ROOT/v121_boot_sendcan_gate_${TS}"
  mkdir -p "$BK"
  cp -a "$CC_FILE" "$BK/carcontroller.py"
  echo "backup: $BK"

  python3 - "$CC_FILE" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
text = path.read_text(encoding="utf-8")

if "XNOR_V121_BOOT_SENDCAN_GATE" in text:
  print("source already contains XNOR_V121_BOOT_SENDCAN_GATE")
  raise SystemExit(0)

replacements = []

old = 'ROADWORKS_DEFAULT_KPH = 50.0 * CV.MPH_TO_KPH\n'
new = '''ROADWORKS_DEFAULT_KPH = 50.0 * CV.MPH_TO_KPH

# XNOR_V121_BOOT_SENDCAN_GATE:
# Debug guard to avoid Tesla AP/HUD boot-warning feedback before CarState is valid.
XNOR_BOOT_SENDCAN_GUARD_FRAMES = 4500  # 90s at 50 Hz once controlsd starts calling update().
XNOR_BOOT_SENDCAN_MIN_VALID_FRAMES = 25
'''
replacements.append((old, new, "boot guard constants"))

old = '''    self._telemetry_prev_active = False

    self._roadworks_main_pulls_ms: list[int] = []'''
new = '''    self._telemetry_prev_active = False

    self._xnor_boot_gate_valid_frames = 0
    self._xnor_boot_gate_released = False
    self._xnor_boot_gate_last_log_frame = -100000

    self._roadworks_main_pulls_ms: list[int] = []'''
replacements.append((old, new, "boot guard state"))

methods = '''  def _boot_gate_gear_name(self, CS) -> str:
    cs_out = getattr(CS, "out", None)
    for obj, attr in ((cs_out, "gearShifter"), (CS, "gear_shifter"), (CS, "gearShifter")):
      if obj is None:
        continue
      try:
        value = getattr(obj, attr)
      except Exception:
        continue
      if value is not None:
        return str(value).lower()
    return "unknown"

  def _boot_gate_carstate_valid(self, CC, CS) -> bool:
    cs_out = getattr(CS, "out", None)
    if cs_out is None:
      return False

    try:
      v_ego = float(getattr(cs_out, "vEgo", 0.0))
      if not np.isfinite(v_ego):
        return False
    except Exception:
      return False

    gear = self._boot_gate_gear_name(CS)
    if (not gear) or ("unknown" in gear) or ("none" in gear):
      return False

    try:
      if not bool(self.params.get_bool("ControlsReady")):
        return False
    except Exception:
      return False

    return True

  def _reset_boot_gated_outputs(self) -> None:
    self._stw_release_frame = -1
    self._stw_sequence = []
    self._virtual_turn_prev = 0
    self._telemetry_prev_active = False
    self._body_controls_prev_turn = 0

  def _boot_sendcan_guard_active(self, CC, CS) -> bool:
    if bool(getattr(self, "_xnor_boot_gate_released", False)):
      return False

    if self._boot_gate_carstate_valid(CC, CS):
      self._xnor_boot_gate_valid_frames = int(getattr(self, "_xnor_boot_gate_valid_frames", 0)) + 1
    else:
      self._xnor_boot_gate_valid_frames = 0

    frame_ready = int(self.frame) >= int(XNOR_BOOT_SENDCAN_GUARD_FRAMES)
    valid_ready = int(self._xnor_boot_gate_valid_frames) >= int(XNOR_BOOT_SENDCAN_MIN_VALID_FRAMES)

    if frame_ready and valid_ready:
      self._xnor_boot_gate_released = True
      cloudlog.warning(
        "[XNOR_V121_BOOT_SENDCAN_GATE] released frame=%d valid_frames=%d gear=%s",
        int(self.frame),
        int(self._xnor_boot_gate_valid_frames),
        self._boot_gate_gear_name(CS),
      )
      return False

    if (int(self.frame) - int(getattr(self, "_xnor_boot_gate_last_log_frame", -100000))) >= 250:
      self._xnor_boot_gate_last_log_frame = int(self.frame)
      cloudlog.warning(
        "[XNOR_V121_BOOT_SENDCAN_GATE] holding sendcan frame=%d/%d valid_frames=%d/%d gear=%s",
        int(self.frame),
        int(XNOR_BOOT_SENDCAN_GUARD_FRAMES),
        int(self._xnor_boot_gate_valid_frames),
        int(XNOR_BOOT_SENDCAN_MIN_VALID_FRAMES),
        self._boot_gate_gear_name(CS),
      )

    return True

'''
old = '  def _speed_limit_sync(self, CC, CS, can_sends) -> None:\n'
new = methods + old
replacements.append((old, new, "boot guard methods"))

old = '''    self._refresh_cached_params()
    self._emit_internal_0x659(CS, can_sends)

    autopilot_disabled = bool(self._cached_autopilot_disabled)
'''
new = '''    self._refresh_cached_params()

    if self._boot_sendcan_guard_active(CC, CS):
      self._reset_boot_gated_outputs()
      try:
        self.apply_angle_last = float(getattr(CS.out, "steeringAngleDeg", self.apply_angle_last))
      except Exception:
        pass
      new_actuators = actuators.as_builder()
      new_actuators.steeringAngleDeg = float(self.apply_angle_last)
      self.frame += 1
      return new_actuators, can_sends

    self._emit_internal_0x659(CS, can_sends)

    autopilot_disabled = bool(self._cached_autopilot_disabled)
'''
replacements.append((old, new, "early update return"))

for old, new, label in replacements:
  if old not in text:
    raise SystemExit(f"patch anchor not found: {label}")
  text = text.replace(old, new, 1)

path.write_text(text, encoding="utf-8")
print("patched carcontroller.py")
PY

  echo "== verifying syntax =="
  python3 -m py_compile "$CC_FILE"

  echo "== clearing python cache =="
  find "$ROOT/opendbc_repo/opendbc/car/tesla" -type d -name "__pycache__" -prune -exec rm -rf {} + 2>/dev/null || true

  grep -q "$MARKER" "$CC_FILE" || die "marker missing after patch"

  echo "v121 apply PASS"
}

restore_patch() {
  need_file

  latest="$(ls -dt "$BACKUP_ROOT"/v121_boot_sendcan_gate_* 2>/dev/null | head -1 || true)"
  [ -n "$latest" ] || die "no v121 backup found under $BACKUP_ROOT"

  [ -f "$latest/carcontroller.py" ] || die "backup missing carcontroller.py: $latest"

  TS="$(date -u +%Y%m%d_%H%M%S)"
  PRE="$BACKUP_ROOT/v121_restore_pre_${TS}"
  mkdir -p "$PRE"
  cp -a "$CC_FILE" "$PRE/carcontroller.py"

  cp -a "$latest/carcontroller.py" "$CC_FILE"
  python3 -m py_compile "$CC_FILE"
  find "$ROOT/opendbc_repo/opendbc/car/tesla" -type d -name "__pycache__" -prune -exec rm -rf {} + 2>/dev/null || true

  echo "restored from: $latest"
  echo "pre-restore backup: $PRE"
  echo "v121 restore PASS"
}

case "${1:-audit}" in
  audit)
    audit
    ;;
  apply)
    apply_patch
    ;;
  restore|uninstall)
    restore_patch
    ;;
  *)
    echo "usage: $0 {audit|apply|restore}"
    exit 2
    ;;
esac
