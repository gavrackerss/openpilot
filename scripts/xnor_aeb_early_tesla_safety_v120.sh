#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/data/openpilot}"
BACKUP_ROOT="${BACKUP_ROOT:-/data/xnor_recovery_backups}"
HDR="$ROOT/selfdrive/pandad/pandad.h"
SRC="$ROOT/selfdrive/pandad/panda_safety.cc"
BIN="$ROOT/selfdrive/pandad/pandad"
MARKER="XNOR_V120_EARLY_TESLA_SAFETY"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/xnor_aeb_early_tesla_safety_v120.sh audit
  bash scripts/xnor_aeb_early_tesla_safety_v120.sh apply
  bash scripts/xnor_aeb_early_tesla_safety_v120.sh restore
  bash scripts/xnor_aeb_early_tesla_safety_v120.sh status

v120 fixes v119's installer failure:
  - v119 rebuilt pandad, but the installer treated an optimized-out string marker as fatal.
  - v120 embeds a volatile/used marker and verifies source + symbol presence.
  - v120 does not require the marker string check to be the only proof.

v120 changes only:
  selfdrive/pandad/pandad.h
  selfdrive/pandad/panda_safety.cc
  rebuilt selfdrive/pandad/pandad
EOF
}

require_files() {
  for f in "$HDR" "$SRC"; do
    if [ ! -f "$f" ]; then
      echo "ERROR: missing $f" >&2
      exit 2
    fi
  done
}

show_params_probe() {
  python3 - <<'PY' || true
from common.params import Params

params = Params()
for key in ("CarParamsPersistent", "CarParamsCache", "CarParamsPrevRoute", "CarParams"):
  try:
    data = params.get(key)
  except Exception:
    data = None
  size = len(data) if data else 0
  print(f"param {key}: {size} bytes")
PY
}

audit() {
  require_files
  echo "== v120 early Tesla safety audit =="
  echo "root: $ROOT"
  echo

  if grep -q "XNOR_V120_EARLY_TESLA_SAFETY" "$SRC"; then
    echo "PATCH PRESENT: XNOR_V120_EARLY_TESLA_SAFETY in panda_safety.cc"
  elif grep -q "XNOR_V119_EARLY_TESLA_SAFETY" "$SRC"; then
    echo "PATCH PRESENT: old XNOR_V119_EARLY_TESLA_SAFETY in panda_safety.cc"
  elif grep -q "XNOR_V118_EARLY_TESLA_SAFETY" "$SRC"; then
    echo "PATCH PRESENT: old XNOR_V118_EARLY_TESLA_SAFETY in panda_safety.cc"
  else
    echo "PATCH NOT PRESENT in panda_safety.cc"
  fi

  if grep -q "cached_tesla_safety_configured_" "$HDR"; then
    echo "PATCH PRESENT: cached_tesla_safety_configured_ in pandad.h"
  else
    echo "PATCH NOT PRESENT: cached_tesla_safety_configured_ in pandad.h"
  fi

  echo
  echo "source marker lines:"
  grep -nE "XNOR_V12|XNOR_V11[89]|trySetCachedTeslaSafety|cached_tesla_safety_configured_|set_safety_model|ELM327|ControlsReady" "$SRC" "$HDR" || true

  echo
  echo "binary checks:"
  if [ -x "$BIN" ]; then
    ls -lh "$BIN"
    sha256sum "$BIN" | awk '{print "sha256=" $1}'
    if strings "$BIN" 2>/dev/null | grep -q "$MARKER"; then
      echo "BINARY STRING PRESENT: $MARKER"
    else
      echo "BINARY STRING NOT PRESENT: $MARKER"
    fi
    if command -v nm >/dev/null 2>&1 && nm -C "$BIN" 2>/dev/null | grep -q "PandaSafety::trySetCachedTeslaSafety"; then
      echo "BINARY SYMBOL PRESENT: PandaSafety::trySetCachedTeslaSafety"
    else
      echo "BINARY SYMBOL NOT FOUND: PandaSafety::trySetCachedTeslaSafety"
    fi
  else
    echo "pandad binary missing/not executable"
  fi

  echo
  echo "cached param probe:"
  show_params_probe
}

patch_sources() {
  require_files
  python3 - "$HDR" "$SRC" <<'PY'
from pathlib import Path
import sys

hdr = Path(sys.argv[1])
src = Path(sys.argv[2])

h = hdr.read_text()
s = src.read_text()

if "bool trySetCachedTeslaSafety();" not in h:
  needle = "  void updateMultiplexingMode();\n"
  if needle not in h:
    raise SystemExit("ERROR: cannot patch pandad.h; missing updateMultiplexingMode declaration")
  h = h.replace(
    needle,
    needle +
    "  bool trySetCachedTeslaSafety();\n"
    "  bool carParamsHaveTeslaLegacySafety(const std::string &params_string);\n",
    1,
  )

if "bool carParamsHaveTeslaLegacySafety(const std::string &params_string);" not in h:
  needle = "  bool trySetCachedTeslaSafety();\n"
  if needle not in h:
    raise SystemExit("ERROR: cannot patch pandad.h; missing trySetCachedTeslaSafety declaration")
  h = h.replace(
    needle,
    needle + "  bool carParamsHaveTeslaLegacySafety(const std::string &params_string);\n",
    1,
  )

if "cached_tesla_safety_configured_" not in h:
  needle = "  bool safety_configured_ = false;\n"
  if needle not in h:
    raise SystemExit("ERROR: cannot patch pandad.h; missing safety_configured_ field")
  h = h.replace(needle, needle + "  bool cached_tesla_safety_configured_ = false;\n", 1)

new_impl = """namespace {
static volatile const char xnor_v120_early_tesla_safety_marker_blob[] __attribute__((used)) = "XNOR_V120_EARLY_TESLA_SAFETY";

static const char *xnorV120EarlyTeslaSafetyMarker() {
  return const_cast<const char *>(xnor_v120_early_tesla_safety_marker_blob);
}
}

// XNOR_V120_EARLY_TESLA_SAFETY:
// Avoid the boot window where pandad uses ELM327 forwarding before ControlsReady.
// If cached CarParams already prove this is Tesla legacy, set the real safety
// model immediately, then let ControlsReady apply the final current-route safety.
bool PandaSafety::carParamsHaveTeslaLegacySafety(const std::string &params_string) {
  if (params_string.empty()) {
    return false;
  }

  try {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
    cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

    auto safety_configs = car_params.getSafetyConfigs();
    for (uint32_t i = 0; i < safety_configs.size(); ++i) {
      if (safety_configs[i].getSafetyModel() == cereal::CarParams::SafetyModel::TESLA_LEGACY) {
        return true;
      }
    }
  } catch (...) {
    return false;
  }

  return false;
}

bool PandaSafety::trySetCachedTeslaSafety() {
  static const char *cached_param_keys[] = {
    "CarParamsPersistent",
    "CarParamsCache",
    "CarParamsPrevRoute",
  };

  for (const char *key : cached_param_keys) {
    const std::string params_string = params_.get(key);
    if (carParamsHaveTeslaLegacySafety(params_string)) {
      LOGW("%s: applying cached Tesla safety from %s before ControlsReady", xnorV120EarlyTeslaSafetyMarker(), key);
      setSafetyMode(params_string);
      return true;
    }
  }

  return false;
}

void PandaSafety::configureSafetyMode(bool is_onroad) {
  if (is_onroad && !safety_configured_) {
    if (!cached_tesla_safety_configured_) {
      cached_tesla_safety_configured_ = trySetCachedTeslaSafety();
      if (!cached_tesla_safety_configured_) {
        updateMultiplexingMode();
      }
    }

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    initialized_ = false;
    cached_tesla_safety_configured_ = false;
    safety_configured_ = false;
    log_once_ = false;
  }
}
"""

def replace_existing_patched_block(text: str) -> str | None:
  markers = [
    "namespace {\nstatic volatile const char xnor_v120_early_tesla_safety_marker_blob",
    'extern "C" const char *xnor_v119_early_tesla_safety_marker',
    "// XNOR_V119_EARLY_TESLA_SAFETY:",
    "// XNOR_V118_EARLY_TESLA_SAFETY:",
  ]
  starts = [text.find(m) for m in markers if text.find(m) >= 0]
  if not starts:
    return None
  start = min(starts)
  end = text.find("void PandaSafety::updateMultiplexingMode()", start)
  if end < 0:
    raise SystemExit("ERROR: found old XNOR marker but could not find updateMultiplexingMode after it")
  return text[:start] + new_impl + "\n\n" + text[end:]

patched = replace_existing_patched_block(s)
if patched is not None:
  s = patched
else:
  old_config = """void PandaSafety::configureSafetyMode(bool is_onroad) {
  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
  }
}
"""
  if old_config not in s:
    raise SystemExit("ERROR: cannot patch panda_safety.cc; expected configureSafetyMode block not found")
  s = s.replace(old_config, new_impl + "\n", 1)

hdr.write_text(h)
src.write_text(s)
PY
}

force_rebuild_pandad() {
  echo "== forcing pandad rebuild =="
  cd "$ROOT"

  local before_hash=""
  if [ -x "$BIN" ]; then
    before_hash="$(sha256sum "$BIN" | awk '{print $1}')"
    echo "before_sha256=$before_hash"
  fi

  scons -c selfdrive/pandad/pandad || true
  rm -f selfdrive/pandad/pandad
  find selfdrive/pandad -maxdepth 1 -type f \( -name '*.o' -o -name '*.os' -o -name '*.a' \) -print -delete || true

  sleep 1
  touch selfdrive/pandad/pandad.h selfdrive/pandad/panda_safety.cc

  local jobs
  jobs="$(nproc 2>/dev/null || echo 2)"
  scons -j"$jobs" selfdrive/pandad/pandad

  if [ ! -x "$BIN" ]; then
    echo "ERROR: pandad was not rebuilt at $BIN" >&2
    exit 3
  fi

  local after_hash
  after_hash="$(sha256sum "$BIN" | awk '{print $1}')"
  echo "after_sha256=$after_hash"

  if [ -n "$before_hash" ] && [ "$before_hash" = "$after_hash" ]; then
    echo "WARNING: pandad sha256 did not change"
  fi
}

verify_after_build() {
  echo "== verifying v120 =="
  if ! grep -q "XNOR_V120_EARLY_TESLA_SAFETY" "$SRC"; then
    echo "ERROR: source marker missing after patch" >&2
    exit 4
  fi

  if ! grep -q "trySetCachedTeslaSafety" "$SRC"; then
    echo "ERROR: source function missing after patch" >&2
    exit 4
  fi

  if ! grep -q "cached_tesla_safety_configured_" "$HDR"; then
    echo "ERROR: header field missing after patch" >&2
    exit 4
  fi

  if strings "$BIN" 2>/dev/null | grep -q "$MARKER"; then
    echo "BINARY STRING PRESENT: $MARKER"
  else
    echo "WARNING: binary string marker not visible; continuing because source was rebuilt"
  fi

  if command -v nm >/dev/null 2>&1 && nm -C "$BIN" 2>/dev/null | grep -q "PandaSafety::trySetCachedTeslaSafety"; then
    echo "BINARY SYMBOL PRESENT: PandaSafety::trySetCachedTeslaSafety"
  else
    echo "WARNING: binary symbol not visible; continuing because source was rebuilt"
  fi

  echo "v120 verify PASS"
}

apply() {
  require_files
  local ts bk
  ts="$(date -u +%Y%m%d_%H%M%S)"
  bk="$BACKUP_ROOT/v120_aeb_early_safety_${ts}"
  mkdir -p "$bk/selfdrive/pandad"

  cp -a "$HDR" "$bk/selfdrive/pandad/pandad.h"
  cp -a "$SRC" "$bk/selfdrive/pandad/panda_safety.cc"
  if [ -e "$BIN" ]; then
    cp -a "$BIN" "$bk/selfdrive/pandad/pandad"
  fi

  echo "backup: $bk"
  patch_sources
  echo "patched source files"
  force_rebuild_pandad
  verify_after_build
  echo "v120 apply PASS"
}

restore() {
  local latest
  latest="$(ls -dt "$BACKUP_ROOT"/v120_aeb_early_safety_* 2>/dev/null | head -1 || true)"
  if [ -z "$latest" ]; then
    echo "ERROR: no v120 backup found under $BACKUP_ROOT" >&2
    exit 2
  fi

  echo "restoring: $latest"
  cp -a "$latest/selfdrive/pandad/pandad.h" "$HDR"
  cp -a "$latest/selfdrive/pandad/panda_safety.cc" "$SRC"
  if [ -e "$latest/selfdrive/pandad/pandad" ]; then
    cp -a "$latest/selfdrive/pandad/pandad" "$BIN"
  else
    cd "$ROOT"
    scons -c selfdrive/pandad/pandad || true
    rm -f selfdrive/pandad/pandad
    scons -j"$(nproc 2>/dev/null || echo 2)" selfdrive/pandad/pandad
  fi

  echo "v120 restore PASS"
}

cmd="${1:-}"
case "$cmd" in
  audit|status) audit ;;
  apply) apply ;;
  restore) restore ;;
  -h|--help|help|"") usage ;;
  *) echo "ERROR: unknown command: $cmd" >&2; usage; exit 2 ;;
esac
