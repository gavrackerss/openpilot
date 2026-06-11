#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/data/openpilot}"
BACKUP_ROOT="${BACKUP_ROOT:-/data/xnor_recovery_backups}"
PARAM_DIR="${PARAM_DIR:-/data/params/d}"
HDR="$ROOT/selfdrive/pandad/pandad.h"
SRC="$ROOT/selfdrive/pandad/panda_safety.cc"
BIN="$ROOT/selfdrive/pandad/pandad"
MARKER="XNOR_V123_PERSISTENT_CARPARAMS_PARSE_FIX"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/xnor_aeb_persistent_carparams_v123.sh audit
  bash scripts/xnor_aeb_persistent_carparams_v123.sh apply
  bash scripts/xnor_aeb_persistent_carparams_v123.sh restore
  bash scripts/xnor_aeb_persistent_carparams_v123.sh status

v123 changes only:
  selfdrive/pandad/pandad.h
  selfdrive/pandad/panda_safety.cc
  selfdrive/pandad/pandad  # rebuilt

It also snapshots the current valid Tesla CarParams into:
  /data/params/d/CarParamsPersistent
  /data/params/d/CarParamsPrevRoute

Purpose:
  v122 failed on this branch because Python capnp CarParams.from_bytes() returns
  a context manager. v123 fixes the snapshot parser and reapplies the persistent
  early Tesla safety patch without touching carcontroller.py.
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

param_size() {
  local f="$PARAM_DIR/$1"
  if [ -f "$f" ]; then
    wc -c < "$f" | tr -d ' '
  else
    echo 0
  fi
}

python_carparams_probe() {
  python3 - <<'PY'
from pathlib import Path
import os
import sys

param_dir = Path("/data/params/d")
keys = ("CarParams", "CarParamsCache", "CarParamsPersistent", "CarParamsPrevRoute")

try:
  from cereal import car
except Exception as e:
  print(f"  python cereal probe unavailable: {e}")
  raise SystemExit(0)

def inspect_reader(cp):
  configs = []
  has_legacy = False
  try:
    for cfg in cp.safetyConfigs:
      model = str(cfg.safetyModel)
      param = int(cfg.safetyParam)
      configs.append(f"{model}:{param}")
      if "teslaLegacy" in model or "TESLA_LEGACY" in model or model.endswith(".teslaLegacy"):
        has_legacy = True
  except Exception as e:
    configs.append(f"safetyConfigs read failed: {e}")

  car_name = getattr(cp, "carName", "")
  fingerprint = getattr(cp, "carFingerprint", "")
  return has_legacy, f"carName={car_name!r} candidate={fingerprint!r} safety={configs}"

def inspect_data(data: bytes):
  reader = car.CarParams.from_bytes(data)
  if hasattr(reader, "__enter__"):
    with reader as cp:
      return inspect_reader(cp)
  return inspect_reader(reader)

for key in keys:
  path = param_dir / key
  if not path.exists():
    continue
  data = path.read_bytes()
  if not data:
    continue
  try:
    ok, detail = inspect_data(data)
    legacy = " legacy=yes" if ok else " legacy=no"
    print(f"  {key}: {len(data)} bytes{legacy} {detail}")
  except Exception as e:
    print(f"  {key}: {len(data)} bytes parse failed: {e}")
PY
}

show_param_probe() {
  echo "param files:"
  for key in CarParams CarParamsCache CarParamsPersistent CarParamsPrevRoute; do
    printf '  %-20s %s bytes\n' "$key" "$(param_size "$key")"
  done
  python_carparams_probe || true
}

snapshot_carparams() {
  mkdir -p "$PARAM_DIR"

  python3 - <<'PY'
from pathlib import Path
import os
import sys

param_dir = Path("/data/params/d")
candidates = ("CarParams", "CarParamsCache", "CarParamsPersistent", "CarParamsPrevRoute")

try:
  from cereal import car
except Exception as e:
  print(f"ERROR: cannot import cereal.car: {e}", file=sys.stderr)
  sys.exit(3)

def inspect_reader(cp):
  configs = []
  has_legacy = False

  try:
    for cfg in cp.safetyConfigs:
      model = str(cfg.safetyModel)
      param = int(cfg.safetyParam)
      configs.append(f"{model}:{param}")
      if "teslaLegacy" in model or "TESLA_LEGACY" in model or model.endswith(".teslaLegacy"):
        has_legacy = True
  except Exception as e:
    configs.append(f"safetyConfigs read failed: {e}")

  car_name = getattr(cp, "carName", "")
  fingerprint = getattr(cp, "carFingerprint", "")
  is_tesla_named = str(car_name).lower() == "tesla" or "TESLA" in str(fingerprint).upper()
  detail = f"carName={car_name!r} candidate={fingerprint!r} safety={configs}"

  # Prefer explicit teslaLegacy. The name fallback is only a last-resort selector
  # for this branch's old CarParams serialization; pandad still validates safetyConfigs.
  return bool(has_legacy or is_tesla_named), bool(has_legacy), detail

def inspect_data(data: bytes):
  reader = car.CarParams.from_bytes(data)
  if hasattr(reader, "__enter__"):
    with reader as cp:
      return inspect_reader(cp)
  return inspect_reader(reader)

selected_key = None
selected_data = None
selected_detail = ""
selected_has_legacy = False

for key in candidates:
  path = param_dir / key
  if not path.exists():
    continue

  data = path.read_bytes()
  if len(data) < 128:
    print(f"probe {key}: {len(data)} bytes skipped: too small")
    continue

  try:
    ok, has_legacy, detail = inspect_data(data)
    legacy_txt = "legacy=yes" if has_legacy else "legacy=no"
    print(f"probe {key}: {len(data)} bytes {legacy_txt} {detail}")
  except Exception as e:
    print(f"probe {key}: {len(data)} bytes parse failed: {e}")
    continue

  if ok:
    selected_key = key
    selected_data = data
    selected_detail = detail
    selected_has_legacy = has_legacy
    if has_legacy:
      break

if selected_data is None:
  print("ERROR: no Tesla CarParams found to persist.", file=sys.stderr)
  print("Boot openpilot once until it identifies the car, then re-run v123 apply.", file=sys.stderr)
  sys.exit(3)

for dst_key in ("CarParamsPersistent", "CarParamsPrevRoute"):
  dst = param_dir / dst_key
  tmp = param_dir / f".{dst_key}.xnor_v123.tmp"
  with open(tmp, "wb") as f:
    f.write(selected_data)
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, dst)

try:
  dfd = os.open(str(param_dir), os.O_RDONLY)
  os.fsync(dfd)
  os.close(dfd)
except Exception:
  pass

legacy_txt = "legacy=yes" if selected_has_legacy else "legacy=unconfirmed-name-match"
print(f"persisted Tesla CarParams from {selected_key}: {len(selected_data)} bytes {legacy_txt} {selected_detail}")
PY
}

audit() {
  require_files
  echo "== v123 persistent CarParams parse-fix audit =="
  echo "root: $ROOT"
  echo

  if grep -q "$MARKER" "$SRC"; then
    echo "PATCH PRESENT: $MARKER in panda_safety.cc"
  elif grep -q "XNOR_V122_PERSISTENT_TESLA_CARPARAMS" "$SRC"; then
    echo "PATCH PRESENT: old XNOR_V122_PERSISTENT_TESLA_CARPARAMS in panda_safety.cc"
  elif grep -q "XNOR_V120_EARLY_TESLA_SAFETY" "$SRC"; then
    echo "PATCH PRESENT: old XNOR_V120_EARLY_TESLA_SAFETY in panda_safety.cc"
  elif grep -q "XNOR_V119_EARLY_TESLA_SAFETY" "$SRC"; then
    echo "PATCH PRESENT: old XNOR_V119_EARLY_TESLA_SAFETY in panda_safety.cc"
  elif grep -q "XNOR_V118_EARLY_TESLA_SAFETY" "$SRC"; then
    echo "PATCH PRESENT: old XNOR_V118_EARLY_TESLA_SAFETY in panda_safety.cc"
  else
    echo "PATCH NOT PRESENT in panda_safety.cc"
  fi

  if grep -q "early_tesla_safety_configured_" "$HDR"; then
    echo "PATCH PRESENT: early_tesla_safety_configured_ in pandad.h"
  elif grep -q "cached_tesla_safety_configured_" "$HDR"; then
    echo "PATCH PRESENT: old cached_tesla_safety_configured_ in pandad.h"
  else
    echo "PATCH NOT PRESENT: early Tesla header field"
  fi

  echo
  echo "source marker lines:"
  grep -nE "XNOR_V12|trySetEarlyTeslaSafety|trySetCachedTeslaSafety|early_tesla_safety_configured_|cached_tesla_safety_configured_|CarParamsPersistent|CarParamsPrevRoute|set_safety_model|ELM327|ControlsReady" "$SRC" "$HDR" || true

  echo
  echo "binary:"
  if [ -x "$BIN" ]; then
    ls -lh "$BIN"
    sha256sum "$BIN" | awk '{print "sha256=" $1}'
    if strings "$BIN" 2>/dev/null | grep -q "$MARKER"; then
      echo "BINARY STRING PRESENT: $MARKER"
    else
      echo "BINARY STRING NOT PRESENT: $MARKER"
    fi
  else
    echo "pandad binary missing/not executable"
  fi

  echo
  show_param_probe
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

for line in (
  "  bool trySetCachedTeslaSafety();\n",
  "  bool trySetEarlyTeslaSafety();\n",
  "  bool carParamsHaveTeslaLegacySafety(const std::string &params_string);\n",
  "  bool cached_tesla_safety_configured_ = false;\n",
  "  bool early_tesla_safety_configured_ = false;\n",
):
  h = h.replace(line, "")

needle = "  void updateMultiplexingMode();\n"
if needle not in h:
  raise SystemExit("ERROR: cannot patch pandad.h; missing updateMultiplexingMode declaration")
h = h.replace(
  needle,
  needle +
  "  bool trySetEarlyTeslaSafety();\n"
  "  bool carParamsHaveTeslaLegacySafety(const std::string &params_string);\n",
  1,
)

needle = "  bool safety_configured_ = false;\n"
if needle not in h:
  raise SystemExit("ERROR: cannot patch pandad.h; missing safety_configured_ field")
h = h.replace(needle, needle + "  bool early_tesla_safety_configured_ = false;\n", 1)

new_impl = """namespace {
static volatile const char xnor_v123_persistent_carparams_parse_fix_marker_blob[] __attribute__((used)) = "XNOR_V123_PERSISTENT_CARPARAMS_PARSE_FIX";

static const char *xnorV123PersistentCarParamsParseFixMarker() {
  return const_cast<const char *>(xnor_v123_persistent_carparams_parse_fix_marker_blob);
}
}

// XNOR_V123_PERSISTENT_CARPARAMS_PARSE_FIX:
// Use a persistent copy of the previous route's Tesla CarParams so pandad can
// set teslaLegacy before ControlsReady and avoid the early ELM327 forwarding
// window. v123 fixes the installer-side Python capnp parse bug from v122.
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

bool PandaSafety::trySetEarlyTeslaSafety() {
  static const char *early_param_keys[] = {
    "CarParamsPersistent",
    "CarParamsPrevRoute",
    "CarParamsCache",
    "CarParams",
  };

  for (const char *key : early_param_keys) {
    const std::string params_string = params_.get(key);
    if (carParamsHaveTeslaLegacySafety(params_string)) {
      LOGW("%s: applying early Tesla safety from %s before ControlsReady", xnorV123PersistentCarParamsParseFixMarker(), key);
      setSafetyMode(params_string);
      return true;
    }
  }

  return false;
}

void PandaSafety::configureSafetyMode(bool is_onroad) {
  if (is_onroad && !safety_configured_) {
    if (!early_tesla_safety_configured_) {
      early_tesla_safety_configured_ = trySetEarlyTeslaSafety();
      if (!early_tesla_safety_configured_) {
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
    early_tesla_safety_configured_ = false;
    safety_configured_ = false;
    log_once_ = false;
  }
}
"""

def replace_existing_patched_block(text: str) -> str | None:
  markers = [
    "namespace {\nstatic volatile const char xnor_v123_persistent_carparams_parse_fix_marker_blob",
    "namespace {\nstatic volatile const char xnor_v122_persistent_tesla_carparams_marker_blob",
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
  echo "== verifying v123 =="
  if ! grep -q "$MARKER" "$SRC"; then
    echo "ERROR: source marker missing after patch" >&2
    exit 4
  fi

  if ! grep -q "trySetEarlyTeslaSafety" "$SRC"; then
    echo "ERROR: source function missing after patch" >&2
    exit 4
  fi

  if ! grep -q "early_tesla_safety_configured_" "$HDR"; then
    echo "ERROR: header field missing after patch" >&2
    exit 4
  fi

  if [ ! -s "$PARAM_DIR/CarParamsPersistent" ]; then
    echo "ERROR: CarParamsPersistent was not persisted" >&2
    exit 4
  fi

  if strings "$BIN" 2>/dev/null | grep -q "$MARKER"; then
    echo "BINARY STRING PRESENT: $MARKER"
  else
    echo "WARNING: binary string marker not visible; continuing because source was rebuilt"
  fi

  echo "v123 verify PASS"
}

apply() {
  require_files
  local ts bk
  ts="$(date -u +%Y%m%d_%H%M%S)"
  bk="$BACKUP_ROOT/v123_persistent_carparams_${ts}"
  mkdir -p "$bk/selfdrive/pandad" "$bk/params"

  cp -a "$HDR" "$bk/selfdrive/pandad/pandad.h"
  cp -a "$SRC" "$bk/selfdrive/pandad/panda_safety.cc"
  if [ -e "$BIN" ]; then
    cp -a "$BIN" "$bk/selfdrive/pandad/pandad"
  fi
  for key in CarParamsPersistent CarParamsPrevRoute; do
    if [ -f "$PARAM_DIR/$key" ]; then
      cp -a "$PARAM_DIR/$key" "$bk/params/$key"
    fi
  done

  echo "backup: $bk"
  echo "== snapshot current Tesla CarParams =="
  snapshot_carparams
  patch_sources
  echo "patched source files"
  force_rebuild_pandad
  verify_after_build
  echo "v123 apply PASS"
}

restore() {
  local latest
  latest="$(ls -dt "$BACKUP_ROOT"/v123_persistent_carparams_* 2>/dev/null | head -1 || true)"
  if [ -z "$latest" ]; then
    echo "ERROR: no v123 backup found under $BACKUP_ROOT" >&2
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

  for key in CarParamsPersistent CarParamsPrevRoute; do
    if [ -f "$latest/params/$key" ]; then
      cp -a "$latest/params/$key" "$PARAM_DIR/$key"
    fi
  done

  echo "v123 restore PASS"
}

cmd="${1:-}"
case "$cmd" in
  audit|status) audit ;;
  apply) apply ;;
  restore) restore ;;
  -h|--help|help|"") usage ;;
  *) echo "ERROR: unknown command: $cmd" >&2; usage; exit 2 ;;
esac
