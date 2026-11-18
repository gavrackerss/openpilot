#!/usr/bin/env bash
set -euo pipefail
cd /data/openpilot
export PYTHONPATH=/data/openpilot${PYTHONPATH:+:$PYTHONPATH}
exec python3 -m openpilot.jetlink_shadow_bench "$@"
