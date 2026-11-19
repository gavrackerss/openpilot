#!/usr/bin/env bash
set -euo pipefail
# Arm for one bounded on-road shadow-only observation (15 mins maximum).
# No modeld, controls, CAN, Panda or phone configuration is changed.
mkdir -p /data/jetlink_xnor
rm -f /data/jetlink_xnor/ONROAD_SHADOW_ENABLED
: > /data/jetlink_xnor/ONROAD_SHADOW_ENABLED
printf 'XNOR on-road SHADOW armed for 2 hours or the next 15-minute on-road session.\n'
printf 'Observe status: cat /data/jetlink_shadow_onroad/status.json\n'
printf 'Stop: rm -f /data/jetlink_xnor/ONROAD_SHADOW_ENABLED\n'
