#!/usr/bin/env bash

# XNOR_AEB_BOOT_GUARD_START
if [ -x /data/openpilot/tools/xnor_aeb_boot_guard.py ]; then
  mkdir -p /data/openpilot/aeb_boot_guard
  if ! pgrep -f "xnor_aeb_boot_guard.py --duration 300" >/dev/null 2>&1; then
    (
      cd /data/openpilot
      nohup python3 tools/xnor_aeb_boot_guard.py --duration 300 --output-dir /data/openpilot/aeb_boot_guard \
        >/data/openpilot/aeb_boot_guard/boot_guard_launcher.log 2>&1 &
    )
  fi
fi
# XNOR_AEB_BOOT_GUARD_END


# XNOR_AEB_BOOT_WATCH_START
# XNOR_DISABLED_BY_V115 if [ -x /data/openpilot/tools/xnor_aeb_boot_watch.py ]; then
# XNOR_DISABLED_BY_V115   mkdir -p /data/openpilot/aeb_boot_watch
# XNOR_DISABLED_BY_V115   if ! pgrep -f "xnor_aeb_boot_watch.py --duration 240" >/dev/null 2>&1; then
    (
      cd /data/openpilot
# XNOR_DISABLED_BY_V115       nohup python3 tools/xnor_aeb_boot_watch.py --duration 240 --output-dir /data/openpilot/aeb_boot_watch \
# XNOR_DISABLED_BY_V115         >/data/openpilot/aeb_boot_watch/boot_hook_launcher.log 2>&1 &
    )
  fi
fi
# XNOR_AEB_BOOT_WATCH_END


exec ./launch_chffrplus.sh
