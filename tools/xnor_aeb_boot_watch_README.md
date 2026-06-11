# XNOR AEB boot watcher v3

This captures the early boot window automatically so the Tesla HUD `Automatic Emergency Braking unavailable` flash is not missed.

It does **not** change driving behaviour and raw CAN logging is **off by default**.

## Install automatic boot capture

From a copied/unzipped bundle on the comma/openpilot device:

```bash
cd /data/openpilot
chmod +x tools/xnor_aeb_boot_watch.py tools/xnor_aeb_boot_watch_launch_hook.sh
tools/xnor_aeb_boot_watch_launch_hook.sh install
```

Then reboot / power-cycle the device/car.

The watcher is inserted near the top of `/data/openpilot/launch_openpilot.sh`, so it starts before the rest of openpilot has fully come up.

Outputs are written here:

```text
/data/openpilot/aeb_boot_watch/xnor_aeb_boot_watch_v3_*.txt
/data/openpilot/aeb_boot_watch/xnor_aeb_boot_watch_v3_*.jsonl
/data/openpilot/aeb_boot_watch/aeb_boot_watch_auto_launcher.log
```

## Check it is installed/running

```bash
cd /data/openpilot
tools/xnor_aeb_boot_watch_launch_hook.sh status
```

## Remove the auto-start hook

```bash
cd /data/openpilot
tools/xnor_aeb_boot_watch_launch_hook.sh uninstall
```

If you need to restore the original launch script from the automatic backup:

```bash
tools/xnor_aeb_boot_watch_launch_hook.sh restore-backup
```

## Duration

Default capture duration is 360 seconds. To change it during install:

```bash
XNOR_AEB_BOOT_DURATION=600 tools/xnor_aeb_boot_watch_launch_hook.sh install
```

## Do not commit captures

The capture folder should stay local. Add this to `.gitignore` if it is not already present:

```gitignore
/aeb_boot_watch/
*xnor_aeb_boot_watch_*.jsonl
*xnor_aeb_boot_watch_*.jsonl.gz
*xnor_aeb_boot_watch_*.txt
*aeb_boot_watch_*launcher.log
```
