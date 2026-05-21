# xnor AEB boot watcher

Captures CAN traffic and openpilot state during the boot window where the Tesla HUD briefly shows:

```text
Automatic Emergency Braking unavailable
```

It does not modify driving behaviour.

## One-shot manual capture

```bash
cd /data/openpilot
python3 tools/xnor_aeb_boot_watch.py --duration 180 --stdout
```

Outputs:

```text
/data/openpilot/aeb_boot_watch/xnor_aeb_boot_watch_*.txt
/data/openpilot/aeb_boot_watch/xnor_aeb_boot_watch_*.jsonl
```

## Automatic capture at boot, read-only `/etc` safe

On comma, `/etc/systemd/system` may be read-only. Use the launch hook instead of the systemd service:

```bash
cd /data/openpilot
chmod +x tools/xnor_aeb_boot_watch_launch_hook.sh
tools/xnor_aeb_boot_watch_launch_hook.sh install
```

Then reboot or power-cycle. The watcher will start from `/data/openpilot/launch_openpilot.sh`.

Check:

```bash
tools/xnor_aeb_boot_watch_launch_hook.sh status
ls -lt /data/openpilot/aeb_boot_watch | head
```

Remove when finished:

```bash
tools/xnor_aeb_boot_watch_launch_hook.sh uninstall
```

## Manual background capture

```bash
cd /data/openpilot
tools/xnor_aeb_boot_watch_launch_hook.sh start-bg
tools/xnor_aeb_boot_watch_launch_hook.sh status
```

## Systemd option

Only use this if your root filesystem is writable:

```bash
tools/xnor_aeb_boot_watch_service.sh install
```

Useful clues are CAN gaps, panda `controlsAllowed` / `faultStatus`, controls alerts, manager process startup timing, and CAN addresses that start/change when the HUD warning disappears.
