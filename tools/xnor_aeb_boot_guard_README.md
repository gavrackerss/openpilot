# xnor AEB boot guard v1

This is a separate diagnostic/guard helper for the Tesla HUD message:

```text
Automatic Emergency Braking unavailable
```

The earlier watcher showed the warning lining up with Panda safety state transitions such as:

```text
teslaLegacy -> noOutput -> elm327 -> teslaLegacy
```

This helper watches Panda safety state during boot and publishes guard params so other xnor code can later choose to avoid custom actions until Panda is stable.

## Files

```text
openpilot/tools/xnor_aeb_boot_guard.py
openpilot/tools/xnor_aeb_boot_guard_launch_hook.sh
openpilot/tools/xnor_aeb_boot_guard_README.md
```

## Apply

```bash
cd /data
unzip -o /mnt/data/xnor_aeb_boot_guard_v1.zip

cd /data/openpilot
chmod +x tools/xnor_aeb_boot_guard.py tools/xnor_aeb_boot_guard_launch_hook.sh
```

## Install launch hook

This does not write to `/etc`, so it is safe for the read-only root filesystem.

```bash
cd /data/openpilot
tools/xnor_aeb_boot_guard_launch_hook.sh install
```

Then reboot or power-cycle and reproduce the AEB warning.

## Check status/logs

```bash
cd /data/openpilot
tools/xnor_aeb_boot_guard_launch_hook.sh status
tools/xnor_aeb_boot_guard_launch_hook.sh logs
```

Outputs:

```text
/data/openpilot/aeb_boot_guard/xnor_aeb_boot_guard_*.txt
/data/openpilot/aeb_boot_guard/xnor_aeb_boot_guard_*.jsonl
```

Send the newest `.txt` and `.jsonl` after a boot where the warning appears.

## Params written

```text
XnorAebBootGuardActive
XnorAebBootGuardReady
XnorAebBootGuardState
```

Meaning:

```text
Active=1: Panda safety is not yet stable, or a bad transient state is being observed.
Ready=1: Panda safety has been stable as teslaLegacy for the configured stable window.
```

## Recovery mode

Default install is log/param-only. It does **not** kill or restart boardd.

Manual recovery mode is available:

```bash
cd /data/openpilot
tools/xnor_aeb_boot_guard_launch_hook.sh start-bg-recover
```

That starts the guard with:

```text
--recover-after 12
```

Recovery is gated to:

```text
vEgo <= 0.5 m/s
gas not pressed
brake not pressed
controls not enabled
controls not active
```

Then it runs:

```text
pkill -f boardd
```

so manager can restart boardd and reapply Tesla safety. Use this only for testing if the warning gets stuck rather than clearing.

## Remove

```bash
cd /data/openpilot
tools/xnor_aeb_boot_guard_launch_hook.sh uninstall
```
