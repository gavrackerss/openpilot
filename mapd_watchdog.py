#!/usr/bin/env python3
"""mapd liveness watchdog.

Problem: sometimes mapd boots but never publishes `mapdOut`. systemd `Restart=always`
does NOT help -- the process is alive, it just isn't producing data -- so a process-level
restart can't detect it. This watchdog watches the *cereal* stream and restarts mapd only
when it has actually stopped publishing.

Restart sequence mirrors the manual one: stop the transient unit (so its own Restart=always
doesn't race us), kill strays, clear failed state, then re-launch under systemd-run with
Restart=always. Needs root (systemctl/systemd-run/pkill) and the openpilot env.

Run it from /data/openpilot. Tunables via env: MAPD_STALE_S, MAPD_BOOT_GRACE_S,
MAPD_RESTART_GRACE_S.
"""
import os
import sys
import time
import subprocess

sys.path.insert(0, "/data/openpilot")

STALE_S = float(os.environ.get("MAPD_STALE_S", "15"))          # no mapdOut for this long => dead
BOOT_GRACE_S = float(os.environ.get("MAPD_BOOT_GRACE_S", "30"))  # ignore the first N s after watchdog start
RESTART_GRACE_S = float(os.environ.get("MAPD_RESTART_GRACE_S", "25"))  # wait after a restart before judging again
UNIT = "mapd-manual"
# The watchdog itself runs as the normal openpilot user (so cereal imports work); only the
# privileged restart actions are elevated. Set MAPD_SUDO=0 if the watchdog already runs as root.
SUDO = [] if os.environ.get("MAPD_SUDO", "1") == "0" else ["sudo", "-n"]


def log(msg: str) -> None:
  print(f"[mapd_watchdog] {time.strftime('%H:%M:%S')} {msg}", flush=True)


def restart_mapd() -> None:
  log("mapdOut stale -> restarting mapd")
  # Stop the unit first so its Restart=always doesn't respawn the stuck process under us.
  subprocess.run(SUDO + ["systemctl", "stop", UNIT], check=False)
  subprocess.run(SUDO + ["pkill", "-f", "./selfdrive/mapd"], check=False)
  subprocess.run(SUDO + ["systemctl", "reset-failed", UNIT], check=False)
  time.sleep(1.0)
  subprocess.run(
    SUDO + [
      "systemd-run",
      f"--unit={UNIT}",
      "--description=manual openpilot mapd (watchdog)",
      "--working-directory=/data/openpilot",
      "--property=Restart=always",
      "--property=RestartSec=2",
      "/bin/bash", "-lc", "cd /data/openpilot && exec ./selfdrive/mapd",
    ],
    check=False,
  )


def main() -> int:
  try:
    import cereal.messaging as messaging
  except Exception as exc:
    log(f"cereal unavailable ({exc!r}); is this running from /data/openpilot? exiting")
    return 1

  sm = messaging.SubMaster(["mapdOut"])
  start = time.monotonic()
  last_alive = start
  last_restart = 0.0
  log(f"started (stale={STALE_S}s boot_grace={BOOT_GRACE_S}s restart_grace={RESTART_GRACE_S}s)")

  while True:
    sm.update(2000)  # block up to 2 s for a fresh mapdOut
    now = time.monotonic()
    if sm.updated["mapdOut"]:
      last_alive = now

    if now - start < BOOT_GRACE_S:
      continue
    if now - last_restart < RESTART_GRACE_S:
      continue
    if now - last_alive >= STALE_S:
      restart_mapd()
      last_restart = now
      last_alive = now  # give the fresh process time before re-judging


if __name__ == "__main__":
  raise SystemExit(main())
