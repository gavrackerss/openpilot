# XNOR Jetlink — real-world ON-ROAD shadow observation v1

**Exact base**: `gavrackerss/openpilot`, branch `xnor-dev`, preserves `xnorjetlinkd` off-road probe, current stock/SunnyPilot `modeld`, Tesla CAN/Panda/controls, and all model selection logic.

**Phone**: keep the **already installed, signed LS2 Jetlink V0.5 StageTrace Android app**, original compiled full LS2 FP16 model / Tensor G6 / LiteRT 2.3 / burst(5). No APK changes, re-import, Kaggle or SCons needed.

**Comma**: this is a direct replacement of existing V5.6 shadow bench **plus** one small on-road observer service and a manager registration. No `unzip` command is required on the comma. Extract this ZIP on your PC and copy the extracted folder/files onto the comma.

## Purpose and boundaries

- **Real-world** means *live driving conditions*, 2-camera VisionIPC input, the real USB NCM/UDP upload and Tensor G6 inference with measured latency, frame age, drop rates and live output samples.
- The current CPU nearest-neighbour warp is **not proven numerically equivalent** to openpilot's production warp. `desire` and `action_t` inputs remain test values and current recurrent-state handling is **not yet reference-parity validated**. These predictions must NOT drive or publish into vehicle planning/control.
- The process **never replaces stock `modeld`**, does not publish `modelV2`, `drivingModelData`, `cameraOdometry`, write Params, send CAN or command Panda. Existing driving remains with existing authoritative modeld.
- Defaults to *parked-only mode*. Only the explicitly managed `--onroad-shadow` observer bypasses the prior parked check and instead validates live on-road state. An arm marker expires after 2 hours and the observer performs a maximum of 5 × 180-second tests (~15 minutes) per arm.
- It runs at low OS scheduling priority (`nice +10`), no real-time priority. If you notice interference with ordinary driving, **disarm** and stop observing; do not rely on the remote model.

## Install — NO unzip on comma

Extract the archive on a PC, upload its `XNOR_Jetlink_Onroad_v1/` directory to `/data/XNOR_Jetlink_Onroad_v1/` on your comma, then **while parked and disengaged**:

```bash
cd /data/openpilot
python3 /data/XNOR_Jetlink_Onroad_v1/tools/install_into_repo.py /data/openpilot --dry-run
python3 /data/XNOR_Jetlink_Onroad_v1/tools/install_into_repo.py /data/openpilot
python3 -m py_compile system/manager/process_config.py openpilot/jetlink_shadow_bench.py openpilot/jetlink_onroad_shadow.py

git status --short
git diff -- system/manager/process_config.py
```

Installer backs up changed files to `/data/jetlink_shadow_onroad/backups/<timestamp>/` and writes a rollback manifest. There is **no automatic activation**. It refuses to modify an unfamiliar manager, and does **not** install/restart Android.

On a parked/disengaged restart of openpilot/manager, the process table loads the new predicate. Do not forcibly restart manager while moving.

## Activate once and observe a drive

**After manager has reloaded, before starting your test drive:**

```bash
cd /data/openpilot
./tools/enable_jetlink_onroad.sh
```

The marker is valid for the next two hours. When the car transitions on-road, `xnorjetlinkshadowd` starts automatically *alongside stock modeld*, observing 20 Hz road and wide camera data, over the established USB route (UDP 65k sendmsg, TCP responses, depth 2, QUICKACK **off**). It records at most five sequential 180-second segments, with a 15-second bounded retry backoff on a failed segment. Three failures terminate this activation.

**Status:**

```bash
cat /data/jetlink_shadow_onroad/status.json
ls -lt /data/jetlink_shadow_onroad/runs/ | head
```

Each `drive-YYYYMMDD-HHMMSS...` folder may contain `summary.json`, `frames.csv`, `joined_stage_frames.csv`, `phone_stage_report.json`, `output_samples.npz`, plus a sibling `.log`. Transfer one clean segment's five standard files and, if available, any error trace. The observer never alters `modeld` outputs.

**Disarm/stop:**

```bash
cd /data/openpilot
./tools/disable_jetlink_onroad.sh
```

Manager stops the observer; its supervisor terminates only the isolated Jetlink child process. The enable marker is also removed when an observation ends, so it cannot start on a later drive without another deliberate arm.

## Commit the exact files

```bash
cd /data/openpilot
git add system/manager/process_config.py openpilot/jetlink_shadow_bench.py \
  openpilot/jetlink_onroad_shadow.py start_jetlink_shadow.sh \
  tools/enable_jetlink_onroad.sh tools/disable_jetlink_onroad.sh
git diff --cached --stat
```

Optional commit when satisfied: `git commit -m "Add opt-in passive on-road Jetlink Tensor G6 shadow observation"`.

## Rollback

Disarm first. While parked and disengaged, restore the exact backup path printed by the installer:

```bash
python3 /data/XNOR_Jetlink_Onroad_v1/tools/rollback_install.py \
  /data/jetlink_shadow_onroad/backups/<exact-timestamp>
```

Restart manager while parked/disengaged to reload the previous process configuration. Your previously imported Android model is untouched.

## Integration into authoritative modeld (NOT done)

This step collects connected real-world latency and output snapshots only. Comparing predictions against production modeld or adopting remote outputs for any on-road authority requires source warp parity, correct LS2 recurrent-state/desire/action input histories, output schema alignment, and independent failover validation. No vehicle-control changes are authorized by this package.

## Deploy from GitHub branch

This GitHub change adds the manager registration and the full V5.6-onroad runner.
On the comma, **while parked and disengaged**, update from this feature branch,
then restart manager safely to load the new process. The worker remains DISABLED
until you run `./tools/enable_jetlink_onroad.sh`. No Android APK change needed.

The standalone ZIP in the associated ChatGPT conversation is an alternative
for devices that cannot use unzip; extract on a PC, copy the directory and run
its Python installer. Never run two shadow observers simultaneously.
