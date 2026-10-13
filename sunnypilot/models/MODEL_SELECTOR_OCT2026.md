# XNOR model selector upgrade — October 2026

Base branch: `xnor-dev` at `6911d7020740efe372e464499e38e715aa36592e`.

## Changes
- Preserve the Sunnypilot v17 / selector-15 legacy catalogue, including existing
  local `opm19v3` model selection and hashes.
- Fetch Sunnypilot development v23 / selector-20 catalogue separately and append
  **new refs only** at UI/download indices >=1000. Both manifests are cached
  independently; the model manager honours the existing offroad-only UI.
- Add chunked models and artifact chunks to `cereal/custom.capnp`.
- Parse chunk hashes from v23 and download/verify/resume chunks without an
  external chunkmanifest. Keep legacy chunkmanifest handling intact.
- Present `Mountain Dew (OP master; stage/download only - not runnable)` as an
  explicit local selection. Selecting this only stages the checksum-pinned
  official comma.ai 2026-10-10 big-model artifact. The previous active
  model remains unchanged, even when the staged file is verified.

## Mountain Dew

Source: https://github.com/commaai/openpilot/pull/39047 (merged Oct 10, 2026).
Current master blob: `big_driving_tinygrad.pkl`, SHA-256:
`256f2178366479a211933701c03518621ed3970289e6a8a43e5a2bcb15e55b5f`,
exactly 799,942,179 bytes. Source commit:
`2713630e7950a61dc072467b5394a1043e34547f`.

Offroad UI: Models -> Select Model -> Mountain Dew (stage/download only).

Or manually, from a parked device:
```
cd /data/openpilot
python3 -m openpilot.sunnypilot.models.mountain_dew
# If you already acquired the exact upstream LFS file:
python3 -m openpilot.sunnypilot.models.mountain_dew --source /path/to/big_driving_tinygrad.pkl
```

The staged file is under `Paths.model_root()/experimental_mountain_dew/`.
It is **not an active model bundle** and is not passed to modeld. It requires
a separate AMD/chestnut-capable runtime, safe package loader, model metadata
contract check, and hardware / replay validation. The current Qualcomm
comma 3X runner does not support it directly.

## Validation

Local review / manual checks after deploying these files:
1. Build the Cap'n Proto schema and check all imports on the target openpilot.
2. Offroad, refresh the selector and verify `opm19v3` remains present.
3. Verify nine additional July/August 2026 model refs show after the v23
   manifest fetch; review timestamps, suffixes and model filenames.
4. Verify existing v15 model selection, download and re-activation survive
   upgrade; no stock fallback unless the local hashes are invalid.
5. Verify one v23 chunked model can be fully downloaded, hashed, activated
   and **loaded on the target device offroad before any driving**.
6. Selecting Mountain Dew must not change ModelManager_ActiveBundle;
   stage status should show staged only, with exact size and SHA.
7. Run a shadow/non-controlling replay of newly loaded models and compare
   predicted trajectories, speed, timing and errors. Do not treat a UI entry
   as proof of usable driving model compatibility.

No new model has been verified on physical comma hardware in this change.
There is deliberately no change to Tesla CAN control, hybrid lateral,
longitudinal, planner, steering, or speed-limit logic.
