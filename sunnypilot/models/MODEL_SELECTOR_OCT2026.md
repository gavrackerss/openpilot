# XNOR / comma 3X — focused Sunnypilot model selector (October 2026)

Baseline: `xnor-dev` speed245 at `6911d7020740efe372e464499e38e715aa36592e`.

## Scope and model sources

This changed-files overlay ONLY updates model selection, Cap'n Proto model
schema, manifest caching, and verified model downloads. No changes are made
to Tesla steering, lateral/longitudinal control, CAN, ALC, vehicle interface,
planner, VSL or UI speed-limit controls.

It reads official Sunnypilot standard model catalogue v23 (selector version
20), the catalogue currently used by Sunnypilot dev. This catalogue contains
the newest non-Chestnut Gen12 models through August 15, 2026. v26 lists the
same model identities but uses a newer manifest/package selector version.
We deliberately use v23 to match the current public Sunnypilot dev loader.
The previous v17 catalogue is used ONLY to preserve the exact hashes,
metadata and installed artifacts of the working OPM10V3 fallback, or an
already active older model.

The picker displays these **11 standard (is_big=false) 20 Hz Gen12 models**,
newest first:

| Label | Release date |
|---|---|
| TSFDOM — Terrible Super Fantastic Do Over | 2026-08-15 |
| Falling Phoenix | 2026-08-14 |
| Kumars Vibe | 2026-08-14 |
| Terrible Super Fantastic | 2026-08-12 |
| RDF Model V2 | 2026-08-11 |
| RDF Model | 2026-08-05 |
| Rebellious Hope | 2026-07-27 |
| Get Your Hopes Up | 2026-07-25 |
| Rebel Legion | 2026-07-22 |
| Michael RL V2 | 2026-07-18 |
| OP Model 16 Deep | 2026-06-03 |

Also displayed: OPM10V3 (April 19, 2026) as the known existing fallback,
plus an earlier legacy model ONLY while it is the active installed model.
The XNOR stock model option remains unchanged. Older historical models are
hidden rather than removed from disk.

No Chestnut / big / USBGPU model is displayed. **Mountain Dew is excluded**
from this overlay and is neither downloaded nor activated.

## Technical behavior

- Adds `ModelManagerSP.Chunk`, `Artifact.chunks`, and a `chunked`
  model enumeration, with existing Cap'n Proto numbers unchanged.
- Pins separate cache keys for the v17 and v23 manifests.
- Filters from authoritative catalogue metadata: `is_big=false`,
  `generation=12`, `runner=tinygrad`, `is_20hz=true`,
  model date >= 2026-06-01.
- Each new model keeps a stable `1000 + upstream index` selector index.
- Accepts legacy selector-15 bundles and dev selector-20 bundles so the
  user's existing verified OPM10V3 remains a valid active bundle.
- Fetches new model chunks with per-chunk SHA-256 verification and resume.
  Hash verification streams files with bounded memory, including
  chunkmanifest-based PKLs. The existing download fallback remains.
- A missing or failed remote manifest does not silently reset the previously
  selected model.

## Deployment validation (parked / offroad)

1. Run `python3 -m compileall -q sunnypilot/models selfdrive/ui/layouts/settings/models.py`.
2. Build the Cap'n Proto schema via the existing openpilot build.
3. Refresh Models and verify **12 entries** (11 new + OPM10V3) in addition
   to XNOR Default. Another entry may appear if a different legacy model
   is already active.
4. Verify Mountain Dew/Cinque Terre/BMRLNAP are **not** listed.
5. Select and download TSFDOM only when parked. Confirm its chunk hashes.
6. Confirm the existing OPM10V3 selection still works if TSFDOM cannot be
   loaded. Validate modeld init, input/output compatibility, stable
   inference rates and trajectories before any road use.
7. Check offroad-to-onroad transitions, warnings, and logs. Use
   shadow/non-controlling validation; **do not rely on an unvalidated model
   for live steering or braking**.

This updates model *availability* and downloader support. Neither upstream
catalogue membership nor successful download proves that the model is fully
compatible with the current xnor-dev modeld runtime. Device-specific
testing is still required.
