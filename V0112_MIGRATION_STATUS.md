# XNOR / openpilot v0.11.2 upgrade — engineering migration tracker

> **NOT AN ON-ROAD BUILD.** This is a staging/compatibility branch and still declares
> `COMMA_VERSION 0.11.1`. Do not run LS2 longitudinal authority or interpret this
> branch as a completed v0.11.2 release.

## Verified references

- Installed user fork before migration: `24d304bb06b6beef6b0a0633bf966dbbc23e9d39`
  (`fix/ls2-remove-dec-enable-smoothing-20261122`). DEC removed; remote
  action smoothing enabled; Tesla small model works.
- Official comma v0.11.2 release: `6939d172e85e3b91f16d90cc8de72e8f938cc196`.
- Exact unmodified release snapshot inside this fork:
  `upgrade/upstream-comma-v0.11.2-reference`. This snapshot **does not**
  include Tesla, SunnyPilot or Jetlink.
- XNOR parent `xnor-tech/openpilot:xnor-dev` at `44be035782288ed94d1a15116907cc08c0a7c83d`
  also declares 0.11.2, but lacks the user's Tesla, SunnyPilot and Jetlink integration.

## Why the upgrade cannot be treated as a simple git merge

The user's installed fork has **no mergeable common ancestor** with the tagged
comma v0.11.2 history (GitHub compare returns `No common ancestor`).
Compared with the upstream 0.11.1 tag's object tree, the installed fork contains
292 modified existing files and 2,925 added files. Many additions are vendored
third-party sources and custom model/runtime components, not simply application
code. The v0.11.2 tree relocated the main code from root-level `cereal/`,
`common/`, `selfdrive/`, `system/`, etc. into `openpilot/`, and switched
`opendbc_repo`, `panda`, `msgq_repo`, `rednose_repo`,
`teleoprtc_repo`, and `tinygrad_repo` to submodule pins.
The installed fork contains custom in-tree copies of these components.

A straight checkout of the tagged release removes critical Tesla HW1 support and
the Pixel remote model integration. Bumping `version.h` would not upgrade the
runtime and must not be done in this staging branch.

## Changes actually ported in this branch

- Ported the comma v0.11.2 asymmetric lead-probability filtering into the existing
  XNOR `selfdrive/controls/radard.py`, preserving its legacy `status` field,
  radar measurements, and Tesla-specific code. Both lead candidates rise
  immediately with high confidence and decay gradually across isolated drops.
- The port changes only `radard`; it is **not yet process-replay verified** and
  does not guarantee safe lead following or adequate LS2 braking.
- DEC-free two-candidate planner and 0.3-second remote action smoothing from the
  already-installed base commit remain untouched.

## Required work before we can claim the fork is v0.11.2

1. **Complete v0.11.2 codebase and dependency migration.**
   Align package paths, Python imports, SCons, AGNOS environment, native
   dependencies, and submodule or vendor strategy; verify compiled artifacts.
2. **Tesla compatibility.** Rebase customised Tesla HW1 interface, CAN parsing,
   longitudinal sender, EPAS/AEB, safety/panda and firmware controls against
   the newer `opendbc` and `panda` contracts.
3. **Cereal/API contracts.** Port user-defined message schemas, parameters and
   custom services to the new `openpilot.cereal` location. Resolve
   `radarState.leadOne.status` vs new `present` semantics consistently.
4. **SunnyPilot model selection.** Reconcile `modeld_v2`, model selection and
   custom tinygrad with the newer official tinygrad and model bundle APIs.
5. **Jetlink/Zoompilot.** Rebase the joining/fallback state and USB remote
   inference bridge onto the new v0.11.2 `modeld` loop, with full-frame/camera
   warp, frame freshness, recurrent context, parser heads and action timing
   tests. Do not assume a working connection implies correct model predictions.
6. **Longitudinal policy.** Port v0.11.2's MPC/cruise/E2E candidate handling,
   new stop semantics and lead tracking without bypassing Tesla's safety gates.
   Retain DEC removal; avoid arbitrary braking gain adjustments.
7. **UI/manager/other extensions.** Rebase user-specific UI, ALC/VSL,
   settings, processes, lifecycle and other files only after core services
   and car safety checks are preserved.

## Test gates (all required, in order)

- Source compilation / dependency validation and a clean build;
- Tesla parameter/fingerprint/CAN/safety/panda tests;
- Model contracts and pixel-level image preprocessor equivalence checks;
- Replay of old and new `radard`, `plannerd` and `controlsd` on the same
  recorded big-model and small-model drives;
- Lead-following metrics, emergency-stop prediction checks, planner/actuator
  consistency, stale-output and demotion tests;
- Parked-only hardware validation and independent review;
- **LS2 retains no longitudinal authority until all safety gates pass.**

This staging branch is intentionally not merged or installed automatically.

## Incremental v0.11.2 ports (2026-11-23, staging only)

The following changes have been added on top of the original lead-confidence port.
These are **source ports**, not a complete v0.11.2 runtime:

1. **Action-head stop decision**: `drive_helpers.should_stop(v_ego,a_target)`
   now implements comma v0.11.2's low-speed (`v_ego < 0.3` m/s and
   `a_target < 0.1` m/s²) predicate. The LS2 direct-action branch in
   `selfdrive/modeld/modeld.py` uses it before smoothing. The stock
   small-model branch still uses XNOR's legacy trajectory-derived stop flag.
   This affects stopping semantics at/near standstill, **not** ordinary
   30–40 mph lead braking.
2. **Independent cruise candidate**:
   `selfdrive/controls/lib/xnor_v0112_candidates.py` contains adapted
   upstream v0.11.2 cruise-acceleration and three-candidate arbitration
   calculations. **Deliberately disconnected from live planning**:
   XNOR's existing longitudinal MPC includes a cruise constraint, whereas
   v0.11.2 separates lead-only MPC from cruise acceleration. Plugging this
   candidate into the legacy MPC without the matching solver migration
   would not be a faithful port.
3. **Short-action-horizon stability**:
   `drive_helpers.get_accel_from_plan` and
   `get_curvature_from_plan` now use 0.3-second interpolation for shorter
   action horizons, matching the v0.11.2 helper logic. The existing
   XNOR two-value `get_accel_from_plan` return signature is retained.
   Unlike the LS2-only stop change, this interpolation change **also
   affects small-model and MPC computations**, and therefore needs replay
   regression before the branch is used for driving.
4. **Legacy/new radar schema compatibility**:
   `drive_helpers.radar_lead_present` understands the existing
   `.status` field and v0.11.2 `.present` field. Staging MPC and
   planner call sites now use it. This does **not** migrate the Cap'n Proto
   schemas themselves.
5. **Regression test additions**:
   `tests/test_xnor_lead_filter.py`,
   `tests/test_xnor_v0112_action_semantics.py`,
   `tests/test_xnor_v0112_candidates.py`, and
   `tests/test_xnor_v0112_drive_helpers.py`.

### Staging validation commands

```bash
cd /data/openpilot
python3 -m py_compile \
  selfdrive/controls/radard.py \
  selfdrive/modeld/modeld.py \
  selfdrive/controls/lib/drive_helpers.py \
  selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py \
  selfdrive/controls/lib/longitudinal_planner.py \
  selfdrive/controls/lib/xnor_v0112_candidates.py
python3 tests/test_xnor_lead_filter.py
python3 tests/test_xnor_v0112_action_semantics.py
python3 tests/test_xnor_v0112_candidates.py
python3 tests/test_xnor_v0112_drive_helpers.py
```

**NOT RUN ON VEHICLE.** These tests are checked into GitHub but do not
constitute process replay, hardware qualification or on-road sign-off.
Keep the user's comma on the known-good small model, not this staging branch.

### Still blocking a full 0.11.2 upgrade

- New package layout, dependencies, native builds and submodule pins;
- Port custom Tesla HW1 firmware, CAN, safety, panda and car interfaces;
- Port the actual lead-only MPC/three-candidate planner together, including
  the regenerated native solver and Tesla-specific acceleration limits;
- Merge new Cap'n Proto schemas while retaining all XNOR messages and services;
- Reconcile Tinygrad/SunnyPilot model selection and Jetlink's phone inference,
  remote recurrent state, camera warp, action timing and fallback;
- Run matching small/big model inference and full radard/plannerd/controlsd
  offline process replay; test lead braking and fallback against interventions.

Do **not** bump the version string, merge this staging branch into `xnor-dev`,
or enable LS2 longitudinal authority based on these commits.

## Full parallel v0.11.2 MPC and planner port (staged, not active)

A complete **source-level** v0.11.2 longitudinal MPC and planner have now been
copied into the quarantined `selfdrive/controls/lib/v0112_migration/` package:

- `long_mpc.py` — full upstream v0.11.2 **lead-only** MPC; excludes the
  legacy cruise obstacle, maintains .status/.present lead compatibility,
  preserves the Tesla `followDistanceS` override, and uses the existing
  Acados solver Python ABI. The symbolic OCP problem-generator methods in
  legacy and v0.11.2 sources differ only by a comment, **not** by their
  equations. This does not establish native binary parity.
- `longitudinal_planner.py` — full upstream v0.11.2 independent
  MPC/cruise/E2E arbitration, including AccelBoost, adapted for legacy
  `liveParameters`, `get_accel_from_plan` tuple return and stop semantics.
  The legacy Cereal `longitudinalPlan` has no `accelBoost` field, so that
  optional diagnostic field is not published. The policy behaviour remains
  isolated from the active manager and actual driver.
- `accel_boost.py` — staged upstream v0.11.2 override-dependent
  acceleration boost (not connected to the car).
- `__init__.py` — marks this as a separate migration package.
- `tests/test_xnor_v0112_lead_only_mpc.py` — checks lead-only obstacle
  selection, status/present schemas and Tesla gap settings using a fake solver.
- `tests/test_xnor_v0112_staged_planner.py` — checks actual staged
  Experimental and non-Experimental arbitration using an injected fake MPC.
- `tests/test_xnor_v0112_port_contract.py` — zero-dependency AST comparison
  of the symbolic MPC generators, plus a safety assertion that active
  `selfdrive/controls/plannerd.py` still imports the old planner.
- `.github/workflows/xnor-v0112-port-contract.yaml` — contains an
  independent source-contract and syntax CI workflow. As of this update,
  **no workflow run has been confirmed**.
- `tools/xnor_v0112_compare_planners.py` — read-only, open-loop comparison
  of legacy vs staged planners using saved LS2 rlogs. This script has not
  yet been run against the user's actual rlogs or on a comma device.

### Replay when a parked/offroad or workstation environment is available

```bash
cd /data/openpilot

# Run zero-dependency structural checks first:
python3 tests/test_xnor_v0112_port_contract.py -v

# Run contract and policy tests with the XNOR venv/native dependencies:
python3 tests/test_xnor_v0112_lead_only_mpc.py -v
python3 tests/test_xnor_v0112_staged_planner.py -v

# Compare old and new MPC + planner over recorded messages. PARKED/OFFROAD ONLY:
python3 tools/xnor_v0112_compare_planners.py /data/ls2_replay/rlogls2.zip \
  --out /data/xnor_v0112_planner_replay
```

Outputs: `old_vs_0112_planner.csv`, `planner_summary.json`, and
`replay_error.json` if a process fails. The replay runs **two Acados
solver instances** in Python, but is read-only: no CAN publication, no
Pixel inference, no camera or manager processes. If run on comma hardware,
the vehicle must be parked and openpilot offroad.

### Safety/readiness reminder

**Ported source does not equal deployable v0.11.2 software.** The main
`selfdrive/controls/lib/longitudinal_planner.py`, active
`long_mpc.py`, native solver, camera pipeline, Tesla/panda binaries,
system manager, dependency tree, and `COMMA_VERSION` have not been
switched to v0.11.2. LS2 braking failures remain unresolved and its
longitudinal authority must remain disabled.
