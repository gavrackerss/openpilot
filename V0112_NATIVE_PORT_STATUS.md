# XNOR native comma v0.11.2 integration — source migration status

**Status: DEVELOPMENT / NEVER INSTALL FOR ON-ROAD USE.** No road-control
qualification, no native compiled build, and no Jetlink authority. The installed
small-model Tesla branch remains unchanged.

## Versions and immutable source baselines

- Official comma v0.11.2 release commit:
  `6939d172e85e3b91f16d90cc8de72e8f938cc196`.
- Native integration branch: `upgrade/xnor-v0112-native-integration`
  (a descendant of the exact official release commit).
- Original installed XNOR branch:
  `24d304bb06b6beef6b0a0633bf966dbbc23e9d39`
  (COMMA_VERSION 0.11.1, DEC removal, LS2 smoothing).
- Older compatibility staging:
  `upgrade/xnor-0.11.2-compat-staging` (complete parallel upstream
  lead-only MPC, cruise/E2E planner and replay tools, but on 0.11.1 layout).
- Native dependency reference: `xnor-tech/openpilot:xnor-dev` commit
  `44be035782288ed94d1a15116907cc08c0a7c83d`.

## Actual migration commits on this native branch

1. **Native source tree:** forked directly from upstream 0.11.2, rather
   than renaming or bumping `version.h` in 0.11.1.
2. **Jetlink sources retained:** 40 original `jetlink/` files plus 13
   XNOR Pixel adapter files `openpilot/jetlink_*.py`. They are not
   connected to the native modeld runtime.
3. **Native ABI bridge:** `openpilot/jetlink_v0112_native.py` verifies
   the 0.11.2 `ModelState.run(..., after_enqueue)`, dual camera names,
   and `state_pairs/input_queues/packed_input`. It calls the stock
   v0.11.2 GPU-state reset preparation, not the old incompatible
   `_Small` / `_reset_small` implementation. Its parked probe passes
   `in_control=lambda: True` to the joiner, **preventing ALL model swaps**.
4. **XNOR-compatible submodules:** moved the 0.11.2 `opendbc_repo`,
   `panda`, `tinygrad_repo`, and `rednose_repo` gitlinks to the exact
   XNOR parent 0.11.2 refs:
   - `opendbc_repo`: `3f86206a13c3181f0224e8f1424b836592e2b2d2`
   - `panda`: `fa83a40ad799c827ab9cbda4560cf3426531ceff`
   - `tinygrad_repo`: `8611fe22a7fcc7d1928bbde19ded66277cb12f3e`
   - `rednose_repo`: `28d4a7f69e80e1c3e0d24ca0733d7daeaeade3d0`
   Updated `.gitmodules` for XNOR panda and opendbc repositories.
5. **Tesla preservation:** 30 user-fork source blobs saved verbatim in
   `migration/tesla_opendbc_legacy/`. They include all the custom Model S/X
   HW1/HW2/HW3 platform definitions, Tesla CAN/DBC files, safety header
   variants, and Tesla safety tests.
   Against XNOR's pinned opendbc tree, 23 active Tesla/documentation/safety
   files comprise **4 identical, 13 modified, 6 additional variants**.
   **Those 13 modified sources are NOT yet applied to the active submodule.**
6. **SunnyPilot preservation:** 18 model selector and `modeld_v2` sources
   saved verbatim in `migration/sunnypilot_legacy/` for later API port.
   The old Tinygrad/modeld runner is not enabled.
7. **XNOR Cereal extensions:** ported `ModelManagerSP`,
   `MapdExtendedOut`, `MapdIn`, and `MapdOut` to
   `openpilot/cereal/custom.capnp`. Reused existing reserved wire IDs
   and corresponding Event union slots **108, 143, 144, 145**, respectively.
   Registered XNOR service names in `openpilot/cereal/services.py`.
   Old and native schema compatibility has been source-checked, not compiled.
8. **Parameter registry:** added **63** legacy XNOR keys missing from
   official v0.11.2: Tesla/Tinkla, model manager, mapd, Vision Speed Limit,
   etc. Also added typed JetlinkLink (INT, default `0` = **off**),
   JetlinkProgress (JSON), JetlinkSpec (JSON), and JetlinkChargePhone
   (BOOL, default `0`). Kept upstream v0.11.2 settings for
   `ExperimentalMode` and `LastGPSPosition`, which conflict with
   old type/default semantics and need an explicit data migration decision.
9. **Source tests:** `tests/test_xnor_v0112_native_contract.py`,
   `tests/test_xnor_v0112_schema_contract.py`,
   `tests/test_xnor_v0112_params_contract.py`, plus
   `.github/workflows/xnor-v0112-native-contract.yaml`.

A separate static check of the current GitHub source validated: native
version 0.11.2, unique custom struct IDs, four preserved Event slot
numbers, unique service and Params names, Tesla/XNOR submodule URLs,
native modeld not importing Jetlink, and the offroad probe's never-swap
guard. **It is not a native build, Cap'n Proto compilation, or runtime test.**

## STOP: components that remain non-working/unverified

- **Tesla safety/custom submodule integration:** apply and reconcile
  the 13 modified Tesla files plus custom safety/CAN support in a writable
  pinned opendbc fork (or deliberately vendored dependency); verify
  Panda vs opendbc safety hooks and all Tesla test suites.
- **Native v0.11.2 modeld Jetlink:** the copied legacy
  `openpilot/jetlink_native_authority.py` still invokes the incompatible
  0.11.1 `_Small` wrapper. `openpilot/jetlink_v0112_native.py` provides
  a compatible *offroad-only* path but is not wired into modeld or manager.
  Do not activate the copied legacy adapters.
- **Cereal and parameter build:** regenerate/compile native Cereal,
  native C++ Params and services; run SCons; validate custom messages
  against the old logs. No such build has run.
- **SunnyPilot model selection:** migrate model catalog, modeld_v2,
  model lifecycle/manager, UI, and all VSL/ALC extensions to native paths
  and pin-compatible Tinygrad.
- **Camera and LS2 inference:** compare exact road/wide frame warp pixels,
  latency, frame freshness, recurrent context and raw action head on both
  model pipelines; native modeld's use of calibrated camera inputs
  must match the Pixel expectations.
- **Longitudinal replay:** rerun native 0.11.2 `radard`, `plannerd`,
  controlsd and Tesla actuators against the recorded lead and intervention
  segments; compare small-model vs big-model predictions when camera
  recordings exist. A new planner alone cannot guarantee LS2 braking.
- **Operational testing:** native device build, unit tests, panda safety
  tests, parked-only integration, independent review, and shadow validation.
  Until these pass, LS2 longitudinal authority remains disabled.

## No-dependency tests (run against this branch on a workstation)

```bash
python3 tests/test_xnor_v0112_native_contract.py -v
python3 tests/test_xnor_v0112_schema_contract.py -v
python3 tests/test_xnor_v0112_params_contract.py -v
python3 -m py_compile openpilot/jetlink_v0112_native.py
```

**Not yet executed here:** GitHub Actions has not reported a successful
workflow for this staging branch; this environment cannot compile or run
the car's native Cereal, opendbc, panda or tinygrad binaries. Do not
merge this staging branch to xnor-dev or install it for driving.
