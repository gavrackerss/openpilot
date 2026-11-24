# XNOR-native 0.11.2: user fork migration

> **Development-only branch. Do not install for on-road use.**
> Tesla HW2 Panda/opendbc source is staged, compiled and host safety-tested.
> Runtime CarInterface construction passed in scratch CI. Real CAN/replay,
> complete SunnyPilot/Jetlink/LS2 authority, and on-road safety are NOT qualified.

## Why this branch exists

XNOR `xnor-tech/openpilot:xnor-dev` is already at 0.11.2,
pinned commit `44be035782288ed94d1a15116907cc08c0a7c83d`.
The user's 0.11.1 `gavrackerss/openpilot:xnor-dev` is extensively
customised. A previous port
`upgrade/xnor-v0112-native-integration` started from official comma 0.11.2
and staged preservation, but official comma's native modeld **differs** from
XNOR's native 0.11.2 modeld. Do not mix those runtimes or reuse their
small-model adapters interchangeably.

This branch starts **at the exact XNOR 0.11.2 commit**, leaving XNOR's
live modeld, helpers, native managers, USB GPU support and submodule
commit pointers intact. The old user and prior staging branches are
not modified by these commits.

## Changes actually integrated (static schema and configuration)

1. `migration/` is grafted verbatim from the previous staging branch,
   preserving custom Tesla HW1/2/3, Panda safety, SunnyPilot and other
   historical reference sources. It is NOT a runtime import.
1a. All legacy Jetlink runtime sources and 14 adapter Python modules are now
    archived under `migration/jetlink_legacy/` and
    `migration/jetlink_adapters_legacy/`. They are **not** imported by the
    XNOR-native modeld or manager, and the old official-0.11.2 bridge remains
    an incompatible reference, not an active adapter.
2. `openpilot/cereal/custom.capnp`: restored the four user-fork reserved
   struct definitions (ModelManagerSP, MapdExtendedOut, MapdIn, MapdOut),
   reusing their original struct IDs.
3. `openpilot/cereal/log.capnp`: renamed **only** the four matching
   Event union slots, numbered 108, 143, 144 and 145. XNOR's native
   `chestnutState`, navigation deprecations and other schema elements
   are preserved. The official-comma port's complete `log.capnp`
   must NOT be copied here.
4. `openpilot/cereal/services.py`: added seven missing model-manager,
   mapd and raw-data services, retaining native XNOR service settings,
   including existing `customReservedRawData0`.
5. `openpilot/common/params_keys.h`: imported **65 missing custom keys**
   without duplicating the two already present
   (`ExperimentalModeConfirmed`, `Offroad_NeosUpdate`). Preserved
   native XNOR `UsbGpuActive`/`UsbGpuLoading`,
   `ExperimentalMode`, `LastGPSPosition` and
   `IsLiveStreaming` definitions. JetlinkLink defaults to **off**.
6. `tests/test_xnor_v0112_base_contract.py` and a branch-specific
   GitHub Actions workflow guard the static ABI, params and Cereal
   contracts. **Test execution is not yet independently confirmed.**

## 2026-11-23 continuation: XNOR-native Jetlink and SunnyPilot staging

7. **Jetlink native model ABI:** archived Jetlink libraries and original
   adapters were preserved in `migration/jetlink_legacy/` and
   `migration/jetlink_adapters_legacy/` without linking them to modeld.
   `migration/jetlink_xnor_v0112/native_contract.py` validates the actual
   XNOR 0.11.2 `ModelState.run(bufs, transforms, inputs)`, QCOM
   model state, two camera inputs and XNOR input queues. It rejects the
   official-comma four-argument/packed-input ABI.
   - Offline decoder inspects the Pixel's **2,068-float** response and
     **four raw action-head floats at indices 2062–2065**.
     XNOR model constants define `ACTION_WIDTH = 2`. A raw head
     length mismatch alone cannot establish correct or incorrect
     semantics; **do not truncate or wire these raw values into modeld**.
   - Explicit authority gate always raises. The staged module has no
     socket, CAN or process hooks. Regression tests validate malformed
     packets and incompatible model interfaces.
8. **SunnyPilot model catalogue:** the legacy model-selector *models*
   package (not obsolete `modeld_v2`) is now staged under
   `openpilot/sunnypilot/models/`. Its imports are moved to
   `openpilot.cereal` and a native `model_root()` shim uses the XNOR
   hardware paths. The 0.11.2 modeld and manager process configuration
   are unchanged. This does **not** make downloaded models runnable.
   - `openpilot/sunnypilot/models/catalog_guard.py` inspects JSON
     metadata without file downloads or execution. It always reports
     effective runner = **stock** until separately validated.
   - `tools/xnor_v0112_model_selection_status.py --offline-json
     /path/to/saved-bundle.json` performs a standalone read-only
     report; invoked without `--offline-json` it reads saved native
     Params only when `IsOffroad=True`.
   - Static and CLI regression tests have been added to the dedicated
     workflow; successful workflow/device execution is **not verified**.
9. **No writable opendbc fork found:** the available repositories do
   not include a user-owned opendbc repository. XNOR's pinned
   `opendbc_repo` and `panda` remain unchanged; Tesla safety,
   controller and actuator customizations are still staged and **inactive**.

## 2026-11-23 installed Tesla longitudinal baseline and isolated state machine

10. **Locked installed reference:** the seven exact Git blob objects from
    `24d304bb06b6beef6b0a0633bf966dbbc23e9d39` are preserved in
    `migration/installed_24d304b/`: `longitudinal_planner.py`,
    `long_mpc.py`, `controlsd.py`, `modeld.py`, `radard.py`,
    `drive_helpers.py` and `cruise.py`. They are non-runtime sources.
    The script `tools/xnor_v0112_longitudinal_migration_audit.py`
    verifies their exact Git blob hashes and reports remaining gaps.
    - **Do not treat `gavrackerss/openpilot:xnor-dev` as the last installed
      driving baseline**: that newer branch's 430-line longitudinal planner
      contains `XnorDynamicExperimentalControl` and release-boost logic,
      whereas the installed `24d304b` planner contains neither.
    - The XNOR-native 0.11.2 planner remains DEC-free. Do **not** merge the
      newer DEC implementation into the native active planner to solve
      LS2 lead following.
    - The custom hybrid brake latch in installed `controlsd.py` and
      custom following-distance helper in installed `long_mpc.py`
      are identified as unported.
11. **Offline Tesla long-owner reference:** the installed V207/V234 hybrid
    brake cancellation, regen hold and manual/Experimental auto-resume
    decision logic has been extracted into the **pure, unconnected**
    `migration/tesla_v0112/hybrid_long_owner.py`.
    `tests/test_xnor_v0112_tesla_hybrid_long_owner.py` covers brake
    latch, 1,000 ms regen/brake-release wait, manual physical MAIN/RES,
    gas/ACC fault/override gates and disengagement.
    - This state machine returns a *hypothetical* LONG eligibility result,
      **not** `CarControl`, Panda CAN or ECU acceptance. It is not hooked
      into native `controlsd`. It needs source/wire validation against
      Tesla CarState and independent replay before any future integration.
    - Native XNOR `controlsd`, Tesla opendbc and Panda remain unchanged.

## 2026-11-23 Tesla Panda/opendbc native overlay

12. **Native XNOR Tesla overlay is staged**, **not** applied to the live
    `opendbc_repo` gitlink (commit `3f86206a`) or `panda`
    (commit `fa83a40a`). See
    [overlay README](migration/tesla_v0112/opendbc_overlay/README.md).
    The active `opendbc` implementation still runs XNOR's original source.
    - `car/tesla/values.py`: copied from pinned native source, with
      legacy `CruiseButtons` and curve-assist configuration added as
      passive declarations. Native `AngleSteeringLimitsVM` and
      `TeslaSafetyFlags` are preserved (no stock-AEB override).
    - `dbc/tesla_can.dbc`: contains all pinned native messages and a
      single new legacy `DAS_telemetry` message `937/0x3A9`, inserted
      before the value-table declarations. No native signal layout
      is changed.
    - `car/tesla/radar_interface.py`: restores `aRel`, `yvRel`
      and `measured` while preserving XNOR's cached last Bosch radar
      result when no new 8 Hz sample arrives. This is a **behavioural**
      port needing radar-log replay and is not included in the passive
      patch.
    - `safety/modes/tesla_legacy.h`: exact, byte-identical copy of
      pinned XNOR Panda safety (283 lines). The old 2,223-line
      custom safety header with 0x659 owner carrier, templates and
      AEB overrides remains archived but **unmerged**.
13. **Source contract and passive patch:** 
    `tools/xnor_v0112_tesla_panda_preflight.py` locks exact native blob
    IDs and fails if AEB TX rejection, native DBC signals, native
    radar timing/cache, VM safety flags or the baseline header change.
    `tools/xnor_v0112_export_tesla_opendbc_patch.py` generates a
    **review-only** git-apply patch of *only* native Tesla values additions
    and `0x3A9` DBC telemetry. Safety is excluded.
    The branch CI workflow now checks syntax, performs the preflight
    and asserts `git apply --check` with the exact pinned submodule.
    **No CI run is confirmed yet.**
14. **Panda HW2 negative tests staged:**
    `migration/tesla_v0112/opendbc_overlay/safety/tests/test_xnor_tesla_hw23_negative.py`
    tests 0x659/0x2B9 TX rejection on external Panda, OP-injected AEB
    rejection, native AEB forwarding and suppression of 0x2BF OP TX,
    and the unsupported AEB-ignore flag. These require compiled
    `opendbc.safety.tests.libsafety` and have **not been executed**.
15. **Unresolved compatibility gates:**
    - `DI_state` `0x368` native and installed `DI_cruiseSet` /
      `DI_digitalSpeed` move between bits 32 and 48 and differ in
      scaling. Read-only dual decoder
      `migration/tesla_v0112/di_state_layout.py` exposes both
      interpretations, with *no automatic layout choice*.
    - `DAS_status2` `0x389` native `DAS_lssState` bits 31–33
      overlap installed `DAS_relaxCruiseLimits` and `DAS_csaState`.
    - Native Tesla HW2 dual-Panda LONG_CONTROL flag placement and
      archived 0x659/0x2B9/0x2BF template ownership must be
      reconciled in an independently buildable opendbc fork before
      attempting custom longitudinal control.
    - No active Tesla CarState/CarController, opendbc safety, Panda
      firmware or SCons/native build has been changed/validated.

## 2026-11-23 Full Hybrid Panda source candidate — the coordinated port

16. **Complete HW1/HW2/HW3 Tesla Hybrid Panda candidate staged** under
    [`migration/tesla_v0112/hybrid_panda_candidate/`](migration/tesla_v0112/hybrid_panda_candidate/README.md).
    This includes the entire legacy **2,223-line Tesla Hybrid C safety
    mode**, ported opendbc safety declarations/dispatcher, both STM32F4
    bxCAN and STM32H7 FD-CAN forwarding drivers, dual-Panda Tesla
    `CarInterface` safety params, Tesla values, native-compatible
    radar/DBC, CFFI packet API, compiled host smoke and Python packet
    safety tests.
    - A new `safety_fwd_hook_pkt(int,CANPacket_t *)` preserves
      XNOR's original address-only `safety_fwd_hook(int,int)`.
      Pandas forward a **private RX copy**, with packet metadata
      immutable and original safety RX unmodified.
    - Real stock AEB event=1 never reaches the packet mutation hook;
      it is forwarded unmodified if forwarding is otherwise allowed.
      The old optional warning-scrub setting cannot author/suppress
      actual physical AEB in the new dispatcher.
    - Native opendbc RX minimum-frequency checks and safety mode
      initialisation are retained.
    - The original experimental `GTW_carConfig` MITM default is
      deliberately disabled. Legacy `0x659`, V229 ownership, valid
      0x2B9/0x2BF templates, fallback/neutral semantics and
      native counters/checksums are staged for compiled validation.
    - The native 0.11.2 `opendbc/safety/ignition.h` already includes
      Model S 0x348 counter-based ignition detection; the older Panda
      ignition subsystem is **not** copied over it.
17. **Isolated patch/export and build gate:**
    `tools/xnor_v0112_full_hybrid_panda_preflight.py` verifies exact
    pinned Git blob IDs for 8 opendbc files and 2 Panda files.
    `tools/xnor_v0112_export_full_hybrid_panda_patch.py` generates
    one full opendbc and one Panda patch and performs read-only
    `git apply --check` without modifying either live submodule.
    `.github/workflows/xnor-v0112-hybrid-panda-compiled.yml`
    creates disposable clones, applies both patches and compiles the C
    safety module plus host synthetic AEB/V229 tests.
    **No GitHub Actions CI run or compiled C result has been observed.**
18. **Still blocking any driving use:** the actual user-custom
    `CarController`, `CarState` and 0x659 sender are still archived,
    not natively adapted or enabled in the XNOR 0.11.2 car stack.
    All physical dual-Panda CAN replay, hardware firmware build,
    native Tesla safety regression, LS2 lead-braking validation and
    on-vehicle independent sign-off are outstanding.
    The primary `xnor-dev` branch and installed Panda remain untouched.

## 2026-11-23 Tesla HW2 CarState/CarController and 0x659 sender staged

19. Full installed Tesla HW1/HW2/HW3 `carstate.py`,
    `carcontroller.py`, `teslacan.py`,
    `teslacan_legacy.py`, and six installed CFG/BLNK/ALC/HSO/
    ACC/LONG support modules were staged under
    `migration/tesla_v0112/hybrid_panda_candidate/opendbc/car/tesla/`.
    They were imported from the exact installed `24d304b` code,
    then adapted to native `openpilot.cereal` and staged opendbc
    Python package paths.
20. Tesla legacy `CarController` now uses XNOR native vehicle-model-
    based steering rate/acceleration/jerk limiting:
    `apply_steer_angle_limits_vm(..., CarControllerParams, VM)` with
    the native Model S HW3 vehicle model. The old speed-only steering
    wind-up/unwind table (11.5/11.0/4.6° wind-up and 11.5/11.5/6.2°
    unwind at 0/5/15 m/s) has **not** replaced the modern limiter.
    The user-specific lane-positioning bias remains separate and must
    be assessed by steering replay.
21. Tesla `car.capnp` now stages the additional old diagnostic fields
    without overwriting native `carNotReady @61`:
    follow distance moves to `@66`, other Tesla-only fields use
    `@62`–`@65`, and compatibility brake is reintroduced at `@5`.
    Raw CAN `DI_state` 0x368 native layout stays intact. Removed old
    heuristic which could multiply native `DI_cruiseSet` by two.
22. Non-legacy Tesla Model 3/Y `CarState` and
    `CarController` delegate to exact pinned 0.11.2 source copies,
    not the old Hybrid control path. Candidate defaults disable
    experimental GTW config injection, unsupported extra 0x2B9
    overlay, and warning/HUD/AEB suppression pending separate tests.
23. The **full** opendbc review patch now includes
    all Tesla car sources, six support modules, additive DBC,
    non-colliding `car.capnp` and Panda safety integration.
    `tests/test_xnor_v0112_tesla_vehicle_contract.py` adds
    dependency-free source-function tests for deterministic cruise
    decoding, 0x659 physical stalk edges and native LKAS handback.
    The compiled workflow includes those checks.
    **No compiled Panda/opendbc source test or live car acceptance
    test is confirmed at this stage.** Both active gitlinks,
    device firmware and XNOR native modeld remain unchanged.

## 2026-11-23 additional C safety and test-build hardening

24. **Removed unqualified V229 priming** in C safety capture and
    substitution and in userspace `_v229_stopgo_long`. Old
    `ACC_ON, 0 km/h, -1.4/+1.8 m/s²` priming is no longer
    eligible while OP longitudinal authority is absent. On
    unauthorised capture, C safety rejects the template; when
    authorisation is revoked before a genuine AP frame arrives,
    C safety forwards the native frame unchanged. Obsolete
    priming helpers/constants were removed.
25. **0x659 owner bit is now conditional** on active engaged OP long,
    CANCEL clear, valid/non-timed-out CAN, Drive gear, no brake,
    accelerator, stock AEB or ACC fault, and no genuine native
    Autosteer ownership. Previous owner selection was too broad:
    it requested substitution with openpilot disengaged. This
    change means low-speed prime/DI-arming parity is not proven
    or automatically preserved; do not restore without full safety
    tests and replay.
26. **Panda RX copy initialization** fixed in both candidate F4
    `bxcan.h` and H7 `fdcan.h` so the new full
    `CANPacket_t to_send = to_push` path cannot inherit
    uninitialised classic-CAN trailing payload bytes.
27. **Standalone opendbc safety test imports** moved old CFG/BLNK/
    ALC/HSO and LONG module dependencies to the legacy
    `CarState`/`CarController` construction path. The
    Model 3/Y native fallback and Tesla interface can now be
    imported for standalone tests without loading openpilot
    messaging/Params. Actually creating the HW2 controller
    still requires the full installed openpilot runtime.
28. **New scratch-only CI gates**:
    - `xnor-v0112-hybrid-panda-compiled.yml` attempts
      Cap'n Proto schema validation, standalone Tesla import,
      host safety C compile and synthetic ownership/AEB smoke,
      then native HW1/HW2/HW3/Model3 safety regressions with
      the new packet-aware tests.
    - `xnor-v0112-hybrid-panda-arm-compile.yml` uses pinned
      source clones, exact review patches, `arm-none-eabi-gcc`
      and SCons to build scratch Panda F4/H7 images without
      flashing or distributing them.
    - Additional source/AST negative tests cover unauthorised
      V229 owner requests, revoked templates, off-Drive
      no-template handling, and RX copy initialisation.

**No GitHub Actions run was confirmed for this branch; no CFFI,
Cap'n Proto or ARM firmware result has been observed.**
Do not enable this Hybrid port on a car or use it as evidence
that LS2 lead/curve braking is resolved.

## 2026-11-23 scratch full-stack source integration

29. Added `tools/xnor_v0112_assemble_hybrid_candidate.py`.
    On a **workstation staging checkout only**, it verifies the
    native XNOR Panda/opendbc pins and source preflight, creates a
    separate openpilot Git clone, clones the exact two pinned
    dependencies into that separate checkout, applies the full
    Hybrid review patches, and emits a manifest. Its hard guards
    refuse AGNOS, /data, the active repo, changed original
    submodules and a pre-existing destination. It never flashes.
30. Added a non-actuating assembly test suite
    `tests/test_xnor_v0112_hybrid_scratch_assemble.py`
    and connected it to the compiled safety workflow.
31. Added a scratch full-stack SCons job
    `.github/workflows/xnor-v0112-hybrid-full-build.yml`;
    it applies both real source patches, bootstraps remaining
    native submodules and dependencies, and attempts to compile
    XNOR 0.11.2 against the staged HW2 Tesla integration. No
    on-road install or Panda flash action is included.
32. Opened [draft staging-only PR #6](https://github.com/gavrackerss/openpilot/pull/6)
    in an attempt to run the review builds without merging into
    the user's production `xnor-dev`.

**Not a finished deployable upgrade:** the physical comma build,
Panda ARM/CFFI regression outcomes, whole fork's original
customisations, LS2 big-model action mapping, native road/lead
braking authority, SunnyPilot runtime activation and actual CAN
replay still require independent validation. The main `xnor-dev`
branch and the comma installation are unchanged.

## 2026-11-23 verified full-stack migration checkpoints

33. **F4/H7 Panda firmware build PASS** — source patched into
    exact XNOR 0.11.2 Panda/opendbc in disposable clones:
    [successful ARM build](https://github.com/gavrackerss/openpilot/actions/runs/45258321608).
    Images were built in CI, not distributed, flashed or connected to a car.
34. **Tesla compiled safety regression PASS** — native XNOR Tesla
    HW1/HW2/HW3/Model3 regression and negative Hybrid packet suite:
    **308 passed, 133 skipped, zero failed**;
    [successful compiled safety run](https://github.com/gavrackerss/openpilot/actions/runs/45258321463).
    To preserve the native safety envelope, ordinary Tesla safety
    uses a mechanically namespaced copy of upstream safety;
    custom Hybrid only activates with explicit 0x40 OP_STALK_ENABLE.
    0x659 carrier forwarding is suppressed only in Hybrid mode.
    No disengaged V229 priming and no stock AEB suppression enabled.
35. **Full native XNOR SCons build PASS** in a disposable desktop
    checkout with the complete reviewed Panda/opendbc patches:
    [full build](https://github.com/gavrackerss/openpilot/actions/runs/45258532125).
    Corrected the scratch venv PATH for Cython and added hosted-runner
    libusb development headers; original gitlinks were unchanged.
36. **HW2 and Model Y host runtime object construction PASS**:
    [full build + runtime probe](https://github.com/gavrackerss/openpilot/actions/runs/45259610612)
    instantiated real CarInterface, CarState, CarController and
    six HW2 CAN parsers / two Model Y CAN parsers, with temporary
    Params and no CAN transmission. Both reported authority=false.
37. **Native Cap'n Proto compatibility PASS**: removed accidental
    duplicate brake @5, retained native carNotReady @61 and
    introduced non-colliding Tesla diagnostics @62–@66.
    Removed deprecated CarParams root setters and the removed
    CarState root brake write.
38. All the verified updates are merged into the dedicated
    `upgrade/xnor-v0112-xnor-base` staging branch; the
    actual `xnor-dev`, installed comma and both live submodules
    are NOT changed.

**Scope of validated result:** source assembly, F4/H7 compilation,
full desktop SCons, synthetic Tesla safety regressions, and
non-actuating CarInterface construction. **This is not full
feature parity or a vehicle-qualified release.** Physical Tesla
CAN replay, timing, dual-Panda bus topology, genuine AEB,
brake/regen override, native EPAS/ALC handover, LS2 lead/curve
braking, SunnyPilot runtime model selection and all original
custom features require separate validation.

## Important implementation differences

| Interface | XNOR's actual v0.11.2 | Official comma v0.11.2 |
|---|---|---|
| ModelState constructor | `ModelState(cam_w, cam_h, usbgpu)` | `ModelState(cam_w, cam_h, chestnut)` |
| ModelState.run | `run(bufs, transforms, inputs)` | `run(bufs, transforms, inputs, after_enqueue=None)` |
| Recurrent model buffers | `input_queues` and `npy`, XNOR policy/warp JIT | `state_pairs`, `packed_input`, native run JIT |
| Modeld helper | `usbgpu_compiled` / XNOR OOB | `chestnut_compiled` / official OOB |

The earlier official-comma-based Jetlink probe checks for
`after_enqueue` and `packed_input`, so it **will reject XNOR's
0.11.2 modeld**. This is an expected safe incompatibility, not evidence
of successful integration. Do not copy or enable the legacy wrapper.

## Still outstanding before any vehicle test

- Validate the **staged and successfully host-built** Tesla HW2
  CarState/CarController/Panda source using the real vehicle's
  dual-Panda CAN replay and safety constraints before enabling it.
  Legacy UI/HUD variants and physical handover are not yet qualified.
- Validate the staged SunnyPilot model catalogue and Params JSON
  lifecycle, then port actual model selection, checksum validation,
  UI and speed-limit overlays to the XNOR-native modeld contract.
  The current modeld continues to use stock model inference.
- Reimplement Jetlink/LS2 against the **XNOR native** model-state ABI;
  demonstrate frame, recurrent-state, latency, output/action-head and
  fallback parity *offline first*. No LS2 vehicle authority.
- Replay lead, braking, curves and disengagement cases from actual
  recordings. A red path or recognised radar lead alone is not proof
  of the commanded acceleration.
- Host Cereal/Params, SCons, Panda ARM and Tesla synthetic
  safety gates now PASS. Still run the broader all-car native suite,
  real Tesla process/CAN replay, parked hardware-in-the-loop tests
  and independently reviewed shadow trials.

This document is an engineering inventory, **not** confirmation of
a working Tesla installation or safety qualification.
