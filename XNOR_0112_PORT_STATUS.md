# XNOR-native 0.11.2: user fork migration

> **Development-only branch. Do not install for on-road use.**
> Tesla safety/CAN, SunnyPilot, Jetlink/LS2 and speed-limit customisations
> are **not integrated or qualified** on this branch.

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

- Reconcile the user's substantial Tesla `carstate.py`,
  `carcontroller.py`, interface, radar, UI/HUD and historical Panda
  safety differences into a writable, **separately tested** opendbc/Panda
  stack. The XNOR reference dependency already defines HW1/HW2/HW3,
  but not the user's additional safety ownership and steering behaviour.
- Port SunnyPilot model catalog, persistence, native model-selection
  lifecycle and speed-limit overlays without altering the XNOR
  modeld execution contract.
- Reimplement Jetlink/LS2 against the **XNOR native** model-state ABI;
  demonstrate frame, recurrent-state, latency, output/action-head and
  fallback parity *offline first*. No LS2 vehicle authority.
- Replay lead, braking, curves and disengagement cases from actual
  recordings. A red path or recognised radar lead alone is not proof
  of the commanded acceleration.
- Compile Cereal and Params, run SCons, Panda safety tests,
  full native test suite, then separate parked-only integration and
  independently reviewed shadow tests.

This document is an engineering inventory, **not** confirmation of
a working Tesla installation or safety qualification.
