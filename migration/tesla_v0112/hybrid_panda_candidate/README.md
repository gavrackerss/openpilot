# XNOR 0.11.2 — Full Tesla Hybrid Panda candidate

**Status: full Panda safety and Tesla HW2 vehicle-source candidate staged,
NOT compiled/qualified/installed.**
This is a coordinated **opendbc + Panda safety + Tesla CarState/
CarController + 0x659 sender** source port from installed `24d304b`,
adapted to XNOR 0.11.2 APIs. The independent OP longitudinal planner,
LS2 action contract, capnp code generation, actual CAN/ECU acceptance
and safety regression tests remain separate gates. The live car
software and both submodule pointers are unchanged.

## Exact base dependencies (unchanged in openpilot)

| Live submodule | XNOR upstream pin |
| --- | --- |
| `opendbc_repo` | `3f86206a13c3181f0224e8f1424b836592e2b2d2` |
| `panda` | `fa83a40ad799c827ab9cbda4560cf3426531ceff` |

There is no writable `gavrackerss/opendbc` available in the connected
GitHub account. All changes are staged under this folder, **not** in
the active submodules or original `xnor-dev`.

## Integrated source files

| Submodule | Candidate path | Port |
| --- | --- | --- |
| opendbc | `opendbc/safety/modes/tesla_legacy.h` | Full 2,223-line original Hybrid safety mode: 0x659 internal carrier; HW1/HW2/HW3; external Panda; stock Autosteer/AEB arbitration; V229 0x2B9/0x2BF templates; native-frame counters/checksums; HIL ownership, HUD, stalk, braking |
| opendbc | `opendbc/safety/declarations.h` | Adds `fwd_msg_hook`, dual forward ABIs and Tesla lateral-only brake state |
| opendbc | `opendbc/safety/safety.h` | Adds packet-aware `safety_fwd_hook_pkt`; preserves legacy address-only API and **new XNOR RX-frequency/fail-closed guards** |
| opendbc | `opendbc/car/tesla/values.py` | Preserves native `AngleSteeringLimitsVM` while adding legacy Hybrid flags 64/128, stalk constants and curve-assist settings |
| opendbc | `opendbc/car/tesla/interface.py` | Configures both HW2/HW3 Panda safety parameters, MAIN/RES owner flag, Hybrid `pcmCruise`, OP long setting |
| opendbc | `opendbc/car/tesla/radar_interface.py` | Restores radar `aRel`, `yvRel`, `measured` without losing XNOR's Bosch last-frame cache |
| opendbc | `opendbc/dbc/tesla_can.dbc` | Adds 0x3A9 lane telemetry, retains **native 0x368 DI speed layout** and native 0x389 LSS layout |
| opendbc | `opendbc/safety/tests/libsafety/libsafety_py.py` | Adds CFFI binding for packet-aware host safety tests |
| opendbc | `opendbc/safety/tests/test_tesla_hybrid_packet.py` | Synthetic packet safety negatives, stock AEB and stale-owner tests |
| panda | `board/drivers/bxcan.h` | Routes a **copy** of RX frame through packet-aware safety hook, preserving real RX data |
| panda | `board/drivers/fdcan.h` | Same on FD-CAN, with Tesla safety denials not bypassed by fallback routes; preserves native body handling |
| host | `host_tests/test_hybrid_panda.c` | Standalone compiled C fixture for native AEB and V229 template/counter behaviour |

**Two deliberate changes from the old installed firmware:**

1. `TESLA_LEGACY_MITM_BLOCK_STOCK_GTW` defaults to **0**, not the
   original bench-only value of 1. A Hybrid port must not cut native
   GTW 0x398 traffic just because an unrelated old experiment was left on.
   `TESLA_LEGACY_OVERRIDE_GTW_AUTOPILOT` stays 0.
2. For genuine stock DAS_control event **AEB=1**, the XNOR packet-aware
   dispatcher **never calls the mutable forwarding hook**; the frame
   is forwarded unchanged if otherwise permitted. This rule is
   independent of `TinklaIgnoreStockAeb`, which must not suppress
   genuine emergency braking. It also preserves the original unmodified
   RX packet for safety evaluation.

## 2026-11-23 safety hardening and reproducible build gates

The original V229 controller requested single-owner longitudinal authority
even while openpilot was disengaged, using a non-neutral "priming" packet.
The original Panda source also allowed a disengaged 0 km/h prime envelope to
be cached and overlaid onto stock AP traffic. **Both exceptions have been
removed in the native 0.11.2 candidate.** The Panda now requires genuine
longitudinal permission when *capturing* AND when *applying* an OP DAS_control
template; in the absence of permission, the genuine AP frame is unchanged.

The Tesla controller now emits the V229 0x659 ownership bit only when:
OP is enabled and longitudinally active; CANCEL is false; CAN is valid and
not timed out; gear is Drive; there is no physical brake, gas, AEB or ACC
fault; and native Tesla Autosteer does not own lateral control. The
controller no longer generates non-Drive or disengaged prime templates.
**Do not assume the old zero-speed DI priming/auto-arming feature survives
this change.** Restoring it would need independent hardware safety
review, replay and validation of a safe, non-actuating mechanism.

The Panda RX drivers now zero-initialise the received CAN packet before
taking a full-size private copy for packet-aware forwarding. This avoids
copying uninitialised trailing bytes from a classic 8-byte frame into
the forwarded packet.

Old source modules that depend on `openpilot.common.Params`,
`openpilot.cereal.messaging` or the legacy LONG/ALC/CFG modules now
import those only when constructing an HW1/HW2/HW3 controller/state.
A standalone opendbc install can import `CarInterface` and compile
the stock Tesla C safety regressions **without** loading the openpilot
runtime. Attempting to construct actual Hybrid state/control without
that runtime intentionally raises an error.

New CI gates:

- `.github/workflows/xnor-v0112-hybrid-panda-compiled.yml`:
  source preflight, patch export and application in scratch clones;
  host C safety compile and AEB/0x659 tests; native Cap'n Proto
  schema load, standalone Tesla import check, and native
  `test_tesla.py`, `test_tesla_hw1.py`, `test_tesla_hw23.py`
  plus packet-aware Hybrid CFFI tests.
- `.github/workflows/xnor-v0112-hybrid-panda-arm-compile.yml`:
  cross-compiles scratch STM32F4 and STM32H7 Panda firmware using
  the paired patched opendbc safety headers and the original
  debug certificate. **Never flashes, connects to a Panda or
  uploads a usable firmware binary.**

The tests are present as GitHub Actions jobs but **no workflow execution
or successful C/ARM compile is confirmed**. Even green scratch builds
would leave full all-car regression, replay, vehicle-specific bus
mapping, independent code review and LS2 lead/curve braking unresolved.

## Tesla vehicle-side source port (2026-11-23 continuation)

The staged opendbc candidate now also includes:

- `opendbc/car/tesla/carstate.py`: recovered installed ~1,800-line
  Tesla state logic, rehomed to `openpilot.cereal` and the staged
  `opendbc.car.tesla.legacy_modules`. Uses **the pinned native 0x368
  DBC layout** without guessing or doubling cruise-set speed.
- `opendbc/car/tesla/carcontroller.py`: recovered ~2,000-line Hybrid
  controller, including the 0x659 dual-Panda carrier, V229 owner request,
  real stalk edges, 0x2B9/0x2BF template creation, and EPAS
  handback logic. Steering now uses
  `apply_steer_angle_limits_vm(..., CarControllerParams, self.VM)` and
  native Model S HW3 vehicle model, matching XNOR's Model S/X steering
  limiter contract. Physical steering/accelerator limits and safety
  approval still require dedicated replay.
- `opendbc/car/tesla/teslacan.py` and
  `teslacan_legacy.py`: native 0.11.2 CAN helper preserved with added
  0x659, STW CRC/counter, lane telemetry, chassis and powertrain
  V229 template helpers.
- `opendbc/car/tesla/legacy_modules/`: exact installed
  CFG/BLNK/ALC/HSO/ACC/LONG controller sources, with native package
  imports adapted, pending runtime integration tests. This includes
  the older longitudinal/lead logic but **does not prove that LS2
  braking works**.
- `carstate_native_0112.py` and `carcontroller_native_0112.py`:
  byte-for-byte XNOR 0.11.2 implementations. Non-legacy Model 3/Y
  requests delegate to these, rather than entering HW1/2/3 Hybrid.
- `opendbc/car/car.capnp`: restores Tesla diagnostic fields
  `adaptiveCruiseEnabled` (62), `teslaAlcHandoverReady` (63),
  `teslaEacStatus` (64), `teslaEacError` (65) and
  `followDistanceS` (**66**). Native XNOR
  `carNotReady @61` is untouched. The removed analog
  `brake @5` is restored for compatibility with the legacy state.

**Deliberately disabled pending replay:** inherited experimental
0x398 gateway injection, the extra standalone 0x2B9 template, legacy
0x399/0x389 HUD warning override, and the
`TinklaIgnoreStockAeb` flag. The old
`DAS_csaState` in `DAS_status2` overlaps the native
`DAS_lssState` definition; do not re-enable old HUD builders
without resolving that conflict. The V229 ownership/template source
remains staged for compiled and on-bench validation.

The native `DI_cruiseSet` is **already scaled by CANParser**:
`32|9@1+` at factor 0.5. The installed heuristic could
multiply that value by two under plausible speed conditions.
The staged logic selects that native decoded value when cruise
is engaged, and refuses to substitute current dashboard speed
for a missing engaged setpoint. Vehicle-specific raw CAN replay
is required to verify that the pinned DBC is correct on this HW2 car.

Additional dependency-free source-function tests:
`python3 tests/test_xnor_v0112_tesla_vehicle_contract.py -v`.
These execute the actual staged function ASTs to verify the 0x659
MAIN/CANCEL edges, V229 native LKAS handback and lack of
cruise-set-speed doubling. They are **not hardware validation**.

## What the packet-aware bridge does

The baseline XNOR `safety_fwd_hook(bus, addr)` remains available so
existing opendbc safety tests/callers do not break. The new
`safety_fwd_hook_pkt(bus, CANPacket_t *)` first runs the regular relay,
CAN address and mode checks. If permitted, it invokes only the Tesla
mode's registered `fwd_msg` callback. Any attempt to mutate frame
address, bus, DLC or CAN-FD metadata is rejected.

On STM32F4 bxCAN and STM32H7 FD-CAN, Panda passes a **private RX copy**
through the new hook, recomputes the transport checksum and sends only
that copy; the original RX frame continues through `safety_rx_hook`
without alteration. This is required for Hybrid's V229 substitution
of validated OP templates onto genuine AP 0x2B9/0x2BF cadence, retaining
native rolling counters and a calculated Tesla additive checksum.

## Workstation validation and patch export

Run from the Openpilot repository root, in a **disposable workstation
checkout**, *not* on the installed comma:

```bash
git checkout upgrade/xnor-v0112-xnor-base
git submodule update --init opendbc_repo panda
python3 tools/xnor_v0112_full_hybrid_panda_preflight.py
python3 tests/test_xnor_v0112_full_hybrid_panda.py -v
python3 tests/test_xnor_v0112_tesla_vehicle_contract.py -v
python3 tools/xnor_v0112_export_full_hybrid_panda_patch.py \
  --output-dir /tmp/xnor-v0112-hybrid-review
```

The export verifies all **15 pinned Git blob hashes**, checks
the full safety/driver source contract, writes two independent patches
and runs read-only `git apply --check` in both pinned submodules.

The files are:

- `/tmp/xnor-v0112-hybrid-review/xnor-v0112-tesla-hybrid-opendbc.patch`
- `/tmp/xnor-v0112-hybrid-review/xnor-v0112-tesla-hybrid-panda.patch`

To test the patched dependencies without changing the live gitlinks:

```bash
mkdir -p /tmp/xnor-v0112-hybrid-work
git clone --no-hardlinks opendbc_repo /tmp/xnor-v0112-hybrid-work/opendbc
git clone --no-hardlinks panda /tmp/xnor-v0112-hybrid-work/panda
git -C /tmp/xnor-v0112-hybrid-work/opendbc apply \
  /tmp/xnor-v0112-hybrid-review/xnor-v0112-tesla-hybrid-opendbc.patch
git -C /tmp/xnor-v0112-hybrid-work/panda apply \
  /tmp/xnor-v0112-hybrid-review/xnor-v0112-tesla-hybrid-panda.patch
cc -Wall -Wextra -Werror -std=gnu11 -DALLOW_DEBUG \
  -I /tmp/xnor-v0112-hybrid-work/opendbc \
  migration/tesla_v0112/hybrid_panda_candidate/host_tests/test_hybrid_panda.c \
  -o /tmp/xnor-v0112-hybrid-work/tesla-hybrid-smoke -lm
/tmp/xnor-v0112-hybrid-work/tesla-hybrid-smoke
```

The dedicated GitHub workflow
`.github/workflows/xnor-v0112-hybrid-panda-compiled.yml`
performs that source verification, disposable-patch application and
compiled C host smoke. **No passing workflow or native compiled result
has been observed as of this staging commit.**

## Safety/replay gates before any firmware flash or onroad authority

- Confirm the new scratch STM32F4/STM32H7 build workflow succeeds
  and independently inspect the resulting compiler diagnostics. It is
  not a substitute for hardware-in-the-loop testing or independent review.
- Run the existing full opendbc Tesla safety test suite on the
  candidate: `test_tesla_hw1.py`, `test_tesla_hw23.py`, `test_tesla.py`.
  Run the new packet suite against a compiled patched C safety library.
- Verify dual-Panda bus mapping, rolling counter and checksum, RX
  freshness, real physical MAIN/CANCEL, brake/regen, stock AEB,
  native LKAS/ALC, stale/forged 0x659, stale template, host/software
  restarts, emergency neutralisation and no double 0x2B9 sender.
- Validate the mismatched `DI_state` 0x368 fields and native
  `DAS_status2` 0x389 occupancy against this specific vehicle's
  **raw** rlogs before porting CarState and cruise speeds.
- Build and replay-test the **now-staged** native Tesla
  `CarController`, `CarState`, supporting modules and 0x659 sender,
  including generated `car.capnp` bindings. Confirm actual
  dual-Panda bus routing; the source port is not yet runtime validated.
  The independent longitudinal planner, LS2 model action path and
  actual lead/curve braking remain unqualified.
- Independently prove that LS2's observed lead-following and curve
  deceleration problem is resolved by replay/diagnostics. **Do not**
  treat a successful Hybrid Panda build as evidence of sufficient braking.

**Stop at the candidate until these gates pass. No active gitlink,
installed Panda firmware, production Tesla controller, or vehicle
actuator has been changed.**
