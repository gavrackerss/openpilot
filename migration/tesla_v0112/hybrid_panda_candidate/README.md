# XNOR 0.11.2 — Full Tesla Hybrid Panda candidate

**Status: fully staged source port, NOT compiled/qualified/installed.**
This is a coordinated **opendbc + Panda safety** port of the installed
`24d304b` Tesla Hybrid safety implementation. It is not a complete
0.11.2 vehicle integration: the custom Tesla CarState/CarController,
independent OP longitudinal planner, LS2 action contract and ECU
actuation replay are separate gates.

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
python3 tools/xnor_v0112_export_full_hybrid_panda_patch.py \
  --output-dir /tmp/xnor-v0112-hybrid-review
```

The export verifies all **10 pinned Git blob hashes**, checks
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

- Build on the exact hardware-target Panda toolchain and independently
  validate the resulting STM32F4/STM32H7 firmware. Host C success
  alone does not prove hardware integration.
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
- Port the installed custom `CarController`, `CarState`, remaining
  Tesla modules and actual 0x659 sender to native XNOR 0.11.2.
  A Panda safety port **alone cannot run Hybrid** with XNOR's stock
  Tesla controller.
- Independently prove that LS2's observed lead-following and curve
  deceleration problem is resolved by replay/diagnostics. **Do not**
  treat a successful Hybrid Panda build as evidence of sufficient braking.

**Stop at the candidate until these gates pass. No active gitlink,
installed Panda firmware, production Tesla controller, or vehicle
actuator has been changed.**
