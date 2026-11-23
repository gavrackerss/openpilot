# Tesla Panda/opendbc native 0.11.2 staging overlay

**Development/review only — not a runnable car integration.**

This directory is deliberately **outside** `opendbc_repo/` and `panda/`.
The live XNOR 0.11.2 submodules remain pinned to:
- `opendbc_repo` — `3f86206a13c3181f0224e8f1424b836592e2b2d2`
- `panda` — `fa83a40ad799c827ab9cbda4560cf3426531ceff`

Panda's `board/main.c` includes `opendbc/safety/safety.h`, so
Panda safety work belongs to an independently testable **opendbc source
revision**, not a blind edit to the active `panda` pointer.

## Staged source

| Path under `opendbc_overlay/` | Port | Status |
| --- | --- | --- |
| `car/tesla/values.py` | XNOR native steering `AngleSteeringLimitsVM` and safety flags unchanged. Legacy `CruiseButtons` enums and curve assist constants added as inactive declarations | Passive definitions only |
| `dbc/tesla_can.dbc` | All native XNOR messages retained exactly, plus legacy lane telemetry `DAS_telemetry` message 937 / `0x3A9` | Additive DBC, not wired to CAN |
| `car/tesla/radar_interface.py` | Restores `aRel`, `yvRel`, `measured` on radar points while retaining native Bosch last-frame caching for its slower 8 Hz update | Staged; **excluded** from passive review patch |
| `safety/modes/tesla_legacy.h` | Byte-for-byte **unchanged** pinned XNOR safety header. Keeps stock AEB suppression of OP TX and refuses any OP-injected AEB event | Safety reference, **not** custom hybrid mode |

There is intentionally **no** staged modified active `carcontroller.py`,
`carstate.py`, `interface.py` or new 0x659 safety hook. The long
customizations are archived separately in
`migration/tesla_opendbc_legacy/opendbc/`.

## Critical differences requiring real CAN replay and separate safety work

1. **DI_state (0x368) layout:** native `DI_cruiseSet` is
   `32|9@1+ (0.5,0)`, native `DI_digitalSpeed` is `48|8@1+`.
   The user's installed DBC instead places `DI_cruiseSet` at
   `48|8@1+` and `DI_digitalSpeed` at `32|8@1+`.
   These are **different raw bytes and factors**. Neither layout
   should be selected automatically from plausible speed values.
   `migration/tesla_v0112/di_state_layout.py` decodes both
   interpretations offline with an explicit unqualified result.
2. **DAS_status2 (0x389) layout:** native `DAS_lssState` occupies
   bits 31–33, overlapping the installed fork's
   `DAS_relaxCruiseLimits` and `DAS_csaState`. The pinned
   signal layout remains untouched; no duplicate or overlapping
   fields were added.
3. **Legacy dual-Panda ownership:** the archived `interface.py`
   applies `LONG_CONTROL` to every safety config and adds
   `OP_STALK_ENABLE (64)` and `IGNORE_STOCK_AEB (128)`.
   The pinned native interface uses different flag ownership and the
   native safety header recognises neither extra flag.
4. **0x659 internal carrier and hybrid templates:** the legacy
   2,223-line `tesla_legacy.h` contains extensive custom ownership
   logic, a 0x659 internal carrier and 0x2B9/0x2BF templates
   overlaid on genuine AP frames. Pinned XNOR safety is 283 lines and
   does not support that mechanism. **Do not merge or enable it**.
5. **AEB:** the legacy code contains configurable AEB warning/stock
   intervention suppression. That is intentionally not ported into
   the safety overlay. A recognised lead, `CC.longActive` and a
   transmitted `DAS_control` template are not proof that Panda
   authorised or the ECU accepted an actual brake request.
6. **Bosch radar cadence:** native 0.11.2 caches the previous Bosch
   radar result on no-update frames. The archived legacy interface
   returns empty radar data instead. The staged radar overlay
   retains native cache behaviour; replay must validate impact on
   tracking and lead acceleration.

## Reproducible validation (workstation or CI; never onroad)

From the `openpilot` repository root:

```bash
git submodule update --init opendbc_repo
python3 tools/xnor_v0112_tesla_panda_preflight.py
python3 tests/test_xnor_v0112_tesla_panda_overlay.py -v
python3 tests/test_xnor_v0112_tesla_passive_patch.py -v
```

Generate a **passive review-only** patch into an external path:

```bash
python3 tools/xnor_v0112_export_tesla_opendbc_patch.py \
  --output /tmp/tesla-v0112-passive.patch
(cd opendbc_repo && git apply --check /tmp/tesla-v0112-passive.patch)
```

The patch only covers `values.py` and the additional DBC telemetry
message, never the staged radar behavioural change or Panda safety.
`git apply --check` checks applicability but does **not** apply it.
Do not run `git apply` in the active submodule.

## Gates before a future vehicle-capable integration

- Maintain a writable, versioned opendbc revision, an actual Panda
  firmware build and an exact two-Panda safety firmware pin.
- Require `opendbc/safety/tests/test_tesla_hw1.py`,
  `test_tesla_hw23.py` and `test_tesla.py` to pass on the pinned
  baseline and **new** negative tests for the custom 0x659 carrier,
  stale or forged templates, hands-on state, active AEB, disabled
  controls and main/external Panda bus routing.
- Check CAN traces for `0x368`, `0x389`, `0x2B9`, `0x2BF`,
  `0x659`, brake/regen/gear, physical MAIN/RES, radar leads and
  checksum/counter validity. Confirm ECU application, not just
  outgoing CAN requests.
- Run log replay of active braking, lead selection and curves, then
  parked-only validation, independent code review and explicit staged
  shadow/authority approval.

**No active CarInterface, Panda safety binary, acceleration sender,
steering limit or stock-AEB behaviour has been changed by this overlay.**
