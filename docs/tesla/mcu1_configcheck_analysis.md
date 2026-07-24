# Tesla MCU1 ConfigCheckBypass configuration-read analysis

Analyzed binary: `Tesla_MCU1_ConfigCheckBypass.bin` (2,097,152 bytes). The image is a raw big-endian PowerPC/VLE-style MCU application image with an apparent flash mapping at `0x40000000` (for example the previously identified RAM words live in the `0x4004xxxx` range while file offsets map naturally to `0x400xxxxx` code/data addresses).

## Decoded configuration payload

The image contains a plain-text `internal.dat` payload at file offset `0x1c448`. It is not encrypted or compressed. The relevant tail of the decoded text is:

```text
packconfig 81
...
chargingID CIKS5M1A6GCF13DBN
autopilot 2
```

There is also an older active-looking `fileCrc 51af79f8` string earlier in the image at `0x183ce`, while additional `fileCrc` labels appear near the file parser strings at `0x25870` and `0x25dbc`. The embedded payload itself terminates with erased flash bytes (`0xff`), not with an adjacent active checksum field after `autopilot 2`.

## Where configuration parameters are read

The important finding is that the config reader is table/key driven, not hard-coded only around the ASCII line `autopilot 2`.

### 1. Name pool used by the parsed configuration API

A contiguous, NUL-padded pool of canonical configuration names starts before `0x23f00` and runs through at least `0x245cf`. It contains the names used by the firmware-facing config API in camelCase form, including:

- `updateWhatYouSee` at file offset `0x2401c`.
- `tunerHwid`, `tpmsType`, `dasHw`, `steeringWheel`, and `steeringHeat` at `0x24060`-`0x2409f`.
- `forwardRadarHw`, `navigationAllowed`, `rearDriveUnitType`, `seatType`, `rearSeatType`, and `autopilot` at `0x24110`-`0x2416f`.
- `softPackConfig` and other drivetrain/chassis parameters at `0x241a8` onward.
- `gtwSoftFactoryGated`, `plcAdapterType`, `deliveryStatus`, `autopilotCameraType`, `mcuFPGAVersion`, and `chargingID` later in the same pool.

This pool is the clearest static evidence for where the firmware knows which configuration parameters exist. The stored `internal.dat` keys are mostly lower-case (`forwardradarhw`, `autopilot`, `autopilotcameratype`), while this pool uses firmware API spelling (`forwardRadarHw`, `autopilot`, `autopilotCameraType`), which strongly indicates a parser/canonicalizer layer between raw text and runtime values.

### 2. The likely read path

Based on the decoded strings and layout, configuration parameters are read in this order:

1. A file/parser layer opens or references `internal.dat` (`internal.dat` strings at `0x25840` and `0x60c14`).
2. The parser recognizes optional `fileCrc` records (`fileCrc` labels at `0x25870` and `0x25dbc`) but the decoded embedded payload has no active trailing checksum that explains the observed AP mismatch.
3. Parsed key/value records are looked up through the canonical name pool around `0x23f00`-`0x245cf`.
4. Individual firmware users request parameters through named accessors/cached fields such as `packConfig`, `updateWhatYouSee`, `tunerHwid`, `dasHw`, `forwardRadarHw`, `autopilot`, `softPackConfig`, `brakeHwType`, `gtwSoftFactoryGated`, `autopilotCameraType`, and `mcuFPGAVersion`. These accessor/debug labels appear together near the configuration logic strings and match the name pool entries.
5. The parsed values are then copied into the known packed runtime words, including the AP representations already identified at `0x4004AA38`, `0x40047CAC`, and `0x40049DA4`.

### 3. Why the reboot is probably not controlled by this local read path

The binary confirms that the local gateway has everything needed to parse and publish `autopilot 2` from `internal.dat`: the embedded file contains `autopilot 2`, the canonical name pool contains `autopilot`, and the runtime code has AP-specific packed output words. That agrees with the prior tests showing the callback can encode AP2 into all three runtime words.

What this image does **not** expose is a simple outbound serializer that reads the AP word and directly decides the pending/reboot outcome. The static string evidence instead points to a broader update/configuration ecosystem: `updateWhatYouSee`, `gtwSoftFactoryGated`, `deliveryStatus`, `autopilotCameraType`, `mcuFPGAVersion`, `chargingID`, module-info acquisition strings, UDS updater strings, and Tegra reset/update strings all coexist near the config and update handling code.

Therefore the best current model is:

- The gateway reads `internal.dat` locally through the table/key-driven parser described above.
- The plain AP value is only one input into runtime/publication state.
- The final pending/reboot decision is likely made after a Tegra/update-service comparison against a richer desired-state object: capability/entitlement bitmap, signed configuration package, module-info database, revision/hash, `updateWhatYouSee` state, or a backend-derived configuration record.

## Practical next reverse-engineering targets

1. Treat the name pool at `0x23f00`-`0x245cf` as the anchor for the parser schema. Recover the adjacent table that indexes these strings; it should reveal each parameter's type, default, destination/cache slot, and possibly a publish flag.
2. Trace users of the accessor/debug-label cluster (`packConfig`, `updateWhatYouSee`, `autopilot`, `gtwSoftFactoryGated`, `autopilotCameraType`, `mcuFPGAVersion`) rather than only users of the three AP runtime words.
3. Prioritize `updateWhatYouSee`, `deliveryStatus`, `gtwSoftFactoryGated`, `chargingID`, `autopilotCameraType`, and `mcuFPGAVersion` as comparison inputs because they are present in the same decoded schema and are more likely to participate in a Tegra-side desired-state or entitlement check than a literal AP integer.
4. If tooling permits, load the raw image at base `0x40000000` in a PowerPC/VLE-aware disassembler and mark strings at `0x40023f00`-`0x400245cf`, `internal.dat` at `0x40025840`, and the embedded config blob at `0x4001c448`. Then search for code that computes string addresses via `lis/addi` pairs rather than direct 32-bit pointers; this explains why simple absolute-pointer searches do not find complete cross-references.

## Immediate next step: prove the Tegra-side comparison input

Do **not** spend the next pass patching `autopilot` again. The highest-value next pass is to capture or locate the object that the Tegra/update service treats as authoritative and compare it against the gateway's locally parsed values.

### Recommended order

1. **Diff all persistent Tegra/update state before and after the failed check.** Capture the update database, desired-configuration cache, vehicle profile, entitlement/capability files, and any `internal.dat` mirror immediately before pressing check, immediately after the new `Last checked` timestamp appears, and after the reboot rolls the timestamp back. The field that changes temporarily and then reverts is the best candidate owner of the reboot decision.
2. **Trace readers of comparison-oriented config keys, not AP runtime words.** Start with `updateWhatYouSee`, `deliveryStatus`, `gtwSoftFactoryGated`, `chargingID`, `autopilotCameraType`, and `mcuFPGAVersion`; those keys are more likely than plain `autopilot` to participate in a backend-desired-state or entitlement comparison.
3. **Recover the schema table adjacent to the canonical name pool.** The pool proves the key names; the adjacent descriptor table should identify value type, default, cached destination, publish behavior, and possibly whether a key is included in a hash/revision calculation.
4. **Instrument the transport at the boundary.** Instead of forcing the local AP value, log complete request/response payloads between the MCU gateway and the Tegra updater/configuration service during the check. Look for a capability bitmap, config revision, hash, package signature result, or module-info block that changes when AP is edited.
5. **Only after the owner is identified, patch the decision point.** The likely patch is not another `autopilot 2` hard-code; it is either aligning the Tegra-side desired-state record, neutralizing a rejected revision/hash comparison, or making the MCU-published compound config object match what the updater expects.

### Fast falsification test

If changing `autopilot` alone still reboots, but changing `autopilot`, `autopilotCameraType`, `updateWhatYouSee`, and the Tegra cached desired-state record together avoids the pending state, then the reboot is confirmed to be a multi-field desired-state rejection rather than a local gateway config-read bug.

## Follow-up: runtime-read descriptor table for the software-update comparison

A concrete runtime-read component is now identified. The contiguous descriptor table starts at file offset `0x249d0` and contains 118 candidate 16-byte records through `0x25120`. Each record has this layout:

```text
+0x00 word0 / optional cached address or default pointer
+0x04 flags
+0x08 canonical-name string offset
+0x0c per-key runtime read callback offset
```

The `autopilot` record is:

```text
entry      0x024d50
word0      0x00000000
flags      0x00000000
name_off   0x024164  -> "autopilot"
read_off   0x0870f0  -> autopilot runtime-read callback
```

The bytes at callback `0x870f0` include the low halves of all three known AP runtime words (`0xaa38`, `0x7cac`, and `0x9da0`/near `0x9da4`), matching the earlier dynamic findings. That makes `0x870f0` the per-parameter read function that supplies the AP runtime value to the generic descriptor-table consumer.

The surrounding comparison-oriented records are also relevant:

| entry | name | read callback |
| --- | --- | --- |
| `0x024c10` | `updateWhatYouSee` | `0x085b96` |
| `0x024d00` | `forwardRadarHw` | `0x087834` |
| `0x024d50` | `autopilot` | `0x0870f0` |
| `0x0250a0` | `deliveryStatus` | `0x0865be` |
| `0x0250b0` | `autopilotCameraType` | `0x08657c` |
| `0x0250c0` | `mcuFPGAVersion` | `0x08654e` |

This changes the next patch target. The likely software-update check path is a generic table walker/comparator that iterates these descriptors, calls the read callback at `entry + 0x0c`, and compares the returned runtime value against the desired/configured value. For AP specifically, adjust either the descriptor entry at `0x24d50` or the read callback at `0x870f0`; do not keep patching only `internal.dat` or the AP application callback.

### Adjustment options

1. **Safest diagnostic patch:** redirect only the `autopilot` descriptor's read callback (`0x24d5c`, currently `0x000870f0`) to a known compatible callback that returns the desired comparison value and has the same calling convention. This proves whether the descriptor-table consumer is the update-check comparator.
2. **AP-specific callback patch:** patch `0x870f0` so its return path reports the comparison-safe value while leaving the functional AP runtime words unchanged. This is narrower than changing all AP runtime state.
3. **Multi-field alignment patch:** if AP alone still fails, patch the neighboring descriptor callbacks for `autopilotCameraType`, `updateWhatYouSee`, and `deliveryStatus` to match the desired-state object captured from Tegra.
4. **Avoid broad table suppression:** do not disable the whole descriptor-table walker unless the goal is only a temporary boot experiment. The table covers many vehicle configuration fields, so a broad bypass risks hiding unrelated required update/precondition failures.

The helper script `tools/tesla/mcu1_config_table.py` extracts this table from the raw image and can be used to verify callback offsets after each binary edit.

## Next trace result: AP runtime reader has a direct code reference

A raw immediate-reference trace found one non-table reference to the `autopilot` read callback offset `0x870f0`: file offset `0x6f549`. The same trace did **not** find equivalent non-table references for `updateWhatYouSee`, `deliveryStatus`, `autopilotCameraType`, or `mcuFPGAVersion` callbacks, which makes the `0x6f520`-area code the strongest AP-specific software-update comparison lead currently identified.

The relevant bytes around the reference are:

```text
0x06f540: 70 ff e7 fe 62 00 1b 81 08 08 70 f0 c0 10 d0 07
                                  ^^^^^^^^
                                  0x0870f0 callback immediate/reference
```

Interpretation: the gateway firmware appears to have both a generic descriptor-table representation of runtime config reads and at least one AP-specific code path that directly references the AP runtime-read callback. If the software-update check is comparing runtime values, the next binary adjustment should target this chain first:

```text
software-update/config check code near 0x6f520
  -> direct callback reference at 0x6f549
  -> autopilot runtime reader at 0x870f0
  -> known AP runtime words 0x4004AA38 / 0x40047CAC / 0x40049DA4
```

### Recommended patch experiment

Patch the call/reference site near `0x6f549` or the AP reader at `0x870f0` so the update-check path receives the comparison-safe AP value, while leaving the functional AP runtime state unchanged. If the pending/reboot state changes after this patch, `0x6f520` is the local gateway-side read component feeding the update comparison. If it does not change, keep the descriptor-table evidence but move the trace to the Tegra desired-state object and transport payloads because the gateway AP reader is no longer the decisive input.

The extractor now supports `--refs` to surface this kind of raw callback-reference hit:

```sh
python3 tools/tesla/mcu1_config_table.py Tesla_MCU1_ConfigCheckBypass.bin --refs
```

## Bench-test export script for Ghidra-side patching

Because the firmware image is a raw flash dump rather than an ELF, the safest way to reproduce bench-test artifacts on the machine that has the loaded Ghidra project is to patch Ghidra memory and export from that same address map. The repository now includes `tools/tesla/ghidra_scripts/Mcu1PatchExport.java` for that workflow.

The script reads from the first loaded Ghidra memory block by default, so it works whether the raw image was imported at `0x0`, `0x40000000`, or another RAM/flash base. If needed, pass `image_base=0x...` to force the loaded-image base and `srec_base=0x40000000` to control exported S-record addresses. It reads/pads a full `0x200000`-byte image with `0xff`, applies user-supplied byte patches, recalculates the observed header CRC field at file offset `0x8` over the default application range `[0x0c, 0x200000)`, writes the refreshed CRC back into both Ghidra memory and the exported image, and emits:

- `<out>.bin`: padded raw flash image.
- `<out>.srec`: Motorola S-record output using S3 records with addresses based at `0x40000000`.

Example Ghidra script arguments:

```text
out=/tmp/Tesla_MCU1_ConfigCheckBypass_patched srec_base=0x40000000 patch=0x24d5c:000870f0 patch=0x6f549:0870f0
```

The checked-in Java script intentionally has an empty default `PATCHES` list so the operator can paste the exact bench patch bytes after confirming the desired replacement callback or instruction sequence in their own Ghidra database. For the AP runtime-comparison experiment, the two offsets to consider first remain the descriptor callback word at `0x24d5c` and the direct AP callback reference near `0x6f549`.
