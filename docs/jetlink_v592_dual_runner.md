# XNOR Jetlink V5.9.2 — pinned Zoompilot joiner + both modeld runners

**Stage only. Remote driving authority is disabled unless independently
qualified. No vehicle-control tests were performed by this package.**

This update retains the **custom XNOR Pixel Android APK** and the existing V5.8
`JetlinkTCP` / `PipelinedInference` USB-tether protocol. It does **not**
install or require the stock Zoompilot phone application or gadget owner.

## Full source provenance

Vendored `jetlink/` source is byte-identical to
[`gavrackerss/jetlink` commit `28c5837e621ed989a642118f594a661dca614eea`](https://github.com/gavrackerss/jetlink/commit/28c5837e621ed989a642118f594a661dca614eea)
(Jetlink Python 0.9.0), including the upstream MIT licence in
`jetlink/LICENSE`. No unpinned pip installation or moving submodule is needed.
The vendor package is imported from the openpilot repository root.

Existing Zoompilot implementations retained:
`jetlink.openpilot.joining.JoiningModelState`,
`jetlink.openpilot.model_state.JetlinkModelState._note_hold`,
`jetlink.openpilot.events.SwapEvents`.
The custom APK protocol adapter remains in `openpilot/jetlink_authority.py`.

## Changes

- `jetlink/`: vendored pinned upstream Python library and MIT licence
- `openpilot/jetlink_authority.py`: V5.9.2 custom APK adapter; adds the tinygrad
  runner's variable image/desire key mapping and the stock LS2 action parser,
  while retaining the original TCP/UDP transport and 2,068-float predictions
- `sunnypilot/modeld_v2/modeld.py`: selected tinygrad model is retained as
  the small/local fallback; optional Zoompilot handover wrapper, remote LS2
  action timing, dropped-frame handover filtering, and `modelV2.big`

**No new cereal schema change in this update.** The original V5.9
`cereal/log.capnp` change still requires a successful build once if not
already built.

## Installation on the comma (with xnor-dev checked out)

From the openpilot checkout, copy the complete replacement files and the
`jetlink/` folder with their existing relative paths. Or fetch just the
feature-branch changes from GitHub:

```bash
cd /data/openpilot
git branch --show-current
git fetch origin feature/xnor-jetlink-v592-dual-modeld-pinned
git checkout origin/feature/xnor-jetlink-v592-dual-modeld-pinned -- \
  jetlink openpilot/jetlink_authority.py sunnypilot/modeld_v2/modeld.py \
  docs/jetlink_v592_dual_runner.md
python3 -m py_compile \
  openpilot/jetlink_authority.py sunnypilot/modeld_v2/modeld.py
python3 -c 'from jetlink.openpilot.joining import JoiningModelState; from jetlink.openpilot.model_state import JetlinkModelState; print("Pinned Zoompilot joiner import OK")'
python3 -c 'import jetlink; print("Jetlink package:",jetlink.__version__)'
git status --short
```

Keep both `/data/jetlink_xnor/AUTHORITY_ENABLED` and
`/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED` absent while validating.
A missing or incompatible Pixel transport does not justify bypassing
the qualification gate.

## Runner behavior

- **stock modeld**: uses its existing V5.9.1 `jetlink_authority.attach()` call;
  its selected local split model remains the fallback.
- **modeld_tinygrad**: new call uses `runner="tinygrad"`, retains whichever
  local tinygrad bundle is selected, and switches only through Zoompilot's
  disengaged joining mechanism. The remote LS2 prediction is still parsed
  using the **stock LS2** output layout, not the active local bundle's parser.
  The remote action uses stock LS2 plan-to-action logic rather than arbitrary
  selected-bundle generation-specific semantics. Camera names are mapped
  dynamically between the two runners.
- The onroad V5.8 shadow process remains suppressed if
  `AUTHORITY_ENABLED` is present, to avoid sharing the Pixel inference port.
- Only stock and tinygrad are supported. No companion APK changes.

## Validation boundaries

Source integrity for the pinned library and isolated custom protocol
unit tests have been checked. This is **not** an on-device end-to-end
validation or a driving qualification. Before enabling remote authority,
verify the actual cereal build, import/runtime with the selected runner,
same-frame camera warp, recurrent state, loss-of-link and rejoin behavior,
modelV2.big transitions, action validity, and disengaged handover on a
stationary controlled setup. Rebuilding modeld/cereal is required if V5.9's
schema change has not already been compiled.

Do not enable remote predictions for vehicle control based solely on
a passing source-level or offline test.
