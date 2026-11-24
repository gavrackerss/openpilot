# XNOR 0.11.2 release promotion readiness

Status: **NOT APPROVED FOR PRODUCTION PROMOTION OR VEHICLE INSTALLATION**.

This is a release preparation record, not an installable release. The staging
branch remains separate from xnor-dev. Do not use a green workflow alone as
evidence of feature parity, real-time behavior, or vehicle safety.

## Verified historical evidence

These results apply to their recorded scratch revisions, not automatically
to the current branch head:

- F4/H7 Panda scratch compile: https://github.com/gavrackerss/openpilot/actions/runs/45258321608
- Tesla compiled synthetic safety suite: 308 passed, 133 skipped, zero failed:
  https://github.com/gavrackerss/openpilot/actions/runs/45258321463
- Full native desktop SCons scratch build:
  https://github.com/gavrackerss/openpilot/actions/runs/45258532125
- HW2 / Model Y non-actuating runtime construction:
  https://github.com/gavrackerss/openpilot/actions/runs/45259610612
- Earlier offline integration suite, 51 passing:
  https://github.com/gavrackerss/openpilot/actions/runs/45288534685
- Real TSFDOM source metadata inspected (not compiled in this particular run):
  https://github.com/gavrackerss/openpilot/actions/runs/45364367675

## Latest independently checked results

- Exact committed Hybrid dependency trees verified and built:
  [F4/H7](https://github.com/gavrackerss/openpilot/actions/runs/45404285671),
  [compiled Tesla regressions](https://github.com/gavrackerss/openpilot/actions/runs/45404276370)
  (308 passed, 133 skipped, three subtests passed),
  [full scratch desktop build](https://github.com/gavrackerss/openpilot/actions/runs/45404266825).
- Immutable pinned review branch commit `ace8f1bb16e66410b759ab1cbf6745588f966a4b`:
  [full desktop build and runtime construction](https://github.com/gavrackerss/openpilot/actions/runs/45409569794).
  Native stock synthetic inference, actual Params persistence and offroad sign interaction also passed.
- [TSFDOM native compile and paired replay](https://github.com/gavrackerss/openpilot/actions/runs/45409289945):
  both camera resolutions, recurrent state, 12 shared synthetic NV12 pairs.
- [OPM10V3 split graph and NV12 reference](https://github.com/gavrackerss/openpilot/actions/runs/45414068079):
  12 synthetic pairs, 11 recurrent feature changes, NumPy reference warp self-test.
  Pixel-level equivalence with the original accelerated warp, hardware latency
  and raw action semantics are not established.

- Consolidated pinned desktop review: [cd50b28 build](https://github.com/gavrackerss/openpilot/actions/runs/45414559559).
- Native startup selector and fallback tests: [offline gate](https://github.com/gavrackerss/openpilot/actions/runs/45419642661);
  [stock native runtime/UI smoke](https://github.com/gavrackerss/openpilot/actions/runs/45419642116).
- Updated isolated loader and dependencies: [TSFDOM desktop](https://github.com/gavrackerss/openpilot/actions/runs/45420642269).

## Target hardware evidence

The user supplied a successful QCOM bench report on 2026-11-24 using
the exact native compiler and tinygrad revision. Both camera warp JIT
compiles and round trips passed; the 126440436-byte artifact loaded and
three synthetic camera pairs ran. Warm frame times were 32.215 and
33.193 ms. See `migration/sunnypilot_legacy/tsfdom_qcom_bench_evidence.json`
for exact hashes and reported scope. The artifact has not been independently
retrieved or reviewed. This closes the short QCOM execution milestone,
not the action-semantics, actual driving regression or release approval gates.
The production approval registry remains empty.

The verified Hybrid dependency revisions are pinned in the separate
`review/xnor-v0112-pinned-desktop` branch, not in production. No build
above establishes safe operation on an actual vehicle.

## Owner-confirmed scope exclusions

- Legacy XNOR Dynamic Experimental Control (DEC) is not to be imported.
  Retain native 0.11.2 MPC/cruise/E2E arbitration. Differences caused by
  dropping DEC are intentional, not a feature-parity failure. The offline
  differential comparison is retained as characterization only.
- LS2/Jetlink is a separate project, remains isolated, and is not an upgrade
  release criterion.
- Short QCOM execution is complete; no repeat of that milestone is required.

The pinned promotion branch's exact Tesla host tests passed: 308 passed,
133 skipped, three subtests passed, zero failures
([run 45424414155](https://github.com/gavrackerss/openpilot/actions/runs/45424414155)).
The source comparison still identifies missing higher-level Hybrid
brake/regen cancellation and automatic ACC resume integration.
These are separate from DEC and the compiled safety tests.

The draft runtime now includes a Tesla Hybrid physical-brake longitudinal
inhibition latch in controlsd. Regen inhibits while held. The latch clears on
native disengagement, not automatically while engaged. This only narrows the
existing native eligibility gate and is a conservative subset of the legacy
behavior, not full V207/V234 parity.
The native selfdrived pedal disengagement remains unchanged. The legacy
independent lateral engagement and automatic re-arm are not connected to
live controls; the offline reference is not an approved vehicle port.

## Required before promotion

- [ ] Final release revision and exact Panda/opendbc/tinygrad commits pinned,
  including the actual reviewed Hybrid source changes in deployable dependencies
  (the staging source branch keeps its original pins; the separate desktop review branch uses the verified Hybrid pins).
- [ ] Fresh full desktop build, ARM F4/H7 builds, schema and regression tests
  on the exact candidate and submodule revisions.
- [x] TSFDOM desktop compile and short QCOM execution, as recorded above.
- [ ] Native action semantics and recorded behavior regression; approved model
  selection across restarts with truthful effective-runtime status.
- [ ] Real recorded Tesla HW2 CAN replay, brake/gas/regen override, physical
  stalk and AEB forwarding, dual-Panda topology and handback independently
  reviewed.
- [ ] V279 interactive offroad sign widget reviewed; live cruise SET integration
  not yet implemented/validated. No automatic engagement, no alert obstruction,
  no stale manual request after transitions, and verified vehicle feedback
  required before any live controls.
- [ ] Independent safety review, isolated hardware qualification and recorded
  release decision. No public-road testing of an unqualified build.
- [ ] Preserve existing xnor-dev revision and device settings; prepare a
  tested recovery plan. No uninstall/reinstall is required merely to change
  branches. Do not merge, flash or install this staging revision.

## Current UI scope

Native modeld now reads the persistent preference at startup; unqualified,
missing or failed candidate artifacts fall back to stock. The settings page
provides a persistent requested model preference separately
from the effective native runtime and a non-actuating V279 demonstration.
The speed signs use memory-only selections and display the configured
offset, not a CAN-confirmed Tesla set speed. NSL is labelled 70 mph for the
motorway preset, not a general UK road limit. The HUD and live cruise command
paths are unchanged.

## Promotion policy

After the above gates pass, pin the reviewed release commit and verify
the production branch has not advanced unexpectedly. Promote the reviewed
candidate through a PR with the exact evidence linked. Keep the current
working device install until qualification and approval. Do not treat this
document or any checked-off source-only item as permission to actuate.
