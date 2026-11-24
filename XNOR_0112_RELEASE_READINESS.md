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

The latest TSFDOM desktop build was reported completed by the user; its exact
run, logs and results have not independently been recorded here.

## Required before promotion

- [ ] Final release revision and exact Panda/opendbc/tinygrad commits pinned,
  including the actual reviewed Hybrid source changes in deployable dependencies
  (the current live gitlinks still point at stock pins).
- [ ] Fresh full desktop build, ARM F4/H7 builds, schema and regression tests
  on the exact candidate and submodule revisions.
- [ ] Actual TSFDOM compilation/inference CI logs reviewed, native device
  backend and timing validated; supported alternative model selection verified
  across process restart and offroad/onroad transitions without false UI status.
- [ ] Real recorded Tesla HW2 CAN replay, brake/gas/regen override, physical
  stalk and AEB forwarding, dual-Panda topology and handback independently
  reviewed.
- [ ] V279 interactive offroad sign widget reviewed; live cruise SET integration
  not yet implemented/validated. No automatic engagement, no alert obstruction,
  no stale manual request after transitions, and verified vehicle feedback
  required before any live controls.
- [ ] LS2/Jetlink remains isolated pending verified output semantics and
  lead/curve braking parity; neither synthetic outputs nor an accepted
  handover alone establish longitudinal safety.
- [ ] Independent safety review, isolated hardware qualification and recorded
  release decision. No public-road testing of an unqualified build.
- [ ] Preserve existing xnor-dev revision and device settings; prepare a
  tested recovery plan. No uninstall/reinstall is required merely to change
  branches. Do not merge, flash or install this staging revision.

## Current UI scope

The settings page provides a persistent requested model preference separately
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
