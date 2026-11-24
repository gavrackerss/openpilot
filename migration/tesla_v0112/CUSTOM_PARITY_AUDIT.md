# XNOR 0.11.1 -> 0.11.2 custom parity audit

Reference working tree: c62e642.
0.11.2 production parent: e1a98f3.
Pinned Tesla opendbc: d0f7552.

## Already present in pinned 0.11.2 opendbc
- Tesla custom CarState parameter loading and independent MAIN/CANCEL engagement latch.
- ALC/HSO helper functions and Hybrid ALC readiness.
- Tesla stock-cruise state/set-speed tracking.
- V229 stop/go controller implementation.
- V241 bounded auto-resume stalk helper.
- V279 manual cruise synchronisation helper.
- Hybrid EPAS/co-op recovery state.
- Tesla custom speed-limit and set-speed helpers.

## Missing or disconnected in 0.11.2 openpilot
- old card.py XNOR integration was dropped:
  - Tesla659Carrier injection
  - Hybrid longitudinal/direct-owner bookkeeping
  - force-fingerprint handoff inside card
  - V220 passive longitudinal owner observer
- process_config no longer starts the old mapd, vision-speed-limit reader,
  V275 shadow, Jetlink shadow, or legacy model-manager/modeld_tinygrad paths.
- live V279 manual speed-limit HUD bar is not connected.
- Tesla/Models/OSM settings navigation was removed (restored in PR8).
- sound mute params existed but were not consumed by 0.11.2 soundd.

## Authority changes intentionally not re-enabled here
The 0.11.2 migration changed selfdrived/controlsd and Tesla safety ownership.
In particular, the old V229 non-neutral low-speed priming lifecycle that armed
DI/TACC from 0 mph is rejected by the new safety gate unless longitudinal
authority is already active. Physical brake handling also takes the native
0.11.2 disengagement path before the retained V241 helper can perform the
old Hybrid auto-resume lifecycle.

Those paths affect vehicle motion/engagement and are not altered by this
restoration branch. This branch restores passive diagnostics and benign UI
preferences only, so the remaining differences can be validated independently.

DEC and LS2 remain out of scope.
