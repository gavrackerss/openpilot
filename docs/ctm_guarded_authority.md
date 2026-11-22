# CTM Original: guarded production stock modeld integration

**Implemented in xnor-dev. NOT approved or automatically enabled for steering.**

The adapter is a separately selected model path in
\`selfdrive/modeld/modeld.py\`, alongside the unchanged LS2 path.
The installed local stock model remains resident and is the default.
With an explicit CTM request and an independently reviewed qualification,
\`modeld\` can instantiate the CTM-specific native
\`JetlinkClient -> JetlinkModelState -> JoiningModelState\`.
Zoompilot manages joining, holding late frames and fallback to stock.
The source SHA is pinned to original CTM:
\`e8d821733be15ebe9e27498bc27ad8bbbd741980ece37d77f377294010b8ff28\`.

## Choice precedence

| CTM request | Qualification/evidence | Result at modeld startup |
| --- | --- | --- |
| absent | any | Existing stock/LS2 selection is unchanged |
| present | missing, mismatched or incomplete | Local stock only; LS2 is NOT selected implicitly |
| present | independently verified and accepted | Guarded CTM native joiner may connect; local stock remains fallback |
| present, then removed/STOP added | previously valid | CTM demotes on the next frame and latches revoked until modeld restarts |

CTM cannot be selected by the Sunnypilot on-screen model name alone.
This path supports the *stock Python modeld* only. The tinygrad modeld_v2
runner has not been qualified for CTM; the existing selector is unchanged.

## Gate contract

The gate is \`openpilot/jetlink_ctm_authority_gate.py\`. It does NOT generate
qualification records. Its per-frame lease is deliberately separate from the
LS2 authority gate.

- \`/data/jetlink_xnor/CTM_AUTHORITY_ENABLED\`: opt-in request whose entire
  text must be the original CTM source SHA above.
- \`/data/jetlink_xnor/CTM_AUTHORITY_QUALIFIED\`: a JSON approval record,
  independently created after numerical parity, actual trajectory semantics,
  on-road shadow and fallback review. The gate checks model identity, explicit
  approval, and three supporting SHA-256-verified reports.
- \`/data/jetlink_xnor/ctm_evidence/numerical_parity.json\`
- \`/data/jetlink_xnor/ctm_evidence/onroad_shadow.json\`
- \`/data/jetlink_xnor/ctm_evidence/parked_fallback.json\`
- \`/data/jetlink_xnor/CTM_AUTHORITY_STOP\`: unconditional kill switch.
- Conflict markers: \`CTM_AUTHORITY_NOT_QUALIFIED\`,
  \`CTM_PARKED_TEST_ENABLED\`, \`AUTHORITY_ENABLED\`,
  \`LS2_AUTHORITY_QUALIFIED\`, \`ONROAD_SHADOW_ENABLED\`.

Each evidence JSON must identify the original CTM SHA and have \`passed: true\`
and a hash matching the qualification's \`reports\` section. A passing field
and hash **do not prove** scientific parity or safety: the evidence and approval
must be independently assessed before they are supplied. The gate checks
file identity and tampering, not independent test quality or authenticity.

The approval record format is schema 1:
\`\`\`json
{
  "schema": 1,
  "source_sha256": "<original CTM SHA256>",
  "approved_for_vehicle_control": false,
  "reports": {
    "numerical_parity": "<sha256 of reviewed numerical_parity.json>",
    "onroad_shadow": "<sha256 of reviewed onroad_shadow.json>",
    "parked_fallback": "<sha256 of reviewed parked_fallback.json>"
  }
}
\`\`\`
The example deliberately has \`approved_for_vehicle_control: false\`.
Changing it to true without real verification is not qualification.

The two positive markers never appear in the repository and are never created
by the adapter, CI, startup code or status scripts. Therefore **installing this
code alone cannot authorise CTM steering.**

## Installation and read-only verification

While the car is safely parked/off-road, update the checkout:

\`\`\`bash
cd /data/openpilot
git pull --ff-only origin xnor-dev
python3 -m unittest discover -s tests -p 'test_jetlink_ctm_contract.py' -v
python3 -m unittest discover -s tests -p 'test_jetlink_ctm_authority_gate.py' -v
python3 tools/check_jetlink_ctm_authority.py
\`\`\`

The status tool reports CTM request, qualified/eligible state, blockers
and, with \`--live\`, the fresh \`modelV2.big\` remote/local bit. That bit alone
does NOT identify which remote model is publishing; corroborate using the
\`CTM modelV2 publisher: Cinque Terre Tensor G6\` **modeld log**
after a qualified guarded join. During ordinary installation the gate will
report \`ctm_not_requested\` or a blocked reason; that is intended.

The code does not modify LS2's model identifier, Tensor G6 Android APK,
Panda, the selected Sunnypilot model bundle or vehicle controls directly.

## Revoking the CTM joiner

A modeld CTM authority request may be revoked using:

\`\`\`bash
cd /data/openpilot
bash tools/disarm_jetlink_ctm_authority.sh
python3 tools/check_jetlink_ctm_authority.py --live
\`\`\`

This places STOP **before** removing the arm marker. In a running CTM joiner
the next inference frame rechecks the lease, demotes to the resident local
model, stops further joins and closes the remote client off the realtime
thread. If \`modeld\` is blocked, unhealthy, or no frames are being consumed,
the shell command alone cannot guarantee physical fallback. Check the vehicle
state and modeld runtime logs; do not use a file-based kill switch as a
substitute for normal vehicle disengagement.

The gate also checks qualification-file contents and the original evidence
files' inode/size/mtime on every frame, without re-hashing megabytes
inside the realtime model loop. Revocation is irreversible within that
\`modeld\` process: approval cannot be toggled back during a drive.

## Remaining validation before actual authority approval

- Real CTM model-output numeric parity against a trusted reference, not just
  expected output shapes. In particular, plan, lateral curvature and action
  semantics must be checked.
- Sustained comma camera-to-reply and whole modeld frame deadlines under
  vehicle-like background load, including held frames and data stalls.
- Independent on-road shadow with the local model authoritative and
  trajectory/planner plausibility checked across real road scenarios.
- Fault injection of USB loss, slow replies, malformed outputs and runtime
  marker revocation, with the real local model still able to publish.
- Review of the exact checkout, transport, model source and approval reports.

A parked 3-frame CTM handover PASS and one sub-50 ms shadow benchmark **do not**
alone meet these qualifications. When a gate is not satisfied, leave the local
model in charge. No unreviewed driving-authority activation procedure is
provided by this integration.
