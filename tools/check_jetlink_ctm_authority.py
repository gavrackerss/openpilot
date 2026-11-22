#!/usr/bin/env python3
"""Read-only CTM modeld gate and live modelV2 publisher probe.

The gate is config eligibility, NOT proof modeld actually chose CTM. A fresh
modelV2.big=true is remote, but does not identify CTM unless corroborated by
modeld's 'CTM modelV2 publisher' log after startup. No markers are written.
"""
from __future__ import annotations

import argparse
import json
from openpilot.jetlink_ctm_authority_gate import (
    SOURCE_SHA, ROOT, ARM_NAME, QUAL_NAME, KILL_NAME, CONFLICTS, eligible, requested,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true",
                        help="Also read a fresh modelV2 sample (only while modeld is running)")
    args = parser.parse_args()
    approval = eligible()
    report = {
        "ctm_model": "Cinque Terre Original",
        "source_sha256": SOURCE_SHA,
        "ctm_requested": requested(),
        "ctm_gate_eligible": approval.allowed,
        "ctm_gate_reason": approval.reason,
        "arm_present": (ROOT / ARM_NAME).exists(),
        "qualification_present": (ROOT / QUAL_NAME).exists(),
        "stop_marker_present": (ROOT / KILL_NAME).exists(),
        "conflicting_markers": [s for s in CONFLICTS if (ROOT / s).exists()],
        "actual_running_model": "unknown (read modeld publisher logs to identify CTM)",
    }
    if args.live:
        try:
            from cereal import messaging
            sm = messaging.SubMaster(["modelV2"])
            for _ in range(6):
                sm.update(1000)
                if sm.updated.get("modelV2"):
                    report["modelV2_big"] = bool(sm["modelV2"].big)
                    report["modelV2_frame_id"] = int(sm["modelV2"].frameId)
                    report["modelV2_meaning"] = (
                        "remote model publisher (CTM or LS2; verify modeld startup log)"
                        if report["modelV2_big"] else "local model publisher"
                    )
                    break
            else:
                report["modelV2_status"] = "no live modelV2 frame received"
        except Exception as exc:
            report["modelV2_status"] = f"probe failed: {type(exc).__name__}: {exc}"
    print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
