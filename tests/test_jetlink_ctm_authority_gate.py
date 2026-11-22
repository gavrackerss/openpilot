#!/usr/bin/env python3
"""Production CTM selection, evidence integrity, kill switch, LS2 isolation.

All evidence in this test is DUMMY data created under TemporaryDirectory.
No installed vehicle/Pixel/modeld/manager state is ever modified.
"""
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from openpilot import jetlink_ctm_authority_gate as G
from openpilot import jetlink_ctm_adapter as A


def make_qualification(root: Path) -> None:
    evidence = root / "ctm_evidence"
    evidence.mkdir()
    checks = {}
    for key in G.REPORTS:
        data = json.dumps({"source_sha256": G.SOURCE_SHA, "passed": True,
                           "test_only": True, "name": key}).encode()
        (evidence / f"{key}.json").write_bytes(data)
        checks[key] = hashlib.sha256(data).hexdigest()
    (root / G.QUAL_NAME).write_text(json.dumps({
        "schema": 1,
        "source_sha256": G.SOURCE_SHA,
        "approved_for_vehicle_control": True,
        "reports": checks,
    }))
    (root / G.ARM_NAME).write_text(G.SOURCE_SHA + "\n")


class GateTests(unittest.TestCase):
    def test_default_local_no_remote_authority(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            self.assertFalse(G.requested(root))
            gate = G.eligible(root)
            self.assertFalse(gate.allowed)
            self.assertEqual(gate.reason, "ctm_not_requested")
            self.assertFalse(G.lease_alive(gate, root))

    def test_explicit_ctm_request_requires_qualification(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            (root / G.ARM_NAME).write_text(G.SOURCE_SHA)
            self.assertTrue(G.requested(root))
            self.assertFalse(G.eligible(root).allowed)

    def test_provisioned_fake_unit_test_evidence_only(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            make_qualification(root)
            gate = G.eligible(root)
            self.assertTrue(gate.allowed, gate.reason)
            self.assertTrue(G.lease_alive(gate, root))
            # A valid gate in this isolated test DOES NOT make global A.enabled true.
            # The production files at /data are never written here.

    def test_fail_closed_if_qualification_or_arm_revoked(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            make_qualification(root)
            gate = G.eligible(root)
            self.assertTrue(gate.allowed)
            (root / G.ARM_NAME).unlink()
            self.assertFalse(G.lease_alive(gate, root))
            (root / G.ARM_NAME).write_text(G.SOURCE_SHA)
            (root / G.KILL_NAME).touch()
            self.assertFalse(G.lease_alive(gate, root))
            self.assertFalse(G.eligible(root).allowed)

    def test_hash_mismatch_or_evidence_mutation_denied(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            make_qualification(root)
            gate = G.eligible(root)
            (root / "ctm_evidence" / "numerical_parity.json").write_text("{}")
            self.assertFalse(G.eligible(root).allowed)
            self.assertTrue(G.lease_alive(gate, root), "Lease only hashes the approval document; evidence changes require requalification on restart")

    def test_changed_approval_kills_active_lease(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            make_qualification(root)
            gate = G.eligible(root)
            manifest = root / G.QUAL_NAME
            manifest.write_text(manifest.read_text() + " ")
            self.assertFalse(G.lease_alive(gate, root))

    def test_mutually_exclusive_with_ls2_shadow_and_parked(self):
        for blocker in G.CONFLICTS:
            with self.subTest(blocker=blocker), TemporaryDirectory() as temp:
                root = Path(temp)
                make_qualification(root)
                (root / blocker).touch()
                self.assertFalse(G.eligible(root).allowed)
                self.assertIn(blocker, G.eligible(root).reason)

    def test_wrong_model_or_report_claim_rejected(self):
        with TemporaryDirectory() as temp:
            root = Path(temp)
            make_qualification(root)
            approval = root / G.QUAL_NAME
            d = json.loads(approval.read_text())
            d["source_sha256"] = "0" * 64
            approval.write_text(json.dumps(d))
            self.assertFalse(G.eligible(root).allowed)

    def test_no_auto_grant_of_qualification(self):
        source = Path(A.__file__).read_text()
        self.assertIn("authority_gate()", source)
        self.assertIn("lease_alive(approval)", source)
        self.assertIn("GuardedCTMJoiner", source)
        self.assertIn("self._demote(LOST)", source)
        self.assertNotIn("CTM_QUALIFIED.touch", source)

    def test_production_selection_preserves_ls2_and_actual_publisher(self):
        modeld = Path(__file__).resolve().parents[1] / "selfdrive/modeld/modeld.py"
        source = modeld.read_text()
        self.assertIn("if jetlink_ctm_adapter.requested():", source)
        self.assertIn("jetlink_ctm_adapter.attach_authority(", source)
        self.assertIn("model = jetlink_authority.attach(", source)
        self.assertIn("modelv2_send.modelV2.big = jetlink_authority.remote_active(model)", source)


if __name__ == "__main__":
    unittest.main()
