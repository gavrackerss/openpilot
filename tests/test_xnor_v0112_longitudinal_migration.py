#!/usr/bin/env python3
"""No-dependency source audit tests for last installed, DEC-free Tesla build."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
FILE = ROOT / "tools/xnor_v0112_longitudinal_migration_audit.py"
SPEC = importlib.util.spec_from_file_location("native_long_migration_audit", FILE)
audit_module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit_module)


class NativeLongitudinalMigration(unittest.TestCase):
  def test_original_installed_source_blobs_match(self):
    for filename, expected in audit_module.EXPECTED_BLOBS.items():
      with self.subTest(file=filename):
        data = (audit_module.OLD / filename).read_bytes()
        self.assertEqual(audit_module.git_blob_sha(data), expected)

  def test_native_and_installed_planners_remain_dec_free(self):
    report = audit_module.audit()
    self.assertFalse(report["installed_dec_present"])
    self.assertFalse(report["native_dec_present"])
    self.assertFalse(report["port_ready_for_vehicle_control"])

  def test_control_ownership_and_follow_distance_still_unported(self):
    report = audit_module.audit()
    self.assertTrue(report["custom_hybrid_brake_owner_original"])
    self.assertFalse(report["custom_hybrid_brake_owner_ported"])
    self.assertTrue(report["custom_follow_distance_original"])
    self.assertFalse(report["custom_follow_distance_ported"])

  def test_direct_head_parsing_needs_independent_replay(self):
    report = audit_module.audit()
    self.assertTrue(report["native_modeld_direct_action_head_present"])
    self.assertFalse(report["port_ready_for_vehicle_control"])


if __name__ == "__main__":
  unittest.main(verbosity=2)
