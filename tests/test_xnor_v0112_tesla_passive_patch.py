#!/usr/bin/env python3
"""Check the review-only Tesla opendbc patch applies to pinned source."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT / "tools/xnor_v0112_export_tesla_opendbc_patch.py"


class TeslaPassivePatch(unittest.TestCase):
  def test_review_patch_applies_to_pinned_xnor(self):
    with tempfile.TemporaryDirectory() as tmp:
      output = Path(tmp) / "tesla-passive.patch"
      created = subprocess.run(
        [sys.executable, str(CLI), "--output", str(output)],
        cwd=ROOT, text=True, capture_output=True, timeout=20,
      )
      self.assertEqual(created.returncode, 0, created.stderr)
      self.assertTrue(output.is_file())
      patch = output.read_text()
      self.assertEqual(patch.count("--- a/"), 2)
      self.assertEqual(patch.count("+++ b/"), 2)
      self.assertIn("BO_ 937 DAS_telemetry", patch)
      self.assertIn("class CruiseButtons", patch)
      self.assertNotIn("opendbc/safety/modes", patch)
      self.assertNotIn("IGNORE_STOCK_AEB", patch)
      self.assertNotIn("OP_STALK_ENABLE", patch)
      self.assertNotIn("0x659", patch)
      checked = subprocess.run(
        ["git", "apply", "--check", str(output)],
        cwd=ROOT / "opendbc_repo", text=True, capture_output=True, timeout=20,
      )
      self.assertEqual(checked.returncode, 0, checked.stderr)

  def test_never_writes_inside_active_opendbc(self):
    forbidden = ROOT / "opendbc_repo" / "should-not-write.patch"
    self.assertFalse(forbidden.exists())
    denied = subprocess.run(
      [sys.executable, str(CLI), "--output", str(forbidden)],
      cwd=ROOT, text=True, capture_output=True, timeout=20,
    )
    self.assertNotEqual(denied.returncode, 0)
    self.assertIn("Refusing to write inside active", denied.stderr)
    self.assertFalse(forbidden.exists())

  def test_refuses_overwriting_existing_patch(self):
    with tempfile.TemporaryDirectory() as tmp:
      target = Path(tmp) / "unchanged.patch"
      target.write_text("do not replace")
      denied = subprocess.run(
        [sys.executable, str(CLI), "--output", str(target)],
        cwd=ROOT, text=True, capture_output=True, timeout=20,
      )
      self.assertNotEqual(denied.returncode, 0)
      self.assertEqual(target.read_text(), "do not replace")


if __name__ == "__main__":
  unittest.main(verbosity=2)
