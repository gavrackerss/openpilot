#!/usr/bin/env python3
"""Model startup resolver regression tests; never load executable pickle data."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from openpilot.sunnypilot.models import native_selection as ns
from openpilot.sunnypilot.models.requested_model import STOCK, TSFDOM, OPM10V3


class NativeSelectionTests(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.root = Path(self.temp.name)

  def test_unqualified_qcom_keeps_requested_preference(self):
    result = ns.select_native(TSFDOM, None, "QCOM", self.root)
    self.assertEqual((result.requested, result.effective), (TSFDOM, STOCK))
    self.assertIsNone(result.path)
    self.assertIn("qualification pending", result.reason)

  def test_stock_and_unsupported_architectures(self):
    for saved, legacy, expected in ((STOCK, {"ref": TSFDOM}, STOCK),
                                    (None, {"ref": OPM10V3}, OPM10V3),
                                    ("other", None, "other")):
      r = ns.select_native(saved, legacy, "QCOM", self.root)
      self.assertEqual(r.requested, expected)
      self.assertEqual(r.effective, STOCK)

  def test_missing_candidate_and_wrong_backend(self):
    with patch.object(ns, "APPROVED_NATIVE_ARTIFACTS", {TSFDOM: ("model.pkl", "a"*64)}):
      self.assertEqual(ns.select_native(TSFDOM, None, "QCOM", self.root).effective, STOCK)
      (self.root / "model.pkl").write_bytes(b"fixture, not an executable artifact")
      self.assertEqual(ns.select_native(TSFDOM, None, "CPU", self.root).effective, STOCK)
      self.assertEqual(ns.select_native(TSFDOM, None, "AMD", self.root).effective, STOCK)
      r = ns.select_native(TSFDOM, None, "QCOM", self.root)
      self.assertEqual(r.effective, TSFDOM)
      self.assertEqual(r.path, self.root / "model.pkl")

  def test_invalid_filename_and_symlink_rejected(self):
    with patch.object(ns, "APPROVED_NATIVE_ARTIFACTS", {TSFDOM: ("../model.pkl", "a"*64)}):
      self.assertEqual(ns.select_native(TSFDOM, None, "QCOM", self.root).effective, STOCK)
    actual = self.root / "actual"
    actual.write_bytes(b"test")
    (self.root / "model.pkl").symlink_to(actual)
    with patch.object(ns, "APPROVED_NATIVE_ARTIFACTS", {TSFDOM: ("model.pkl", "a"*64)}):
      self.assertEqual(ns.select_native(TSFDOM, None, "QCOM", self.root).effective, STOCK)


if __name__ == "__main__":
  unittest.main(verbosity=2)
