#!/usr/bin/env python3
"""No-dependency functional tests of read-only native SunnyPilot status CLI."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT / "tools/xnor_v0112_model_selection_status.py"
VALID = {
  "displayName": "My saved model",
  "ref": "saved-model-test",
  "minimumSelectorVersion": 20,
  "models": [{
    "artifact": {
      "fileName": "driving_tinygrad.pkl",
      "downloadUri": {"sha256": "f" * 64},
    },
  }],
}


class NativeCatalogCLI(unittest.TestCase):
  def run_offline(self, contents):
    with tempfile.TemporaryDirectory() as tmp:
      path = Path(tmp) / "bundle.json"
      path.write_text(contents, encoding="utf-8")
      return subprocess.run(
        [sys.executable, str(CLI), "--offline-json", str(path)],
        cwd=ROOT, capture_output=True, text=True, timeout=10,
      )

  def test_valid_json_remains_stock_only(self):
    result = self.run_offline(json.dumps(VALID))
    self.assertEqual(result.returncode, 0, result.stderr)
    report = json.loads(result.stdout)
    self.assertEqual(report["requested_name"], "My saved model")
    self.assertEqual(report["effective_runner"], "stock")
    self.assertTrue(report["preserved_selection"])
    self.assertFalse(report["ready_for_runtime"])
    self.assertFalse(report["catalog_manager_activated"])
    self.assertFalse(report["custom_model_authority"])
    self.assertEqual(report["metadata_issues"], [])

  def test_missing_checksum_still_preserves_choice(self):
    bad = {**VALID, "models": [{"artifact": {"fileName": "model.pkl"}}]}
    result = self.run_offline(json.dumps(bad))
    self.assertEqual(result.returncode, 0, result.stderr)
    report = json.loads(result.stdout)
    self.assertTrue(report["metadata_issues"])
    self.assertEqual(report["effective_runner"], "stock")
    self.assertTrue(report["preserved_selection"])

  def test_unsupported_selector_and_bad_json(self):
    result = self.run_offline(json.dumps({**VALID, "minimumSelectorVersion": 999}))
    self.assertEqual(result.returncode, 0)
    self.assertIn("Unsupported selector version", json.loads(result.stdout)["metadata_issues"])
    bad = self.run_offline("{broken")
    self.assertEqual(bad.returncode, 2)
    self.assertIn("Cannot inspect selection", bad.stderr)

  def test_no_writes_or_downloads_in_status_command(self):
    src = CLI.read_text()
    guard = (ROOT / "openpilot/sunnypilot/models/catalog_guard.py").read_text()
    self.assertNotIn("params.put(", src + guard)
    self.assertNotIn("params.remove(", src + guard)
    self.assertNotIn("requests.get(", src + guard)
    self.assertNotIn("socket.", src + guard)


if __name__ == "__main__":
  unittest.main(verbosity=2)
