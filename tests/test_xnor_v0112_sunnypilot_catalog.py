#!/usr/bin/env python3
"""Pure Python staging tests for SunnyPilot's selected-bundle preservation."""
import ast
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "migration/sunnypilot_v0112/catalog_guard.py"
spec = importlib.util.spec_from_file_location("sunnypilot_catalog_staged", MODULE)
# dataclasses resolves module through sys.modules when constructing a class.
import sys
catalog = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = catalog
spec.loader.exec_module(catalog)

VALID = {
  "displayName": "Saved alternative model",
  "ref": "saved-example",
  "minimumSelectorVersion": 20,
  "models": [{"artifact": {
    "fileName": "driving_tinygrad.pkl",
    "downloadUri": {"sha256": "a" * 64},
  }}],
}


class SunnyPilotNativeCatalogTest(unittest.TestCase):
  def test_valid_metadata_still_cannot_execute(self):
    result = catalog.audit_bundle(VALID)
    self.assertTrue(result.preserved_selection)
    self.assertEqual(result.requested_name, "Saved alternative model")
    self.assertEqual(result.metadata_issues, ())
    self.assertFalse(result.ready_for_runtime)
    self.assertEqual(result.effective_runner, "stock")

  def test_old_selector_versions(self):
    for version in (15, 20):
      with self.subTest(version=version):
        selected = dict(VALID, minimumSelectorVersion=version)
        self.assertFalse(catalog.audit_bundle(selected).metadata_issues)
    selected = dict(VALID, minimumSelectorVersion=999)
    self.assertIn("Unsupported selector version", catalog.audit_bundle(selected).metadata_issues)

  def test_bad_model_or_hash_is_never_enabled(self):
    bad = [
      dict(VALID, models=[]),
      dict(VALID, models=[{"artifact": {"fileName": "model.pkl"}}]),
      dict(VALID, models=[{"artifact": {"fileName": "../model.pkl", "downloadUri": {"sha256": "a" * 64}}}]),
      dict(VALID, models=[{"artifact": {"fileName": "/tmp/model.pkl", "downloadUri": {"sha256": "a" * 64}}}]),
      dict(VALID, models=[{"artifact": {"fileName": "model.pkl", "chunks": [{"sha256": "bad"}]}}]),
      "not a model bundle",
    ]
    for item in bad:
      with self.subTest(item=str(item)[:40]):
        result = catalog.audit_bundle(item)
        self.assertTrue(result.preserved_selection)
        self.assertTrue(result.metadata_issues)
        self.assertFalse(result.ready_for_runtime)
        self.assertEqual(result.effective_runner, "stock")

  def test_chunked_artifact_metadata_is_parsed_without_io(self):
    selected = dict(VALID, models=[{"artifact": {
      "fileName": "model.pkl",
      "chunks": [{"sha256": "a" * 64}, {"sha256": "b" * 64}],
    }}])
    self.assertEqual(catalog.audit_bundle(selected).metadata_issues, ())

  def test_never_opens_or_deletes_model_or_params(self):
    source = MODULE.read_text()
    tree = ast.parse(source)
    imports = set()
    for node in ast.walk(tree):
      if isinstance(node, ast.Import):
        imports.update(alias.name for alias in node.names)
      elif isinstance(node, ast.ImportFrom):
        imports.add(node.module)
    self.assertEqual(imports, {"__future__", "dataclasses", "re", "typing"})
    for keyword in ("put_bool(", "params.remove(", "os.remove(", "unlink(", "subprocess", "jetlink"):
      self.assertNotIn(keyword, source)

  def test_native_modeld_not_loading_staged_custom_models(self):
    native = (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text()
    self.assertNotIn("sunnypilot_v0112", native)
    self.assertNotIn("sunnypilot.models.helpers", native)
    legacy = (ROOT / "migration/sunnypilot_legacy/sunnypilot/models/helpers.py").read_text()
    self.assertIn("SUPPORTED_JSON_VERSIONS = frozenset((15, 20))", legacy)


if __name__ == "__main__":
  unittest.main(verbosity=2)
