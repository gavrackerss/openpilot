#!/usr/bin/env python3
"""Offline tests: model bytes are never deserialised."""
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from openpilot.sunnypilot.models.artifact_audit import verify_artifacts


def sha(data):
  return hashlib.sha256(data).hexdigest()


def bundle(artifact):
  return {"displayName": "Test", "ref": "test", "minimumSelectorVersion": 20,
          "models": [{"artifact": artifact}]}


class TestArtifactAudit(unittest.TestCase):
  def test_direct(self):
    with tempfile.TemporaryDirectory() as d:
      root = Path(d)
      (root / "model.pkl").write_bytes(b"not a pickle")
      selection = bundle({"fileName": "model.pkl", "downloadUri": {"sha256": sha(b"not a pickle")}})
      result = verify_artifacts(selection, root)
      self.assertEqual(result.issues, ())
      self.assertEqual(result.verified_files, 1)
      self.assertFalse(result.native_abi_verified)
      self.assertEqual(result.effective_runner, "stock")
      (root / "model.pkl").write_bytes(b"corrupted")
      self.assertTrue(verify_artifacts(selection, root).issues)

  def test_chunked(self):
    with tempfile.TemporaryDirectory() as d:
      root = Path(d)
      selection = bundle({"fileName": "model.pkl", "chunks": [{"sha256": sha(b"a")}, {"sha256": sha(b"b")}]})
      (root / "model.pkl.chunkmanifest").write_text("2")
      (root / "model.pkl.chunk01of02").write_bytes(b"a")
      (root / "model.pkl.chunk02of02").write_bytes(b"b")
      self.assertEqual(verify_artifacts(selection, root).verified_bytes, 2)
      (root / "model.pkl").write_bytes(b"ab")
      self.assertTrue(verify_artifacts(selection, root).issues)
      (root / "model.pkl").unlink()
      (root / "model.pkl.chunk02of02").unlink()
      self.assertTrue(verify_artifacts(selection, root).issues)

  def test_bad_path_and_symlink(self):
    with tempfile.TemporaryDirectory() as d:
      root = Path(d)
      self.assertTrue(verify_artifacts(bundle({"fileName": "../escape", "downloadUri": {"sha256": sha(b"a")}}), root).issues)
      (root / "target").write_bytes(b"a")
      (root / "model.pkl").symlink_to(root / "target")
      self.assertTrue(verify_artifacts(bundle({"fileName": "model.pkl", "downloadUri": {"sha256": sha(b"a")}}), root).issues)


if __name__ == "__main__":
  unittest.main(verbosity=2)
