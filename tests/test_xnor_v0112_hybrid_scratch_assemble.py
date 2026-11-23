#!/usr/bin/env python3
"""Non-actuating contract tests for the 0.11.2 scratch Hybrid assembler."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

from xnor_v0112_assemble_hybrid_candidate import prohibit_live_install, PINS


class TestHybridScratchAssembly(unittest.TestCase):
  def test_refuses_any_path_inside_active_openpilot(self):
    for path in (ROOT, ROOT / "tmp/hybrid", ROOT / "migration/tesla_v0112"):
      with self.subTest(path=path), self.assertRaises(RuntimeError):
        prohibit_live_install(path)

  def test_refuses_any_existing_output(self):
    with tempfile.TemporaryDirectory() as tmp:
      with self.assertRaises(FileExistsError):
        prohibit_live_install(Path(tmp))

  def test_refuses_a_missing_output_parent(self):
    with tempfile.TemporaryDirectory() as tmp:
      with self.assertRaises(FileNotFoundError):
        prohibit_live_install(Path(tmp) / "missing-parent" / "hybrid")

  def test_accepts_new_disposable_workstation_path(self):
    with tempfile.TemporaryDirectory() as tmp:
      # no changes to original source are performed by this guard
      prohibit_live_install(Path(tmp) / "new-candidate")

  def test_sources_are_pinned_to_known_native_xnor_revisions(self):
    self.assertEqual(
      PINS["panda"], "fa83a40ad799c827ab9cbda4560cf3426531ceff"
    )
    self.assertEqual(
      PINS["opendbc_repo"], "3f86206a13c3181f0224e8f1424b836592e2b2d2"
    )

  def test_no_vehicle_install_entrypoint(self):
    source = (ROOT / "tools/xnor_v0112_assemble_hybrid_candidate.py").read_text()
    self.assertNotIn("flash_panda(", source)
    self.assertNotIn("can_send(", source)
    self.assertIn('"vehicle_authority_qualified": False', source)
    self.assertIn('"onroad_install_supported": False', source)
    self.assertIn('"device_flash_performed": False', source)
    self.assertIn("Refusing to assemble on AGNOS/comma hardware", source)


if __name__ == "__main__":
  unittest.main(verbosity=2)
