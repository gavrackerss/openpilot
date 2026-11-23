#!/usr/bin/env python3
"""Source-contract tests for complete staged XNOR 0.11.2 Tesla Hybrid Panda."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
sys.path.insert(0, str(TOOLS))
from xnor_v0112_full_hybrid_panda_preflight import audit, BASE, NATIVE_BLOBS, SOURCES, git_blob
from xnor_v0112_export_full_hybrid_panda_patch import export, PATCH_NAMES, PATCH_FILES


class FullHybridPandaSourceContracts(unittest.TestCase):
  def test_all_native_sources_match_exact_xnor_v0112_git_blobs(self):
    for component, files in NATIVE_BLOBS.items():
      for rel, pinned_sha in files.items():
        with self.subTest(component=component, path=rel):
          self.assertEqual(git_blob(SOURCES[component] / rel), pinned_sha)

  def test_staged_complete_panda_safety_source_preflight(self):
    report = audit()
    self.assertEqual(report["native_blob_sources_verified"], 10)
    self.assertTrue(report["full_hybrid_staged"])
    self.assertTrue(report["packet_aware_fwd_present"])
    self.assertTrue(report["both_panda_drivers_ported"])
    self.assertTrue(report["genuine_stock_AEB_unmodified"])
    self.assertEqual(report["GTW_bench_MITM_default"], "OFF")
    self.assertFalse(report["compiled_safety_tests_executed"])
    self.assertFalse(report["suitable_for_vehicle_control"])

  def test_aeb_forwarding_checks_precede_mutable_hooks(self):
    safety = (BASE / "opendbc/opendbc/safety/safety.h").read_text()
    start = safety.index("int safety_fwd_hook_pkt(int bus_num, CANPacket_t *to_fwd)")
    aeb = safety.index("if (real_aeb)", start)
    mutable = safety.index("current_hooks->fwd_msg(bus_num, to_fwd)", start)
    self.assertLess(aeb, mutable)
    self.assertIn("if (block || (to_fwd->addr != before.addr)", safety)
    self.assertIn("return -1;", safety[aeb:mutable])

  def test_panda_copies_the_original_receive_frame(self):
    for controller in ("bxcan.h", "fdcan.h"):
      source = (BASE / "panda/board/drivers" / controller).read_text()
      self.assertIn("CANPacket_t to_send = to_push;", source)
      self.assertIn("safety_rx_hook(&to_push)", source)
      self.assertIn("safety_fwd_hook_pkt(bus_number, &to_send)", source)
      self.assertNotIn("safety_fwd_hook(bus_number, to_push.addr)", source)
    fdcan = (BASE / "panda/board/drivers/fdcan.h").read_text()
    self.assertIn("current_safety_mode != SAFETY_TESLA_LEGACY", fdcan)

  def test_0x659_never_directly_written_to_vehicle(self):
    source = (BASE / "opendbc/opendbc/safety/modes/tesla_legacy.h").read_text()
    self.assertIn("if (addr == 0x659)", source)
    self.assertIn("return false;\n  }\n\n  // V229", source)
    self.assertIn(".fwd_msg = tesla_legacy_fwd_msg_hook", source)

  def test_dual_panda_interface_can_set_hybrid_safety_flags(self):
    interface = (BASE / "opendbc/opendbc/car/tesla/interface.py").read_text()
    values = (BASE / "opendbc/opendbc/car/tesla/values.py").read_text()
    self.assertIn("for cfg in ret.safetyConfigs:", interface)
    self.assertIn("ret.pcmCruise = hybrid_native_ap", interface)
    self.assertIn("OP_STALK_ENABLE = 64", values)
    self.assertIn("IGNORE_STOCK_AEB = 128", values)

  def test_export_both_patches_cleanly_without_changing_gitlinks(self):
    before = subprocess.run(
      ["git", "ls-tree", "HEAD", "opendbc_repo", "panda"],
      cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout
    with tempfile.TemporaryDirectory() as tmp:
      export_dir = Path(tmp) / "patches"
      exported = export(export_dir)
      self.assertEqual(set(exported), set(PATCH_NAMES))
      self.assertIn("safety/modes/tesla_legacy.h", exported["opendbc"].read_text())
      self.assertIn("safety_fwd_hook_pkt", exported["opendbc"].read_text())
      self.assertIn("board/drivers/bxcan.h", exported["panda"].read_text())
      self.assertIn("board/drivers/fdcan.h", exported["panda"].read_text())
      self.assertIn("safety/tests/test_tesla_hybrid_packet.py", exported["opendbc"].read_text())
      self.assertGreater(exported["opendbc"].stat().st_size, 100_000)
      self.assertEqual(len(PATCH_FILES["opendbc"]), 9)
      self.assertEqual(len(PATCH_FILES["panda"]), 2)
    after = subprocess.run(
      ["git", "ls-tree", "HEAD", "opendbc_repo", "panda"],
      cwd=ROOT, capture_output=True, text=True, check=True,
    ).stdout
    self.assertEqual(before, after)

  def test_export_refuses_writes_into_submodules_and_active_openpilot(self):
    with self.assertRaises(ValueError):
      export(ROOT / "opendbc_repo")
    with self.assertRaises(ValueError):
      export(ROOT / "panda")
    with self.assertRaises(ValueError):
      export(ROOT)


if __name__ == "__main__":
  unittest.main(verbosity=2)
