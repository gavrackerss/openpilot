#!/usr/bin/env python3
"""Offline, source-only contract checks for the XNOR-native 0.11.2 base.

Never enables vehicle controls. Does not load Tinygrad, Panda, Cap'n Proto or
CAN interfaces. Runtime and safety qualification must be done separately.
"""
from __future__ import annotations

import ast
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


def source(path: str) -> str:
  return (ROOT / path).read_text(encoding="utf-8")


class XnorNativeBaseContract(unittest.TestCase):
  def test_xnor_native_0112_modeld_abi_not_official_commas(self):
    text = source("openpilot/selfdrive/modeld/modeld.py")
    model = next(
      n for n in ast.parse(text).body
      if isinstance(n, ast.ClassDef) and n.name == "ModelState"
    )
    def args(method):
      fn = next(n for n in model.body if isinstance(n, ast.FunctionDef) and n.name == method)
      return [n.arg for n in fn.args.args]
    self.assertEqual(args("__init__"), ["self", "cam_w", "cam_h", "usbgpu"])
    self.assertEqual(args("run"), ["self", "bufs", "transforms", "inputs"])
    self.assertIn("self.input_queues, self.npy = make_input_queues(", text)
    self.assertIn("self.run_policy", text)
    self.assertNotIn("after_enqueue", str(args("run")))
    self.assertNotIn("jetlink_v0112_native", text)

  def test_xnor_native_model_helpers_not_official_chestnut(self):
    text = source("openpilot/selfdrive/modeld/helpers.py")
    self.assertIn("def usbgpu_compiled()", text)
    self.assertIn("def modeld_pkl_path(usbgpu: bool)", text)
    self.assertNotIn("def chestnut_compiled()", text)

  def test_reserved_wire_ids_reused_exactly(self):
    c = source("openpilot/cereal/custom.capnp")
    log = source("openpilot/cereal/log.capnp")
    fields = (
      ("ModelManagerSP", "modelManagerSP", 108, "0xaedffd8f31e7b55d"),
      ("MapdExtendedOut", "mapdExtendedOut", 143, "0xa30662f84033036c"),
      ("MapdIn", "mapdIn", 144, "0xc86a3d38d13eb3ef"),
      ("MapdOut", "mapdOut", 145, "0xa4f1eb3323f5f582"),
    )
    for typename, field, slot, wire_id in fields:
      with self.subTest(typename=typename):
        self.assertIn(f"struct {typename} @{wire_id}", c)
        pattern = rf"\b{field}\s+@{slot}\s*:\s*Custom\.{typename}\s*;"
        self.assertRegex(log, pattern)
        self.assertEqual(len(re.findall(rf"\b{field}\s+@{slot}\s*:", log)), 1)
    self.assertNotIn("customReserved1 @108", log)
    self.assertNotIn("customReserved17 @143", log)
    self.assertNotIn("customReserved18 @144", log)
    self.assertNotIn("customReserved19 @145", log)
    for marker in ("chestnutState", "navInstructionDEPRECATED", "navRouteDEPRECATED"):
      self.assertIn(marker, log)

  def test_services_merged_not_overwritten(self):
    services = source("openpilot/cereal/services.py")
    keys = re.findall(r'^\s*"([^"]+)"\s*:', services, re.M)
    self.assertEqual(len(keys), len(set(keys)), "Duplicate service registration")
    for service in ("modelManagerSP", "mapdOut", "mapdExtendedOut", "mapdIn", "mapdCli",
                    "customReservedRawData0", "customReservedRawData1", "customReservedRawData2"):
      self.assertIn(service, keys)
    self.assertRegex(services, r'"chestnutState"\s*:\s*\(True,\s*10\.,\s*10\)')

  def test_native_params_preserved_and_extensions_unique(self):
    params = source("openpilot/common/params_keys.h")
    keys = re.findall(r'\{"([^"]+)",\s*\{', params)
    self.assertEqual(len(keys), len(set(keys)), "Duplicate Params keys")
    for name in ("UsbGpuActive", "UsbGpuLoading", "ModelManager_ActiveBundle",
                 "TinklaHybridNativeAP", "VisionSpeedLimitDetection", "JetlinkLink",
                 "JetlinkProgress", "MapdSettings"):
      self.assertIn(name, keys)
    self.assertIn('{"ExperimentalMode", {PERSISTENT, BOOL}}', params)
    self.assertIn('{"LastGPSPosition", {PERSISTENT, STRING}}', params)
    self.assertIn('{"JetlinkLink", {PERSISTENT, INT, "0"}}', params)

  def test_staged_custom_safety_code_not_activated(self):
    self.assertTrue((ROOT / "migration/tesla_opendbc_legacy/opendbc/safety/modes/tesla_legacy.h").is_file())
    self.assertTrue((ROOT / "migration/sunnypilot_legacy/sunnypilot/models/manager.py").is_file())
    runtime_modeld = source("openpilot/selfdrive/modeld/modeld.py")
    manager = source("openpilot/system/manager/process_config.py")
    self.assertNotIn("migration/", runtime_modeld + manager)
    self.assertNotIn("jetlink", runtime_modeld + manager)
    self.assertNotIn("sunnypilot_legacy", runtime_modeld + manager)


if __name__ == "__main__":
  unittest.main(verbosity=2)
