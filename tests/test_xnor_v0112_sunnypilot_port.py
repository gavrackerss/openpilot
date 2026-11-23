#!/usr/bin/env python3
"""Static guard for staged SunnyPilot model-manager code on native XNOR 0.11.2.

This is *not* a model runner test and does not download or execute models.
"""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "openpilot/sunnypilot/models"


def imports(src):
  out = set()
  for node in ast.walk(ast.parse(src)):
    if isinstance(node, ast.ImportFrom):
      out.add(node.module)
    elif isinstance(node, ast.Import):
      out.update(alias.name for alias in node.names)
  return out


class NativeSunnyPilotPort(unittest.TestCase):
  def test_catalog_modules_present_and_compile(self):
    for basename in ("__init__.py", "constants.py", "fetcher.py", "helpers.py",
                     "manager.py", "model_name.py", "split_model_constants.py", "paths.py"):
      with self.subTest(module=basename):
        path = PKG / basename
        self.assertTrue(path.is_file(), path)
        compile(path.read_text(), str(path), "exec")

  def test_all_old_cereal_and_hardware_imports_fixed(self):
    for name in ("fetcher.py", "helpers.py", "manager.py"):
      with self.subTest(name=name):
        modules = imports((PKG / name).read_text())
        self.assertNotIn("cereal", modules)
        self.assertNotIn("openpilot.system.hardware.hw", modules)
        self.assertIn("openpilot.cereal", modules)

    self.assertIn("openpilot.common.hardware.hw", imports((PKG / "paths.py").read_text()))
    self.assertIn("openpilot.sunnypilot.models.paths", imports((PKG / "helpers.py").read_text()))
    self.assertIn("openpilot.sunnypilot.models.paths", imports((PKG / "manager.py").read_text()))

  def test_saved_user_selection_remains_persistent(self):
    params = (ROOT / "openpilot/common/params_keys.h").read_text()
    self.assertIn('{"ModelManager_ActiveBundle", {PERSISTENT, JSON}}', params)
    self.assertIn('{"ModelRunnerTypeCache", {CLEAR_ON_ONROAD_TRANSITION, INT}}', params)
    helpers = (PKG / "helpers.py").read_text()
    self.assertIn("def get_active_bundle(", helpers)
    self.assertIn("def validate_active_bundle(", helpers)
    self.assertIn("def get_active_model_runner(", helpers)
    self.assertIn("runtime must use stock", helpers)
    self.assertNotIn("params.remove('ModelManager_ActiveBundle')", helpers)
    self.assertNotIn('params.remove("ModelManager_ActiveBundle")', helpers)

  def test_model_manager_schema_service_exists(self):
    custom = (ROOT / "openpilot/cereal/custom.capnp").read_text()
    services = (ROOT / "openpilot/cereal/services.py").read_text()
    log = (ROOT / "openpilot/cereal/log.capnp").read_text()
    self.assertIn("struct ModelManagerSP @0xaedffd8f31e7b55d", custom)
    self.assertIn('"modelManagerSP": (False, 1., 1, QueueSize.BIG)', services)
    self.assertIn("modelManagerSP @108 :Custom.ModelManagerSP", log)

  def test_offroad_manager_not_registered_and_native_modeld_not_replaced(self):
    processes = (ROOT / "openpilot/system/manager/process_config.py").read_text()
    modeld = (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text()
    self.assertNotIn("sunnypilot.models.manager", processes)
    self.assertNotIn("sunnypilot.models.helpers", modeld)
    self.assertNotIn("ModelManager_ActiveBundle", modeld)
    self.assertIn("self.input_queues, self.npy = make_input_queues(", modeld)

  def test_old_modeld_v2_only_archived(self):
    self.assertTrue((ROOT / "migration/sunnypilot_legacy/sunnypilot/modeld_v2/modeld.py").is_file())
    self.assertFalse((ROOT / "openpilot/sunnypilot/modeld_v2/modeld.py").exists())


if __name__ == "__main__":
  unittest.main(verbosity=2)
