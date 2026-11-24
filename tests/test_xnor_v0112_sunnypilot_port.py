#!/usr/bin/env python3
"""Static/runtime-contract guard for SunnyPilot model selection on XNOR 0.11.2."""
import ast
from pathlib import Path
import stat
import unittest

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "openpilot/sunnypilot/models"
RUNTIME = ROOT / "openpilot/sunnypilot/modeld_v2"


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

  def test_downloaded_model_runtime_present_and_compiles(self):
    for basename in ("__init__.py", "constants.py", "parse_model_outputs.py",
                     "parse_model_outputs_split.py", "runtime_helpers.py", "modeld.py"):
      with self.subTest(module=basename):
        path = RUNTIME / basename
        self.assertTrue(path.is_file(), path)
        compile(path.read_text(), str(path), "exec")

    wrapper = RUNTIME / "modeld"
    self.assertTrue(wrapper.is_file())
    self.assertTrue(wrapper.stat().st_mode & stat.S_IXUSR)
    wrapper_text = wrapper.read_text()
    self.assertIn("preserving selection and using stock modeld", wrapper_text)
    self.assertNotIn('remove("ModelManager_ActiveBundle")', wrapper_text)
    self.assertNotIn("remove('ModelManager_ActiveBundle')", wrapper_text)

  def test_runtime_uses_current_0112_interfaces(self):
    runtime = (RUNTIME / "modeld.py").read_text()
    self.assertIn("VISION_STREAM_NARROW_ROAD", runtime)
    self.assertIn('"narrowRoadCameraState"', runtime)
    self.assertIn('"extrinsicsCalibration"', runtime)
    self.assertIn('"lateralDelay"', runtime)
    self.assertIn("open_file_chunked", runtime)
    self.assertIn("fill_driving_model_data", runtime)
    self.assertIn("should_stop", runtime)
    self.assertNotIn("roadCameraState", runtime)
    self.assertNotIn("liveCalibration", runtime)
    self.assertNotIn("read_file_chunked", runtime)

  def test_manager_enforces_single_modeld_owner(self):
    processes = (ROOT / "openpilot/system/manager/process_config.py").read_text()
    self.assertIn("get_active_model_runner", processes)
    self.assertIn("def is_tinygrad_model", processes)
    self.assertIn("def is_stock_model", processes)
    self.assertIn('PythonProcess("modeld", "openpilot.selfdrive.modeld.modeld", and_(only_onroad, is_stock_model))', processes)
    self.assertIn('NativeProcess("modeld_tinygrad", "openpilot/sunnypilot/modeld_v2"', processes)
    self.assertIn('PythonProcess("models_manager", "openpilot.sunnypilot.models.manager", only_offroad)', processes)
    self.assertIn("return not is_tinygrad_model", processes)
    self.assertIn("COMMA_HARDWARE and _safe_model_runner", processes)

  def test_saved_selection_is_verified_and_persistent(self):
    params = (ROOT / "openpilot/common/params_keys.h").read_text()
    helpers = (PKG / "helpers.py").read_text()
    self.assertIn('{"ModelManager_ActiveBundle", {PERSISTENT, JSON}}', params)
    self.assertIn('{"ModelRunnerTypeCache", {CLEAR_ON_ONROAD_TRANSITION, INT}}', params)
    self.assertIn("def get_active_model_runner(", helpers)
    self.assertIn("_bundle_integrity_issues", helpers)
    self.assertNotIn("params.remove('ModelManager_ActiveBundle')", helpers)
    self.assertNotIn('params.remove("ModelManager_ActiveBundle")', helpers)

  def test_model_manager_schema_service_exists(self):
    custom = (ROOT / "openpilot/cereal/custom.capnp").read_text()
    services = (ROOT / "openpilot/cereal/services.py").read_text()
    self.assertIn("struct ModelManagerSP @0xaedffd8f31e7b55d", custom)
    self.assertIn('tinygrad @1;', custom)
    self.assertIn('"modelManagerSP": (False, 1., 1, QueueSize.BIG)', services)


if __name__ == "__main__":
  unittest.main(verbosity=2)
