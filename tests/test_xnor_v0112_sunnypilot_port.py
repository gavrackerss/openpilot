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
    self.assertNotIn("roadCameraState", runtime)
    self.assertNotIn("liveCalibration", runtime)
    self.assertNotIn("read_file_chunked", runtime)


  def test_runtime_preserves_final_0111_action_and_recurrent_semantics(self):
    runtime = (RUNTIME / "modeld.py").read_text()

    # Final 0.11.1 SunnyPilot did NOT consume a raw action head for vehicle
    # control. Acceleration and shouldStop came from the plan trajectory.
    self.assertIn("def _legacy_get_accel_from_plan(", runtime)
    self.assertIn("v_target_1sec = np.interp(action_t + 1.0", runtime)
    self.assertIn("v_target < v_ego_stopping and v_target_1sec < v_ego_stopping", runtime)
    action_start = runtime.index("  def action(")
    action_end = runtime.index("\n\ndef main(", action_start)
    action = runtime[action_start:action_end]
    self.assertIn("_legacy_get_accel_from_plan(", action)
    self.assertNotIn('output["action"]', action)

    # Final 0.11.1 curvature path predates 0.11.2 MIN_STABLE_DELAY blending.
    self.assertIn("def _legacy_get_curvature_from_plan(", runtime)
    self.assertIn("psi_target = np.interp(action_t, t_idxs, yaws)", runtime)

    # Generation >= 11 used MLSIM semantics: recurrent desired curvature is
    # zeroed and direct desired_curvature output is ignored for action.
    self.assertIn("self.mlsim = self.generation >= 11", runtime)
    self.assertIn('outputs["desired_curvature"][0, :] if not self.mlsim else 0', runtime)
    self.assertIn('if not self.mlsim and "desired_curvature" in output:', runtime)

    # Preserve the final 0.11.1 desire-edge encoding and local input contract.
    self.assertIn("desire[0] = 0", runtime)
    self.assertIn('for key in ("traffic_convention", "lateral_control_params"):', runtime)
    self.assertNotIn('for key in ("traffic_convention", "lateral_control_params", "action_t"):', runtime)


  def test_manager_enforces_single_modeld_owner(self):
    processes = (ROOT / "openpilot/system/manager/process_config.py").read_text()
    self.assertIn("get_active_model_runner", processes)
    self.assertIn("def is_tinygrad_model", processes)
    self.assertIn("def is_stock_model", processes)
    self.assertIn('PythonProcess("modeld", "openpilot.selfdrive.modeld.modeld", and_(only_onroad, is_stock_model))', processes)
    self.assertIn('NativeProcess("modeld_tinygrad", "openpilot/sunnypilot/modeld_v2"', processes)
    self.assertIn('PythonProcess("models_manager", "openpilot.sunnypilot.models.manager", only_offroad)', processes)
    self.assertIn("return jetlink_ls2_requested(started) or not is_tinygrad_model", processes)
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

  def test_sunnypilot_helpers_defer_capnp_annotations(self):
    helpers = (PKG / "helpers.py").read_text()

    # pycapnp schema modules are not Python runtime types and cannot take part
    # in PEP 604 unions during module import. Deferred annotations keep the
    # selector/manager importable on the 0.11.2 Python runtime.
    self.assertIn("from __future__ import annotations", helpers)
    compile(helpers, str(PKG / "helpers.py"), "exec")


  def test_model_selector_uses_live_catalogue_not_migration_experiment(self):
    ui = (ROOT / "openpilot/selfdrive/ui/layouts/settings/models.py").read_text()

    self.assertIn("from openpilot.sunnypilot.models.fetcher import ModelFetcher", ui)
    self.assertIn("def _available_bundles(self):", ui)
    self.assertIn("manager.availableBundles", ui)
    self.assertIn("ModelFetcher(self._params).get_available_bundles()", ui)
    self.assertIn('"Select SunnyPilot model"', ui)
    self.assertIn('"CD210 / XNOR Default"', ui)
    self.assertIn('"ModelManager_LastSyncTime_v23"', ui)

    self.assertNotIn("Native artifact experiment", ui)
    self.assertNotIn("Compatibility runner required", ui)
    self.assertNotIn("from openpilot.sunnypilot.models.requested_model import", ui)


  def test_model_selector_exposes_full_compatible_catalogue_with_ascii_labels(self):
    fetcher = (PKG / "fetcher.py").read_text()
    ui = (ROOT / "openpilot/selfdrive/ui/layouts/settings/models.py").read_text()

    self.assertIn("def _compatible_small_model(bundle: dict) -> bool:", fetcher)
    self.assertIn('bundle.get("is_big") is not False', fetcher)
    self.assertIn('str(bundle.get("runner")) != "tinygrad"', fetcher)
    self.assertIn('str(bundle.get("generation")) != "12"', fetcher)
    self.assertIn('return bool(bundle.get("is_20hz"))', fetcher)
    self.assertNotIn("_recent_small_model", fetcher)
    self.assertNotIn("datetime.strptime", fetcher)

    self.assertIn('"Shows all compatible standard non-Chestnut SunnyPilot models.', ui)
    self.assertIn('f"{bundle.internalName} - {bundle.displayName}"', ui)
    self.assertNotIn(" — ", ui)


  def test_model_manager_schema_service_exists(self):
    custom = (ROOT / "openpilot/cereal/custom.capnp").read_text()
    services = (ROOT / "openpilot/cereal/services.py").read_text()
    self.assertIn("struct ModelManagerSP @0xaedffd8f31e7b55d", custom)
    self.assertIn('tinygrad @1;', custom)
    self.assertIn('"modelManagerSP": (False, 1., 1, QueueSize.BIG)', services)


if __name__ == "__main__":
  unittest.main(verbosity=2)
