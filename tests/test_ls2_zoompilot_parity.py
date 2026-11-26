#!/usr/bin/env python3
"""Dependency-free regression guards for LS2 Zoompilot/Clarity parity work."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools/ls2_recorded_parity.py"


def _literal_assignment(source: str, name: str):
  tree = ast.parse(source)
  for node in tree.body:
    if isinstance(node, ast.Assign):
      for target in node.targets:
        if isinstance(target, ast.Name) and target.id == name:
          return ast.literal_eval(node.value)
  raise AssertionError(f"{name} not found")


class LS2ZoompilotParity(unittest.TestCase):
  def test_real_road_gate_covers_reported_heads(self):
    source = TOOL_PATH.read_text()
    self.assertEqual(
      _literal_assignment(source, "REQUIRED_HEADS"),
      ("plan", "lead", "lead_prob", "road_edges", "lane_lines", "action"),
    )
    self.assertEqual(
      _literal_assignment(source, "LS2_SHA256"),
      "909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8",
    )
    self.assertEqual(_literal_assignment(source, "LS2_NBYTES"), 768_749_892)
    self.assertGreaterEqual(_literal_assignment(source, "MIN_CORR"), 0.999)

  def test_recorded_gate_requires_32_frames_and_real_cameras(self):
    source = TOOL_PATH.read_text()
    self.assertIn('if args.n < 32:', source)
    self.assertIn('FrameReader(args.fcamera, pix_fmt="nv12"', source)
    self.assertIn('FrameReader(args.ecamera, pix_fmt="nv12"', source)
    self.assertIn('_record_context(args.rlog)', source)
    self.assertIn('warped = np.array(warp.run_arrays(', source)
    self.assertNotIn("make_inputs(", source)

  def test_nv12_repacker_restores_camerad_layout(self):
    source = TOOL_PATH.read_text()
    self.assertIn("get_nv12_info(width, height)", source)
    self.assertIn("tight = width * height * 3 // 2", source)
    self.assertIn("y_dst[:height, :width] = y", source)
    self.assertIn("uv_dst[:height // 2, :width] = uv", source)

  def test_authority_uses_standard_action_slice(self):
    authority = (ROOT / "openpilot/jetlink_authority.py").read_text()
    modeld = (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text()
    self.assertIn("'action': (2062, 2066)", authority)
    self.assertNotIn("def get_ls2_action_from_model(", modeld)
    self.assertIn('action_fn = getattr(model, "get_action_from_model", get_action_from_model)', modeld)

  def test_authority_uses_native_0112_warp_not_v58_indices(self):
    native = (ROOT / "openpilot/jetlink_native_authority.py").read_text()
    self.assertIn("class NativeModeldWarp:", native)
    self.assertIn("self._model.warp(", native)
    self.assertIn("WARP_INPUTS", native)
    self.assertIn("def run_arrays(", native)
    self.assertNotIn("warp_indices(", native)
    self.assertNotIn("class ExistingCameraWarp", native)

  def test_ls2_bypasses_dec_and_release_state(self):
    planner = (ROOT / "openpilot/selfdrive/controls/lib/longitudinal_planner.py").read_text()
    self.assertIn("ls2_parity_active = bool(self._xnor_ls2_session and sm['modelV2'].big)", planner)
    self.assertIn("and not ls2_parity_active", planner)
    self.assertIn("self._xnor_release_prev_e2e_stop = False", planner)
    self.assertIn("self._xnor_release_stop_frames = 0", planner)
    self.assertIn("self._xnor_release_boost_frames = 0", planner)
    self.assertIn("[XNOR_LS2_PARITY] dec=bypassed release=bypassed", planner)

  def test_reference_contract_comes_from_unmodified_onnx(self):
    source = TOOL_PATH.read_text()
    self.assertIn("spec_from_onnx(str(onnx)", source)
    self.assertIn("if digest != LS2_SHA256:", source)
    self.assertIn("if spec.output_slices != link_spec.output_slices:", source)
    for head in ("plan", "lead", "lead_prob", "road_edges", "lane_lines", "action"):
      self.assertIn(f'"{head}"', source)


if __name__ == "__main__":
  unittest.main(verbosity=2)
