#!/usr/bin/env python3
"""Regression guards for LS2 Zoompilot/Clarity parity work."""
from pathlib import Path
import importlib.util
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TOOL_PATH = ROOT / "tools/ls2_recorded_parity.py"
SPEC = importlib.util.spec_from_file_location("ls2_recorded_parity", TOOL_PATH)
tool = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = tool
SPEC.loader.exec_module(tool)


class LS2ZoompilotParity(unittest.TestCase):
  def test_real_road_gate_covers_reported_heads(self):
    self.assertEqual(
      tool.REQUIRED_HEADS,
      ("plan", "lead", "lead_prob", "road_edges", "lane_lines", "action"),
    )
    self.assertEqual(tool.LS2_SHA256, "909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8")
    self.assertEqual(tool.LS2_NBYTES, 768_749_892)
    self.assertGreaterEqual(tool.MIN_CORR, 0.999)

  def test_recorded_gate_requires_32_frames_and_real_cameras(self):
    source = TOOL_PATH.read_text()
    self.assertIn('if args.n < 32:', source)
    self.assertIn('FrameReader(args.fcamera, pix_fmt="nv12"', source)
    self.assertIn('FrameReader(args.ecamera, pix_fmt="nv12"', source)
    self.assertIn('_record_context(args.rlog)', source)
    self.assertNotIn("make_inputs(", source)

  def test_nv12_repacker_preserves_visible_planes_and_padding(self):
    from openpilot.system.camerad.cameras.nv12_info import get_nv12_info

    width, height = 128, 64
    tight = np.arange(width * height * 3 // 2, dtype=np.uint8)
    padded = tool._tight_to_camerad_nv12(tight, width, height)
    stride, y_height, uv_height, size = get_nv12_info(width, height)
    self.assertEqual(padded.size, size)

    y = padded[:stride * y_height].reshape(y_height, stride)
    uv_off = stride * y_height
    uv = padded[uv_off:uv_off + stride * uv_height].reshape(uv_height, stride)
    np.testing.assert_array_equal(y[:height, :width].reshape(-1), tight[:width * height])
    np.testing.assert_array_equal(uv[:height // 2, :width].reshape(-1), tight[width * height:])

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


if __name__ == "__main__":
  unittest.main(verbosity=2)
