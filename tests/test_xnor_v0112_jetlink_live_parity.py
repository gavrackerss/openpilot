#!/usr/bin/env python3
"""Regression guards for the live XNOR 0.11.1 Jetlink/LS2 parity port."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
LIVE = ROOT / "jetlink"
ARCHIVED = ROOT / "migration/jetlink_legacy"


class JetlinkLiveParity(unittest.TestCase):
  def test_live_jetlink_core_is_exact_preserved_0111_source(self):
    archived = sorted(p.relative_to(ARCHIVED) for p in ARCHIVED.rglob("*") if p.is_file())
    live = sorted(p.relative_to(LIVE) for p in LIVE.rglob("*") if p.is_file())
    self.assertEqual(live, archived)
    for rel in archived:
      with self.subTest(path=str(rel)):
        self.assertEqual((LIVE / rel).read_bytes(), (ARCHIVED / rel).read_bytes())

  def test_confirmed_ls2_identity_and_markers_are_preserved(self):
    authority = (ROOT / "openpilot/jetlink_authority.py").read_text()
    native = (ROOT / "openpilot/jetlink_native_authority.py").read_text()

    self.assertIn("VERSION = '5.9.3-xnor-native-jetlink-ls2'", authority)
    self.assertIn("909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8", authority)
    self.assertIn("AUTHORITY_ENABLED", authority)
    self.assertIn("LS2_AUTHORITY_QUALIFIED", authority)
    self.assertIn("'action': (2062, 2066)", authority)
    self.assertIn("5.9.4-r1-native-modeld-warp-parity", native)
    self.assertIn("JoiningModelState", native)
    self.assertIn("JetlinkModelState", native)
    self.assertIn("if msg_type == P.Msg.INFER_REQ and self.datagrams:", native)
    self.assertIn("self.send_datagrams(msg_type, seq, parts, flags)", native)

  def test_0112_adapter_changes_only_the_resident_small_model_boundary(self):
    authority = (ROOT / "openpilot/jetlink_authority.py").read_text()

    self.assertIn('hasattr(model, "input_queues")', authority)
    self.assertIn('hasattr(model, "npy")', authority)
    self.assertIn("value = self._model.run(bufs, transforms, inputs)", authority)
    self.assertIn('for name in ("img_q", "big_img_q", "feat_q", "desire_q"):', authority)
    self.assertIn("tensor.assign(0).realize()", authority)
    self.assertIn("from openpilot.cereal import messaging", authority)

  def test_modeld_uses_active_model_face_for_ls2_action(self):
    modeld = (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text()

    self.assertNotIn("def get_ls2_action_from_model(", modeld)
    self.assertIn("jetlink_authority.attach(", modeld)
    self.assertIn("jetlink_authority.handed_over(model)", modeld)
    self.assertIn("jetlink_authority.remote_active(model)", modeld)
    self.assertIn('setattr(model, "frame_drop_ratio", frame_drop_ratio)', modeld)
    self.assertIn("def get_action_from_model(", modeld)
    self.assertIn('action_fn = getattr(model, "get_action_from_model", get_action_from_model)', modeld)
    self.assertIn("action = action_fn(model_output, prev_action, lat_action_t, long_action_t, v_ego)", modeld)

  def test_native_ls2_uses_resident_0112_compiled_warp(self):
    native = (ROOT / "openpilot/jetlink_native_authority.py").read_text()
    self.assertIn("class NativeModeldWarp:", native)
    self.assertIn("self._model.warp(", native)
    self.assertIn("from openpilot.selfdrive.modeld.compile_modeld import WARP_INPUTS", native)
    self.assertIn('self._model.npy["tfm"][:, :] = tfm', native)
    self.assertIn('self._model.npy["big_tfm"][:, :] = big_tfm', native)
    self.assertNotIn("warp_indices(", native)
    self.assertNotIn("ExistingCameraWarp", native)

  def test_ls2_parity_bypasses_xnor_dec_and_release_shaping(self):
    planner = (ROOT / "openpilot/selfdrive/controls/lib/longitudinal_planner.py").read_text()
    self.assertIn("ls2_parity_active = bool(self._xnor_ls2_session and sm['modelV2'].big)", planner)
    self.assertIn("dec_active = bool(experimental_mode and self._xnor_dec.enabled() and not ls2_parity_active)", planner)
    self.assertIn("and not ls2_parity_active", planner)
    self.assertIn("[XNOR_LS2_PARITY] dec=bypassed release=bypassed", planner)

  def test_ls2_owns_stock_modeld_only_when_both_old_markers_exist(self):
    manager = (ROOT / "openpilot/system/manager/process_config.py").read_text()

    self.assertIn("def jetlink_ls2_requested(started: bool)", manager)
    self.assertIn('/data/jetlink_xnor/AUTHORITY_ENABLED', manager)
    self.assertIn('/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED', manager)
    self.assertIn('/data/jetlink_xnor/CTM_PARKED_TEST_ENABLED', manager)
    self.assertIn('/data/jetlink_xnor/CTM_AUTHORITY_ENABLED', manager)
    self.assertIn("_XNOR_LS2_SESSION_STARTED", manager)
    self.assertIn("_XNOR_LS2_SESSION_REQUESTED", manager)
    self.assertIn("_jetlink_ls2_markers_present()", manager)
    self.assertIn("if jetlink_ls2_requested(started):", manager)
    self.assertIn("return False", manager)
    self.assertIn("return jetlink_ls2_requested(started) or not is_tinygrad_model", manager)

  def test_port_does_not_create_or_auto_arm_authority_markers(self):
    paths = [
      ROOT / "openpilot/jetlink_authority.py",
      ROOT / "openpilot/jetlink_native_authority.py",
      ROOT / "openpilot/selfdrive/modeld/modeld.py",
      ROOT / "openpilot/system/manager/process_config.py",
    ]
    source = "\n".join(path.read_text() for path in paths)
    self.assertNotIn('touch("AUTHORITY_ENABLED"', source)
    self.assertNotIn('touch("LS2_AUTHORITY_QUALIFIED"', source)
    self.assertNotIn("AUTHORITY_ENABLED.write", source)
    self.assertNotIn("LS2_AUTHORITY_QUALIFIED.write", source)


if __name__ == "__main__":
  unittest.main(verbosity=2)
