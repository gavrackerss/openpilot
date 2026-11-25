#!/usr/bin/env python3
"""Regression guards for the live XNOR 0.11.1 Jetlink/LS2 parity port."""
from pathlib import Path
import hashlib
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
    self.assertIn("5.9.3-r4-native-jetlink-v58-camera-warp", native)
    self.assertIn("JoiningModelState", native)
    self.assertIn("JetlinkModelState", native)

    native_bytes = (ROOT / "openpilot/jetlink_native_authority.py").read_bytes()
    git_blob = b"blob " + str(len(native_bytes)).encode() + b"\0" + native_bytes
    self.assertEqual(hashlib.sha1(git_blob, usedforsecurity=False).hexdigest(),
                     "41a05abeaba6e84ae728b9d42e7032f5027f413f")

  def test_0112_adapter_changes_only_the_resident_small_model_boundary(self):
    authority = (ROOT / "openpilot/jetlink_authority.py").read_text()

    self.assertIn('hasattr(model, "input_queues")', authority)
    self.assertIn('hasattr(model, "npy")', authority)
    self.assertIn("value = self._model.run(bufs, transforms, inputs)", authority)
    self.assertIn('for name in ("img_q", "big_img_q", "feat_q", "desire_q"):', authority)
    self.assertIn("buf.copyin(memoryview(bytearray(buf.nbytes)))", authority)
    self.assertIn("from openpilot.cereal import messaging", authority)

  def test_modeld_keeps_stock_behavior_separate_from_ls2_behavior(self):
    modeld = (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text()

    self.assertIn("def get_ls2_action_from_model(", modeld)
    self.assertIn("remote_action = np.asarray(", modeld)
    self.assertIn("max(2.5, v_ego) ** 2", modeld)
    self.assertIn("v_target_1sec", modeld)
    self.assertIn("smooth_value(float(remote_action[1])", modeld)
    self.assertIn("lat_delay + DT_MDL, long_delay + DT_MDL, v_ego", modeld)
    self.assertIn("jetlink_authority.attach(", modeld)
    self.assertIn("jetlink_authority.handed_over(model)", modeld)
    self.assertIn("jetlink_authority.remote_active(model)", modeld)
    self.assertIn('setattr(model, "frame_drop_ratio", frame_drop_ratio)', modeld)

    # Stock 0.11.2 action extraction remains a separate function and path.
    self.assertIn("def get_action_from_model(", modeld)
    self.assertIn("action = get_action_from_model(", modeld)

  def test_ls2_owns_stock_modeld_only_when_both_old_markers_exist(self):
    manager = (ROOT / "openpilot/system/manager/process_config.py").read_text()

    self.assertIn("def jetlink_ls2_requested(started: bool)", manager)
    self.assertIn('/data/jetlink_xnor/AUTHORITY_ENABLED', manager)
    self.assertIn('/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED', manager)
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
