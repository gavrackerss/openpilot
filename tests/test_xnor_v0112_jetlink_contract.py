#!/usr/bin/env python3
"""Source and functional tests for XNOR-native 0.11.2 passive Jetlink port."""
import ast
import importlib.util
import math
from pathlib import Path
import struct
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "migration/jetlink_xnor_v0112/native_contract.py"
SPEC = importlib.util.spec_from_file_location("xnvr_jetlink_contract", MODULE_PATH)
module = importlib.util.module_from_spec(SPEC)
import sys
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class FakeSmall:
  usbgpu = False
  frame_skip = 4
  vision_input_names = ["big_img", "img"]
  input_queues = {}
  npy = {"desire": object(), "traffic_convention": object(), "action_t": object()}
  prev_desire = SimpleNamespace(shape=(8,))
  run_policy = staticmethod(lambda: None)
  warp = staticmethod(lambda: None)

  def run(self, bufs, transforms, inputs):
    raise AssertionError("No small-model inference is allowed in ABI tests")


class NativeXnorJetlinkContracts(unittest.TestCase):
  def test_resident_small_validates_without_running(self):
    module.validate_resident_xnor_model(FakeSmall())

  def test_old_four_argument_run_is_incompatible(self):
    class OldSignature(FakeSmall):
      def run(self, bufs, transforms, inputs, after_enqueue=None):
        raise AssertionError("Should not run")
    with self.assertRaisesRegex(ValueError, "requires run"):
      module.validate_resident_xnor_model(OldSignature())

  def test_official_model_state_is_rejected(self):
    x = FakeSmall()
    x.state_pairs = {"history": "next_history"}
    with self.assertRaisesRegex(ValueError, "Official-comma state ABI"):
      module.validate_resident_xnor_model(x)

  def test_wrong_runner_and_context_rejected(self):
    for field, value in (("usbgpu", True), ("frame_skip", 1),
                         ("vision_input_names", ["img"]),
                         ("npy", {"desire": 1}), ("prev_desire", SimpleNamespace(shape=(16,)))):
      with self.subTest(field=field):
        x = FakeSmall()
        setattr(x, field, value)
        with self.assertRaises(ValueError):
          module.validate_resident_xnor_model(x)

  def test_exact_finite_raw_head_extraction(self):
    floats = [0.] * module.PIXEL_REPLY_FLOATS
    floats[2062:2066] = [1.25, -0.4, 0.75, -0.25]
    floats[2066:2068] = [0.0, 0.0]
    packet = struct.pack("<2068f", *floats)
    report = module.inspect_pixel_reply(packet)
    self.assertEqual(report.raw_head, (1.25, -0.4000000059604645, 0.75, -0.25))
    self.assertEqual(report.tail_padding, (0.0, 0.0))
    self.assertEqual(report.xnor_action_width, 2)
    self.assertFalse(report.authority_allowed)
    self.assertFalse(report.native_action_semantics_proven)

  def test_rejects_short_nonfinite_and_unrecognised_packets(self):
    good = struct.pack("<2068f", *([0.] * 2068))
    for packet in (good[:-4], bytearray(good), b"", b"\0" * (len(good) + 4)):
      with self.subTest(length=len(packet)):
        with self.assertRaises(ValueError):
          module.inspect_pixel_reply(packet)
    for bad in (math.nan, math.inf, -math.inf):
      floats = [0.] * 2068
      floats[917] = bad
      with self.assertRaises(ValueError):
        module.inspect_pixel_reply(struct.pack("<2068f", *floats))

  def test_no_mechanism_can_enable_authority(self):
    with self.assertRaisesRegex(RuntimeError, "unqualified"):
      module.assert_authority_disabled()
    doc = MODULE_PATH.read_text()
    tree = ast.parse(doc)
    imports = set()
    for node in ast.walk(tree):
      if isinstance(node, ast.Import):
        imports.update(a.name for a in node.names)
      if isinstance(node, ast.ImportFrom):
        imports.add(node.module)
    self.assertEqual(imports, {"__future__", "dataclasses", "inspect", "math", "struct", "typing"})
    self.assertNotIn("socket", doc)
    self.assertNotIn("subprocess", doc)
    self.assertNotIn("CANParser", doc)

  def test_native_xnor_model_abi_matches_checkout(self):
    native = (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text()
    tree = ast.parse(native)
    model_cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ModelState")
    run = next(n for n in model_cls.body if isinstance(n, ast.FunctionDef) and n.name == "run")
    self.assertEqual([arg.arg for arg in run.args.args], ["self", "bufs", "transforms", "inputs"])
    self.assertNotIn("after_enqueue", [arg.arg for arg in run.args.args])
    self.assertIn("self.input_queues, self.npy = make_input_queues(", native)

  def test_xnor_model_width_differs_from_pixel_raw_head(self):
    constants = (ROOT / "openpilot/selfdrive/modeld/constants.py").read_text()
    self.assertIn("ACTION_WIDTH = 2", constants)
    old = (ROOT / "migration/jetlink_adapters_legacy/jetlink_authority.py").read_text()
    self.assertIn("REPLY_FLOATS = 2068", old)
    self.assertIn("'action' exists at 2062:2066", old)

  def test_not_hooked_into_modeld_or_manager(self):
    runtime = (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text()
    manager = (ROOT / "openpilot/system/manager/process_config.py").read_text()
    self.assertNotIn("jetlink_xnor_v0112", runtime + manager)


if __name__ == "__main__":
  unittest.main(verbosity=2)
