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
    self.assertEqual(report["native_blob_sources_verified"], 15)
    self.assertTrue(report["full_hybrid_staged"])
    self.assertTrue(report["packet_aware_fwd_present"])
    self.assertTrue(report["both_panda_drivers_ported"])
    self.assertTrue(report["genuine_stock_AEB_unmodified"])
    self.assertEqual(report["GTW_bench_MITM_default"], "OFF")
    self.assertFalse(report["compiled_safety_tests_executed"])
    self.assertFalse(report["suitable_for_vehicle_control"])

  def test_aeb_forwarding_checks_precede_mutable_hooks(self):
    safety = (BASE / "opendbc/safety/safety.h").read_text()
    start = safety.index("int safety_fwd_hook_pkt(int bus_num, CANPacket_t *to_fwd)")
    aeb = safety.index("if (real_aeb)", start)
    mutable = safety.index("current_hooks->fwd_msg(bus_num, to_fwd)", start)
    self.assertLess(aeb, mutable)
    self.assertIn("if (block || (to_fwd->addr != before.addr)", safety)
    self.assertIn("if (current_safety_mode == SAFETY_TESLA_LEGACY && addr == 0x659)", safety)
    self.assertIn("return destination_bus;", safety[aeb:mutable])
    self.assertNotIn("to_fwd->data[", safety[aeb:mutable])

  def test_panda_copies_the_original_receive_frame(self):
    for controller in ("bxcan.h", "fdcan.h"):
      source = (BASE / "panda/board/drivers" / controller).read_text()
      self.assertIn("CANPacket_t to_send = to_push;", source)
      self.assertIn("safety_rx_hook(&to_push)", source)
      self.assertIn("safety_fwd_hook_pkt(bus_number, &to_send)", source)
      self.assertNotIn("safety_fwd_hook(bus_number, to_push.addr)", source)
    fdcan = (BASE / "panda/board/drivers/fdcan.h").read_text()
    self.assertIn("current_safety_mode != SAFETY_TESLA_LEGACY", fdcan)

  def test_v229_disengaged_priming_and_revoked_authority_never_modify_ap(self):
    safety = (BASE / "opendbc/safety/modes/tesla_legacy.h").read_text()
    capture = safety[safety.index("static void tesla_legacy_v229_capture_das_control("):
                     safety.index("static void tesla_legacy_v229_observe_di_state(")]
    substitute = safety[safety.index("static bool tesla_legacy_v229_apply_das_control("):
                        safety.index("// --- TX hook ---")]
    self.assertIn("if (!long_allowed) {", capture)
    self.assertIn("tesla_legacy_v229_tmpl_valid = false;", capture)
    self.assertNotIn("!long_allowed && !neutral && !priming", capture)
    self.assertIn("if (!controls_allowed || !get_longitudinal_allowed()) {", substitute)
    self.assertIn("return false;", substitute)
    smoke = (BASE / "host_tests/test_hybrid_panda.c").read_text()
    self.assertIn("test_disengaged_priming_never_rewrites_a_genuine_ap_frame", smoke)
    self.assertIn("test_captured_template_revoked_before_stock_frame_is_not_applied", smoke)

  def test_v229_controller_does_not_take_longitudinal_without_driver_permission(self):
    controller = (BASE / "opendbc/car/tesla/carcontroller.py").read_text()
    gate = controller[controller.index("def _v229_compute_owner(self, CC, CS)"):
                      controller.index("def _v229_stopgo_long(", controller.index("def _v229_compute_owner(self, CC, CS)"))]
    for token in (
      '"longActive", False', '"enabled", False', '"cancel", True',
      '"canValid", False', '"canTimeout", True', '"brakePressed", True',
      '"gasPressed", True', '"stockAeb", True', '"accFaulted", True',
      "structs.CarState.GearShifter.drive",
    ):
      self.assertIn(token, gate)

  def test_both_hardware_drivers_zero_initialise_complete_forward_copy(self):
    for driver in ("bxcan.h", "fdcan.h"):
      source = (BASE / "panda/board/drivers" / driver).read_text()
      rx = source[source.index("void can_rx("):]
      self.assertIn("CANPacket_t to_push = {0};", rx, driver)
      self.assertIn("CANPacket_t to_send = to_push;", rx, driver)
      self.assertIn("safety_rx_hook(&to_push)", rx, driver)

  def test_native_tesla_safety_preserved_without_hybrid_selection(self):
    staged = (BASE / "opendbc/safety/modes/tesla_legacy_native_0112.h").read_text()
    original = (SOURCES["opendbc"] / "opendbc/safety/modes/tesla_legacy.h").read_text()
    for src, target in (
        ("tesla_legacy_", "tesla_native_legacy_"),
        ("tesla_external_panda", "tesla_native_external_panda"),
        ("tesla_hw1", "tesla_native_hw1"),
        ("chassis_bus", "tesla_native_chassis_bus"),
        ("das_control_msg", "tesla_native_das_control_msg"),
        ("di_torque1_msg", "tesla_native_di_torque1_msg"),
    ):
      original = original.replace(src, target)
    self.assertEqual(staged, original)
    hybrid = (BASE / "opendbc/safety/modes/tesla_legacy.h").read_text()
    self.assertIn("tesla_hybrid_extension_active = GET_FLAG(param, 64U);", hybrid)
    self.assertIn("return tesla_native_legacy_hooks.init(param);", hybrid)

  def test_0x659_never_directly_written_to_vehicle(self):
    source = (BASE / "opendbc/safety/modes/tesla_legacy.h").read_text()
    self.assertIn("if (addr == 0x659)", source)
    self.assertIn("return false;\n  }\n\n  // V229", source)
    self.assertIn(".fwd_msg = tesla_hybrid_dispatch_fwd_msg", source)

  def test_dual_panda_interface_can_set_hybrid_safety_flags(self):
    interface = (BASE / "opendbc/car/tesla/interface.py").read_text()
    values = (BASE / "opendbc/car/tesla/values.py").read_text()
    self.assertIn("for cfg in ret.safetyConfigs:", interface)
    self.assertIn("ret.pcmCruise = hybrid_native_ap", interface)
    self.assertIn("OP_STALK_ENABLE = 64", values)
    self.assertIn("IGNORE_STOCK_AEB = 128", values)

  def test_native_car_schema_adds_no_colliding_tesla_tags(self):
    capnp = (BASE / "opendbc/car/car.capnp").read_text()
    native = (SOURCES["opendbc"] / "opendbc/car/car.capnp").read_text()
    self.assertIn("carNotReady @61 :Bool;", capnp)
    self.assertIn("followDistanceS @66 :UInt8;", capnp)
    self.assertIn("adaptiveCruiseEnabled @62 :Bool;", capnp)
    self.assertIn("teslaAlcHandoverReady @63 :Bool;", capnp)
    self.assertIn("teslaEacStatus @64 :UInt8;", capnp)
    self.assertIn("teslaEacError @65 :UInt8;", capnp)
    self.assertEqual(capnp.count("brake @5 :Float32;"), 1)
    self.assertIn("deprecated :group {", capnp)
    # Native schema text must survive without deleting or renumbering
    # any existing CarState entry.
    self.assertTrue(all(line in capnp.splitlines() for line in native.splitlines()))

  def test_model3_y_implementations_are_exact_xnor_source(self):
    for path, native in (
        ("carstate_native_0112.py", "carstate.py"),
        ("carcontroller_native_0112.py", "carcontroller.py")):
      with self.subTest(path=path):
        staged = (BASE / "opendbc/car/tesla" / path).read_bytes()
        pinned = (SOURCES["opendbc"] / "opendbc/car/tesla" / native).read_bytes()
        self.assertEqual(staged, pinned)

  def test_tesla_legacy_native_vm_and_no_unsupported_cruise_scaling(self):
    state = (BASE / "opendbc/car/tesla/carstate.py").read_text()
    ctrl = (BASE / "opendbc/car/tesla/carcontroller.py").read_text()
    interface = (BASE / "opendbc/car/tesla/interface.py").read_text()
    self.assertIn("apply_steer_angle_limits_vm(", ctrl)
    self.assertIn("def get_safety_CP():", ctrl)
    self.assertIn('get_non_essential_params("TESLA_MODEL_Y")', ctrl)
    self.assertIn('VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))', ctrl)
    self.assertNotIn("apply_std_steer_angle_limits(", ctrl)
    self.assertIn("DI_cruiseSet_native_0112", state)
    self.assertNotIn("_cruise_set_scale", state)
    self.assertIn("_GTW_TX_AUTOPILOT2_BUS2 = False", ctrl)
    self.assertIn("_UNITY_2B9_OVERLAY_TEMPLATE = False", ctrl)
    self.assertIn("_ALLOW_LEGACY_HUD_WARNING_OVERLAY = False", ctrl)
    self.assertIn("ignore_hud_aeb = False", interface)
    self.assertIn("def _emit_internal_0x659(", ctrl)
    self.assertIn("def _v229_stopgo_long(", ctrl)

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
      self.assertEqual(len(PATCH_FILES["opendbc"]), 24)
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
