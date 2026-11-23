#!/usr/bin/env python3
"""Dependency-free unit checks of staged Tesla Hybrid CarState/Controller logic.

Extracts the *actual function ASTs* from the staged files. Exercises the
unmodified functions without importing opendbc binaries, opening CAN, or
activating Panda. Tests source-level semantics only, not live vehicle safety.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
TESLA = ROOT / "migration/tesla_v0112/hybrid_panda_candidate/opendbc/car/tesla"


def detached_function(filename: str, class_name: str | None, function_name: str, globals_dict: dict):
  tree = ast.parse((TESLA / filename).read_text(encoding="utf-8"))
  container = next(
    (n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name),
    None
  ) if class_name else tree
  if container is None:
    raise LookupError(f"Missing class {class_name}")
  original = next((n for n in container.body if isinstance(n, ast.FunctionDef) and n.name == function_name), None)
  if original is None:
    raise LookupError(f"Missing {class_name}.{function_name}")
  # Make the actual body executable without altering the committed source.
  code = compile(ast.fix_missing_locations(ast.Module(body=[original], type_ignores=[])), filename, "exec")
  namespace = dict(globals_dict)
  exec(code, namespace)
  return namespace[function_name]


class TeslaHybridVehicleFunctionContracts(unittest.TestCase):
  @classmethod
  def setUpClass(cls):
    cls.pick_cruise = staticmethod(detached_function(
      "carstate.py", "CarState", "_pick_stock_cruise_set_u", {"math": math}
    ))
    cls.create_carrier = staticmethod(detached_function(
      "teslacan.py", None, "create_fake_das_msg", {}
    ))
    cls.emit_carrier = staticmethod(detached_function(
      "carcontroller.py", "CarController", "_emit_internal_0x659", {
        "BTN_MAIN": 2, "BTN_CANCEL": 1,
        "CANBUS": SimpleNamespace(party=0, powertrain=4),
        "LEGACY_CARS": ("TESLA_MODEL_S_HW2",),
        "CAR": SimpleNamespace(TESLA_MODEL_S_HW2="TESLA_MODEL_S_HW2"),
        "os": SimpleNamespace(path=SimpleNamespace(exists=lambda p: False)),
        "create_fake_das": cls.create_carrier,
      }
    ))
    cls.compute_owner = staticmethod(detached_function(
      "carcontroller.py", "CarController", "_v229_compute_owner", {
        "structs": SimpleNamespace(
          CarState=SimpleNamespace(GearShifter=SimpleNamespace(drive="drive"))
        ),
      }
    ))

  def test_native_cruise_setpoint_is_not_doubled(self):
    data = {"DI_cruiseSet": 35.0, "DI_digitalSpeed": 70.0}
    self.assertEqual(self.pick_cruise(None, data, 31., True, "MPH"),
                     (35.0, "DI_cruiseSet_native_0112"))
    self.assertEqual(self.pick_cruise(None, data, 31., False, "MPH"),
                     (70.0, "DI_digitalSpeed_native_0112"))
    self.assertEqual(self.pick_cruise(None, {"DI_cruiseSet": 0., "DI_digitalSpeed": 70.}, 0., True, "MPH"),
                     (0., "DI_cruiseSet_unavailable"))

  def test_cruise_rejects_unknown_nonfinite_and_out_of_range(self):
    for value in (float("nan"), float("inf"), -10., 300., "bad"):
      with self.subTest(value=value):
        out = self.pick_cruise(None, {"DI_cruiseSet": value, "DI_digitalSpeed": 80.}, 30., True, "MPH")
        self.assertEqual(out[0], 0.)

  def test_carrier_bit_fields_and_bus_identity(self):
    addr, data, bus = self.create_carrier(
      False, False, 4,
      stalk_main=True, hybrid_native_ap=True, v229_stopgo_owner=True
    )
    self.assertEqual(addr, 0x659)
    self.assertEqual(bus, 4)
    self.assertEqual(len(data), 8)
    self.assertEqual(data[5] & 0x4A, 0x4A)
    self.assertEqual(data[5] & 0x01, 0)
    self.assertEqual(data[4], 0)

  @staticmethod
  def dummy_controller():
    return SimpleNamespace(
      frame=0,
      _op659_prev_btn=0,
      _v229_owner_last_sent=None,
      _native_alc_last_sent=0,
      _cached_pedal_enabled=False,
      _cached_autopilot_disabled=False,
      _cached_hybrid_native_ap=True,
      _cached_autosteer_247_test=False,
      _v213_offhighway_alc_enable=False,
      _v216_acc_from_zero_enable=False,
      _v217_autopilot_always_on_enable=False,
      CP=SimpleNamespace(carFingerprint="TESLA_MODEL_S_HW2"),
    )

  def test_0x659_main_cancel_edges_and_ownership_release(self):
    sender = self.dummy_controller()
    CS = SimpleNamespace(cruise_buttons=2)
    packets = []
    self.emit_carrier(sender, CS, packets, v229_stopgo_owner=True)
    self.assertEqual({p[2] for p in packets}, {0, 4})
    self.assertTrue(all(p[0] == 0x659 and p[1][5] & 0x4A == 0x4A for p in packets))

    sender.frame = 1
    packets = []
    self.emit_carrier(sender, CS, packets, v229_stopgo_owner=True)
    self.assertEqual(packets, [])  # no duplicate main edge

    CS.cruise_buttons = 1
    packets = []
    self.emit_carrier(sender, CS, packets, v229_stopgo_owner=False)
    self.assertEqual(len(packets), 2)
    self.assertTrue(all(p[1][5] & 0x01 for p in packets))
    self.assertTrue(all((p[1][5] & 0x08) == 0 for p in packets))

  @staticmethod
  def eligible_v229_states():
    sender = SimpleNamespace(
      _v229_stopgo_selected=True, _cached_autopilot_disabled=False,
      _cached_hybrid_native_ap=True, _cached_enable_acc=True
    )
    CC = SimpleNamespace(
      enabled=True, longActive=True,
      cruiseControl=SimpleNamespace(cancel=False)
    )
    CS = SimpleNamespace(
      out=SimpleNamespace(
        stockLkas=False, canValid=True, canTimeout=False,
        gearShifter="drive", brakePressed=False, gasPressed=False,
        stockAeb=False, accFaulted=False
      ),
      enableACC=True,
    )
    return sender, CC, CS

  def test_v229_hands_back_to_native_autosteer(self):
    sender, CC, CS = self.eligible_v229_states()
    self.assertTrue(self.compute_owner(sender, CC, CS))
    CS.out.stockLkas = True
    self.assertFalse(self.compute_owner(sender, CC, CS))
    sender._v229_stopgo_selected = False
    CS.out.stockLkas = False
    self.assertFalse(self.compute_owner(sender, CC, CS))

  def test_v229_has_no_disengaged_or_pedal_override_ownership(self):
    sender, CC, CS = self.eligible_v229_states()
    for field in ("enabled", "longActive"):
      setattr(CC, field, False)
      self.assertFalse(self.compute_owner(sender, CC, CS), field)
      setattr(CC, field, True)
    CC.cruiseControl.cancel = True
    self.assertFalse(self.compute_owner(sender, CC, CS))
    CC.cruiseControl.cancel = False
    for field in ("brakePressed", "gasPressed", "stockAeb", "accFaulted"):
      setattr(CS.out, field, True)
      self.assertFalse(self.compute_owner(sender, CC, CS), field)
      setattr(CS.out, field, False)
    self.assertTrue(self.compute_owner(sender, CC, CS))

  def test_v229_requires_valid_CAN_and_drive_gear(self):
    sender, CC, CS = self.eligible_v229_states()
    CS.out.canValid = False
    self.assertFalse(self.compute_owner(sender, CC, CS))
    CS.out.canValid = True
    CS.out.canTimeout = True
    self.assertFalse(self.compute_owner(sender, CC, CS))
    CS.out.canTimeout = False
    for gear in (None, "park", "reverse", "neutral"):
      CS.out.gearShifter = gear
      self.assertFalse(self.compute_owner(sender, CC, CS))
    CS.out.gearShifter = "drive"
    self.assertTrue(self.compute_owner(sender, CC, CS))

  def test_v229_absent_CC_or_state_fails_closed(self):
    sender, CC, CS = self.eligible_v229_states()
    self.assertFalse(self.compute_owner(sender, None, CS))
    self.assertFalse(self.compute_owner(sender, CC, SimpleNamespace(out=None)))
    self.assertFalse(self.compute_owner(sender, CC, SimpleNamespace(out=SimpleNamespace())))


if __name__ == "__main__":
  unittest.main(verbosity=2)
