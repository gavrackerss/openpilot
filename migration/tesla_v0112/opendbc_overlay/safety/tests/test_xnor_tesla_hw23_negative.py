#!/usr/bin/env python3
"""Native Panda safety negative tests for Tesla HW2 external-Panda baseline.

REQUIRES separately built opendbc.safety.tests.libsafety (not provided by
source-only GitHub CI). Run on a workstation after compiling pinned opendbc.

These tests never flash Panda or send physical CAN. The native safety
library is exercised with synthetic CANPacket_t inputs.

This is a *baseline* set: before implementing any new hybrid ownership,
the same negative cases must pass on the proposed firmware revision.
"""
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import TeslaSafetyFlags
from opendbc.safety.tests.common import CANPackerSafety, make_msg
from opendbc.safety.tests.libsafety import libsafety_py

HW2_EXTERNAL = int(TeslaSafetyFlags.FLAG_HW2 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)
LONG_MSG = 0x2BF
MAIN_CHASSIS_LONG_MSG = 0x2B9
INTERNAL_CARRIER = 0x659


class TestXnorTeslaHW2ExternalPandaNegative(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, HW2_EXTERNAL)
    self.safety.init_tests()
    self.packer = CANPackerSafety("tesla_powertrain")

  def long_msg(self, *, aeb_event: int = 0, bus: int = 0, accel: float = 0.0):
    return self.packer.make_can_msg_safety("DAS_control", bus, {
      "DAS_setSpeed": 10,
      "DAS_accState": 4,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": 0.0,
      "DAS_jerkMax": 0.0,
      "DAS_accelMin": accel,
      "DAS_accelMax": max(accel, 0.0),
    })

  def test_unrecognised_internal_carrier_cannot_transmit(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.safety.safety_tx_hook(make_msg(0, INTERNAL_CARRIER)))
    self.assertFalse(self.safety.safety_tx_hook(make_msg(0, MAIN_CHASSIS_LONG_MSG)))

  def test_openpilot_cannot_inject_aeb(self):
    self.safety.set_controls_allowed(True)
    for event in (1, 2, 3):
      with self.subTest(event=event):
        self.assertFalse(self.safety.safety_tx_hook(self.long_msg(aeb_event=event)))

  def test_native_aeb_forwards_and_blocks_op_longitudinal_tx(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.safety_rx_hook(self.long_msg(bus=2, aeb_event=0)))
    self.assertEqual(self.safety.safety_fwd_hook(2, LONG_MSG), -1)
    self.assertTrue(self.safety.safety_tx_hook(self.long_msg()))

    self.assertTrue(self.safety.safety_rx_hook(self.long_msg(bus=2, aeb_event=1)))
    self.assertEqual(self.safety.safety_fwd_hook(2, LONG_MSG), 0)
    self.assertFalse(self.safety.safety_tx_hook(self.long_msg()))

  def test_unknown_legacy_aeb_ignore_bit_does_not_change_baseline(self):
    # The pinned header does not parse bit 128 as IGNORE_STOCK_AEB.
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, HW2_EXTERNAL | 128)
    self.safety.init_tests()
    self.safety.set_controls_allowed(True)
    self.assertTrue(self.safety.safety_rx_hook(self.long_msg(bus=2, aeb_event=1)))
    self.assertEqual(self.safety.safety_fwd_hook(2, LONG_MSG), 0)
    self.assertFalse(self.safety.safety_tx_hook(self.long_msg()))

  def test_unrecognised_carrier_cannot_rearm_disengaged_controls(self):
    self.safety.set_controls_allowed(False)
    self.safety.safety_rx_hook(make_msg(0, INTERNAL_CARRIER, dat=b"\xff" * 8))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self.long_msg(accel=0.5)))


if __name__ == "__main__":
  unittest.main(verbosity=2)
