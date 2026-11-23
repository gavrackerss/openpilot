#!/usr/bin/env python3
"""Compiled-safety negative and packet-forward tests for full Hybrid candidate.

This module is intentionally **not** run by source-only CI. It requires the
pinned XNOR opendbc C safety library rebuilt with ALL candidate source files
in an isolated, disposable worktree. No Panda firmware or vehicle connection.

The fixture calls the C dispatcher and tests copied packets; it never talks
to a real CAN device or unlocks driving authority.
"""
import unittest

from opendbc.car.structs import CarParams
from opendbc.car.tesla.values import TeslaSafetyFlags
from opendbc.safety.tests.libsafety import libsafety_py

MSG_HYBRID_CARRIER = 0x659
DAS_MAIN = 0x2B9
DAS_EXTERNAL = 0x2BF
HW2_MAIN = int(TeslaSafetyFlags.FLAG_HW2 | TeslaSafetyFlags.LONG_CONTROL | TeslaSafetyFlags.OP_STALK_ENABLE)
HW2_EXTERNAL = int(HW2_MAIN | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA)


def pkt(addr: int, bus: int, data: bytes):
  return libsafety_py.make_CANPacket(addr, bus, data)


def payload(packet) -> bytes:
  return bytes(packet[0].data[0:8])


class TestXnorHybridPandaPacket(unittest.TestCase):
  def setUp(self):
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, HW2_EXTERNAL)
    self.safety.init_tests()
    self.safety.set_timer(10_000)

  @staticmethod
  def carrier(*, hybrid=True, owner=True, main_edge=False):
    d = bytearray(8)
    if hybrid:
      d[5] |= 0x40
    if owner:
      d[5] |= 0x08
    if main_edge:
      d[5] |= 0x02
    return pkt(MSG_HYBRID_CARRIER, 0, bytes(d))

  @staticmethod
  def das(*, bus=2, aeb=0, count=3, accel_min=0., accel_max=0., set_speed=60):
    # Emit with a native CAN packer so all CRC/counter checks operate on
    # a properly formatted Tesla DAS_control payload.
    from opendbc.safety.tests.common import CANPackerSafety
    pk = CANPackerSafety("tesla_powertrain")
    return pk.make_can_msg_safety("DAS_control", bus, {
      "DAS_setSpeed": set_speed,
      "DAS_accState": 4,
      "DAS_aebEvent": aeb,
      "DAS_jerkMin": 0.,
      "DAS_jerkMax": 0.,
      "DAS_accelMin": accel_min,
      "DAS_accelMax": accel_max,
      "DAS_controlCounter": count,
    })

  def test_packet_abi_and_address_only_compatibility(self):
    self.assertTrue(hasattr(self.safety, "safety_fwd_hook_pkt"))
    frame = self.das()
    address_fwd = self.safety.safety_fwd_hook(2, DAS_EXTERNAL)
    packet_fwd = self.safety.safety_fwd_hook_pkt(2, frame)
    self.assertEqual(packet_fwd, address_fwd)
    self.assertEqual(frame[0].addr, DAS_EXTERNAL)
    self.assertEqual(frame[0].bus, 2)

  def test_no_carrier_or_unsolicited_main_can_transmission(self):
    carrier = self.carrier()
    self.assertFalse(self.safety.safety_tx_hook(carrier))
    self.assertEqual(self.safety.safety_fwd_hook_pkt(0, carrier), -1)
    self.assertFalse(self.safety.safety_tx_hook(pkt(DAS_MAIN, 0, bytes(8))))
    # The return value of the consumed internal carrier is never permission
    # to send it over CAN, even with controls_allowed manually set by test.
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.safety.safety_tx_hook(self.carrier()))

  def test_real_stock_aeb_intervention_cannot_be_rewritten(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.safety.safety_tx_hook(self.carrier()))
    aeb = self.das(aeb=1)
    before = payload(aeb)
    self.assertEqual(self.safety.safety_fwd_hook_pkt(2, aeb), 0)
    self.assertEqual(payload(aeb), before)

    # An RX of genuine AEB cancels OP longitudinal activity and its TX.
    self.assertTrue(self.safety.safety_rx_hook(self.das(aeb=1, bus=2)))
    self.assertFalse(self.safety.safety_tx_hook(self.das(bus=0)))

  def test_owner_is_not_granted_by_stale_carrier_or_invalid_template(self):
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.safety.safety_tx_hook(self.carrier()))
    original = self.das(bus=2, aeb=0)
    raw = payload(original)
    # No template and no authorisation from a 0x659 request alone.
    self.assertEqual(self.safety.safety_fwd_hook_pkt(2, original), 0)
    self.assertEqual(payload(original), raw)

    # Advance beyond owner-request freshness; cannot substitute a stale
    # payload even if userspace previously requested ownership.
    self.safety.set_timer(2_000_000)
    newer = self.das(bus=2)
    native = payload(newer)
    self.assertEqual(self.safety.safety_fwd_hook_pkt(2, newer), 0)
    self.assertEqual(payload(newer), native)

  def test_injected_aeb_event_always_rejected(self):
    self.safety.set_controls_allowed(True)
    for event in (1, 2, 3):
      with self.subTest(event=event):
        self.assertFalse(self.safety.safety_tx_hook(self.das(bus=0, aeb=event)))

  def test_real_aeb_preempts_even_legacy_warning_flag(self):
    self.safety.set_safety_hooks(
      CarParams.SafetyModel.teslaLegacy,
      HW2_EXTERNAL | int(TeslaSafetyFlags.IGNORE_STOCK_AEB),
    )
    self.safety.init_tests()
    self.safety.set_timer(10_000)
    self.safety.set_controls_allowed(True)
    self.assertFalse(self.safety.safety_tx_hook(self.carrier()))
    real = self.das(aeb=1)
    before = payload(real)
    self.assertEqual(self.safety.safety_fwd_hook_pkt(2, real), 0)
    self.assertEqual(payload(real), before)
    self.assertTrue(self.safety.safety_rx_hook(self.das(aeb=1)))
    self.assertFalse(self.safety.safety_tx_hook(self.das(bus=0)))

  def test_mode_reset_clears_prior_owner_and_rejects_carrier(self):
    self.safety.set_controls_allowed(True)
    self.safety.safety_tx_hook(self.carrier())
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, HW2_EXTERNAL)
    self.safety.init_tests()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.safety_tx_hook(self.carrier()))
    raw = self.das()
    before = payload(raw)
    self.assertEqual(self.safety.safety_fwd_hook_pkt(2, raw), 0)
    self.assertEqual(payload(raw), before)


if __name__ == "__main__":
  unittest.main(verbosity=2)
