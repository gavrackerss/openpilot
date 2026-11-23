#!/usr/bin/env python3
"""Read-only regression tests for the comma 0.11.2 lead-confidence backport.

Requires XNOR's normal cereal/opendbc runtime. Do not run on-road.
Execute from the repository root:
    python3 -m unittest tests.test_xnor_lead_filter -v
"""
import unittest
from types import SimpleNamespace

from openpilot.selfdrive.controls.radard import RadarD


class FakeSM(dict):
  def __init__(self, lead1_prob, lead2_prob=0.1):
    make_lead = lambda p: SimpleNamespace(
      prob=p, x=[35.0], y=[0.0], v=[9.0], a=[0.0])
    super().__init__(
      carState=SimpleNamespace(vEgo=15.0),
      modelV2=SimpleNamespace(
        velocity=SimpleNamespace(x=[15.0]),
        leadsV3=[make_lead(lead1_prob), make_lead(lead2_prob)]),
    )
    self.seen = {"modelV2": True}
    self.logMonoTime = {"modelV2": 100, "carState": 100}
    self.recv_frame = {"carState": 1}

  def all_checks(self):
    return True

  def set_prob(self, first_prob, second_prob=0.1):
    self["modelV2"].leadsV3[0].prob = first_prob
    self["modelV2"].leadsV3[1].prob = second_prob


class LeadFilterTest(unittest.TestCase):
  def setUp(self):
    self.rd = RadarD()
    self.sm = FakeSM(0.9)
    self.rr = SimpleNamespace(points=[], errors=[])

  def update(self):
    self.rd.update(self.sm, self.rr)
    return bool(self.rd.radar_state.leadOne.status)

  def test_high_lead_probability_is_accepted_immediately(self):
    self.assertTrue(self.update())

  def test_one_frame_drop_does_not_lose_lead(self):
    self.assertTrue(self.update())
    self.sm.set_prob(0.1)
    self.assertTrue(self.update())

  def test_sustained_low_confidence_does_clear(self):
    self.assertTrue(self.update())
    self.sm.set_prob(0.1)
    for _ in range(50):
      last = self.update()
    self.assertFalse(last)

  def test_immediate_rise_after_low_confidence(self):
    self.sm.set_prob(0.1)
    self.assertFalse(self.update())
    self.sm.set_prob(0.9)
    self.assertTrue(self.update())


if __name__ == "__main__":
  unittest.main(verbosity=2)
