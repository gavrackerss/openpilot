#!/usr/bin/env python3
"""Offline ABI comparison regression tests."""
from pathlib import Path
import sys
import unittest
import copy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from openpilot.sunnypilot.models.native_abi import describe_metadata, compare_native_abi


class NativeABI(unittest.TestCase):
  def setUp(self):
    self.reference = describe_metadata({
      "input_shapes": {"img": (1,12,128,256), "action_t": (1,2)},
      "output_shapes": {"outputs": (1,514)},
      "output_slices": {"hidden_state": slice(0,512), "action": slice(512,514)},
    })

  def test_exact_match(self):
    self.assertEqual(compare_native_abi(self.reference, copy.deepcopy(self.reference)), [])

  def test_incompatible_action_and_recurrent_state(self):
    candidate = copy.deepcopy(self.reference)
    candidate["output_slices"]["action"] = [512,516,None]
    candidate["input_shapes"]["features_buffer"] = [1,99,512]
    self.assertEqual(len(compare_native_abi(self.reference,candidate)), 2)

  def test_missing_sections(self):
    self.assertTrue(compare_native_abi(self.reference, {}))
    self.assertTrue(compare_native_abi({}, {}))

  def test_output_names_and_ordered_slices(self):
    candidate = copy.deepcopy(self.reference)
    candidate["output_slices"]["hidden_state"] = [2,514,None]
    self.assertTrue(compare_native_abi(self.reference,candidate))


if __name__ == "__main__":
  unittest.main(verbosity=2)
