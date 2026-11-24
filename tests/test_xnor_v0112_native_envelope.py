#!/usr/bin/env python3
"""Dependency-free test of native OOB framing; no inference or unpickling."""
import io
import pickle
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from openpilot.sunnypilot.models.native_envelope import inspect_envelope


def envelope(opcodes, *buffers):
  return struct.pack("<q", len(opcodes)) + opcodes + b"".join(
    struct.pack("<q", len(buf)) + buf for buf in buffers)


class NativeEnvelopeTests(unittest.TestCase):
  def test_normal_pickle_is_not_native_envelope(self):
    with self.assertRaises(ValueError):
      inspect_envelope(io.BytesIO(pickle.dumps({}, protocol=5)))

  def test_valid_oob_framing(self):
    ops = pickle.dumps(pickle.PickleBuffer(bytearray(b"abc")), protocol=5,
                       buffer_callback=lambda _: None)
    result = inspect_envelope(io.BytesIO(envelope(ops, b"abc")))
    self.assertEqual(result.buffer_count, 1)
    self.assertEqual(result.buffer_bytes, 3)
    self.assertFalse(result.native_abi_verified)

  def test_missing_trailing_and_invalid_lengths(self):
    ops = pickle.dumps(pickle.PickleBuffer(bytearray(b"abc")), protocol=5,
                       buffer_callback=lambda _: None)
    for data in (envelope(ops), envelope(ops, b"abc")[:-1],
                 envelope(ops, b"abc") + b"x",
                 struct.pack("<q", -1), envelope(ops + b"x", b"abc"),
                 envelope(ops) + struct.pack("<q", -1)):
      with self.subTest(data=data), self.assertRaises(ValueError):
        inspect_envelope(io.BytesIO(data))

  def test_valid_header_does_not_imply_compatible_model(self):
    result = inspect_envelope(io.BytesIO(envelope(pickle.dumps({}, protocol=5))))
    self.assertEqual(result.buffer_count, 0)
    self.assertFalse(result.native_abi_verified)


if __name__ == "__main__":
  unittest.main(verbosity=2)
