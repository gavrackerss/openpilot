"""Structural audit of XNOR's native compiled pickle envelope.

No unpickling or execution. This checks framing, not model compatibility,
trustworthiness, native ABI, lead following, or vehicle readiness.
"""
from __future__ import annotations

from dataclasses import dataclass
import pickletools
import struct
from typing import BinaryIO

MAX_OPCODE_BYTES = 32 * 1024 * 1024
MAX_BUFFER_BYTES = 2 * 1024**3
MAX_TOTAL_BYTES = 8 * 1024**3
MAX_BUFFERS = 100000


@dataclass(frozen=True)
class EnvelopeAudit:
  opcode_bytes: int
  buffer_count: int
  buffer_bytes: int
  native_abi_verified: bool = False


def _read_exact(stream: BinaryIO, n: int) -> bytes:
  data = stream.read(n)
  if len(data) != n:
    raise ValueError("Truncated native model envelope")
  return data


def inspect_envelope(stream: BinaryIO) -> EnvelopeAudit:
  """Check length-prefixed protocol-5 opcodes and out-of-band buffers."""
  n = struct.unpack("<q", _read_exact(stream, 8))[0]
  if not 1 <= n <= MAX_OPCODE_BYTES:
    raise ValueError("Invalid native model opcode length")
  data = _read_exact(stream, n)
  if not data.startswith(b"\x80\x05"):
    raise ValueError("Not a protocol-5 native model envelope")
  expected = 0
  stop = None
  for op, _, pos in pickletools.genops(data):
    if op.name == "NEXT_BUFFER":
      expected += 1
      if expected > MAX_BUFFERS:
        raise ValueError("Too many native model buffers")
    elif op.name == "STOP":
      stop = pos
  if stop != len(data) - 1:
    raise ValueError("Trailing bytes in opcode stream")
  total = 0
  for _ in range(expected):
    size = struct.unpack("<q", _read_exact(stream, 8))[0]
    if not 0 <= size <= MAX_BUFFER_BYTES or total + size > MAX_TOTAL_BYTES:
      raise ValueError("Invalid native model buffer length")
    remaining = size
    while remaining:
      block = _read_exact(stream, min(remaining, 1024 * 1024))
      remaining -= len(block)
    total += size
  if stream.read(1):
    raise ValueError("Unexpected trailing native model data")
  return EnvelopeAudit(n, expected, total)
