#!/usr/bin/env python3
"""Extract the Tesla MCU1 ConfigCheckBypass runtime configuration descriptor table.

The table is a 16-byte-per-entry structure discovered in the raw firmware image:

  word0: optional/default pointer or cached address (0 for most entries)
  word1: flags (0x80 marks a subset of descriptors)
  word2: file offset of the canonical config key string
  word3: file offset of the per-key runtime read callback

This script intentionally works with file offsets because the raw image is not an ELF.
For disassembler work, add the MCU flash base (usually 0x40000000) to these offsets.
"""
from __future__ import annotations

import argparse
import csv
import struct
from pathlib import Path

STRING_MIN = 0x23E00
STRING_MAX = 0x24600
READ_MIN = 0x84000
READ_MAX = 0x89000
ENTRY_SIZE = 16


def c_string(blob: bytes, offset: int) -> str:
  end = blob.find(b"\0", offset)
  if end < 0:
    end = len(blob)
  return blob[offset:end].decode("ascii", "replace")


def iter_descriptor_candidates(blob: bytes):
  for offset in range(0, len(blob) - ENTRY_SIZE, 4):
    word0, flags, name_off, read_off = struct.unpack(">IIII", blob[offset:offset + ENTRY_SIZE])
    if STRING_MIN <= name_off <= STRING_MAX and READ_MIN <= read_off <= READ_MAX:
      yield {
        "entry_off": offset,
        "word0": word0,
        "flags": flags,
        "name_off": name_off,
        "name": c_string(blob, name_off),
        "read_off": read_off,
      }


def main() -> int:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("binary", type=Path, help="raw Tesla MCU1 firmware binary")
  parser.add_argument("--csv", action="store_true", help="emit CSV instead of a text table")
  parser.add_argument("--refs", action="store_true", help="include raw 24-bit callback-offset references outside descriptor entries")
  args = parser.parse_args()

  blob = args.binary.read_bytes()
  entries = list(iter_descriptor_candidates(blob))

  if args.refs:
    for entry in entries:
      pattern = entry["read_off"].to_bytes(3, "big")
      refs = []
      start = 0
      while True:
        ref = blob.find(pattern, start)
        if ref < 0:
          break
        if ref != entry["entry_off"] + 13:  # descriptor stores the low three bytes at +0x0d
          refs.append(ref)
        start = ref + 1
      if refs:
        entry["raw_read_refs"] = ";".join(f"0x{ref:x}" for ref in refs)
      else:
        entry["raw_read_refs"] = ""

  if args.csv:
    import sys
    fieldnames = ["entry_off", "word0", "flags", "name_off", "name", "read_off"]
    if args.refs:
      fieldnames.append("raw_read_refs")
    writer = csv.DictWriter(sys.stdout, fieldnames=fieldnames)
    writer.writeheader()
    for entry in entries:
      writer.writerow({k: f"0x{v:x}" if isinstance(v, int) else v for k, v in entry.items()})
  else:
    print(f"found {len(entries)} candidate descriptors")
    print("entry_off  word0      flags  name_off  read_off  name")
    for entry in entries:
      print(
        f"0x{entry['entry_off']:06x}  "
        f"0x{entry['word0']:08x}  "
        f"0x{entry['flags']:04x}  "
        f"0x{entry['name_off']:06x}  "
        f"0x{entry['read_off']:06x}  "
        f"{entry['name']}"
        f"  refs={entry.get('raw_read_refs', '')}"
      )
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
