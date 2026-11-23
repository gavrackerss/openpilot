#!/usr/bin/env python3
"""No-dependency compatibility checks for XNOR Cereal extension port.

Validates preserved Cap'n Proto struct IDs/event union field numbers and
legacy service registrations in the native comma 0.11.2 source tree.
This does not replace a capnp compiler or interprocess replay.
"""
import ast
from pathlib import Path
import re
import unittest

ROOT=Path(__file__).resolve().parents[1]
CUSTOM=(ROOT/"openpilot/cereal/custom.capnp").read_text()
LOG=(ROOT/"openpilot/cereal/log.capnp").read_text()
SERVICES=(ROOT/"openpilot/cereal/services.py").read_text()

STRUCTS={
    "ModelManagerSP": "0xaedffd8f31e7b55d",
    "MapdExtendedOut": "0xa30662f84033036c",
    "MapdIn": "0xc86a3d38d13eb3ef",
    "MapdOut": "0xa4f1eb3323f5f582",
}
EVENTS={
    "modelManagerSP": (108, "ModelManagerSP"),
    "mapdExtendedOut": (143, "MapdExtendedOut"),
    "mapdIn": (144, "MapdIn"),
    "mapdOut": (145, "MapdOut"),
}
SERVICE_NAMES={
    "modelManagerSP", "mapdOut", "mapdExtendedOut", "mapdIn", "mapdCli",
    "customReservedRawData0", "customReservedRawData1", "customReservedRawData2",
}


class CerealPortContractTest(unittest.TestCase):
  def test_custom_ids_not_changed(self):
    for name, ident in STRUCTS.items():
      with self.subTest(name=name):
        self.assertRegex(CUSTOM, rf"\bstruct\s+{name}\s+@{ident}\b")

  def test_replaced_event_slots_keep_numbers(self):
    for event, (number, cls) in EVENTS.items():
      with self.subTest(event=event):
        self.assertRegex(LOG, rf"\b{event}\s+@{number}\s+:Custom\.{cls}\s*;")
        self.assertEqual(len(re.findall(rf"\b{event}\s+@{number}\b", LOG)), 1)

  def test_ids_unique_in_custom_types(self):
    declared=re.findall(r"\bstruct\s+\w+\s+@(0x[0-9a-fA-F]{16})", CUSTOM)
    self.assertEqual(len(declared), len(set(declared)))

  def test_no_duplicate_service_keys(self):
    names=re.findall(r'^\s*"([^"]+)"\s*:', SERVICES, re.MULTILINE)
    self.assertEqual(len(names),len(set(names)))
    self.assertTrue(SERVICE_NAMES.issubset(set(names)))

  def test_custom_import_path_matches_0112(self):
    self.assertIn('import "/include/c++.capnp"', CUSTOM)
    self.assertIn('using Custom = import "custom.capnp"',LOG)

  def test_native_submodule_sources_are_xnor(self):
    gitmodules=(ROOT/".gitmodules").read_text()
    self.assertIn("https://github.com/xnor-tech/opendbc.git",gitmodules)
    self.assertIn("https://github.com/xnor-tech/panda.git",gitmodules)


if __name__=="__main__":
  unittest.main(verbosity=2)
