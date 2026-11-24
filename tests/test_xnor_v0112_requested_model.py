#!/usr/bin/env python3
"""Requested model persistence and offroad selector contract."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from openpilot.sunnypilot.models.requested_model import (
  REQUEST_KEY, STOCK, TSFDOM, OPM10V3, requested_ref, requested_label, save_request,
)


class FakeParams:
  def __init__(self):
    self.values = {"IsOffroad": True, "ModelManager_ActiveBundle": {"ref": OPM10V3}}
    self.writes = []

  def get_bool(self, key):
    return bool(self.values.get(key))

  def put(self, key, value, *, block):
    self.values[key] = value
    self.writes.append((key, value, block))


class RequestedModelTests(unittest.TestCase):
  def test_legacy_choice_inherited_without_write(self):
    p = FakeParams()
    self.assertEqual(requested_ref(None, p.values["ModelManager_ActiveBundle"]), OPM10V3)
    self.assertEqual(p.writes, [])

  def test_explicit_choice_survives_new_reader_and_keeps_legacy(self):
    p = FakeParams()
    self.assertTrue(save_request(p, TSFDOM, confirmed_offroad=True))
    self.assertEqual(requested_label(p.values[REQUEST_KEY], p.values["ModelManager_ActiveBundle"]), "TSFDOM")
    self.assertEqual(p.values["ModelManager_ActiveBundle"], {"ref": OPM10V3})
    self.assertEqual(p.writes, [(REQUEST_KEY, TSFDOM, True)])
    self.assertTrue(save_request(p, STOCK, confirmed_offroad=True))

  def test_offroad_and_allowlist_both_required(self):
    p = FakeParams()
    self.assertFalse(save_request(p, TSFDOM, confirmed_offroad=False))
    self.assertFalse(save_request(p, "unknown", confirmed_offroad=True))
    p.values["IsOffroad"] = False
    self.assertFalse(save_request(p, TSFDOM, confirmed_offroad=True))
    self.assertEqual(p.writes, [])

  def test_unknown_saved_reference_is_retained(self):
    self.assertEqual(requested_ref("another-model", {"ref": OPM10V3}), "another-model")
    self.assertEqual(requested_label("another-model", None), "Other saved model")

  def test_persistent_parameter_and_native_runner_isolation(self):
    self.assertIn('{"XNORRequestedModelRef", {PERSISTENT, STRING}}',
                  (ROOT / "openpilot/common/params_keys.h").read_text())
    self.assertNotIn(REQUEST_KEY, (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text())


if __name__ == "__main__":
  unittest.main(verbosity=2)
