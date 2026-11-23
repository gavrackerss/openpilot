#!/usr/bin/env python3
"""No-dependency regression checks for XNOR Params registry onto native v0.11.2."""
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
HEADER = (ROOT/"openpilot/common/params_keys.h").read_text()
REGEX = r'^\s*\{"([^"]+)",\s*\{(.+)\}\},?\s*$'


class ParamsPortTest(unittest.TestCase):
  def test_unique_native_registry_keys(self):
    names = re.findall(REGEX, HEADER, re.MULTILINE)
    keys = [x[0] for x in names]
    self.assertEqual(len(keys), len(set(keys)), "Duplicated Param names can change lookup behavior")
    self.assertGreaterEqual(len(keys), 190)

  def test_tesla_legacy_settings_available(self):
    for k in ("TinklaEnableACC", "TinklaEnableALC", "TinklaFollowDistance",
              "TeslaAccBlendEnabled", "TeslaLeadSelectionEnabled",
              "TeslaRadarFilteringEnabled", "TeslaSetSpeedOwnershipEnabled",
              "XnorForceFingerprint"):
      with self.subTest(key=k):
        self.assertIn(f'{{"{k}",', HEADER)

  def test_model_selection_and_jetlink_legacy_keys_available(self):
    for k in ("ModelManager_ActiveBundle", "ModelManager_Favs",
              "ModelRunnerTypeCache", "VisionSpeedLimitDetection",
              "MapdSettings", "VisionSpeedLimitV3Decision"):
      with self.subTest(key=k):
        self.assertIn(f'{{"{k}",', HEADER)

  def test_native_jetlink_is_disabled_by_default(self):
    self.assertIn('{"JetlinkLink", {PERSISTENT, INT, "0"}}', HEADER)
    self.assertIn('{"JetlinkChargePhone", {PERSISTENT, BOOL, "0"}}', HEADER)
    self.assertIn('{"JetlinkProgress", {CLEAR_ON_MANAGER_START, JSON}}', HEADER)
    self.assertIn('{"JetlinkSpec", {PERSISTENT, JSON}}', HEADER)

  def test_native_0112_contracts_not_silently_overwritten(self):
    self.assertIn('{"ExperimentalMode", {PERSISTENT, BOOL, "1"}}', HEADER)
    self.assertIn('{"LastGPSPosition", {PERSISTENT, BYTES}}', HEADER)
    # These differ from XNOR 0.11.1 and need an explicit migration decision;
    # do NOT overwrite the 0.11.2 type/default during a blind port.


if __name__ == "__main__":
  unittest.main(verbosity=2)
