#!/usr/bin/env python3
"""Contract guard for restored XNOR mapd process and UI wiring."""
from pathlib import Path
import stat
import unittest

ROOT = Path(__file__).resolve().parents[1]


class MapdRuntimeContract(unittest.TestCase):
  def test_mapd_executable_and_manager_process(self):
    binary = ROOT / "selfdrive/mapd"
    self.assertTrue(binary.is_file())
    self.assertTrue(binary.stat().st_mode & stat.S_IXUSR)

    processes = (ROOT / "openpilot/system/manager/process_config.py").read_text()
    self.assertIn('NativeProcess("mapd", "selfdrive", ["./mapd"], always_run, enabled=COMMA_HARDWARE)', processes)

  def test_schema_and_services_are_retained(self):
    custom = (ROOT / "openpilot/cereal/custom.capnp").read_text()
    services = (ROOT / "openpilot/cereal/services.py").read_text()
    for marker in ("struct MapdIn", "struct MapdOut", "struct MapdExtendedOut",
                   "speedLimit @3 :Float32", "nextSpeedLimit @4 :Float32"):
      self.assertIn(marker, custom)
    self.assertIn('"mapdOut": (True, 20., 20, QueueSize.MEDIUM)', services)
    self.assertIn('"mapdIn": (False, 0.)', services)

  def test_offroad_download_ui_uses_mapd_in_and_avoids_extended_stream(self):
    osm = (ROOT / "openpilot/selfdrive/ui/layouts/settings/osm.py").read_text()
    self.assertIn("MAPD_DOWNLOAD = 0", osm)
    self.assertIn("MAPD_CANCEL_DOWNLOAD = 27", osm)
    self.assertIn('messaging.PubMaster(["mapdIn"])', osm)
    self.assertIn('messaging.new_message("mapdIn")', osm)
    self.assertIn('return f"nation.{self._country_code}"', osm)
    self.assertIn('return f"us_state.{self._state_code}"', osm)
    self.assertNotIn('SubMaster(["mapdExtendedOut"]', osm)

  def test_onroad_ui_consumes_only_safe_mapd_out(self):
    ui_state = (ROOT / "openpilot/selfdrive/ui/ui_state.py").read_text()
    hud = (ROOT / "openpilot/selfdrive/ui/onroad/manual_speed_limit.py").read_text()
    self.assertIn('"mapdOut"', ui_state)
    self.assertNotIn('"mapdExtendedOut"', ui_state)
    self.assertIn('ui_state.sm["mapdOut"]', hud)
    self.assertIn("OSM", hud)
    self.assertIn("CV.MS_TO_MPH", hud)


if __name__ == "__main__":
  unittest.main(verbosity=2)
