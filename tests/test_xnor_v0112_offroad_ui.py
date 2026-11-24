#!/usr/bin/env python3
"""Static integration guard for the actual native offroad settings page."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
PREVIEW = ROOT / "openpilot/selfdrive/ui/onroad/manual_speed_limit_preview.py"
DEVELOPER = ROOT / "openpilot/selfdrive/ui/layouts/settings/developer.py"
HUD = ROOT / "openpilot/selfdrive/ui/onroad/hud_renderer.py"


class OffroadUIContract(unittest.TestCase):
  def test_preview_contains_original_presets_and_no_control_hooks(self):
    tree = ast.parse(PREVIEW.read_text())
    options = next(n.value for n in tree.body if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == "OPTIONS" for t in n.targets))
    self.assertEqual(ast.literal_eval(options), ((40, "40"), (50, "50"), (60, "60"), (70, "NSL"), (0, "X")))
    names = {n.name for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    self.assertFalse({"_handle_mouse_release", "_handle_mouse_press"} & names)
    self.assertNotIn("XNORManualSpeedLimitMph", PREVIEW.read_text())
    self.assertNotIn("manual_speed_limit_preview", HUD.read_text())

  def test_offroad_settings_are_wired_and_read_only(self):
    text = DEVELOPER.read_text()
    tree = ast.parse(text)
    methods = {n.name: n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)}
    self.assertIn("SpeedSignDemoRow(self._migration_offroad, self._preview_offset_settings)", text)
    self.assertIn("*self._migration_items", text)
    self.assertIn("item.set_visible(self._migration_offroad)", text)
    self.assertIn("ui_state.is_offroad()", ast.unparse(methods["_migration_offroad"]))
    self.assertIn("get_bool('IsOffroad')", ast.unparse(methods["_migration_offroad"]))
    refresh = ast.unparse(methods["_refresh_migration_items"])
    self.assertIn("get('ModelManager_ActiveBundle')", refresh)
    for forbidden in (".put(", ".put_bool(", ".remove(", "load_oob(", "get_active_model_runner("):
      self.assertNotIn(forbidden, refresh)


if __name__ == "__main__":
  unittest.main(verbosity=2)
