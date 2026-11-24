"""Restore the OSM tab without claiming a running mapd service."""
from pathlib import Path
import json
from openpilot.selfdrive.ui.layouts.settings.xnor_panel import XnorPanel
from openpilot.system.ui.widgets.list_view import text_item


class OSMLayout(XnorPanel):
  def __init__(self):
    super().__init__()
    self._root = Path("/data/media/0/osm")
    self._region = text_item("Saved map region", "Not inspected")
    self._files = text_item("Offline map directory", "Not inspected")
    self.set_items([
      text_item("Offline maps", "Service not ported",
                description="Existing map data is preserved. Download, cancel and delete are not available in this revision."),
      self._region, self._files,
    ])

  def refresh(self):
    try:
      selection = self._root / "ui_selection.json"
      if selection.is_file() and selection.stat().st_size <= 4096:
        data = json.loads(selection.read_text())
        if isinstance(data, dict):
          region = " ".join(str(data.get(k, "")) for k in ("country", "state")).strip()
          if not region:
            region = " ".join(str(data.get(k, "")) for k in ("country_code", "state_code")).strip()
        else:
          region = ""
      else:
        region = ""
      self._region.action_item.set_text(" ".join(region.split())[:60] or "Not available")
      self._files.action_item.set_text("Present" if (self._root / "offline").is_dir() else "Not found")
    except (OSError, ValueError, TypeError):
      self._region.action_item.set_text("Unavailable")
      self._files.action_item.set_text("Unavailable")
