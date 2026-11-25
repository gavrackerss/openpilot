"""Offline OSM map management backed by the restored XNOR mapd runtime."""
from __future__ import annotations

import glob
import json
import os
import shutil
import time
from datetime import datetime
from pathlib import Path

from openpilot.cereal import messaging
from openpilot.selfdrive.ui.layouts.settings.xnor_panel import XnorPanel
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import button_item, text_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog

MAPD_DOWNLOAD = 0
MAPD_CANCEL_DOWNLOAD = 27

MAP_ROOT = Path("/data/media/0/osm")
OFFLINE_MAP_PATH = MAP_ROOT / "offline"
TMP_MAP_PATH = MAP_ROOT / "tmp"
SELECTION_PATH = MAP_ROOT / "ui_selection.json"
DOWNLOAD_MENU_PATH = Path(__file__).with_name("osm_download_menu.json")


class OSMLayout(XnorPanel):
  """Manage mapd downloads without subscribing to mapdExtendedOut.

  The retained mapd executable has a known legacy mapdExtendedOut framing issue
  with the current Python msgq client. Download progress is therefore derived
  from mapd's tmp/offline files; ordinary on-road map data uses mapdOut only.
  """

  def __init__(self):
    super().__init__()
    self._pm = None
    self._menu = self._load_download_menu()
    self._country_code = ""
    self._state_code = ""
    self._dialog = None
    self._dialog_kind = None
    self._option_to_code: dict[str, str] = {}

    self._request_pending = False
    self._cancel_pending = False
    self._request_time = 0.0
    self._request_base_bytes = 0
    self._request_base_mtime = 0.0
    self._saw_tmp_activity = False
    self._last_tmp_activity_time = 0.0

    self._installed_bytes = 0
    self._legacy_bytes = 0
    self._tmp_bytes = 0
    self._tmp_files = 0
    self._latest_map_mtime = 0.0
    self._last_storage_refresh = -1000.0

    self._load_selection()

    self._selection = text_item(
      "Selected Map Region", self._selection_label,
      description="The selected country or US state is used for Download / Update. Existing maps for other regions are retained.",
    )
    self._country = button_item(
      "Country", "SELECT", description=self._country_description,
      callback=self._open_country_selector, enabled=self._can_change_selection,
    )
    self._state = button_item(
      "US State", "SELECT", description="Choose a US state, or All United States.",
      callback=self._open_state_selector, enabled=self._can_change_selection,
    )
    self._state.set_visible(lambda: self._country_code == "US")

    self._status = text_item(
      "Download Status", self._download_status,
      description="mapd download state is detected from /data/media/0/osm/tmp and /data/media/0/osm/offline.",
    )
    self._storage = text_item("Downloaded Maps", self._storage_value, description=self._storage_description)
    self._download = button_item(
      "Download / Update Maps", "DOWNLOAD",
      description="Downloads or updates the selected region using mapd. Other downloaded regions are not removed.",
      callback=self._confirm_download, enabled=self._can_download,
    )
    self._cancel = button_item(
      "Cancel Map Download", "CANCEL",
      description="Stops the active map download after the current file finishes.",
      callback=self._cancel_download, enabled=self._is_downloading,
    )
    self._delete = button_item(
      "Delete Downloaded Maps", "DELETE",
      description="Deletes offline tiles plus legacy mapd db/v* data. The selected region is retained.",
      callback=self._confirm_delete, enabled=self._can_delete,
    )

    self.set_items([
      self._selection, self._country, self._state, self._status, self._storage,
      self._download, self._cancel, self._delete,
    ])
    self._refresh_storage(force=True)

  @staticmethod
  def _load_download_menu() -> dict:
    try:
      data = json.loads(DOWNLOAD_MENU_PATH.read_text(encoding="utf-8"))
      return data if isinstance(data, dict) else {}
    except (OSError, json.JSONDecodeError):
      return {}

  def _load_selection(self):
    try:
      data = json.loads(SELECTION_PATH.read_text(encoding="utf-8"))
      country = str(data.get("country", ""))
      state = str(data.get("state", ""))
      if country in self._menu.get("nation", {}):
        self._country_code = country
      if state in self._menu.get("us_state", {}):
        self._state_code = state
    except (OSError, json.JSONDecodeError, TypeError, ValueError):
      pass

  def _save_selection(self):
    try:
      MAP_ROOT.mkdir(parents=True, exist_ok=True)
      tmp = SELECTION_PATH.with_suffix(".tmp")
      tmp.write_text(json.dumps({"country": self._country_code, "state": self._state_code}), encoding="utf-8")
      os.replace(tmp, SELECTION_PATH)
    except OSError:
      pass

  def _country_name(self, code: str | None = None) -> str:
    code = self._country_code if code is None else code
    try:
      return str(self._menu["nation"][code]["full_name"])
    except (KeyError, TypeError):
      return ""

  def _state_name(self, code: str | None = None) -> str:
    code = self._state_code if code is None else code
    if not code:
      return "All United States"
    try:
      return str(self._menu["us_state"][code]["full_name"])
    except (KeyError, TypeError):
      return ""

  def _selection_label(self) -> str:
    if not self._country_code:
      return "Not selected"
    return self._state_name() if self._country_code == "US" else self._country_name()

  def _country_description(self) -> str:
    selected = self._country_name()
    return f"Current: {selected}" if selected else "Select the country whose offline OSM data you want to download."

  def _can_change_selection(self) -> bool:
    return self.is_offroad() and not self._is_downloading()

  def _can_download(self) -> bool:
    return self.is_offroad() and bool(self._country_code) and not self._is_downloading()

  def _can_delete(self) -> bool:
    return self.is_offroad() and self._installed_bytes > 0 and not self._is_downloading()

  def _open_country_selector(self):
    if not self._can_change_selection():
      return
    entries = sorted(
      ((str(data.get("full_name", code)), code) for code, data in self._menu.get("nation", {}).items()),
      key=lambda item: item[0].casefold(),
    )
    if not entries:
      return
    options = [name for name, _ in entries]
    self._option_to_code = dict(entries)
    self._dialog_kind = "country"
    self._dialog = MultiOptionDialog("Select OSM Country", options, current=self._country_name(), callback=self._on_option_selected)
    gui_app.push_widget(self._dialog)

  def _open_state_selector(self):
    if not self._can_change_selection() or self._country_code != "US":
      return
    entries = [("All United States", "")]
    entries.extend(sorted(
      ((str(data.get("full_name", code)), code) for code, data in self._menu.get("us_state", {}).items()),
      key=lambda item: item[0].casefold(),
    ))
    options = [name for name, _ in entries]
    self._option_to_code = dict(entries)
    self._dialog_kind = "state"
    self._dialog = MultiOptionDialog("Select US State", options, current=self._state_name(), callback=self._on_option_selected)
    gui_app.push_widget(self._dialog)

  def _on_option_selected(self, result):
    if result != DialogResult.CONFIRM or self._dialog is None:
      self._dialog = None
      self._dialog_kind = None
      return
    code = self._option_to_code.get(self._dialog.selection)
    if code is None:
      self._dialog = None
      self._dialog_kind = None
      return
    if self._dialog_kind == "country":
      self._country_code = code
      if code != "US":
        self._state_code = ""
    elif self._dialog_kind == "state":
      self._state_code = code
    self._save_selection()
    self._dialog = None
    self._dialog_kind = None

  def _download_location(self) -> str:
    if not self._country_code:
      return ""
    if self._country_code == "US" and self._state_code:
      return f"us_state.{self._state_code}"
    return f"nation.{self._country_code}"

  def _send_mapd(self, msg_type: int, value: str = ""):
    if self._pm is None:
      self._pm = messaging.PubMaster(["mapdIn"])
    msg = messaging.new_message("mapdIn")
    msg.mapdIn.type = msg_type
    if value:
      msg.mapdIn.str = value
    self._pm.send("mapdIn", msg)

  def _confirm_download(self):
    if not self._can_download():
      return
    location = self._selection_label()

    def confirmed(result):
      if result == DialogResult.CONFIRM and self._can_download():
        self._start_download()

    gui_app.push_widget(ConfirmDialog(
      f"Download or update offline OSM data for {location}?", "Download", callback=confirmed,
    ))

  def _start_download(self):
    location = self._download_location()
    if not location:
      return
    self._refresh_storage(force=True)
    self._request_base_bytes = self._installed_bytes
    self._request_base_mtime = self._latest_map_mtime
    self._saw_tmp_activity = False
    self._last_tmp_activity_time = 0.0
    self._cancel_pending = False
    self._send_mapd(MAPD_DOWNLOAD, location)
    self._request_pending = True
    self._request_time = time.monotonic()

  def _cancel_download(self):
    if not self._is_downloading():
      return
    self._send_mapd(MAPD_CANCEL_DOWNLOAD)
    self._cancel_pending = True

  def _is_downloading(self) -> bool:
    return self._tmp_files > 0 or self._request_pending

  def _download_status(self) -> str:
    if self._cancel_pending:
      return "Canceling" if self._is_downloading() else "Cancelled"
    if self._tmp_files > 0:
      return f"Downloading / Updating ({self._format_bytes(self._tmp_bytes)})"
    if self._request_pending:
      return "Starting"
    return "Ready"

  @staticmethod
  def _path_state(path: Path) -> tuple[int, float, int]:
    total = 0
    latest = 0.0
    files = 0
    try:
      if path.is_file() or path.is_symlink():
        stat = path.stat()
        return stat.st_size, stat.st_mtime, 1
      if not path.is_dir():
        return 0, 0.0, 0
      for root, _, names in os.walk(path):
        for name in names:
          try:
            stat = os.stat(os.path.join(root, name))
            total += stat.st_size
            latest = max(latest, stat.st_mtime)
            files += 1
          except OSError:
            pass
    except OSError:
      return 0, 0.0, 0
    return total, latest, files

  def _refresh_storage(self, force: bool = False):
    now = time.monotonic()
    if not force and now - self._last_storage_refresh < 2.0:
      return
    offline_bytes, offline_mtime, _ = self._path_state(OFFLINE_MAP_PATH)
    self._tmp_bytes, _, self._tmp_files = self._path_state(TMP_MAP_PATH)
    legacy_paths = [MAP_ROOT / "db", *(Path(p) for p in glob.glob(str(MAP_ROOT / "v*")))]
    self._legacy_bytes = sum(self._path_state(path)[0] for path in legacy_paths)
    self._installed_bytes = offline_bytes + self._legacy_bytes
    self._latest_map_mtime = offline_mtime
    self._last_storage_refresh = now

  @staticmethod
  def _format_bytes(value: int) -> str:
    size = float(value)
    for unit in ("B", "KB", "MB", "GB"):
      if size < 1024.0 or unit == "GB":
        return f"{size:.1f} {unit}"
      size /= 1024.0
    return "0.0 B"

  def _storage_value(self) -> str:
    return self._format_bytes(self._installed_bytes)

  def _storage_description(self) -> str:
    details = [f"Path: {OFFLINE_MAP_PATH}"]
    if self._latest_map_mtime > 0:
      stamp = datetime.fromtimestamp(self._latest_map_mtime).strftime("%Y-%m-%d %H:%M")
      details.append(f"Latest map file: {stamp}")
    if self._legacy_bytes > 0:
      details.append(f"Legacy mapd data detected: {self._format_bytes(self._legacy_bytes)}")
    return "<br>".join(details)

  def _confirm_delete(self):
    if not self._can_delete():
      return

    def confirmed(result):
      if result == DialogResult.CONFIRM and self._can_delete():
        self._delete_maps()

    gui_app.push_widget(ConfirmDialog(
      "Delete all downloaded OSM map data from this device?", "Delete", callback=confirmed,
    ))

  def _delete_maps(self):
    targets = [OFFLINE_MAP_PATH, TMP_MAP_PATH, MAP_ROOT / "db"]
    targets.extend(Path(p) for p in glob.glob(str(MAP_ROOT / "v*")))
    for target in targets:
      try:
        if target.is_symlink() or target.is_file():
          target.unlink(missing_ok=True)
        elif target.is_dir():
          shutil.rmtree(target)
      except OSError:
        pass
    try:
      OFFLINE_MAP_PATH.mkdir(parents=True, exist_ok=True)
    except OSError:
      pass
    self._request_pending = False
    self._cancel_pending = False
    self._saw_tmp_activity = False
    self._refresh_storage(force=True)

  def _update_state(self):
    self._refresh_storage()
    tmp_active = self._tmp_files > 0
    if self._request_pending:
      elapsed = time.monotonic() - self._request_time
      if tmp_active:
        self._saw_tmp_activity = True
        self._last_tmp_activity_time = time.monotonic()
      else:
        changed = self._installed_bytes != self._request_base_bytes or self._latest_map_mtime > self._request_base_mtime
        tmp_quiet = self._saw_tmp_activity and time.monotonic() - self._last_tmp_activity_time > 4.0
        if tmp_quiet or (changed and elapsed > 4.0 and not self._saw_tmp_activity) or elapsed > 30.0:
          self._request_pending = False
          self._cancel_pending = False
          self._saw_tmp_activity = False
          self._refresh_storage(force=True)
    elif self._cancel_pending and not tmp_active:
      self._cancel_pending = False

  def refresh(self):
    self._refresh_storage(force=True)
