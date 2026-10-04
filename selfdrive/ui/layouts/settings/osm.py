import glob
import json
import os
import shutil
import time
from datetime import datetime
from pathlib import Path

from cereal import messaging
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware.hw import Paths
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import DialogResult, Widget
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import button_item, text_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller


MAPD_DOWNLOAD = 0
MAPD_CANCEL_DOWNLOAD = 27

MAP_ROOT = Path(Paths.mapd_root())
OFFLINE_MAP_PATH = MAP_ROOT / "offline"
TMP_MAP_PATH = MAP_ROOT / "tmp"
SELECTION_PATH = MAP_ROOT / "ui_selection.json"
DOWNLOAD_MENU_PATH = Path(__file__).with_name("osm_download_menu.json")


class OSMLayout(Widget):
  """Offline OSM map management UI for XNOR's existing pfeiferj/mapd.

  Deliberately does not subscribe to mapdExtendedOut. The mapd binary bundled
  with this XNOR build can emit a frame that the current Python msgq client
  treats as zero-length, which aborts the entire UI process in native code.
  Downloads still use mapdIn; status is derived from mapd's tmp/offline files.
  """

  def __init__(self):
    super().__init__()
    self._pm = None
    self._dialog = None
    self._dialog_kind = None
    self._option_to_code = {}

    self._menu = self._load_download_menu()
    self._country_code = ""
    self._state_code = ""
    self._load_selection()

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
    self._last_storage_refresh = 0.0

    self._selection = text_item(
      lambda: tr("Selected Map Region"),
      self._selection_label,
      lambda: tr("The selected country or US state is used for Download / Update. Existing maps for other regions are not removed."),
    )
    self._country = button_item(
      lambda: tr("Country"),
      lambda: tr("SELECT"),
      self._country_description,
      callback=self._open_country_selector,
      enabled=self._can_change_selection,
    )
    self._state = button_item(
      lambda: tr("US State"),
      lambda: tr("SELECT"),
      lambda: tr("Choose a single US state, or All United States."),
      callback=self._open_state_selector,
      enabled=self._can_change_selection,
    )
    self._state.set_visible(lambda: self._country_code == "US")

    self._status = text_item(
      lambda: tr("Download Status"),
      self._download_status,
      lambda: tr("Download state is detected from /data/media/0/osm/tmp and /data/media/0/osm/offline. The UI does not subscribe to mapdExtendedOut."),
    )
    self._storage = text_item(
      lambda: tr("Downloaded Maps"),
      self._storage_value,
      self._storage_description,
    )
    self._download = button_item(
      lambda: tr("Download / Update Maps"),
      lambda: tr("DOWNLOAD"),
      lambda: tr("Downloads the selected region using the existing mapd service. Re-running this updates/replaces the selected map tiles without removing other regions."),
      callback=self._confirm_download,
      enabled=self._can_download,
    )
    self._cancel = button_item(
      lambda: tr("Cancel Map Download"),
      lambda: tr("CANCEL"),
      lambda: tr("Stops the active map download after the current file finishes."),
      callback=self._cancel_download,
      enabled=self._is_downloading,
    )
    self._delete = button_item(
      lambda: tr("Delete Downloaded Maps"),
      lambda: tr("DELETE"),
      lambda: tr("Deletes offline tiles plus legacy mapd db/v* data from /data/media/0/osm. Your selected region is retained."),
      callback=self._confirm_delete,
      enabled=self._can_delete,
    )

    self._scroller = Scroller(
      [self._selection, self._country, self._state, self._status, self._storage, self._download, self._cancel, self._delete],
      line_separator=True,
      spacing=0,
    )
    self._refresh_storage(force=True)

  @staticmethod
  def _load_download_menu():
    try:
      with open(DOWNLOAD_MENU_PATH, "r", encoding="utf-8") as f:
        data = json.load(f)
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

  def _country_name(self, code=None):
    code = self._country_code if code is None else code
    try:
      return str(self._menu["nation"][code]["full_name"])
    except (KeyError, TypeError):
      return ""

  def _state_name(self, code=None):
    code = self._state_code if code is None else code
    if not code:
      return tr("All United States")
    try:
      return str(self._menu["us_state"][code]["full_name"])
    except (KeyError, TypeError):
      return ""

  def _selection_label(self):
    if not self._country_code:
      return tr("Not selected")
    if self._country_code == "US":
      return self._state_name()
    return self._country_name()

  def _country_description(self):
    selected = self._country_name()
    if selected:
      return f"{tr('Current')}: {selected}"
    return tr("Select the country whose offline OSM data you want to download or update.")

  def _can_change_selection(self):
    return ui_state.is_offroad() and not self._is_downloading()

  def _can_download(self):
    return ui_state.is_offroad() and bool(self._country_code) and not self._is_downloading()

  def _can_delete(self):
    return ui_state.is_offroad() and self._installed_bytes > 0 and not self._is_downloading()

  def _open_country_selector(self):
    entries = []
    self._option_to_code = {}
    for code, data in self._menu.get("nation", {}).items():
      name = str(data.get("full_name", code))
      entries.append((name, code))
    entries.sort(key=lambda x: x[0].casefold())
    options = [name for name, _ in entries]
    self._option_to_code = {name: code for name, code in entries}
    if not options:
      return

    current = self._country_name()
    self._dialog_kind = "country"
    self._dialog = MultiOptionDialog(tr("Select OSM Country"), options, current=current, callback=self._on_option_selected)
    gui_app.push_widget(self._dialog)

  def _open_state_selector(self):
    if self._country_code != "US":
      return
    entries = [(tr("All United States"), "")]
    for code, data in self._menu.get("us_state", {}).items():
      name = str(data.get("full_name", code))
      entries.append((name, code))
    entries[1:] = sorted(entries[1:], key=lambda x: x[0].casefold())
    options = [name for name, _ in entries]
    self._option_to_code = {name: code for name, code in entries}

    self._dialog_kind = "state"
    self._dialog = MultiOptionDialog(tr("Select US State"), options, current=self._state_name(), callback=self._on_option_selected)
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

  def _download_location(self):
    if not self._country_code:
      return ""
    if self._country_code == "US" and self._state_code:
      return f"us_state.{self._state_code}"
    return f"nation.{self._country_code}"

  def _confirm_download(self):
    location = self._selection_label()
    if not location:
      return

    def cb(result):
      if result == DialogResult.CONFIRM:
        self._start_download()

    msg = tr("Download or update offline OSM data for") + f" {location}?"
    gui_app.push_widget(ConfirmDialog(msg, tr("Download"), callback=cb))

  def _send_mapd(self, msg_type, value=""):
    # Create the publisher only when an action is requested. Merely opening
    # Settings therefore adds no mapd messaging sockets to the UI process.
    if self._pm is None:
      self._pm = messaging.PubMaster(["mapdIn"])
    msg = messaging.new_message("mapdIn")
    msg.mapdIn.type = msg_type
    if value:
      msg.mapdIn.str = value
    self._pm.send("mapdIn", msg)

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
    self._send_mapd(MAPD_CANCEL_DOWNLOAD)
    self._cancel_pending = True

  def _is_downloading(self):
    if self._tmp_files > 0:
      return True
    if self._request_pending:
      # Keep controls locked while mapd has a chance to create its first .part
      # file. If no activity occurs, _update_state clears this after 30 s.
      return True
    return False

  def _download_status(self):
    if self._cancel_pending:
      return tr("Cancelling") if self._is_downloading() else tr("Cancelled")

    if self._tmp_files > 0:
      return f"{tr('Downloading / Updating')} ({self._format_bytes(self._tmp_bytes)})"

    if self._request_pending:
      return tr("Starting")

    return tr("Ready")

  @staticmethod
  def _path_state(path):
    total = 0
    latest = 0.0
    files = 0
    try:
      if path.is_file() or path.is_symlink():
        st = path.stat()
        return st.st_size, st.st_mtime, 1
    except OSError:
      return 0, 0.0, 0

    if path.is_dir():
      for root, _, names in os.walk(path):
        for name in names:
          try:
            st = os.stat(os.path.join(root, name))
            total += st.st_size
            latest = max(latest, st.st_mtime)
            files += 1
          except OSError:
            pass
    return total, latest, files

  def _refresh_storage(self, force=False):
    now = time.monotonic()
    if not force and now - self._last_storage_refresh < 2.0:
      return

    offline_bytes, offline_mtime, _ = self._path_state(OFFLINE_MAP_PATH)
    tmp_bytes, _, tmp_files = self._path_state(TMP_MAP_PATH)

    legacy_paths = [MAP_ROOT / "db"] + [Path(p) for p in glob.glob(str(MAP_ROOT / "v*"))]
    legacy_bytes = sum(self._path_state(path)[0] for path in legacy_paths)

    self._installed_bytes = offline_bytes + legacy_bytes
    self._legacy_bytes = legacy_bytes
    self._tmp_bytes = tmp_bytes
    self._tmp_files = tmp_files
    self._latest_map_mtime = offline_mtime
    self._last_storage_refresh = now

  @staticmethod
  def _format_bytes(value):
    value = float(value)
    for unit in ("B", "KB", "MB", "GB"):
      if value < 1024.0 or unit == "GB":
        return f"{value:.1f} {unit}"
      value /= 1024.0
    return "0.0 B"

  def _storage_value(self):
    return self._format_bytes(self._installed_bytes)

  def _storage_description(self):
    details = [f"{tr('Path')}: {OFFLINE_MAP_PATH}"]
    if self._latest_map_mtime > 0:
      stamp = datetime.fromtimestamp(self._latest_map_mtime).strftime("%Y-%m-%d %H:%M")
      details.append(f"{tr('Latest map file')}: {stamp}")
    if self._legacy_bytes > 0:
      details.append(f"{tr('Legacy mapd data detected')}: {self._format_bytes(self._legacy_bytes)}")
    return "<br>".join(details)

  def _confirm_delete(self):
    def cb(result):
      if result == DialogResult.CONFIRM:
        self._delete_maps()

    gui_app.push_widget(
      ConfirmDialog(
        tr("Delete all downloaded OSM map data from this device?"),
        tr("Delete"),
        callback=cb,
      )
    )

  def _delete_maps(self):
    targets = [OFFLINE_MAP_PATH, TMP_MAP_PATH, MAP_ROOT / "db"]
    targets.extend(Path(p) for p in glob.glob(str(MAP_ROOT / "v*")))

    for path in targets:
      try:
        if path.is_symlink() or path.is_file():
          path.unlink(missing_ok=True)
        elif path.is_dir():
          shutil.rmtree(path)
      except OSError:
        pass

    try:
      OFFLINE_MAP_PATH.mkdir(parents=True, exist_ok=True)
    except OSError:
      pass

    self._request_pending = False
    self._cancel_pending = False
    self._saw_tmp_activity = False
    self._last_tmp_activity_time = 0.0
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
        storage_changed = (
          self._installed_bytes != self._request_base_bytes or
          self._latest_map_mtime > self._request_base_mtime
        )

        # mapd removes /tmp after a location completes. Seeing tmp activity
        # followed by no tmp files is the strongest filesystem completion cue.
        tmp_quiet = self._saw_tmp_activity and (time.monotonic() - self._last_tmp_activity_time) > 4.0
        if tmp_quiet or (storage_changed and elapsed > 4.0 and not self._saw_tmp_activity) or elapsed > 30.0:
          self._request_pending = False
          self._cancel_pending = False
          self._saw_tmp_activity = False
          self._last_tmp_activity_time = 0.0
          self._refresh_storage(force=True)

    elif self._cancel_pending and not tmp_active:
      self._cancel_pending = False

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    super().show_event()
    self._refresh_storage(force=True)
    self._scroller.show_event()
