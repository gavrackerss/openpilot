import os

from cereal import custom
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware.hw import Paths
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.widgets import Widget, DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import button_item, text_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.system.ui.widgets.scroller_tici import Scroller


class ModelsLayout(Widget):
  """XNOR-native UI front end for Sunnypilot's model manager."""
  def __init__(self):
    super().__init__()
    self.params = Params()
    self._dialog = None
    self._name_to_ref = {}
    self._current = text_item(lambda: tr("Current Model"), self._current_model_name,
                              lambda: tr("Select from Sunnypilot driving-model bundles. Selection is available offroad only."))
    self._download = text_item(lambda: tr("Download Status"), self._download_status)
    self._select = button_item(lambda: tr("Select Model"), lambda: tr("SELECT"),
                               lambda: tr("Downloads are SHA-256 verified before activation. Default always returns to the XNOR stock model."),
                               callback=self._open_selector, enabled=ui_state.is_offroad)
    self._refresh = button_item(lambda: tr("Refresh Model List"), lambda: tr("REFRESH"),
                                callback=self._refresh_models, enabled=ui_state.is_offroad)
    self._clear = button_item(lambda: tr("Clear Downloaded Model Cache"), lambda: tr("CLEAR"),
                              callback=self._clear_cache, enabled=ui_state.is_offroad)
    self._cache = text_item(lambda: tr("Model Cache"), self._cache_size)
    self._scroller = Scroller([self._current, self._download, self._select, self._refresh, self._clear, self._cache],
                              line_separator=True, spacing=0)

  def _manager(self):
    try:
      return ui_state.sm["modelManagerSP"]
    except Exception:
      return None

  def _current_model_name(self):
    mm = self._manager()
    try:
      if mm and mm.activeBundle and mm.activeBundle.ref:
        return mm.activeBundle.internalName or mm.activeBundle.displayName
    except Exception:
      pass
    return "CD210 / XNOR Default"

  def _download_status(self):
    mm = self._manager()
    if mm is None:
      return tr("Manager starting")
    try:
      b = mm.selectedBundle
      if not b or not b.ref:
        return tr("Ready")
      states = {
        custom.ModelManagerSP.DownloadStatus.downloading: tr("Downloading"),
        custom.ModelManagerSP.DownloadStatus.downloaded: tr("Downloaded"),
        custom.ModelManagerSP.DownloadStatus.cached: tr("Cached"),
        custom.ModelManagerSP.DownloadStatus.failed: tr("Failed"),
      }
      state = states.get(b.status, tr("Pending"))
      progresses = [float(m.artifact.downloadProgress.progress) for m in b.models if m.artifact.fileName]
      pct = min(progresses) if progresses else 0.0
      return f"{state}: {b.internalName} {pct:.0f}%" if b.status == custom.ModelManagerSP.DownloadStatus.downloading else f"{state}: {b.internalName}"
    except Exception:
      return tr("Ready")

  def _cache_size(self):
    root = Paths.model_root()
    try:
      total = sum(os.path.getsize(os.path.join(root, f)) for f in os.listdir(root) if os.path.isfile(os.path.join(root, f)))
      return f"{total / (1024 * 1024):.1f} MB"
    except OSError:
      return "0.0 MB"

  def _refresh_models(self):
    # V238: -1 is an explicit refresh request. The model fetcher consumes it and
    # bypasses retry backoff once the wall clock is safe for HTTPS.
    self.params.put("ModelManager_LastSyncTime", -1)

  def _clear_cache(self):
    def cb(result):
      if result == DialogResult.CONFIRM:
        self.params.put_bool("ModelManager_ClearCache", True)
    gui_app.push_widget(ConfirmDialog(tr("Delete all cached custom models except the active one?"), tr("Clear Cache"), callback=cb))

  def _open_selector(self):
    mm = self._manager()
    if mm is None:
      return
    options = ["CD210 / XNOR Default"]
    self._name_to_ref = {options[0]: "Default"}
    try:
      bundles = list(mm.availableBundles)
    except Exception:
      bundles = []
    for b in bundles:
      label = f"{b.internalName} — {b.displayName}" if b.displayName and b.displayName != b.internalName else b.internalName
      if not label:
        continue
      # make duplicate display names deterministic
      if label in self._name_to_ref:
        label = f"{label} [{b.index}]"
      options.append(label)
      self._name_to_ref[label] = b.ref
    current = options[0]
    try:
      if mm.activeBundle and mm.activeBundle.ref:
        current = next((k for k, v in self._name_to_ref.items() if v == mm.activeBundle.ref), options[0])
    except Exception:
      pass
    self._dialog = MultiOptionDialog(tr("Select Driving Model"), options, current=current, callback=self._on_selected)
    gui_app.push_widget(self._dialog)

  def _on_selected(self, result):
    if result != DialogResult.CONFIRM or self._dialog is None:
      self._dialog = None
      return
    selected = self._dialog.selection
    ref = self._name_to_ref.get(selected)
    mm = self._manager()
    if ref == "Default":
      self.params.remove("ModelManager_ActiveBundle")
      self.params.remove("ModelRunnerTypeCache")
    elif ref and mm is not None:
      try:
        bundle = next((b for b in mm.availableBundles if b.ref == ref), None)
        if bundle is not None:
          self.params.put("ModelManager_DownloadIndex", int(bundle.index))
      except Exception:
        pass
    self._dialog = None

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
