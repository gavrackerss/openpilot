"""XNOR SunnyPilot model settings.

Expose the real downloadable SunnyPilot catalogue backed by modelManagerSP.
The verified Tinygrad runner remains fail-closed: missing, corrupt, unsupported,
or incomplete bundles fall back to stock modeld without erasing the selection.
"""
import os

from openpilot.cereal import custom
from openpilot.selfdrive.ui.layouts.settings.xnor_panel import XnorPanel
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.models.fetcher import ModelFetcher
from openpilot.sunnypilot.models.paths import model_root
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import text_item, button_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog


class ModelsLayout(XnorPanel):
  """Offroad front end for the verified SunnyPilot catalogue/download manager."""

  def __init__(self):
    super().__init__()

    self._runtime = text_item(
      "Onroad model runtime", "Stock modeld",
      description="Verified Tinygrad bundles use the dedicated SunnyPilot modeld_v2 process. Any integrity/runtime failure falls back to stock.",
    )
    self._downloaded = text_item(
      "Current model", "CD210 / XNOR Default",
      description="Selected SunnyPilot bundle for the next on-road session, or the XNOR stock model when no custom bundle is selected.",
    )
    self._download_status = text_item("Download status", "Manager starting")
    self._select = button_item(
      "Select SunnyPilot model", "SELECT",
      description="Shows all compatible standard non-Chestnut SunnyPilot models. Downloads are SHA-256 verified before activation.",
      callback=self._select_model, enabled=self.is_offroad,
    )
    self._refresh_catalog = button_item(
      "Refresh model catalogue", "REFRESH",
      callback=self._refresh_models, enabled=self.is_offroad,
    )
    self._clear_cache = button_item(
      "Clear downloaded model cache", "CLEAR",
      description="Keeps the currently selected bundle and removes other cached model files.",
      callback=self._confirm_clear_cache, enabled=self.is_offroad,
    )
    self._cache = text_item("Model cache", "0.0 MB")

    self.set_items([
      self._downloaded,
      self._runtime,
      self._download_status,
      self._select,
      self._refresh_catalog,
      self._clear_cache,
      self._cache,
    ])

  def _manager(self):
    try:
      return ui_state.sm["modelManagerSP"]
    except (KeyError, RuntimeError):
      return None

  def _available_bundles(self):
    """Use the live manager list, with a direct cached/fetcher fallback offroad.

    The UI can be opened before the first modelManagerSP message arrives. In
    that case the old 0.11.2 port produced an empty selector even though the
    upstream catalogue was valid. ModelFetcher shares the same Params caches,
    so this fallback also seeds the manager for the queued download.
    """
    manager = self._manager()
    if manager is not None:
      try:
        bundles = list(manager.availableBundles)
        if bundles:
          return bundles
      except (AttributeError, TypeError):
        pass

    if not self.is_offroad():
      return []

    try:
      return ModelFetcher(self._params).get_available_bundles()
    except Exception:
      return []

  def refresh(self):
    bundle = self._params.get("ModelManager_ActiveBundle")
    if isinstance(bundle, dict):
      name = bundle.get("internalName") or bundle.get("displayName") or bundle.get("ref")
      self._downloaded.action_item.set_text(str(name)[:70] if name else "CD210 / XNOR Default")
    else:
      self._downloaded.action_item.set_text("CD210 / XNOR Default")

    runtime = "Stock modeld"
    if isinstance(bundle, dict):
      runner = bundle.get("runner")
      runner_text = str(runner).lower()
      if runner == 1 or "tinygrad" in runner_text:
        name = bundle.get("internalName") or bundle.get("displayName") or "custom"
        runtime = f"SunnyPilot Tinygrad - {str(name)[:45]}"
    self._runtime.action_item.set_text(runtime)

    self._download_status.action_item.set_text(self._manager_status())
    self._cache.action_item.set_text(self._cache_size())

  def _manager_status(self) -> str:
    manager = self._manager()
    if manager is None:
      return "Manager starting"
    try:
      bundle = manager.selectedBundle
      if not bundle or not bundle.ref:
        count = len(list(manager.availableBundles))
        return f"Ready - {count} models" if count else "Ready - catalogue loading"

      states = {
        custom.ModelManagerSP.DownloadStatus.downloading: "Downloading",
        custom.ModelManagerSP.DownloadStatus.downloaded: "Downloaded",
        custom.ModelManagerSP.DownloadStatus.cached: "Cached",
        custom.ModelManagerSP.DownloadStatus.failed: "Failed",
      }
      state = states.get(bundle.status, "Pending")
      progresses = [float(m.artifact.downloadProgress.progress) for m in bundle.models if m.artifact.fileName]
      progress = min(progresses) if progresses else 0.0
      if bundle.status == custom.ModelManagerSP.DownloadStatus.downloading:
        return f"{state}: {bundle.internalName} {progress:.0f}%"
      return f"{state}: {bundle.internalName}"
    except (AttributeError, TypeError, ValueError):
      return "Ready"

  @staticmethod
  def _cache_size() -> str:
    root = model_root()
    try:
      total = sum(
        os.path.getsize(os.path.join(base, name))
        for base, _, filenames in os.walk(root)
        for name in filenames
        if os.path.isfile(os.path.join(base, name))
      )
      return f"{total / (1024 * 1024):.1f} MB"
    except OSError:
      return "0.0 MB"

  def _refresh_models(self):
    if self.is_offroad():
      # The legacy sentinel is consumed by ModelFetcher as a force-refresh;
      # expiring v23 as well makes the intent explicit after branch upgrades.
      self._params.put("ModelManager_LastSyncTime", -1)
      self._params.put("ModelManager_LastSyncTime_v23", -1)

  def _confirm_clear_cache(self):
    if not self.is_offroad():
      return

    def confirmed(result):
      if result == DialogResult.CONFIRM and self.is_offroad():
        self._params.put_bool("ModelManager_ClearCache", True)

    gui_app.push_widget(ConfirmDialog(
      "Delete cached custom models except the currently selected bundle?",
      "Clear Cache",
      callback=confirmed,
    ))

  def _select_model(self):
    if not self.is_offroad():
      return

    bundles = self._available_bundles()
    options = ["CD210 / XNOR Default"]
    refs = {options[0]: None}

    for bundle in bundles:
      label = (
        f"{bundle.internalName} - {bundle.displayName}"
        if bundle.displayName and bundle.displayName != bundle.internalName
        else bundle.internalName
      )
      if not label:
        continue
      if label in refs:
        label = f"{label} [{bundle.index}]"
      options.append(label)
      refs[label] = (bundle.ref, int(bundle.index))

    current = options[0]
    active_raw = self._params.get("ModelManager_ActiveBundle")
    active_ref = active_raw.get("ref") if isinstance(active_raw, dict) else None
    if active_ref:
      current = next(
        (label for label, value in refs.items() if value is not None and value[0] == active_ref),
        options[0],
      )

    def confirmed(result):
      if result != DialogResult.CONFIRM or not self.is_offroad():
        return

      selected = refs.get(dialog.selection)
      if selected is None:
        self._params.remove("ModelManager_DownloadIndex")
        self._params.remove("ModelManager_ActiveBundle")
        self._params.remove("ModelRunnerTypeCache")
      else:
        _, index = selected
        self._params.remove("ModelRunnerTypeCache")
        self._params.put("ModelManager_DownloadIndex", index)
      self.refresh()

    dialog = MultiOptionDialog("Select Driving Model", options, current=current, callback=confirmed)
    gui_app.push_widget(dialog)
