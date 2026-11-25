"""XNOR model settings.

Restores the SunnyPilot catalogue/download workflow and the verified Tinygrad
on-road runner. Stock modeld remains the fail-closed fallback for missing,
corrupt or unsupported bundles.
"""
import os

from openpilot.cereal import custom
from openpilot.selfdrive.ui.layouts.settings.xnor_panel import XnorPanel
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.sunnypilot.models.requested_model import CHOICES, REQUEST_KEY, requested_label, requested_ref, save_request
from openpilot.sunnypilot.models.paths import model_root
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.widgets.list_view import text_item, button_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog


class ModelsLayout(XnorPanel):
  def __init__(self):
    super().__init__()

    self._request = button_item(
      "Native artifact experiment", "SELECT",
      description="Separate native 0.11.2 artifact preference. This does not override an active verified SunnyPilot Tinygrad bundle.",
      callback=self._select_requested, enabled=self.is_offroad,
    )
    self._runtime = text_item(
      "Onroad model runtime", "Stock modeld",
      description="Verified Tinygrad bundles run through the dedicated SunnyPilot modeld_v2 process. Any runtime/integrity failure falls back to stock.",
    )

    self._downloaded = text_item(
      "Active SunnyPilot bundle", "None",
      description="The downloaded and SHA-256 verified bundle selected for the next on-road session.",
    )
    self._download_status = text_item("Download status", "Manager starting")
    self._download_select = button_item(
      "Download model bundle", "SELECT",
      description="Downloads and SHA-256 verifies a bundle offroad. A Tinygrad bundle becomes the on-road model after the next transition.",
      callback=self._select_download, enabled=self.is_offroad,
    )
    self._refresh_catalog = button_item(
      "Refresh model catalogue", "REFRESH",
      callback=self._refresh_models, enabled=self.is_offroad,
    )
    self._clear_cache = button_item(
      "Clear downloaded model cache", "CLEAR",
      description="Keeps the currently downloaded bundle and removes other cached model files.",
      callback=self._confirm_clear_cache, enabled=self.is_offroad,
    )
    self._cache = text_item("Model cache", "0.0 MB")

    choices = [text_item(c.label, c.status, description=c.architecture) for c in CHOICES]
    self.set_items([
      self._request,
      self._runtime,
      self._downloaded,
      self._download_status,
      self._download_select,
      self._refresh_catalog,
      self._clear_cache,
      self._cache,
      *choices,
    ])

  def _manager(self):
    try:
      return ui_state.sm["modelManagerSP"]
    except (KeyError, RuntimeError):
      return None

  def refresh(self):
    try:
      label = requested_label(self._params.get(REQUEST_KEY), self._params.get("ModelManager_ActiveBundle"))
    except (ValueError, TypeError, OSError):
      label = "Unavailable"
    self._request.action_item.set_value(label)

    bundle = self._params.get("ModelManager_ActiveBundle")
    if isinstance(bundle, dict):
      name = bundle.get("internalName") or bundle.get("displayName") or bundle.get("ref")
      self._downloaded.action_item.set_text(str(name)[:70] if name else "None")
    else:
      self._downloaded.action_item.set_text("None")

    runtime = "Stock modeld"
    if isinstance(bundle, dict):
      runner = bundle.get("runner")
      runner_text = str(runner).lower()
      if runner == 1 or "tinygrad" in runner_text:
        name = bundle.get("internalName") or bundle.get("displayName") or "custom"
        runtime = f"SunnyPilot Tinygrad — {str(name)[:45]}"
    self._runtime.action_item.set_text(runtime)

    self._download_status.action_item.set_text(self._manager_status())
    self._cache.action_item.set_text(self._cache_size())

  def _select_requested(self):
    if not self.is_offroad():
      return

    legacy = self._params.get("ModelManager_ActiveBundle")
    ref = requested_ref(self._params.get(REQUEST_KEY), legacy)
    labels = {f"{choice.label} - {choice.status}": choice.ref for choice in CHOICES}
    current = next((label for label, value in labels.items() if value == ref), "")

    def confirmed(result):
      if result == DialogResult.CONFIRM and self.is_offroad():
        choice = labels.get(dialog.selection)
        if choice is not None:
          save_request(self._params, choice, confirmed_offroad=True)
      self.refresh()

    dialog = MultiOptionDialog("Native 0.11.2 artifact experiment", list(labels), current=current, callback=confirmed)
    gui_app.push_widget(dialog)

  def _manager_status(self) -> str:
    manager = self._manager()
    if manager is None:
      return "Manager starting"
    try:
      bundle = manager.selectedBundle
      if not bundle or not bundle.ref:
        return "Ready"

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
      self._params.put("ModelManager_LastSyncTime", -1)

  def _confirm_clear_cache(self):
    if not self.is_offroad():
      return

    def confirmed(result):
      if result == DialogResult.CONFIRM and self.is_offroad():
        self._params.put_bool("ModelManager_ClearCache", True)

    gui_app.push_widget(ConfirmDialog(
      "Delete cached custom models except the current downloaded bundle?",
      "Clear Cache",
      callback=confirmed,
    ))

  def _select_download(self):
    if not self.is_offroad():
      return
    manager = self._manager()
    if manager is None:
      return

    options = ["No downloaded custom bundle"]
    refs = {options[0]: None}
    try:
      bundles = list(manager.availableBundles)
    except (AttributeError, TypeError):
      bundles = []

    for bundle in bundles:
      label = (
        f"{bundle.internalName} — {bundle.displayName}"
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
    try:
      active = manager.activeBundle
      if active and active.ref:
        current = next(
          (label for label, value in refs.items() if value is not None and value[0] == active.ref),
          options[0],
        )
    except (AttributeError, TypeError):
      pass

    def confirmed(result):
      if result != DialogResult.CONFIRM or not self.is_offroad():
        return
      selected = refs.get(dialog.selection)
      if selected is None:
        self._params.remove("ModelManager_ActiveBundle")
        self._params.remove("ModelRunnerTypeCache")
      else:
        _, index = selected
        self._params.put("ModelManager_DownloadIndex", index)
      self.refresh()

    dialog = MultiOptionDialog("Download model bundle", options, current=current, callback=confirmed)
    gui_app.push_widget(dialog)
