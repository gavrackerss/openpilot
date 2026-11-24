"""Restore the dedicated Models tab using the existing native preference API."""
from openpilot.selfdrive.ui.layouts.settings.xnor_panel import XnorPanel
from openpilot.sunnypilot.models.requested_model import CHOICES, REQUEST_KEY, requested_label, requested_ref, save_request
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.list_view import text_item, button_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog


class ModelsLayout(XnorPanel):
  def __init__(self):
    super().__init__()
    self._request = button_item("Requested model", "SELECT",
        description="Save an offroad preference. Unavailable or unqualified models fall back to stock.",
        callback=self._select, enabled=self.is_offroad)
    self._runtime = text_item("Model runtime", "Native selection",
        description="The selected preference alone does not confirm the model currently running.")
    self.set_items([self._request, self._runtime] +
        [text_item(c.label, c.status, description=c.architecture) for c in CHOICES])

  def refresh(self):
    try:
      label = requested_label(self._params.get(REQUEST_KEY), self._params.get("ModelManager_ActiveBundle"))
    except (ValueError, TypeError, OSError):
      label = "Unavailable"
    self._request.action_item.set_value(label)

  def _select(self):
    if not self.is_offroad():
      return
    ref = requested_ref(self._params.get(REQUEST_KEY), self._params.get("ModelManager_ActiveBundle"))
    labels = {f"{c.label} - {c.status}": c.ref for c in CHOICES}
    current = next((label for label, value in labels.items() if value == ref), "")

    def confirmed(result):
      if result == DialogResult.CONFIRM and self.is_offroad():
        choice = labels.get(dialog.selection)
        if choice is not None:
          save_request(self._params, choice, confirmed_offroad=True)
      self.refresh()

    dialog = MultiOptionDialog("Requested native model", list(labels), current=current, callback=confirmed)
    gui_app.push_widget(dialog)
