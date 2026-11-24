"""Native offroad XNOR panels, sharing the 0.11.2 scroller lifecycle."""
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.scroller_tici import Scroller


class XnorPanel(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()

  def set_items(self, items):
    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def is_offroad(self):
    return bool(ui_state.is_offroad() and self._params.get_bool("IsOffroad"))

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self.refresh()

  def hide_event(self):
    self._scroller.hide_event()
    super().hide_event()

  def refresh(self):
    pass
