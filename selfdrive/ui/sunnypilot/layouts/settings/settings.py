"""Sunnypilot-style settings shell adapted for XNOR.

Visual/UI port only: panel contents remain XNOR's existing implementations.
No Sunnypilot control, MADS, Sunnylink, map or longitudinal services are loaded.
"""
from dataclasses import dataclass

import pyray as rl
from openpilot.selfdrive.ui.layouts.settings import settings as OP
from openpilot.selfdrive.ui.layouts.settings.developer import DeveloperLayout
from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
from openpilot.selfdrive.ui.layouts.settings.firehose import FirehoseLayout
from openpilot.selfdrive.ui.layouts.settings.models import ModelsLayout
from openpilot.selfdrive.ui.layouts.settings.osm import OSMLayout
from openpilot.selfdrive.ui.layouts.settings.software import SoftwareLayout
from openpilot.selfdrive.ui.layouts.settings.tesla import TeslaLayout
from openpilot.selfdrive.ui.layouts.settings.toggles import TogglesLayout
from openpilot.system.ui.lib.application import gui_app, MousePos
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.wifi_manager import WifiManager
from openpilot.system.ui.sunnypilot.lib.styles import style
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.network import NetworkUI
from openpilot.system.ui.widgets.scroller_tici import Scroller

# Match Sunnypilot's darker settings panel while retaining XNOR's panel bodies.
OP.PANEL_COLOR = rl.Color(10, 10, 10, 255)
ICON_SIZE = 70


@dataclass
class PanelInfo(OP.PanelInfo):
  icon: str = ""


class NavButton(Widget):
  def __init__(self, parent, panel_type, panel_info):
    super().__init__()
    self.parent = parent
    self.panel_type = panel_type
    self.panel_info = panel_info

  def _render(self, rect: rl.Rectangle):
    is_selected = self.panel_type == self.parent._current_panel
    text_color = OP.TEXT_SELECTED if is_selected else OP.TEXT_NORMAL
    content_x = rect.x + 90

    if is_selected:
      selected_rect = rl.Rectangle(content_x - 50, rect.y, OP.SIDEBAR_WIDTH - 50, OP.NAV_BTN_HEIGHT)
      rl.draw_rectangle_rounded(selected_rect, 0.2, 5, OP.CLOSE_BTN_COLOR)

    if self.panel_info.icon:
      icon = gui_app.texture(self.panel_info.icon, ICON_SIZE, ICON_SIZE, keep_aspect_ratio=True)
      rl.draw_texture_ex(icon, rl.Vector2(content_x, rect.y + (OP.NAV_BTN_HEIGHT - icon.height) / 2), 0.0, 1.0, rl.WHITE)
      content_x += ICON_SIZE + 20

    panel_name = tr(self.panel_info.name)
    text_size = measure_text_cached(self.parent._font_medium, panel_name, 55)
    rl.draw_text_ex(self.parent._font_medium, panel_name,
                    rl.Vector2(content_x, rect.y + (OP.NAV_BTN_HEIGHT - text_size.y) / 2),
                    55, 0, text_color)
    self.panel_info.button_rect = rect


class SettingsLayoutSP(OP.SettingsLayout):
  """Sunnypilot navigation chrome around XNOR-native settings panels."""
  def __init__(self):
    OP.SettingsLayout.__init__(self)
    self._nav_items: list[Widget] = []
    self._sidebar_scroller = Scroller([], spacing=0, line_separator=False, pad_end=False)

    wifi_manager = WifiManager()
    wifi_manager.set_active(False)

    self._panels = {
      OP.PanelType.DEVICE: PanelInfo(tr_noop("Device"), DeviceLayout(), icon="sunnypilot/offroad/icon_home.png"),
      OP.PanelType.NETWORK: PanelInfo(tr_noop("Network"), NetworkUI(wifi_manager), icon="icons/network.png"),
      OP.PanelType.TOGGLES: PanelInfo(tr_noop("Toggles"), TogglesLayout(), icon="sunnypilot/offroad/icon_toggle.png"),
      OP.PanelType.SOFTWARE: PanelInfo(tr_noop("Software"), SoftwareLayout(), icon="sunnypilot/offroad/icon_software.png"),
      OP.PanelType.MODELS: PanelInfo(tr_noop("Models"), ModelsLayout(), icon="sunnypilot/offroad/icon_models.png"),
      OP.PanelType.OSM: PanelInfo(tr_noop("OSM"), OSMLayout(), icon="icons/road.png"),
      OP.PanelType.TESLA: PanelInfo(tr_noop("Tesla"), TeslaLayout(), icon="sunnypilot/offroad/icon_vehicle.png"),
      OP.PanelType.FIREHOSE: PanelInfo(tr_noop("Firehose"), FirehoseLayout(), icon="sunnypilot/offroad/icon_firehose.png"),
      OP.PanelType.DEVELOPER: PanelInfo(tr_noop("Developer"), DeveloperLayout(), icon="icons/shell.png"),
    }

  def _draw_sidebar(self, rect: rl.Rectangle):
    rl.draw_rectangle_rec(rect, OP.SIDEBAR_COLOR)

    close_btn_rect = rl.Rectangle(rect.x + style.ITEM_PADDING * 3,
                                  rect.y + style.ITEM_PADDING * 2,
                                  style.CLOSE_BTN_SIZE, style.CLOSE_BTN_SIZE)
    pressed = (rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT) and
               rl.check_collision_point_rec(rl.get_mouse_position(), close_btn_rect))
    close_color = OP.CLOSE_BTN_PRESSED if pressed else OP.CLOSE_BTN_COLOR
    rl.draw_rectangle_rounded(close_btn_rect, 1.0, 20, close_color)

    icon_color = rl.Color(220, 220, 220, 255) if pressed else rl.WHITE
    icon_dest = rl.Rectangle(close_btn_rect.x + (close_btn_rect.width - self._close_icon.width) / 2,
                             close_btn_rect.y + (close_btn_rect.height - self._close_icon.height) / 2,
                             self._close_icon.width, self._close_icon.height)
    rl.draw_texture_pro(self._close_icon,
                        rl.Rectangle(0, 0, self._close_icon.width, self._close_icon.height),
                        icon_dest, rl.Vector2(0, 0), 0, icon_color)
    self._close_btn_rect = close_btn_rect

    if not self._nav_items:
      for panel_type, panel_info in self._panels.items():
        nav_button = NavButton(self, panel_type, panel_info)
        nav_button.rect.width = rect.width - 100
        nav_button.rect.height = OP.NAV_BTN_HEIGHT
        self._nav_items.append(nav_button)
        self._sidebar_scroller.add_widget(nav_button)

    nav_rect = rl.Rectangle(rect.x,
                            self._close_btn_rect.height + style.ITEM_PADDING * 4,
                            rect.width,
                            rect.height - 260)
    self._sidebar_scroller.render(nav_rect)

  def _handle_mouse_release(self, mouse_pos: MousePos) -> bool:
    if rl.check_collision_point_rec(mouse_pos, self._close_btn_rect):
      if self._close_callback:
        self._close_callback()
      return True

    for panel_type, panel_info in self._panels.items():
      if (rl.check_collision_point_rec(mouse_pos, panel_info.button_rect) and
          self._sidebar_scroller.scroll_panel.is_touch_valid()):
        self.set_current_panel(panel_type)
        return True
    return False

  def show_event(self):
    super().show_event()
    self._sidebar_scroller.show_event()
