"""Live V279 manual posted-speed selector for Tesla.

The UI writes only XNORManualSpeedLimitMph. Vehicle CAN remains exclusively
owned by the Tesla CarController, which applies the configured offset and uses
the existing bounded STW_ACTN_RQ pulse path to adjust an already-engaged native
Tesla cruise set speed. Numbered presets require a deliberate two-tap confirm.
"""
from __future__ import annotations

import time
import pyray as rl

from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

PARAM = "XNORManualSpeedLimitMph"
OPTIONS = ((40, "40"), (50, "50"), (60, "60"), (70, "NSL"), (0, "X"))
PRESETS = (40, 50, 60, 70)
CONFIRM_SECONDS = 3.0

SIGN_WHITE = rl.Color(255, 255, 255, 255)
SIGN_RED = rl.Color(210, 28, 35, 255)
SIGN_BLACK = rl.Color(20, 20, 20, 255)
SELECTED = rl.Color(74, 239, 163, 255)
PENDING = rl.Color(255, 201, 61, 255)
CANCEL_RED = rl.Color(255, 156, 156, 255)
TEXT = rl.Color(255, 255, 255, 255)
MUTED = rl.Color(225, 232, 232, 255)
SHADOW = rl.Color(0, 0, 0, 215)


class ManualSpeedLimitBar(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._value = 0
    self._last_read = -1000.0
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._smaller_font = gui_app.font(FontWeight.MEDIUM)
    self._status = ""
    self._status_until = 0.0
    self._pending = 0
    self._pending_until = 0.0
    self._actual_set_mph = 0.0
    self._map_speed_mph = 0.0
    self._map_road_name = ""

  @staticmethod
  def _is_tesla() -> bool:
    cp = ui_state.CP
    return bool(ui_state.started and cp is not None and getattr(cp, "brand", "") == "tesla")

  def _update_state(self):
    now = time.monotonic()
    if now - self._last_read >= 0.4:
      self._last_read = now
      try:
        value = int(self._params.get(PARAM) or 0)
        self._value = value if value in PRESETS else 0
      except Exception:
        self._value = 0

      try:
        cruise = ui_state.sm["carState"].cruiseState
        self._actual_set_mph = float(cruise.speedCluster) * CV.MS_TO_MPH if bool(cruise.enabled) else 0.0
      except Exception:
        self._actual_set_mph = 0.0

      try:
        if ui_state.sm.alive["mapdOut"] and ui_state.sm.valid["mapdOut"]:
          mapd = ui_state.sm["mapdOut"]
          speed_ms = float(mapd.speedLimit)
          self._map_speed_mph = speed_ms * CV.MS_TO_MPH if speed_ms > 0.1 else 0.0
          self._map_road_name = str(mapd.roadName or mapd.wayName or "")[:28]
        else:
          self._map_speed_mph = 0.0
          self._map_road_name = ""
      except Exception:
        self._map_speed_mph = 0.0
        self._map_road_name = ""

    if self._pending and now >= self._pending_until:
      self._pending = 0
      self._pending_until = 0.0
    if self._status_until and now >= self._status_until:
      self._status = ""

  def _button_rects(self):
    r = self._rect
    gap = 12.0
    horizontal_padding = 5.0
    inner_w = max(1.0, r.width - horizontal_padding * 2)
    btn_w = (inner_w - gap * 4) / 5
    btn_y = r.y + 35
    btn_h = max(30.0, r.height - 39)
    return [rl.Rectangle(r.x + horizontal_padding + i * (btn_w + gap), btn_y, btn_w, btn_h)
            for i in range(5)]

  def _draw_centered(self, text: str, font, size: float, rect: rl.Rectangle, color):
    dims = measure_text_cached(font, text, max(1, int(round(size))), 0)
    pos = rl.Vector2(rect.x + (rect.width - dims.x) / 2,
                     rect.y + (rect.height - dims.y) / 2)
    rl.draw_text_ex(font, text, rl.Vector2(pos.x + 2, pos.y + 2), size, 0, SHADOW)
    rl.draw_text_ex(font, text, pos, size, 0, color)

  def _draw_sign_text(self, speed: str, center: rl.Vector2, radius: float):
    font_size = radius * 1.22
    dims = measure_text_cached(self._font, speed, max(1, int(round(font_size))), 0)
    if dims.x > radius * 1.28:
      font_size *= (radius * 1.28) / dims.x
      dims = measure_text_cached(self._font, speed, max(1, int(round(font_size))), 0)
    pos = rl.Vector2(center.x - dims.x / 2, center.y - dims.y / 2)
    rl.draw_text_ex(self._font, speed, pos, font_size, 0, SIGN_BLACK)

  def _draw_speed_sign(self, value: int, box: rl.Rectangle, selected: bool, pending: bool):
    art_h = box.height - (21.0 if value == 70 else 0.0)
    radius = max(8.0, min(box.width * 0.45, art_h * 0.46))
    center = rl.Vector2(box.x + box.width / 2, box.y + art_h / 2)

    if selected or pending:
      highlight = PENDING if pending else SELECTED
      rl.draw_ring(center, radius + 2.5, radius + 5.5, 0.0, 360.0, 48, highlight)

    rl.draw_circle_v(center, radius, SIGN_WHITE)
    if value == 70:
      rl.draw_ring(center, radius - 1.0, radius, 0.0, 360.0, 48, SIGN_BLACK)
      p1 = rl.Vector2(center.x - radius * 0.48, center.y - radius * 0.61)
      p2 = rl.Vector2(center.x + radius * 0.48, center.y + radius * 0.61)
      stroke = max(3.0, radius * 0.22)
      rl.draw_line_ex(p1, p2, stroke, SIGN_BLACK)
      rl.draw_circle_v(p1, stroke / 2, SIGN_BLACK)
      rl.draw_circle_v(p2, stroke / 2, SIGN_BLACK)
      detail = rl.Rectangle(box.x, box.y + box.height - 22, box.width, 20)
      self._draw_centered("70 mph", self._smaller_font, 17.0, detail, MUTED)
    else:
      rl.draw_ring(center, radius * 0.80, radius, 0.0, 360.0, 48, SIGN_RED)
      self._draw_sign_text(str(value), center, radius)

  def _draw_cancel(self, box: rl.Rectangle, selected: bool):
    art_h = box.height - 21.0
    radius = max(8.0, min(box.width * 0.40, art_h * 0.40))
    center = rl.Vector2(box.x + box.width / 2, box.y + art_h / 2)
    c = MUTED if selected else CANCEL_RED
    d = radius * 0.49
    stroke = max(3.0, radius * 0.15)
    rl.draw_line_ex(rl.Vector2(center.x - d, center.y - d), rl.Vector2(center.x + d, center.y + d), stroke, c)
    rl.draw_line_ex(rl.Vector2(center.x + d, center.y - d), rl.Vector2(center.x - d, center.y + d), stroke, c)
    if selected:
      rl.draw_ring(center, radius + 2.5, radius + 5.5, 0.0, 360.0, 48, SELECTED)
    detail = rl.Rectangle(box.x, box.y + box.height - 22, box.width, 20)
    self._draw_centered("TESLA", self._smaller_font, 17.0, detail, MUTED)

  def _render(self, rect: rl.Rectangle):
    if not self._is_tesla():
      return

    label = f"POSTED {self._value} mph - OFFSET APPLIES" if self._value else "TESLA SPEED LIMIT - AUTO"
    if self._map_speed_mph > 0:
      label += f" - OSM {round(self._map_speed_mph)} mph"
      if self._map_road_name:
        label += f" {self._map_road_name}"
    if self._actual_set_mph > 0:
      label += f" - TESLA SET {round(self._actual_set_mph)} mph"
    if self._status:
      label += f" - {self._status}"
    rl.draw_text_ex(self._smaller_font, label, rl.Vector2(rect.x + 5, rect.y + 2), 24, 0, TEXT)

    for (value, _), box in zip(OPTIONS, self._button_rects(), strict=False):
      if value == 0:
        self._draw_cancel(box, self._value == 0)
      else:
        self._draw_speed_sign(value, box, value == self._value, value == self._pending)

  def _handle_mouse_release(self, mouse_pos):
    if not self._is_tesla():
      return

    for (value, label), box in zip(OPTIONS, self._button_rects(), strict=False):
      if not rl.check_collision_point_rec(mouse_pos, box):
        continue

      now = time.monotonic()
      if value:
        try:
          cruise = ui_state.sm["carState"].cruiseState
          eligible = bool(ui_state.engaged and cruise.enabled and self._actual_set_mph > 0)
        except Exception:
          eligible = False
        if not eligible:
          self._status = "ENGAGE CRUISE FIRST"
          self._status_until = now + 3.0
          return

        if self._pending != value or now >= self._pending_until:
          self._pending = value
          self._pending_until = now + CONFIRM_SECONDS
          self._status = f"TAP {label} AGAIN"
          self._status_until = self._pending_until
          return

      self._pending = 0
      self._pending_until = 0.0
      try:
        self._params.put(PARAM, int(value))
        self._value = value
        self._last_read = 0.0
        self._status = "TESLA AUTO" if value == 0 else f"POSTED {value}: OFFSET APPLIES"
        self._status_until = now + 3.0
      except Exception:
        self._status = "NOT APPLIED"
        self._status_until = now + 3.0
      return
