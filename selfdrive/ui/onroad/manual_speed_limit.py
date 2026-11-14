"""V279 driver-selected posted limit with offset-aware native Tesla cruise SET.

Every preset is two-tap confirmed within 3 s because the car's CAN-reported offset
may differ from Params. X immediately cancels the manual source, returning to Tesla.
No GUI code transmits CAN messages.

Visual-only adjustment: speed buttons have completely transparent fills.
"""
from __future__ import annotations

import time
import pyray as rl
from openpilot.common.params import Params
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget

PARAM = "XNORManualSpeedLimitMph"
OPTIONS = ((40, "40"), (50, "50"), (60, "60"), (70, "NSL"), (0, "X"))
# No background paint: the driving camera remains fully visible inside every button.
OUTLINE = rl.Color(230, 237, 237, 155)
OUTLINE_SELECTED = rl.Color(74, 239, 163, 255)
OUTLINE_CANCEL = rl.Color(255, 170, 170, 190)
TEXT = rl.Color(255, 255, 255, 255)
MUTED = rl.Color(201, 211, 213, 255)
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
    self._pending = None
    self._pending_until = 0.0
    self._actual_set_mph = 0.0
    self.set_visible(self._is_tesla)

  @staticmethod
  def _is_tesla() -> bool:
    cp = ui_state.CP
    return bool(ui_state.started and cp is not None and getattr(cp, "brand", "") == "tesla")

  def _update_state(self):
    now = time.monotonic()
    if now - self._last_read < 0.4:
      return
    self._last_read = now
    try:
      value = int(self._params.get(PARAM) or 0)
      self._value = value if value in (40, 50, 60, 70) else 0
    except Exception:
      self._value = 0
    try:
      state = ui_state.sm['carState'].cruiseState
      self._actual_set_mph = float(state.speedCluster) * CV.MS_TO_MPH if bool(state.enabled) else 0.0
    except Exception:
      self._actual_set_mph = 0.0
    if self._pending is not None and now >= self._pending_until:
      self._pending = None
    if self._status_until and now >= self._status_until:
      self._status = ''

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
    dims = rl.measure_text_ex(font, text, size, 0)
    pos = rl.Vector2(rect.x + (rect.width - dims.x) / 2,
                     rect.y + (rect.height - dims.y) / 2)
    rl.draw_text_ex(font, text, rl.Vector2(pos.x + 2, pos.y + 2), size, 0, SHADOW)
    rl.draw_text_ex(font, text, pos, size, 0, color)

  def _render(self, rect: rl.Rectangle):
    label = f"POSTED {self._value} mph - OFFSET APPLIES" if self._value else "TESLA SPEED LIMIT - AUTO"
    if self._actual_set_mph > 0:
      label += f" - TESLA SET {round(self._actual_set_mph)} mph"
    if self._status:
      label += f" - {self._status}"
    header_pos = rl.Vector2(rect.x + 5, rect.y + 2)
    rl.draw_text_ex(self._smaller_font, label, rl.Vector2(header_pos.x + 2, header_pos.y + 2), 24, 0, SHADOW)
    rl.draw_text_ex(self._smaller_font, label, header_pos, 24, 0, TEXT)
    for (value, text), box in zip(OPTIONS, self._button_rects()):
      selected = value == self._value
      outline = OUTLINE_SELECTED if selected else (OUTLINE_CANCEL if value == 0 else OUTLINE)
      # Draw an outline only: no rectangular fill, even when selected or cancelled.
      rl.draw_rectangle_rounded_lines_ex(box, 0.22, 8, 4.0 if selected else 2.5, outline)
      self._draw_centered(text, self._font, 46 if text.isdigit() else 33, box, TEXT)
      if value == 70:
        detail = rl.Rectangle(box.x, box.y + box.height - 27, box.width, 25)
        self._draw_centered("70 mph", self._smaller_font, 17, detail, MUTED)
      elif value == 0:
        detail = rl.Rectangle(box.x, box.y + box.height - 27, box.width, 25)
        self._draw_centered("TESLA", self._smaller_font, 17, detail, MUTED)

  def _handle_mouse_release(self, mouse_pos):
    for (value, label), box in zip(OPTIONS, self._button_rects()):
      if not rl.check_collision_point_rec(mouse_pos, box):
        continue
      now = time.monotonic()
      if value:
        try:
          car_state = ui_state.sm['carState'].cruiseState
          eligible = bool(ui_state.engaged and car_state.enabled and self._actual_set_mph > 0)
        except Exception:
          eligible = False
        if not eligible:
          self._status = 'ENGAGE CRUISE FIRST'
          self._status_until = now + 3.0
          return
        # Selected posted speed may increase native SET due to offset.
        # Require two deliberate taps of any numbered preset.
        if self._pending != value or now > self._pending_until:
          self._pending = value
          self._pending_until = now + 3.0
          self._status = f'TAP {label} AGAIN'
          self._status_until = now + 3.0
          return
      self._pending = None
      try:
        self._params.put(PARAM, int(value))  # strictly typed INT
        self._value = value
        self._last_read = 0.0
        self._status = 'TESLA AUTO' if value == 0 else f'POSTED {value}: OFFSET APPLIES'
        self._status_until = now + 3.0
      except Exception:
        self._status = 'NOT APPLIED'
        self._status_until = now + 3.0
      return
