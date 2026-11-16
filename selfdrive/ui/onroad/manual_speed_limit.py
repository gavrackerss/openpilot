"""V279 driver-selected posted limit with offset-aware native Tesla cruise SET.

Every preset is two-tap confirmed within 3 s because the car's CAN-reported offset
may differ from Params. X immediately cancels the manual source, returning to Tesla.
No GUI code transmits CAN messages.

Visual-only update: UK circular speed signs, NSL symbol, and transparent Tesla X.
No rectangular button fills or outlines. Touch areas and control logic unchanged.
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

# Actual sign faces are opaque, but everything OUTSIDE the round sign is transparent.
# The X control is transparent throughout (except its strokes and label).
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
    # Unchanged original hit boxes: artwork does NOT reduce the touch targets.
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

  def _draw_sign_text(self, speed: str, center: rl.Vector2, radius: float):
    # Fit black speed numerals inside the circular white face, not the touch box.
    font_size = radius * 1.30
    dims = rl.measure_text_ex(self._font, speed, font_size, 0)
    if dims.x > radius * 1.40:
      font_size *= (radius * 1.40) / dims.x
      dims = rl.measure_text_ex(self._font, speed, font_size, 0)
    pos = rl.Vector2(center.x - dims.x / 2, center.y - dims.y / 2)
    rl.draw_text_ex(self._font, speed, pos, font_size, 0, SIGN_BLACK)

  def _draw_speed_sign(self, value: int, box: rl.Rectangle, is_selected: bool, is_pending: bool):
    # Reserve a thin bottom strip for the '70 mph' explanation under NSL.
    has_detail = (value == 70)
    art_h = box.height - (21.0 if has_detail else 0.0)
    radius = max(8.0, min(box.width * 0.42, art_h * 0.43))
    center = rl.Vector2(box.x + box.width / 2, box.y + art_h / 2)

    # Circle-only status indicator. No rectangle anywhere in the button graphics.
    if is_selected or is_pending:
      highlight = PENDING if is_pending else SELECTED
      rl.draw_ring(center, radius + 2.5, radius + 5.5, 0.0, 360.0, 48, highlight)

    # UK sign face, with a genuinely transparent background OUTSIDE the disc.
    rl.draw_circle_v(center, radius, SIGN_WHITE)
    if value == 70:
      # UK national speed limit: white circle with a descending black diagonal bar.
      # Semantics remain the pre-existing motorway-specific 70 mph preset.
      rl.draw_ring(center, radius - 1.0, radius, 0.0, 360.0, 48, SIGN_BLACK)
      p1 = rl.Vector2(center.x - radius * 0.48, center.y - radius * 0.61)
      p2 = rl.Vector2(center.x + radius * 0.48, center.y + radius * 0.61)
      stroke = max(3.0, radius * 0.22)
      rl.draw_line_ex(p1, p2, stroke, SIGN_BLACK)
      rl.draw_circle_v(p1, stroke / 2, SIGN_BLACK)
      rl.draw_circle_v(p2, stroke / 2, SIGN_BLACK)
      detail = rl.Rectangle(box.x, box.y + box.height - 22, box.width, 20)
      self._draw_centered('70 mph', self._smaller_font, 17.0, detail, MUTED)
    else:
      rl.draw_ring(center, radius * 0.78, radius, 0.0, 360.0, 48, SIGN_RED)
      self._draw_sign_text(str(value), center, radius)

  def _draw_tesla_cancel(self, box: rl.Rectangle, is_selected: bool):
    # X symbol only: no background panel, square outline or opaque disc.
    art_h = box.height - 21.0
    radius = max(8.0, min(box.width * 0.40, art_h * 0.40))
    center = rl.Vector2(box.x + box.width / 2, box.y + art_h / 2)
    c = MUTED if is_selected else CANCEL_RED
    d = radius * 0.49
    p1, p2 = rl.Vector2(center.x - d, center.y - d), rl.Vector2(center.x + d, center.y + d)
    p3, p4 = rl.Vector2(center.x + d, center.y - d), rl.Vector2(center.x - d, center.y + d)
    stroke = max(3.0, radius * 0.15)
    rl.draw_line_ex(p1, p2, stroke, c)
    rl.draw_line_ex(p3, p4, stroke, c)
    if is_selected:
      rl.draw_ring(center, radius + 2.5, radius + 5.5, 0.0, 360.0, 48, SELECTED)
    detail = rl.Rectangle(box.x, box.y + box.height - 22, box.width, 20)
    self._draw_centered('TESLA', self._smaller_font, 17.0, detail, MUTED)

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
      if value == 0:
        self._draw_tesla_cancel(box, selected)
      else:
        self._draw_speed_sign(value, box, selected, self._pending == value and time.monotonic() < self._pending_until)

  def _handle_mouse_release(self, mouse_pos):
    # UNCHANGED V279 interaction and Tesla control logic.
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
