"""Native 0.11.2 visual-only port of the installed V279 speed sign bar.

Source: installed 24d304b selfdrive/ui/onroad/manual_speed_limit.py.
This is a standalone preview widget, not an onroad control. It never writes
Params, reads vehicle state, changes cruise speed, or handles touch input.
"""
from __future__ import annotations

import pyray as rl
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.lib.text_measure import measure_text_cached

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


class ManualSpeedLimitPreview(Widget):
  def __init__(self):
    super().__init__()
    self.set_enabled(False)
    self._font = gui_app.font(FontWeight.SEMI_BOLD)
    self._smaller_font = gui_app.font(FontWeight.MEDIUM)

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
    dims = measure_text_cached(font, text, max(1, int(round(size))), 0)
    pos = rl.Vector2(rect.x + (rect.width - dims.x) / 2,
                     rect.y + (rect.height - dims.y) / 2)
    rl.draw_text_ex(font, text, rl.Vector2(pos.x + 2, pos.y + 2), size, 0, SHADOW)
    rl.draw_text_ex(font, text, pos, size, 0, color)

  def _draw_sign_text(self, speed: str, center: rl.Vector2, radius: float):
    # Fit black speed numerals inside the circular white face, not the touch box.
    # Keep numerals approximately the same physical size as the previous UI while expanding the rings.
    font_size = radius * 1.22
    dims = measure_text_cached(self._font, speed, max(1, int(round(font_size))), 0)
    # Leave visible white clearance between digits and the red ring.
    if dims.x > radius * 1.28:
      font_size *= (radius * 1.28) / dims.x
      dims = measure_text_cached(self._font, speed, max(1, int(round(font_size))), 0)
    pos = rl.Vector2(center.x - dims.x / 2, center.y - dims.y / 2)
    rl.draw_text_ex(self._font, speed, pos, font_size, 0, SIGN_BLACK)

  def _draw_speed_sign(self, value: int, box: rl.Rectangle, is_selected: bool, is_pending: bool):
    # Reserve a thin bottom strip for the '70 mph' explanation under NSL.
    has_detail = (value == 70)
    art_h = box.height - (21.0 if has_detail else 0.0)
    # About 7% larger discs while staying inside the existing invisible touch areas.
    radius = max(8.0, min(box.width * 0.45, art_h * 0.46))
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
      rl.draw_ring(center, radius * 0.80, radius, 0.0, 360.0, 48, SIGN_RED)
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
    label = "PREVIEW ONLY - NOT A CRUISE CONTROL"
    pos = rl.Vector2(rect.x + 5, rect.y + 2)
    rl.draw_text_ex(self._smaller_font, label, pos, 24, 0, TEXT)
    for (value, _), box in zip(OPTIONS, self._button_rects()):
      if value == 0:
        self._draw_tesla_cancel(box, False)
      else:
        self._draw_speed_sign(value, box, False, False)



class SpeedSignPreviewRow(Widget):
  """Passive, scrollable settings-page sample; never an onroad overlay."""

  def __init__(self):
    super().__init__()
    self.set_rect(rl.Rectangle(0, 0, 0, 170))
    self.set_enabled(False)
    self._preview = self._child(ManualSpeedLimitPreview())

  def set_parent_rect(self, parent_rect: rl.Rectangle):
    super().set_parent_rect(parent_rect)
    self.set_rect(rl.Rectangle(self.rect.x, self.rect.y, parent_rect.width, 170))

  def _render(self, rect: rl.Rectangle):
    width = min(900., rect.width - 20.)
    if width >= 350.:
      self._preview.render(rl.Rectangle(rect.x + (rect.width-width)/2,
                                        rect.y + 19, width, 132))


def main():
  """Desktop-only preview; never start a second UI on the comma device."""
  from openpilot.common.hardware import PC
  if not PC:
    raise RuntimeError("The preview requires a desktop checkout")
  gui_app.init_window("XNOR 0.11.2 speed sign preview")
  try:
    preview = ManualSpeedLimitPreview()
    for should_render, _, _ in gui_app.render():
      if should_render:
        width = min(900., gui_app.width - 20.)
        if width >= 350. and gui_app.height >= 180:
          preview.render(rl.Rectangle((gui_app.width-width)/2,
                                      (gui_app.height-132)/2, width, 132))
  finally:
    gui_app.close()


if __name__ == "__main__":
  main()
