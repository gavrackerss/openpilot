"""Interactive, offroad-only V279 speed sign demonstration.

The demonstration changes in-memory selection only. No cruise requests are
written, even if a preset is confirmed. Never attach to the live HUD.
"""
import time
import pyray as rl

from openpilot.selfdrive.ui.onroad.manual_speed_limit_preview import ManualSpeedLimitPreview, OPTIONS, TEXT
from openpilot.selfdrive.ui.onroad.manual_speed_limit_demo_state import SpeedSignDemo, preview_target_mph


class SpeedSignDemoRow(ManualSpeedLimitPreview):
  def __init__(self, allowed, offset_settings):
    super().__init__()
    self._allowed = allowed
    self._offset_settings = offset_settings
    self._demo = SpeedSignDemo()
    self.set_enabled(allowed)
    self.set_rect(rl.Rectangle(0, 0, 0, 170))

  def set_parent_rect(self, parent_rect):
    super().set_parent_rect(parent_rect)
    self.set_rect(rl.Rectangle(self.rect.x, self.rect.y, parent_rect.width, 170))

  def reset_demo(self):
    self._demo.reset()

  def hide_event(self):
    self.reset_demo()
    super().hide_event()

  def _update_state(self):
    self._demo.tick(time.monotonic(), self._allowed())

  def _button_rects(self):
    width = min(900., self.rect.width - 20.)
    if width < 350.:
      return []
    x = self.rect.x + (self.rect.width - width) / 2
    inner = width - 10.
    btn_w = (inner - 48.) / 5
    return [rl.Rectangle(x + 5. + i * (btn_w + 12.), self.rect.y + 57., btn_w, 93.)
            for i in range(5)]

  def _render(self, rect):
    if not self._allowed():
      return
    label = "DEMO ONLY - NO CRUISE CONTROL"
    if self._demo.pending:
      label += f" | TAP {self._demo.pending} AGAIN"
    elif self._demo.selected:
      try:
        offset, relative, metric = self._offset_settings()
        target = preview_target_mph(self._demo.selected, offset, relative, metric)
      except (ValueError, TypeError, OSError):
        target = None
      label += f" | {self._demo.selected} mph"
      label += f" + offset = {target:.1f} mph" if target is not None else " | offset unavailable"
    else:
      label += " | AUTO"
    rl.draw_text_ex(self._smaller_font, label, rl.Vector2(rect.x + 5, rect.y + 12), 24, 0, TEXT)
    for (value, _), box in zip(OPTIONS, self._button_rects(), strict=False):
      if value == 0:
        self._draw_tesla_cancel(box, self._demo.selected == 0)
      else:
        self._draw_speed_sign(value, box, self._demo.selected == value, self._demo.pending == value)

  def _handle_mouse_release(self, mouse_pos):
    if not self._allowed():
      self._demo.reset()
      return
    for (value, _), box in zip(OPTIONS, self._button_rects(), strict=False):
      if rl.check_collision_point_rec(mouse_pos, box):
        self._demo.tap(value, time.monotonic(), True)
        break
