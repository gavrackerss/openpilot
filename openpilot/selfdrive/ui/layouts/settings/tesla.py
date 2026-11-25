"""Tesla/XNOR settings surface with final 0.11.1 behavior on the 0.11.2 UI API."""
from __future__ import annotations

from openpilot.selfdrive.ui.layouts.settings.xnor_panel import XnorPanel
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.list_view import button_item, toggle_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog


class TeslaLayout(XnorPanel):
  def __init__(self):
    super().__init__()
    self._dialog: MultiOptionDialog | None = None
    self._dialog_handler = None
    self._toggles = {}

    rows = [
      button_item("Follow Distance", self._follow_distance_text,
                  description=self._follow_distance_desc, callback=self._show_follow_distance),
      button_item("Hands-on Threshold", self._hands_on_text,
                  description=self._hands_on_desc, callback=self._show_hands_on_level),
      button_item("Radar Offset", self._radar_offset_text,
                  description=self._radar_offset_desc, callback=self._show_radar_offset),
    ]

    for title, description, key, exclusive in (
      ("Match Speed to Speed Limit",
       "Automatically sets cruise speed to the detected speed limit with an offset.",
       "TinklaAdjustAccWithSpeedLimit", None),
      ("Vision Speed Limit (UK)",
       " ".join((
         "Uses the road camera to recognize UK red-circle 20/30/40/50/60/70 mph signs.",
         "Confirmed vision limits can lower speed-limit matching but never raise above the Tesla/map limit.",
         "Higher signs only release the vision override.",
       )),
       "VisionSpeedLimitDetection", None),
      ("Offset is Percentage",
       "If enabled, offset is a percentage of the speed limit. Otherwise it is an absolute mph/kph offset.",
       "TinklaSpeedLimitUseRelative", None),
    ):
      item = self._toggle(title, description, key, exclusive)
      rows.append(item)

    rows.append(button_item("Speed Limit Offset", self._speed_limit_offset_text,
                            description=self._speed_limit_offset_desc, callback=self._show_speed_limit_offset))

    for title, description, key, exclusive in (
      ("Auto Lane Change",
       "Automatically starts lane changes after a short indicator tap, if clear.",
       "TinklaEnableALC", None),
    ):
      rows.append(self._toggle(title, description, key, exclusive))

    rows.append(button_item("Auto Lane Change Delay", self._alc_delay_text,
                            description=self._alc_delay_desc, callback=self._show_alc_delay))

    for title, description, key, exclusive in (
      ("Radar Upside Down", "Use if your Tesla radar is mounted upside down.",
       "TinklaUseTeslaRadarUpsideDown", None),
      ("Ignore Radar SGU Error", "Ignore SGU hardware fail on Tesla radar.",
       "TinklaTeslaRadarIgnoreSGUError", None),
      ("Ignore Stock AEB", "Ignore Tesla stock Automatic Emergency Braking events.",
       "TinklaIgnoreStockAeb", None),
      ("Autopilot Disabled", "Unity mode: steering assist when Tesla Autopilot is disabled.",
       "TinklaAutopilotDisabled", "TinklaHybridNativeAP"),
      ("Hybrid Native Autopilot",
       " ".join((
         "Experimental: keep native Tesla Autosteer/TACC and IC visuals active while openpilot substitutes only the steering command.",
         "Mutually exclusive with Autopilot Disabled; takes effect after restart/next drive.",
       )),
       "TinklaHybridNativeAP", "TinklaAutopilotDisabled"),
      ("0x247 Autosteer State Test",
       " ".join((
         "Experimental IC test: while normal xnor Autopilot-Disabled control is engaged, rewrite the native AP 0x247 unknown state field",
         "from its standby value to the value observed during genuine Tesla Autosteer. Does not run in Hybrid mode.",
       )),
       "TinklaAutosteer247Test", None),
      ("Tesla VTB Steering",
       "Virtual Torque Blending: blends light driver steering input into Tesla angle control for smoother handoff. Default off.",
       "XnorTeslaVirtualTorqueBlending", None),
      ("Mute Engage/Disengage Sounds", "Disables the start and stop sounds.",
       "TinklaDisableStartStopSounds", None),
      ("Mute Prompt Sounds", "Disables prompt sounds.",
       "TinklaDisablePromptSounds", None),
    ):
      rows.append(self._toggle(title, description, key, exclusive))

    self.set_items(rows)

  def _toggle(self, title: str, description: str, key: str, exclusive_key: str | None):
    def changed(state: bool):
      if state and exclusive_key is not None:
        self._params.put_bool(exclusive_key, False)
        other = self._toggles.get(exclusive_key)
        if other is not None:
          other.action_item.set_state(False)
      self._params.put_bool(key, bool(state))

    item = toggle_item(title, description=description,
                       initial_state=self._params.get_bool(key), callback=changed)
    self._toggles[key] = item
    return item

  def refresh(self):
    # Keep the UI in sync with Params changes made by another surface/process.
    for key, item in self._toggles.items():
      item.action_item.set_state(self._params.get_bool(key))

  def _get_float(self, key: str, default: float) -> float:
    val = self._params.get(key, return_default=True)
    try:
      if isinstance(val, (bytes, bytearray)):
        val = val.decode("utf-8", errors="ignore")
      return float(val)
    except (TypeError, ValueError):
      return float(default)

  @staticmethod
  def _follow_distance_desc() -> str:
    return "Overrides longitudinal time-gap for Tesla (seconds)."

  @staticmethod
  def _hands_on_desc() -> str:
    return "Changes steering wheel touch sensitivity (1-4)."

  @staticmethod
  def _radar_offset_desc() -> str:
    return "Adjusts Tesla radar longitudinal offset (meters)."

  @staticmethod
  def _speed_limit_offset_desc() -> str:
    return "Offset applied when matching speed limits."

  @staticmethod
  def _alc_delay_desc() -> str:
    return "Delay before starting an automatic lane change (seconds)."

  def _follow_distance_text(self) -> str:
    return f"{max(0.5, min(3.0, self._get_float('TinklaFollowDistance', 1.45))):.2f} s"

  def _hands_on_text(self) -> str:
    return str(max(1, min(4, int(round(self._get_float('TinklaHandsOnLevel', 3.0))))))

  def _radar_offset_text(self) -> str:
    return f"{max(-1.0, min(1.0, self._get_float('TinklaRadarOffset', 0.0))):+.1f} m"

  def _speed_limit_offset_text(self) -> str:
    is_metric = self._params.get_bool("IsMetric")
    min_v = -30.0 if is_metric else -20.0
    max_v = 30.0 if is_metric else 20.0
    units = "kph" if is_metric else "mph"
    value = max(min_v, min(max_v, self._get_float("TinklaSpeedLimitOffset", 0.0)))
    return f"{value:.1f} {units}"

  def _alc_delay_text(self) -> str:
    return f"{max(0.0, min(10.0, self._get_float('TinklaAlcDelay', 2.0))):.1f} s"

  def _open_dialog(self, title: str, options: list[str], current: str, on_confirm):
    if self._dialog is not None and gui_app.widget_in_stack(self._dialog):
      return
    self._dialog_handler = on_confirm
    self._dialog = MultiOptionDialog(title, options, current=current, callback=self._handle_dialog)
    gui_app.push_widget(self._dialog)

  def _handle_dialog(self, result: int):
    if result == DialogResult.CONFIRM and self._dialog is not None and self._dialog_handler is not None:
      self._dialog_handler(self._dialog.selection)
    self._dialog = None
    self._dialog_handler = None

  def _show_follow_distance(self):
    options = [f"{v:.2f} s" for v in [x / 100.0 for x in range(50, 301, 5)]]
    self._open_dialog("Follow Distance", options, self._follow_distance_text(), self._set_follow_distance)

  def _set_follow_distance(self, selection: str):
    try:
      value = float(selection.split(maxsplit=1)[0])
    except (ValueError, IndexError):
      return
    self._params.put("TinklaFollowDistance", f"{max(0.5, min(3.0, value)):.2f}")

  def _show_hands_on_level(self):
    options = [str(v) for v in range(1, 5)]
    self._open_dialog("Hands-on Threshold", options, self._hands_on_text(), self._set_hands_on_level)

  def _set_hands_on_level(self, selection: str):
    try:
      value = int(selection)
    except ValueError:
      return
    self._params.put("TinklaHandsOnLevel", str(max(1, min(4, value))))

  def _show_radar_offset(self):
    options = [f"{(x / 10.0):+.1f} m" for x in range(-10, 11)]
    self._open_dialog("Radar Offset", options, self._radar_offset_text(), self._set_radar_offset)

  def _set_radar_offset(self, selection: str):
    try:
      value = float(selection.split(maxsplit=1)[0])
    except (ValueError, IndexError):
      return
    self._params.put("TinklaRadarOffset", f"{max(-1.0, min(1.0, value)):+.1f}")

  def _show_speed_limit_offset(self):
    is_metric = self._params.get_bool("IsMetric")
    min_v = -30 if is_metric else -20
    max_v = 30 if is_metric else 20
    units = "kph" if is_metric else "mph"
    options = [f"{float(v):.1f} {units}" for v in range(min_v, max_v + 1)]
    self._open_dialog("Speed Limit Offset", options, self._speed_limit_offset_text(), self._set_speed_limit_offset)

  def _set_speed_limit_offset(self, selection: str):
    try:
      value = float(selection.split(maxsplit=1)[0])
    except (ValueError, IndexError):
      return
    is_metric = self._params.get_bool("IsMetric")
    min_v = -30.0 if is_metric else -20.0
    max_v = 30.0 if is_metric else 20.0
    self._params.put("TinklaSpeedLimitOffset", f"{max(min_v, min(max_v, value)):.1f}")

  def _show_alc_delay(self):
    options = [f"{(x / 2.0):.1f} s" for x in range(21)]
    self._open_dialog("Auto Lane Change Delay", options, self._alc_delay_text(), self._set_alc_delay)

  def _set_alc_delay(self, selection: str):
    try:
      value = float(selection.split(maxsplit=1)[0])
    except (ValueError, IndexError):
      return
    self._params.put("TinklaAlcDelay", f"{max(0.0, min(10.0, value)):.1f}")
