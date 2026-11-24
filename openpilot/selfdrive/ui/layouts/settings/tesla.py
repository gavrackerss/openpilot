"""Tesla settings migration panel.

Displays retained settings separately from their runtime qualification.
No writes to vehicle-control parameters or messaging publishers.
"""
from openpilot.selfdrive.ui.layouts.settings.xnor_panel import XnorPanel
from openpilot.selfdrive.ui.onroad.manual_speed_limit_demo import SpeedSignDemoRow
from openpilot.system.ui.widgets.list_view import text_item


class TeslaLayout(XnorPanel):
  def __init__(self):
    super().__init__()
    self._rows = []
    rows = [
      text_item("Hybrid runtime", "Native pedal disengage",
                description="Brake inhibition ported. Independent lateral engagement and auto-resume are not active in this revision."),
      text_item("Manual speed presets", "Offroad preview",
                description="40 / 50 / 60 / NSL / cancel. This preview does not control cruise."),
    ]
    self._demo = SpeedSignDemoRow(self.is_offroad, self._offset)
    rows.append(self._demo)
    for title, key in (
        ("Hybrid Native Autopilot", "TinklaHybridNativeAP"),
        ("Autopilot Disabled", "TinklaAutopilotDisabled"),
        ("Auto Resume ACC", "TinklaAutoResumeACC"),
        ("Match Speed to Speed Limit", "TinklaAdjustAccWithSpeedLimit"),
        ("Offset is Percentage", "TinklaSpeedLimitUseRelative"),
        ("Auto Lane Change", "TinklaEnableALC"),
        ("Tesla VTB Steering", "XnorTeslaVirtualTorqueBlending"),
        ("Radar Upside Down", "TinklaUseTeslaRadarUpsideDown"),
        ("Ignore Radar SGU Error", "TinklaTeslaRadarIgnoreSGUError"),
        ("Ignore Stock AEB", "TinklaIgnoreStockAeb"),
        ("Mute Engage/Disengage Sounds", "TinklaDisableStartStopSounds"),
        ("Mute Prompt Sounds", "TinklaDisablePromptSounds"),
    ):
      item = text_item(title, "Unknown", description="Saved preference only. Runtime support may differ.")
      rows.append(item)
      self._rows.append((item, key, True))
    for title, key in (
        ("Speed Limit Offset", "TinklaSpeedLimitOffset"),
        ("Follow Distance", "TinklaFollowDistance"),
        ("Hands On Level", "TinklaHandsOnLevel"),
        ("Radar Offset", "TinklaRadarOffset"),
        ("Lane Change Delay", "TinklaAlcDelay"),
    ):
      item = text_item(title, "Unknown", description="Saved preference only.")
      rows.append(item)
      self._rows.append((item, key, False))
    self.set_items(rows)

  def _offset(self):
    return (float(self._params.get("TinklaSpeedLimitOffset") or 0.),
            self._params.get_bool("TinklaSpeedLimitUseRelative"),
            self._params.get_bool("IsMetric"))

  def refresh(self):
    for item, key, boolean in self._rows:
      try:
        value = self._params.get(key, return_default=True)
        label = ("On" if value else "Off") if boolean and isinstance(value, bool) else (
            "Not set" if value is None else str(value))
      except (ValueError, TypeError, OSError):
        label = "Unavailable"
      item.action_item.set_text(label)

  def hide_event(self):
    self._demo.reset_demo()
    super().hide_event()
