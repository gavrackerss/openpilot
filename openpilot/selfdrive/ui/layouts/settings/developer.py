from openpilot.common.params import Params
from openpilot.selfdrive.ui.widgets.ssh_key import ssh_key_item
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import toggle_item, text_item, button_item
from openpilot.system.ui.widgets.option_dialog import MultiOptionDialog
from openpilot.sunnypilot.models.requested_model import CHOICES, REQUEST_KEY, requested_label, requested_ref, save_request
from openpilot.selfdrive.ui.onroad.manual_speed_limit_demo import SpeedSignDemoRow
from openpilot.sunnypilot.models.catalog_guard import audit_bundle
from openpilot.system.ui.widgets.scroller_tici import Scroller
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult

# Description constants
DESCRIPTIONS = {
  'enable_adb': tr_noop(
    "ADB (Android Debug Bridge) allows connecting to your device over USB or over the network. " +
    "See https://docs.comma.ai/how-to/connect-to-comma for more info."
  ),
  'ssh_key': tr_noop(
    "Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username " +
    "other than your own. A comma employee will NEVER ask you to add their GitHub username."
  ),
  'alpha_longitudinal': tr_noop(
    "<b>WARNING: openpilot longitudinal control is in alpha for this car and may disable Automatic Emergency Braking (AEB).</b><br><br>" +
    "On this car, openpilot defaults to the car's built-in ACC instead of openpilot's longitudinal control. " +
    "Enable this to switch to openpilot longitudinal control. Enabling Experimental mode is recommended when enabling openpilot longitudinal control alpha. " +
    "Changing this setting will restart openpilot if the car is powered on."
  ),
}


class DeveloperLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    self._is_release = self._params.get_bool("IsReleaseBranch")

    # Build items and keep references for callbacks/state updates
    self._adb_toggle = toggle_item(
      lambda: tr("Enable ADB"),
      description=lambda: tr(DESCRIPTIONS["enable_adb"]),
      initial_state=self._params.get_bool("AdbEnabled"),
      callback=self._on_enable_adb,
      enabled=ui_state.is_offroad,
    )

    # SSH enable toggle + SSH key management
    self._ssh_toggle = toggle_item(
      lambda: tr("Enable SSH"),
      description="",
      initial_state=self._params.get_bool("SshEnabled"),
      callback=self._on_enable_ssh,
    )
    self._ssh_keys = ssh_key_item(lambda: tr("SSH Keys"), description=lambda: tr(DESCRIPTIONS["ssh_key"]))

    self._joystick_toggle = toggle_item(
      lambda: tr("Joystick Debug Mode"),
      description="",
      initial_state=self._params.get_bool("JoystickDebugMode"),
      callback=self._on_joystick_debug_mode,
      enabled=ui_state.is_offroad,
    )

    self._long_maneuver_toggle = toggle_item(
      lambda: tr("Longitudinal Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LongitudinalManeuverMode"),
      callback=self._on_long_maneuver_mode,
    )

    self._lat_maneuver_toggle = toggle_item(
      lambda: tr("Lateral Maneuver Mode"),
      description="",
      initial_state=self._params.get_bool("LateralManeuverMode"),
      callback=self._on_lat_maneuver_mode,
    )

    self._alpha_long_toggle = toggle_item(
      lambda: tr("openpilot Longitudinal Control (Alpha)"),
      description=lambda: tr(DESCRIPTIONS["alpha_longitudinal"]),
      initial_state=self._params.get_bool("AlphaLongitudinalEnabled"),
      callback=self._on_alpha_long_enabled,
      enabled=lambda: not ui_state.engaged,
    )

    self._ui_debug_toggle = toggle_item(
      lambda: tr("UI Debug Mode"),
      description="",
      initial_state=self._params.get_bool("ShowDebugInfo"),
      callback=self._on_enable_ui_debug,
    )
    self._on_enable_ui_debug(self._params.get_bool("ShowDebugInfo"))

    # Migration diagnostics: show requested selection separately from native runtime.
    # Runtime diagnostics are read-only. The preference selector is offroad-only.
    self._saved_model_item = text_item("Saved SunnyPilot model", "Not inspected")
    self._native_model_item = text_item("Model runtime", "Native only",
                                        description="Custom model execution is not qualified on this port.")
    self._requested_model_item = button_item("Requested model", "SELECT",
        description="Save a native model preference for modeld startup. Unqualified or unavailable models use stock.",
        callback=self._on_requested_model, enabled=self._migration_offroad)
    self._speed_sign_preview = SpeedSignDemoRow(self._migration_offroad, self._preview_offset_settings)
    self._migration_items = (self._saved_model_item, self._requested_model_item, self._native_model_item, self._speed_sign_preview)
    for item in self._migration_items:
      item.set_visible(self._migration_offroad)
    self._scroller = Scroller([
      self._adb_toggle,
      self._ssh_toggle,
      self._ssh_keys,
      self._joystick_toggle,
      self._long_maneuver_toggle,
      self._lat_maneuver_toggle,
      self._alpha_long_toggle,
      self._ui_debug_toggle,
      *self._migration_items,
    ], line_separator=True, spacing=0)

    # Toggles should be not available to change in onroad state
    ui_state.add_offroad_transition_callback(self._update_toggles)

  def _preview_offset_settings(self):
    return (float(self._params.get('TinklaSpeedLimitOffset') or 0.),
            self._params.get_bool('TinklaSpeedLimitUseRelative'),
            self._params.get_bool('IsMetric'))

  def _migration_offroad(self):
    return bool(ui_state.is_offroad() and self._params.get_bool("IsOffroad"))

  def _refresh_migration_items(self):
    if not self._migration_offroad():
      self._speed_sign_preview.reset_demo()
      self._saved_model_item.action_item.set_text("Not inspected")
      self._requested_model_item.action_item.set_value("")
      return
    try:
      saved = self._params.get("ModelManager_ActiveBundle")
      self._requested_model_item.action_item.set_value(requested_label(self._params.get(REQUEST_KEY), saved))
      report = audit_bundle(saved)
      name = report.requested_name or ("No saved selection" if saved is None else "Invalid selection")
      # TextAction displays plain text; strip newlines/control characters and bound length.
      name = " ".join(name.split())[:60]
      self._saved_model_item.action_item.set_text(name)
      self._saved_model_item.set_description(
        "Saved selection retained. " + ("Catalogue metadata checks passed." if not report.metadata_issues else
        "Catalogue metadata requires review.") + " This does not confirm the selected model is running.")
    except (ValueError, TypeError, OSError):
      self._saved_model_item.action_item.set_text("Unavailable")

  def _on_requested_model(self):
    if not self._migration_offroad():
      return
    legacy = self._params.get("ModelManager_ActiveBundle")
    current_ref = requested_ref(self._params.get(REQUEST_KEY), legacy)
    labels = {f"{choice.label} - {choice.status}": choice.ref for choice in CHOICES}
    current = next((label for label, ref in labels.items() if ref == current_ref), "")

    def confirmed(result):
      if result == DialogResult.CONFIRM and self._migration_offroad():
        selected = labels.get(dialog.selection)
        if selected is not None:
          save_request(self._params, selected, confirmed_offroad=self._migration_offroad())
        self._refresh_migration_items()

    dialog = MultiOptionDialog("Requested native model", list(labels), current=current, callback=confirmed)
    gui_app.push_widget(dialog)

  def _render(self, rect):
    self._scroller.render(rect)

  def show_event(self):
    super().show_event()
    self._scroller.show_event()
    self._update_toggles()

  def _update_toggles(self):
    ui_state.update_params()
    self._refresh_migration_items()

    # Hide non-release toggles on release builds
    # TODO: we can do an onroad cycle, but alpha long toggle requires a deinit function to re-enable radar and not fault
    for item in (self._joystick_toggle, self._long_maneuver_toggle, self._lat_maneuver_toggle, self._alpha_long_toggle):
      item.set_visible(not self._is_release)

    # CP gating
    if ui_state.CP is not None:
      alpha_avail = ui_state.CP.alphaLongitudinalAvailable
      if not alpha_avail or self._is_release:
        self._alpha_long_toggle.set_visible(False)
        self._params.remove("AlphaLongitudinalEnabled")
      else:
        self._alpha_long_toggle.set_visible(True)

      long_man_enabled = ui_state.has_longitudinal_control and ui_state.is_offroad()
      self._long_maneuver_toggle.action_item.set_enabled(long_man_enabled)
      self._lat_maneuver_toggle.action_item.set_enabled(ui_state.is_offroad())
    else:
      self._long_maneuver_toggle.action_item.set_enabled(False)
      self._lat_maneuver_toggle.action_item.set_enabled(False)
      self._alpha_long_toggle.set_visible(False)

    # TODO: make a param control list item so we don't need to manage internal state as much here
    # refresh toggles from params to mirror external changes
    for key, item in (
      ("AdbEnabled", self._adb_toggle),
      ("SshEnabled", self._ssh_toggle),
      ("JoystickDebugMode", self._joystick_toggle),
      ("LongitudinalManeuverMode", self._long_maneuver_toggle),
      ("LateralManeuverMode", self._lat_maneuver_toggle),
      ("AlphaLongitudinalEnabled", self._alpha_long_toggle),
      ("ShowDebugInfo", self._ui_debug_toggle),
    ):
      item.action_item.set_state(self._params.get_bool(key))

  def _on_enable_ui_debug(self, state: bool):
    self._params.put_bool("ShowDebugInfo", state, block=True)
    gui_app.set_show_touches(state)
    gui_app.set_show_fps(state)

  def _on_enable_adb(self, state: bool):
    self._params.put_bool("AdbEnabled", state, block=True)

  def _on_enable_ssh(self, state: bool):
    self._params.put_bool("SshEnabled", state, block=True)

  def _on_joystick_debug_mode(self, state: bool):
    self._params.put_bool("JoystickDebugMode", state, block=True)
    self._params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.action_item.set_state(False)
    self._params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.action_item.set_state(False)

  def _on_long_maneuver_mode(self, state: bool):
    self._params.put_bool("LongitudinalManeuverMode", state, block=True)
    self._params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.action_item.set_state(False)
    self._params.put_bool("LateralManeuverMode", False, block=True)
    self._lat_maneuver_toggle.action_item.set_state(False)

  def _on_lat_maneuver_mode(self, state: bool):
    self._params.put_bool("LateralManeuverMode", state, block=True)
    self._params.put_bool("ExperimentalMode", False, block=True)
    self._params.put_bool("JoystickDebugMode", False, block=True)
    self._joystick_toggle.action_item.set_state(False)
    self._params.put_bool("LongitudinalManeuverMode", False, block=True)
    self._long_maneuver_toggle.action_item.set_state(False)

  def _on_alpha_long_enabled(self, state: bool):
    if state:
      def confirm_callback(result: DialogResult):
        if result == DialogResult.CONFIRM:
          self._params.put_bool("AlphaLongitudinalEnabled", True, block=True)
          self._params.put_bool("OnroadCycleRequested", True, block=True)
          self._update_toggles()
        else:
          self._alpha_long_toggle.action_item.set_state(False)

      # show confirmation dialog
      content = (f"<h1>{self._alpha_long_toggle.title}</h1><br>" +
                 f"<p>{self._alpha_long_toggle.description}</p>")

      dlg = ConfirmDialog(content, tr("Enable"), rich=True, callback=confirm_callback)
      gui_app.push_widget(dlg)

    else:
      self._params.put_bool("AlphaLongitudinalEnabled", False, block=True)
      self._params.put_bool("OnroadCycleRequested", True, block=True)
      self._update_toggles()
