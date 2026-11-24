from openpilot.cereal import log
from openpilot.selfdrive.ui.soundd import apply_xnor_sound_preferences

AudibleAlert = log.SelfdriveState.AudibleAlert


def test_xnor_start_stop_mute_only():
  assert apply_xnor_sound_preferences(AudibleAlert.engage, True, False) == AudibleAlert.none
  assert apply_xnor_sound_preferences(AudibleAlert.disengage, True, False) == AudibleAlert.none
  assert apply_xnor_sound_preferences(AudibleAlert.prompt, True, False) == AudibleAlert.prompt
  assert apply_xnor_sound_preferences(AudibleAlert.warningImmediate, True, True) == AudibleAlert.warningImmediate
  assert apply_xnor_sound_preferences(AudibleAlert.refuse, True, True) == AudibleAlert.refuse


def test_xnor_prompt_mute_only():
  for alert in (AudibleAlert.prompt, AudibleAlert.promptRepeat, AudibleAlert.promptDistracted):
    assert apply_xnor_sound_preferences(alert, False, True) == AudibleAlert.none
  assert apply_xnor_sound_preferences(AudibleAlert.warningSoft, False, True) == AudibleAlert.warningSoft
