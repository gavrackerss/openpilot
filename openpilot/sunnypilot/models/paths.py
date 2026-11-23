"""XNOR-native v0.11.2 local model cache location.

The legacy SunnyPilot code imported openpilot.system.hardware.hw.Paths and
called Paths.model_root(), neither of which exists in XNOR 0.11.2.
No directories are created, and nothing is downloaded or enabled on import.
"""
from pathlib import Path

from openpilot.common.hardware import AGNOS
from openpilot.common.hardware.hw import Paths


def model_root() -> str:
  # The native hardware module supplies comma_home(), not model_root().
  # Mirror the existing SunnyPilot/Jetlink convention, not modeld's
  # compiled stock model directory (openpilot/selfdrive/modeld/models).
  if AGNOS:
    return "/data/media/0/models"
  return str(Path(Paths.comma_home()) / "media" / "0" / "models")
