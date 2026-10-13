#!/usr/bin/env python3
"""Read-only XNOR model-selection diagnostic. Run while parked; no param writes."""
import os
import subprocess
from pathlib import Path

from openpilot.common.params import Params
from openpilot.sunnypilot.models.helpers import get_active_bundle, _bundle_integrity_issues
from openpilot.system.hardware.hw import Paths


def main():
  p = Params()
  raw = p.get('ModelManager_ActiveBundle')
  bundle = get_active_bundle(p)
  print('IsOnroad:', p.get('IsOnroad'))
  print('IsOffroad:', p.get('IsOffroad'))
  print('ModelRunnerTypeCache:', p.get('ModelRunnerTypeCache'), '(1=tinygrad; 2=stock)')
  print('Saved bundle present:', bool(raw))

  if isinstance(raw, dict):
    print('Saved ref:', raw.get('ref'))
    print('Saved name:', raw.get('displayName'))
    print('Saved selector version:', raw.get('minimumSelectorVersion'))
  else:
    print('Saved raw value type:', type(raw).__name__)

  if bundle:
    print('Parsed bundle:', bundle.displayName)
    print('Expected runner:', bundle.runner.raw)
    print('Model storage:', Paths.model_root())
    for model in bundle.models:
      filename = model.artifact.fileName
      if filename:
        path = Path(Paths.model_root()) / filename
        print('PKL:', path, 'file_exists=', path.is_file(),
              'manifest_exists=', Path(str(path) + '.chunkmanifest').is_file())

  issues = _bundle_integrity_issues(bundle) if raw else []
  print('Local integrity:', 'VALID' if not issues and bundle else 'NOT VERIFIED' if not raw else 'INVALID')
  for reason in issues:
    print('  ISSUE:', reason)

  try:
    res = subprocess.run(['pgrep', '-af', 'selfdrive.modeld.modeld'],
                         capture_output=True, text=True, timeout=5, check=False)
    lines = [ln for ln in res.stdout.splitlines() if 'pgrep' not in ln and 'diagnose_model_selection' not in ln]
    print('Running driving modeld:')
    print('\n'.join('  ' + ln for ln in lines) or '  None (normal while offroad)')
  except (OSError, subprocess.TimeoutExpired) as exc:
    print('Could not query modeld processes:', exc)

  if bundle and not issues:
    print('RESULT: saved selection is locally verified; actual running model still requires modeld PKL load log.')
  elif raw:
    print('RESULT: selection must remain saved; stock runner is the safe fallback until listed errors are fixed.')
  else:
    print('RESULT: selection parameter is absent; reselect your downloaded model once while parked.')


if __name__ == '__main__':
  main()
