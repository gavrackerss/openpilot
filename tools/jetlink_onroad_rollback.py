#!/usr/bin/env python3
"""Restore only files backed up by the Jetlink onroad installer."""
import argparse
import json
from pathlib import Path
import shutil
import sys

if __name__ == '__main__':
  ap = argparse.ArgumentParser(description=__doc__)
  ap.add_argument('backup', help='Exact /data/jetlink_shadow_onroad/backups/<timestamp> directory')
  opts = ap.parse_args()
  backup = Path(opts.backup).resolve()
  data = json.loads((backup/'rollback.json').read_text())
  repo = Path(data['repository']).resolve()
  marker = Path('/data/jetlink_xnor/ONROAD_SHADOW_ENABLED')
  marker.unlink(missing_ok=True)
  # Restore manager before other modules: once restarted, no process reference remains.
  for name in data['saved']:
    shutil.copy2(backup/name, repo/name)
  for name in data['created']:
    (repo/name).unlink(missing_ok=True)
  print('Restored originals and removed new files from:', backup)
  print('Manager restart while parked/disengaged required to reload process configuration')
