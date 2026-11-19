#!/usr/bin/env python3
"""Conservatively install passive Jetlink shadow-only on-road observation.

Uses standard-library Python; neither unzip nor scons is required on comma.
Never installs a modeld replacement or touches any vehicle-control module.
"""
from __future__ import annotations
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import py_compile
import shutil
import sys

PRED_BEGIN = '# XNOR_JETLINK_ONROAD_SHADOW_PREDICATE_BEGIN'
PRED_END = '# XNOR_JETLINK_ONROAD_SHADOW_PREDICATE_END'
PROC_BEGIN = '  # XNOR_JETLINK_ONROAD_SHADOW_PROCESS_BEGIN'
PROC_END = '  # XNOR_JETLINK_ONROAD_SHADOW_PROCESS_END'
ANCHOR_PRED = 'def xnor_jetlink_probe('
ANCHOR_PROC = '  PythonProcess("xnorjetlinkd", "openpilot.jetlink_xnor_probe", xnor_jetlink_probe),'
PRED = f'''{PRED_BEGIN}
def xnor_jetlink_onroad_shadow(started: bool, params: Params, CP: car.CarParams) -> bool:
  """Opt-in, bounded Tesla on-road Jetlink performance observer; never modeld."""
  try:
    arm = '/data/jetlink_xnor/ONROAD_SHADOW_ENABLED'
    age = time.time() - os.path.getmtime(arm)
    return (bool(started) and getattr(CP, 'brand', '') == 'tesla'
            and 0 <= age <= 7200)
  except OSError:
    return False

{PRED_END}

'''
PROC = f'''{PROC_BEGIN}
  PythonProcess("xnorjetlinkshadowd", "openpilot.jetlink_onroad_shadow", xnor_jetlink_onroad_shadow),
{PROC_END}'''


def patch_manager(text: str) -> str:
  pair = (PRED_BEGIN in text, PROC_BEGIN in text)
  if pair == (True, True):
    if PRED_END not in text or PROC_END not in text:
      raise RuntimeError('Existing XNOR on-road manager patch is incomplete; refusing overwrite')
    return text
  if any(pair):
    raise RuntimeError('Only part of XNOR on-road manager registration exists; refusing duplicate')
  if text.count(ANCHOR_PRED) != 1 or text.count(ANCHOR_PROC) != 1:
    raise RuntimeError('Expected xnor-dev manager anchors absent/duplicated; manager left unchanged')
  if '\nimport time\n' not in text:
    if 'import os\n' not in text:
      raise RuntimeError('Expected import os anchor not found; refusing manager edit')
    text = text.replace('import os\n', 'import os\nimport time\n', 1)
  text = text.replace(ANCHOR_PRED, PRED + ANCHOR_PRED, 1)
  text = text.replace(ANCHOR_PROC, ANCHOR_PROC + '\n' + PROC, 1)
  compile(text, 'system/manager/process_config.py', 'exec')
  return text


def install(repo: Path, bundle: Path, dry_run: bool = False):
  mgr = repo / 'system/manager/process_config.py'
  if not mgr.is_file():
    raise FileNotFoundError('No manager config: ' + str(mgr))
  sources = {
    'openpilot/jetlink_shadow_bench.py': bundle / 'openpilot/jetlink_shadow_bench.py',
    'openpilot/jetlink_onroad_shadow.py': bundle / 'openpilot/jetlink_onroad_shadow.py',
    'start_jetlink_shadow.sh': bundle / 'start_jetlink_shadow.sh',
    'tools/enable_jetlink_onroad.sh': bundle / 'tools/enable_jetlink_onroad.sh',
    'tools/disable_jetlink_onroad.sh': bundle / 'tools/disable_jetlink_onroad.sh',
  }
  for name, source in sources.items():
    if not source.is_file():
      raise FileNotFoundError('Missing bundle file: ' + str(source))
  if "VERSION = '5.6-onroad-shadow'" not in sources['openpilot/jetlink_shadow_bench.py'].read_text():
    raise RuntimeError('Expected V5.6 on-road extension runner absent; refusing install')
  for path in ('openpilot/jetlink_shadow_bench.py', 'openpilot/jetlink_onroad_shadow.py'):
    py_compile.compile(str(sources[path]), doraise=True)
  old_mgr = mgr.read_text()
  new_mgr = patch_manager(old_mgr)
  modified = [key for key, src in sources.items()
              if not (repo/key).exists() or (repo/key).read_bytes() != src.read_bytes()]
  if old_mgr != new_mgr:
    modified.append('system/manager/process_config.py')
  print('XNOR passive on-road Jetlink installation (' + ('dry-run' if dry_run else 'apply') + ')')
  for name in modified:
    print('  UPDATE ' + name)
  if not modified:
    print('Already installed; no changes')
    return None
  if dry_run:
    return None

  stamp = datetime.now().strftime('%Y%m%d-%H%M%S-%f')
  backup = Path('/data/jetlink_shadow_onroad/backups') / stamp
  backup.mkdir(parents=True, exist_ok=False)
  created = []
  saved = []
  try:
    for name in modified:
      dst = repo / name
      dst.parent.mkdir(parents=True, exist_ok=True)
      if dst.is_file():
        original = backup / name
        original.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(dst, original)
        saved.append(name)
      else:
        created.append(name)
    (backup / 'rollback.json').write_text(json.dumps({'repository': str(repo), 'saved': saved, 'created': created}, indent=2) + '\n')
    # The manager is written LAST, after all its referenced modules exist.
    for name in modified:
      if name == 'system/manager/process_config.py':
        continue
      shutil.copy2(sources[name], repo/name)
      if name.endswith('.sh'):
        (repo/name).chmod(0o755)
    if old_mgr != new_mgr:
      tmp = mgr.with_name('.process_config.jetlink-shadow.tmp')
      tmp.write_text(new_mgr)
      compile(new_mgr, str(mgr), 'exec')
      os.replace(tmp, mgr)
    py_compile.compile(str(mgr), doraise=True)
    print('Installed. Backup: ' + str(backup))
    print('No automatic activation. Arm when ready; restart manager only while safely parked/disengaged.')
    return backup
  except BaseException:
    for name in saved:
      shutil.copy2(backup/name, repo/name)
    for name in created:
      (repo/name).unlink(missing_ok=True)
    raise


if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument('repo', nargs='?', default='/data/openpilot')
  parser.add_argument('--dry-run', action='store_true')
  opts = parser.parse_args()
  install(Path(opts.repo), Path(__file__).resolve().parent.parent, opts.dry_run)
