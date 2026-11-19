#!/usr/bin/env python3
"""XNOR Jetlink passive on-road observer service; never affects vehicle authority.

This process is an optional, low-priority supervisor. It does not import or
replace modeld, send CAN, publish model messages, or write openpilot Params.
The normal modeld remains authoritative. Enable only for an intentional test.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
CONFIG = Path('/data/jetlink_xnor')
ARM = CONFIG / 'ONROAD_SHADOW_ENABLED'
STATUS_DIR = Path('/data/jetlink_shadow_onroad')
STATUS = STATUS_DIR / 'status.json'
RUNS = STATUS_DIR / 'runs'
MAX_ARM_AGE_S = 7200
RUN_SECONDS = 180  # Fits 5000-entry Android V0.5 timing ring; qualify 180s gate
MAX_RUNS = 5        # A bounded 15-minute on-road observation, not indefinite
RETRY_S = 15
_child: subprocess.Popen | None = None
_stopping = False


def write_status(**values):
  STATUS_DIR.mkdir(parents=True, exist_ok=True)
  data = {'time_utc': datetime.now(timezone.utc).isoformat(),
          'shadow_only': True, 'controls_affected': False, **values}
  tmp = STATUS_DIR / f'.status-{os.getpid()}.tmp'
  tmp.write_text(json.dumps(data, indent=2) + '\n')
  os.replace(tmp, STATUS)


def armed():
  try:
    age = time.time() - ARM.stat().st_mtime
    return 0 <= age <= MAX_ARM_AGE_S
  except OSError:
    return False


def stop_child():
  global _child
  p = _child
  if p is None or p.poll() is not None:
    return
  try:
    os.killpg(p.pid, signal.SIGTERM)
  except ProcessLookupError:
    return
  try:
    p.wait(timeout=10)
  except subprocess.TimeoutExpired:
    # Only the isolated Jetlink worker process group, never modeld/manager.
    try:
      os.killpg(p.pid, signal.SIGKILL)
    except ProcessLookupError:
      pass
    p.wait(timeout=3)


def shutdown(_signum, _frame):
  global _stopping
  _stopping = True
  stop_child()


def summary_for(path):
  summary = path / 'summary.json'
  if not summary.is_file():
    return None
  try:
    data = json.loads(summary.read_text())
    return {'frames': data.get('counts', {}).get('frames'),
            'fps': data.get('counts', {}).get('achieved_fps'),
            'reply_cycle_p95_ms': data.get('cadence_gate', {}).get('reply_cycle_p95_ms'),
            'capture_to_reply_p95_ms': data.get('timing_ms', {}).get('capture_to_reply_ms', {}).get('p95'),
            'errors': data.get('errors'),
            'no_control': data.get('meta', {}).get('no_control'),
            'onroad_shadow': data.get('meta', {}).get('onroad_shadow'),
            'comparison_matched_frames': (data.get('prediction_comparison') or {}).get('matched_frames'),
            'full_numerical_parity_verified': False}
  except (ValueError, OSError, AttributeError):
    return None


def runner_args(path, host):
  return [sys.executable, '-m', 'openpilot.jetlink_shadow_bench',
          '--mode', 'camera', '--onroad-shadow', '--pacing', 'timer',
          '--seconds', str(RUN_SECONDS), '--rate', '20', '--pipeline-depth', '2',
          '--transport', 'udp', '--udp-payload-bytes', '65000',
          '--udp-send-mode', 'sendmsg', '--reply-guard-ms', '0',
          '--dump-outputs-every', '20', '--compare-local', '--fetch-stage-report',
          '--host', host, '--output', str(path)]


def main():
  global _child
  # No invocation or enablement without explicit, short-lived arm marker.
  if not armed():
    write_status(state='not_armed', reason='Enable using tools/enable_jetlink_onroad.sh')
    return 0
  signal.signal(signal.SIGTERM, shutdown)
  signal.signal(signal.SIGINT, shutdown)
  os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
  os.environ.setdefault('OMP_NUM_THREADS', '1')
  try:
    os.nice(10)
  except OSError:
    pass
  RUNS.mkdir(parents=True, exist_ok=True)
  host_file = CONFIG / 'HOST'
  host = host_file.read_text().strip() if host_file.exists() else '10.169.232.180'
  failures = 0
  completed = 0
  try:
    for idx in range(1, MAX_RUNS + 1):
      if _stopping or not armed():
        break
      stamp = datetime.now(timezone.utc).strftime('%Y%m%d-%H%M%S-%f')
      path = RUNS / f'drive-{stamp}-{idx:02d}'
      # The worker writes its result files in its own newly-created folder.
      RUNS.mkdir(parents=True, exist_ok=True)
      log_path = RUNS / f'{path.name}.log'
      args = runner_args(path, host)
      write_status(state='running', run=idx, total_runs=MAX_RUNS,
                   output=str(path), log=str(log_path), host=host)
      env = os.environ.copy()
      env['PYTHONPATH'] = str(ROOT) + (':' + env['PYTHONPATH'] if env.get('PYTHONPATH') else '')
      with log_path.open('w') as output_log:
        _child = subprocess.Popen(args, cwd=ROOT, env=env, stdout=output_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        while _child.poll() is None:
          if _stopping or not armed():
            stop_child()
            break
          time.sleep(0.5)
        code = _child.wait()
        _child = None
      stats = summary_for(path)
      write_status(state='stopped' if _stopping else ('complete' if code == 0 else 'failed'),
                   run=idx, exit_code=code, output=str(path), log=str(log_path), stats=stats)
      if _stopping or not armed():
        break
      if code:
        failures += 1
        if failures >= 3:
          write_status(state='aborted', reason='Three failed sessions; automatic retries stopped',
                       output=str(path), log=str(log_path), stats=stats)
          break
        for _ in range(RETRY_S * 2):
          if _stopping or not armed():
            break
          time.sleep(0.5)
      else:
        completed += 1
        failures = 0
        time.sleep(2)
    if not _stopping and completed >= MAX_RUNS:
      write_status(state='finished', completed_runs=completed,
                   reason='Bounded on-road test completed. Re-arm for another drive.')
  except BaseException as exc:
    write_status(state='error', error=f'{type(exc).__name__}: {exc}')
    raise
  finally:
    stop_child()
    # Requiring a fresh arm for the next drive prevents unattended use.
    ARM.unlink(missing_ok=True)
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
