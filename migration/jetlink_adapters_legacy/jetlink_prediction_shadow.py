#!/usr/bin/env python3
"""Passive, file-based LS2 prediction interface for XNOR Jetlink shadow tests.

Reads sampled *remote* Light Speed 2 inference outputs and emits a JSONL
prediction stream plus an atomic, freshness-tagged latest prediction file.
It deliberately cannot publish modelV2, drivingModelData or any other cereal
message and cannot affect controls. The output remains unvalidated for driving.

Use only with the separately armed on-road observer. This is not a remote modeld
runner; readers must regard even well-formed observations as NON-AUTHORITATIVE.
"""
from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time
from typing import Any

import numpy as np

from openpilot.jetlink_prediction_compare import remote_features

SCHEMA = 1
PREDICTION_SOURCE = 'XNOR_Jetlink_LS2_TensorG6'
EXPECTED_SHA256 = '909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8'
DEFAULT_LATEST = Path('/data/jetlink_shadow_onroad/latest_prediction.json')
MAX_FRESH_MS = 120.0             # Observation classification only, NOT an authority gate
LIVE_UPDATE_INTERVAL_S = 0.5  # Bounded writes; never 20Hz fsync


def _list(a: np.ndarray, *, decimals=4) -> list[float]:
  return np.round(np.asarray(a, dtype=np.float64), decimals).tolist()


def _atomic_json(path: Path, item: dict[str, Any]) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_name('.' + path.name + f'.{os.getpid()}.tmp')
  try:
    tmp.write_text(json.dumps(item, separators=(',', ':'), allow_nan=False) + '\n')
    os.replace(tmp, path)
  finally:
    try:
      tmp.unlink(missing_ok=True)
    except OSError:
      pass


def decode_shadow_prediction(output: np.ndarray, spec: dict, *, frame_id: int,
                             capture_ns: int, received_ns: int, npu_ms: float | None = None) -> dict:
  """Validate and decode LS2's public 2068 float output; no action authority.

  The plan's 33 time points follow openpilot's T_IDXS. All arrays are model
  predictions, not measured road geometry or a safety validation.
  """
  if spec.get('sha256') != EXPECTED_SHA256:
    raise ValueError('Remote model SHA-256 mismatch')
  if not isinstance(frame_id, int) or frame_id < 0:
    raise ValueError('Frame ID must be nonnegative')
  if capture_ns <= 0 or received_ns < capture_ns:
    raise ValueError('Invalid comma monotonic capture/reply timestamps')
  heads = remote_features(output, spec)
  required = {'position_x': (33,), 'position_y': (33,), 'position_z': (33,),
              'velocity_x': (33,), 'lane_y': (4, 33), 'lane_z': (4, 33),
              'edge_y': (2, 33), 'edge_z': (2, 33), 'lane_prob': (4,),
              'lead_prob': (3,)}
  for k, shape in required.items():
    v = np.asarray(heads[k])
    if v.shape != shape or not np.isfinite(v).all():
      raise ValueError(f'Invalid {k} output shape or values')
  for field in ('lane_prob', 'lead_prob'):
    vals = heads[field]
    if np.any((vals < 0) | (vals > 1)):
      raise ValueError(f'Invalid {field} probabilities')

  age_ms = round((received_ns - capture_ns) / 1e6, 3)
  idxs = (0, 8, 12, 16, 20, 24, 28, 32)
  t = np.asarray([10.0 * (i / 32.0) ** 2 for i in range(33)])
  return {
    'schema': SCHEMA, 'source': PREDICTION_SOURCE, 'model_sha256': EXPECTED_SHA256,
    'shadow_only': True, 'authoritative': False, 'full_numerical_parity_verified': False,
    'geometry_is_model_prediction_not_ground_truth': True,
    'frame_id': frame_id, 'camera_timestamp_eof_ns': int(capture_ns),
    'reply_monotonic_ns': int(received_ns), 'capture_to_reply_ms': age_ms,
    'fresh_at_reply': age_ms <= MAX_FRESH_MS,
    'phone_npu_ms': round(float(npu_ms), 3) if npu_ms is not None else None,
    'trajectory': {'t_s': _list(t), 'x_m': _list(heads['position_x']),
                   'y_m': _list(heads['position_y']), 'z_m': _list(heads['position_z']),
                   'velocity_x_mps': _list(heads['velocity_x'])},
    'lane_lines': [{'y_m': _list(heads['lane_y'][i]), 'z_m': _list(heads['lane_z'][i]),
                    'prob': round(float(heads['lane_prob'][i]), 5)} for i in range(4)],
    'road_edges': [{'y_m': _list(heads['edge_y'][i]), 'z_m': _list(heads['edge_z'][i])} for i in range(2)],
    'lead_probabilities': _list(heads['lead_prob'], decimals=5),
    'diagnostics': {
      't_1_s_path_y_m': round(float(heads['position_y'][10]), 4),
      't_2_5_s_path_y_m': round(float(heads['position_y'][16]), 4),
      't_3_9_s_path_y_m': round(float(heads['position_y'][20]), 4),
      'predicted_speed_mps': round(float(heads['velocity_x'][0]), 4),
    },
  }


def read_shadow_prediction(path: Path = DEFAULT_LATEST, *, max_age_ms: float = MAX_FRESH_MS,
                           now_monotonic_ns: int | None = None) -> tuple[dict | None, str]:
  """READ ONLY. Fail closed on stale/inactive data; never treat as control-ready.

  The clock epoch is comma's monotonic_ns: only call on the *same device*.
  Not a command, controls or modeld interface. A matching record is still
  categorically NOT authorized for vehicle control.
  """
  try:
    value = json.loads(Path(path).read_text())
    if not value.get('active') or value.get('schema') != SCHEMA:
      return None, 'inactive_or_schema_mismatch'
    if not value.get('shadow_only') or value.get('authoritative') is not False:
      return None, 'not_a_shadow_record'
    if value.get('model_sha256') != EXPECTED_SHA256:
      return None, 'model_hash_mismatch'
    received = int(value['reply_monotonic_ns'])
    now = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
    age_ms = (now - received) / 1e6
    if age_ms < 0 or age_ms > max_age_ms or value.get('fresh_at_reply') is not True:
      return None, 'stale'
    return value, 'shadow_observation_only'
  except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError):
    return None, 'missing_or_invalid'


class ShadowPredictionRecorder:
  """Bounded-rate observer. No messaging, Params, CAN, Panda or modeld calls."""
  def __init__(self, output: Path, spec: dict, *, latest_path: Path = DEFAULT_LATEST):
    if spec.get('sha256') != EXPECTED_SHA256:
      raise ValueError('Unexpected model, refusing prediction export')
    self.output = Path(output)
    self.output.mkdir(parents=True, exist_ok=True)
    self.latest_path = Path(latest_path)
    self.stream_path = self.output / 'shadow_predictions.jsonl'
    self.stream = self.stream_path.open('w', buffering=65536)
    self.seen = 0
    self.valid = 0
    self.invalid = 0
    self.stale_at_reply = 0
    self.non_monotonic_ids = 0
    self.previous_id: int | None = None
    self.previous_record: dict | None = None
    self.last_live_update = -1.0
    self.errors: list[str] = []
    self.ages: list[float] = []
    self.path_y_delta: list[float] = []
    self.sample_intervals: list[float] = []
    self.spec = spec
    self.closed = False
    _atomic_json(self.latest_path, {
      'schema': SCHEMA, 'source': PREDICTION_SOURCE, 'model_sha256': EXPECTED_SHA256,
      'active': False, 'shadow_only': True, 'authoritative': False,
      'state': 'starting', 'reason': 'Awaiting valid remote prediction samples'})

  def offer(self, frame_id: int, capture_ns: int, received_ns: int,
            output: np.ndarray, *, npu_ms: float | None = None) -> None:
    if self.closed:
      return
    self.seen += 1
    try:
      record = decode_shadow_prediction(output, self.spec, frame_id=int(frame_id),
                                       capture_ns=int(capture_ns), received_ns=int(received_ns),
                                       npu_ms=npu_ms)
      if self.previous_id is not None and frame_id <= self.previous_id:
        self.non_monotonic_ids += 1
      if self.previous_record is not None:
        prev = self.previous_record
        interval = (record['camera_timestamp_eof_ns'] - prev['camera_timestamp_eof_ns']) / 1e6
        if interval > 0:
          self.sample_intervals.append(round(interval, 3))
          # Observational temporal delta; NOT motion-compensated stability.
          delta = abs(record['diagnostics']['t_2_5_s_path_y_m'] -
                      prev['diagnostics']['t_2_5_s_path_y_m'])
          self.path_y_delta.append(round(delta, 4))
      record['active'] = True
      self.stream.write(json.dumps(record, separators=(',', ':'), allow_nan=False) + '\n')
      self.valid += 1
      self.previous_id = frame_id
      self.previous_record = record
      self.ages.append(record['capture_to_reply_ms'])
      self.stale_at_reply += int(not record['fresh_at_reply'])
      now_s = time.monotonic()
      if self.last_live_update < 0 or now_s - self.last_live_update >= LIVE_UPDATE_INTERVAL_S:
        _atomic_json(self.latest_path, record)
        self.stream.flush()
        self.last_live_update = now_s
    except (OSError, ValueError, TypeError, KeyError, IndexError) as exc:
      self.invalid += 1
      if len(self.errors) < 8:
        self.errors.append(f'{type(exc).__name__}: {exc}')

  def finish(self) -> dict:
    if self.closed:
      return self.summary
    self.closed = True
    self.stream.flush()
    self.stream.close()
    def percentile(data, p):
      return round(float(np.percentile(data, p)), 3) if data else None
    self.summary = {
      'schema': SCHEMA, 'model_sha256': EXPECTED_SHA256,
      'source': PREDICTION_SOURCE, 'shadow_only': True,
      'authoritative': False, 'remote_control_allowed': False,
      'output_stream': self.stream_path.name,
      'samples_seen': self.seen, 'valid_predictions': self.valid,
      'invalid_predictions': self.invalid, 'stale_at_reply': self.stale_at_reply,
      'non_monotonic_sample_frame_ids': self.non_monotonic_ids,
      'capture_to_reply_ms_p50': percentile(self.ages, 50),
      'capture_to_reply_ms_p95': percentile(self.ages, 95),
      'capture_to_reply_ms_p99': percentile(self.ages, 99),
      'sample_frame_interval_ms_p95': percentile(self.sample_intervals, 95),
      'raw_path_y_t2_5s_temporal_delta_m_p95': percentile(self.path_y_delta, 95),
      'temporal_delta_motion_compensated': False,
      'preprocessing_parity_verified': False,
      'recurrent_state_parity_verified': False,
      'reference_full_model_parity_verified': False,
      'physical_road_geometry_validated': False,
      'errors': self.errors,
      'note': 'No on-road authority. Only a separate decoded observation stream. '
              '20Hz inference throughput is measured elsewhere; JSONL sampling '
              'is independently decimated. Temporal delta is not motion-compensated.',
    }
    (self.output / 'shadow_predictions_summary.json').write_text(json.dumps(self.summary, indent=2)+'\n')
    _atomic_json(self.latest_path, {
      'schema': SCHEMA, 'source': PREDICTION_SOURCE, 'model_sha256': EXPECTED_SHA256,
      'active': False, 'shadow_only': True, 'authoritative': False,
      'state': 'ended', 'run_dir': str(self.output),
      'time_utc': datetime.now(timezone.utc).isoformat(),
      'summary': {'valid_predictions': self.valid, 'invalid_predictions': self.invalid},
    })
    return self.summary
