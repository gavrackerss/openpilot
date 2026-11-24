#!/usr/bin/env python3
"""Passive Jetlink-vs-modelV2 prediction agreement diagnostics.

This is NOT an FP16 numerical parity checker or a control path. It compares
semantic model outputs from exactly matching VisionIPC frame IDs and camera EOF
timestamps. Local stock modeld and the Jetlink full LS2 may have different
weights, preprocessing, scalar inputs, and recurrent state. Their numerical
differences CANNOT certify or disqualify Tensor G6 conversion fidelity.

Never sends or publishes cereal messages; no onroad parameters or vehicle
controls can be changed. Imported only when --compare-local is explicitly set.
"""
from __future__ import annotations

from collections import OrderedDict
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

SCHEMA = 1
MAX_LOCAL = 160  # ~8 seconds at 20 Hz
MAX_REMOTE = 160
TIMESTAMP_TOLERANCE_NS = 5_000_000


def _part(output: np.ndarray, spec: dict, name: str) -> np.ndarray:
  offsets = spec.get('output_slices', {}).get(name)
  if not isinstance(offsets, (list, tuple)) or len(offsets) != 2:
    raise ValueError(f'Jetlink model has no usable {name!r} output slice')
  begin, end = map(int, offsets)
  if begin < 0 or end > output.size or end <= begin:
    raise ValueError(f'Invalid output slice {name}: {offsets}')
  return output[begin:end]


def _mu(part: np.ndarray, shape: tuple[int, ...]) -> np.ndarray:
  n = math.prod(shape)
  if part.size != n * 2:
    raise ValueError(f'MDN mean/std width mismatch: {part.size} != {n * 2}')
  return part[:n].reshape(shape)


def remote_features(output: np.ndarray, spec: dict) -> dict[str, np.ndarray]:
  """Decode the mean half of unimodal output heads, no recurrent activation."""
  out = np.asarray(output, dtype=np.float32).reshape(-1)
  if out.size < 2068 or not np.all(np.isfinite(out)):
    raise ValueError('Jetlink output must be >=2068 finite floats')
  lane = _mu(_part(out, spec, 'lane_lines'), (4, 33, 2))
  edges = _mu(_part(out, spec, 'road_edges'), (2, 33, 2))
  plan = _mu(_part(out, spec, 'plan'), (33, 15))
  llp = _part(out, spec, 'lane_lines_prob')
  lp = _part(out, spec, 'lead_prob')
  if llp.size != 8 or lp.size != 3:
    raise ValueError('Unexpected probability head widths')
  sigmoid = lambda x: 1.0 / (1.0 + np.exp(-np.clip(x, -30.0, 30.0)))
  return {
    'position_x': plan[:, 0].copy(),
    'position_y': plan[:, 1].copy(),
    'position_z': plan[:, 2].copy(),
    'velocity_x': plan[:, 3].copy(),
    'lane_y': lane[:, :, 0].copy(),
    'lane_z': lane[:, :, 1].copy(),
    'edge_y': edges[:, :, 0].copy(),
    'edge_z': edges[:, :, 1].copy(),
    'lane_prob': sigmoid(llp[1::2]),
    'lead_prob': sigmoid(lp),
  }


def local_features(msg: Any) -> dict[str, np.ndarray]:
  def arr(vals):
    return np.asarray(list(vals), dtype=np.float32)
  pos = msg.position
  lanes = list(msg.laneLines)
  edges = list(msg.roadEdges)
  leads = list(msg.leadsV3)
  if len(lanes) != 4 or len(edges) != 2 or len(leads) != 3:
    raise ValueError('Local modelV2 lane/edge/lead structure is not compatible')
  result = {
    'position_x': arr(pos.x),
    'position_y': arr(pos.y),
    'position_z': arr(pos.z),
    'velocity_x': arr(msg.velocity.x),
    'lane_y': np.stack([arr(l.y) for l in lanes]),
    'lane_z': np.stack([arr(l.z) for l in lanes]),
    'edge_y': np.stack([arr(e.y) for e in edges]),
    'edge_z': np.stack([arr(e.z) for e in edges]),
    'lane_prob': arr(msg.laneLineProbs),
    'lead_prob': np.array([float(l.prob) for l in leads], dtype=np.float32),
  }
  if any(not np.isfinite(v).all() for v in result.values()):
    raise ValueError('Local modelV2 emitted non-finite features')
  return result


def frame_agreement(local: dict, remote: dict) -> dict:
  result = {}
  for name, r in remote.items():
    if name not in local:
      raise ValueError(f'Missing reference feature {name}')
    l = local[name]
    if r.shape != l.shape:
      raise ValueError(f'Shape mismatch for {name}: {r.shape} != {l.shape}')
    d = np.abs(r.astype(np.float64) - l.astype(np.float64))
    result[f'{name}_mae'] = round(float(d.mean()), 7)
    result[f'{name}_max_abs'] = round(float(d.max()), 7)
  return result


def _p95(values):
  return round(float(np.percentile(values, 95)), 6) if values else None


class PredictionObserver:
  """Non-blocking best-effort observation of local modelV2, keyed by camera frame.

  One SubMaster update per camera event. Pending remote samples are matched
  asynchronously (modeld may publish after Jetlink returns); unmatched samples
  are reported, NEVER silently compared to nearest/different frames.
  """
  def __init__(self, output: Path, spec: dict, submaster=None):
    self.output = Path(output)
    self.spec = spec
    if submaster is None:
      import cereal.messaging as messaging
      submaster = messaging.SubMaster(['modelV2'])
    self.sm = submaster
    self.local = OrderedDict()
    self.remote = OrderedDict()
    self.rows = []
    self.errors = []
    self.local_messages_seen = 0
    self.remote_samples_seen = 0
    self.timestamp_mismatches = 0
    self.invalid_local = 0

  def poll(self):
    """Never wait for local modeld (must not slow down 20Hz camera path)."""
    try:
      self.sm.update(0)
      if not self.sm.updated.get('modelV2', False):
        return
      msg = self.sm['modelV2']
      fid = int(msg.frameId)
      if fid < 0:
        return
      entry = (int(msg.timestampEof), local_features(msg))
      self.local_messages_seen += 1
      if fid in self.remote:
        remote = self.remote.pop(fid)
        self._compare(fid, entry, remote)
      else:
        self.local[fid] = entry
        self.local.move_to_end(fid)
        while len(self.local) > MAX_LOCAL:
          self.local.popitem(last=False)
    except Exception as exc:
      if len(self.errors) < 8:
        self.errors.append(f'Local modelV2 observation: {type(exc).__name__}: {exc}')

  def offer_remote(self, frame_id: int, capture_ns: int, receive_ns: int, output: np.ndarray):
    self.remote_samples_seen += 1
    try:
      entry = (int(capture_ns), int(receive_ns), remote_features(output, self.spec))
      if int(frame_id) in self.local:
        local = self.local.pop(int(frame_id))
        self._compare(int(frame_id), local, entry)
      else:
        self.remote[int(frame_id)] = entry
        self.remote.move_to_end(int(frame_id))
        while len(self.remote) > MAX_REMOTE:
          self.remote.popitem(last=False)
    except Exception as exc:
      if len(self.errors) < 8:
        self.errors.append(f'Remote output parsing: {type(exc).__name__}: {exc}')

  def _compare(self, frame_id: int, local: tuple, remote: tuple):
    local_ns, local_features_dict = local
    remote_ns, received_ns, remote_features_dict = remote
    if abs(local_ns - remote_ns) > TIMESTAMP_TOLERANCE_NS:
      self.timestamp_mismatches += 1
      return
    try:
      diff = frame_agreement(local_features_dict, remote_features_dict)
    except (ValueError, TypeError) as exc:
      if len(self.errors) < 8:
        self.errors.append(f'Comparison frame {frame_id}: {exc}')
      return
    self.rows.append({'frame_id': frame_id, 'timestamp_eof_ns': remote_ns,
                      'timestamp_delta_ms': round((local_ns - remote_ns)/1e6, 5),
                      'remote_reply_age_ms': round((received_ns - remote_ns)/1e6, 4),
                      **diff})

  def finish(self) -> dict:
    # One final non-blocking attempt before writing artifacts.
    self.poll()
    columns = list(self.rows[0].keys()) if self.rows else ['frame_id','timestamp_eof_ns','timestamp_delta_ms','remote_reply_age_ms']
    with (self.output/'local_vs_jetlink.csv').open('w', newline='') as f:
      w = csv.DictWriter(f, fieldnames=columns)
      w.writeheader()
      w.writerows(self.rows)
    fields = {c.removesuffix('_mae') for c in columns if c.endswith('_mae')}
    aggregates = {}
    for field in sorted(fields):
      xs = [r[f'{field}_mae'] for r in self.rows]
      aggregates[field] = {'mae_mean': round(float(np.mean(xs)), 6),
                           'mae_p95': _p95(xs),
                           'max_abs_over_frames': round(float(max(r[f'{field}_max_abs'] for r in self.rows)), 6)}
    summary = {
      'schema': SCHEMA, 'mode': 'observational_same-frame_semantic_agreement',
      'matched_frames': len(self.rows),
      'remote_output_samples': self.remote_samples_seen,
      'local_model_messages_seen': self.local_messages_seen,
      'timestamp_mismatches': self.timestamp_mismatches,
      'unmatched_remote_pending': len(self.remote),
      'errors': self.errors,
      'by_head': aggregates,
      'full_numerical_parity_verified': False,
      'model_weights_identical_verified': False,
      'camera_warp_parity_verified': False,
      'scalar_and_recurrent_state_parity_verified': False,
      'remote_predictions_authorized': False,
      'reason': ('Same-frame modelV2 comparison only. The local model graph/weights, CPU warp, '
                 'auxiliary inputs and recurrent state have not been proved identical. '
                 'This is not an eligibility decision for replacing or influencing modeld.'),
    }
    (self.output/'local_vs_jetlink_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary
