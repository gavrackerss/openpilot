"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and contributors.

XNOR model selector: v23 standard models and legacy v17 compatibility.

Important: ModelManager_ActiveBundle is the user's requested selection, not a
health-check cache. Model validation must NEVER remove this parameter. An
invalid/unavailable artifact selects the stock *runtime* while keeping the
requested bundle and a diagnostic explanation available for repair.
"""

import hashlib
import os
import pickle
from pathlib import Path

import numpy as np

from cereal import custom
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.sunnypilot.models.constants import Meta, MetaSimPose, MetaTombRaider
from openpilot.system.hardware.hw import Paths

REQUIRED_JSON_VERSION = 20
SUPPORTED_JSON_VERSIONS = frozenset((15, 20))

CUSTOM_MODEL_PATH = Paths.model_root()
METADATA_PATH = Path(__file__).parent / '../models/supercombo_metadata.pkl'
ModelManager = custom.ModelManagerSP

# These values belong to one process. The manager and off-road model manager
# have separate caches, and both revalidate on their first observation of a
# new persisted model selection.
_LAST_VALIDATED_RAW = None
_RUNNER_EVALUATED_RAW = object()
_RUNNER_EVALUATED_RESULT = None


def _compute_hash(file_path: str) -> str | None:
  """Stream SHA256 for a regular or .chunkmanifest-backed file (bounded RAM)."""
  from openpilot.common.file_chunker import get_manifest_path, get_chunk_name

  digest = hashlib.sha256()
  try:
    manifest = get_manifest_path(file_path)
    if os.path.isfile(manifest):
      with open(manifest, encoding='utf-8') as stream:
        count = int(stream.read().strip())
      if count < 1 or count > 9999:
        return None
      files = (get_chunk_name(file_path, i, count) for i in range(count))
    else:
      files = (file_path,)

    for part in files:
      with open(part, 'rb') as stream:
        while block := stream.read(1024 * 1024):
          digest.update(block)
    return digest.hexdigest().lower()
  except (OSError, ValueError):
    return None


async def verify_file(file_path: str, expected_hash: str) -> bool:
  return _verify_file(file_path, expected_hash)


def _verify_file(file_path: str, expected_hash: str) -> bool:
  digest = _compute_hash(file_path)
  return bool(digest and expected_hash and digest == str(expected_hash).lower())


def is_bundle_version_compatible(bundle: dict) -> bool:
  try:
    return int(bundle.get('minimumSelectorVersion', 0)) in SUPPORTED_JSON_VERSIONS
  except (ValueError, TypeError, AttributeError):
    return False


def _bundle_artifacts(bundle: custom.ModelManagerSP.ModelBundle) -> list[tuple[str, str]]:
  """File/digest pairs, independently verifying modern chunks or legacy PKLs."""
  from openpilot.common.file_chunker import get_chunk_name

  result = []
  for model in getattr(bundle, 'models', []) or []:
    for artifact in (getattr(model, 'artifact', None), getattr(model, 'metadata', None)):
      filename = getattr(artifact, 'fileName', None) if artifact is not None else None
      if not filename:
        continue
      chunks = list(getattr(artifact, 'chunks', []) or [])
      if chunks:
        count = len(chunks)
        result.extend((get_chunk_name(filename, i, count), chunk.sha256) for i, chunk in enumerate(chunks))
      elif getattr(artifact, 'downloadUri', None) and getattr(artifact.downloadUri, 'sha256', None):
        result.append((filename, artifact.downloadUri.sha256))
  return result


def _bundle_integrity_issues(bundle: custom.ModelManagerSP.ModelBundle | None) -> list[str]:
  """Read-only verification; never modifies Params or deletes models.

  Legacy v17 artifacts can be standalone files OR have a chunkmanifest, while
  newer v23 artifacts carry an explicit list of per-chunk SHA256 digests.
  Refuse an empty/incomplete model definition; do not treat all([]) as valid.
  """
  from openpilot.common.file_chunker import get_manifest_path

  if bundle is None or not getattr(bundle, 'models', None):
    return ['Saved bundle could not be parsed or contains no models']

  root = Paths.model_root()
  issues = []
  expected_files = _bundle_artifacts(bundle)
  if not expected_files:
    issues.append('Saved bundle has no model artifact checksums')

  for model in bundle.models:
    primary = getattr(model, 'artifact', None)
    if not getattr(primary, 'fileName', None):
      issues.append('Saved bundle has a model with no PKL filename')

    for artifact in (getattr(model, 'artifact', None), getattr(model, 'metadata', None)):
      filename = getattr(artifact, 'fileName', None) if artifact is not None else None
      if not filename:
        continue
      chunks = list(getattr(artifact, 'chunks', []) or [])
      if chunks:
        manifest = get_manifest_path(os.path.join(root, filename))
        try:
          with open(manifest, encoding='utf-8') as stream:
            count = int(stream.read().strip())
          if count != len(chunks):
            issues.append(f'{filename}: chunk manifest count {count} != {len(chunks)}')
        except (OSError, ValueError):
          issues.append(f'{filename}: chunk manifest missing/invalid')
      elif not getattr(getattr(artifact, 'downloadUri', None), 'sha256', None):
        issues.append(f'{filename}: SHA256 missing from saved metadata')

  for filename, expected_hash in expected_files:
    path = os.path.join(root, filename)
    if not (os.path.isfile(path) or os.path.isfile(path + '.chunkmanifest')):
      issues.append(f'{filename}: file missing')
    elif not _verify_file(path, expected_hash):
      issues.append(f'{filename}: SHA256 mismatch / incomplete chunk(s)')

  return issues


def _bundle_is_valid_locally(bundle: custom.ModelManagerSP.ModelBundle) -> bool:
  return not _bundle_integrity_issues(bundle)


def _bundle_needs_reset(active_bundle: custom.ModelManagerSP.ModelBundle, available_bundles=None) -> bool:
  """Legacy compatibility. Only local integrity affects runtime eligibility.

  A hidden catalogue entry or an upstream recompile is NOT grounds to erase a
  previously installed, verified model. The caller must never delete the
  saved selection merely because this function returns True.
  """
  return not _bundle_is_valid_locally(active_bundle)


def get_active_bundle(params: Params | None = None, raw_bundle_dict: dict | bytes | None = None) -> 'custom.ModelManagerSP.ModelBundle | None':
  params = params or Params()
  try:
    raw = raw_bundle_dict if raw_bundle_dict is not None else (params.get('ModelManager_ActiveBundle') or {})
    if raw and is_bundle_version_compatible(raw):
      return custom.ModelManagerSP.ModelBundle(**raw)
  except Exception as exc:
    cloudlog.error(f'Unable to parse saved ModelManager_ActiveBundle: {exc}')
  return None


def validate_active_bundle(params: Params, available_bundles=None) -> None:
  """Log diagnostics once per selected bundle without ever deleting it.

  Safety is enforced by get_active_model_runner, which chooses stock if file
  checks fail. Avoid overwriting ModelRunnerTypeCache here: this function
  executes in the off-road process while the actual runner decision belongs
  to the manager process.
  """
  global _LAST_VALIDATED_RAW
  raw = params.get('ModelManager_ActiveBundle')
  if not raw:
    _LAST_VALIDATED_RAW = None
    return
  if raw == _LAST_VALIDATED_RAW:
    return

  bundle = get_active_bundle(params, raw_bundle_dict=raw)
  issues = _bundle_integrity_issues(bundle)
  if issues:
    cloudlog.error('Saved model selection PRESERVED, but runtime must use stock until integrity is repaired: ' + '; '.join(issues[:8]))
  else:
    cloudlog.info(f'Saved model selection verified: {bundle.internalName} ({bundle.ref})')
  _LAST_VALIDATED_RAW = raw


def get_active_model_runner(params: Params | None = None, force_check: bool = False) -> int:
  """Independently gate custom model execution on verified installed files.

  The process-config manager calls this every loop, and ModelRunnerTypeCache
  is cleared at an on-road transition. We memoize the *result* per saved
  selection so on-road transitions do not repeatedly hash large files.
  A new saved selection is always rechecked, even if the old runner cache
  has not yet expired. A cleared runner cache forces another integrity check
  at an on-road transition.
  """
  global _RUNNER_EVALUATED_RAW, _RUNNER_EVALUATED_RESULT
  params = params or Params()
  raw = params.get('ModelManager_ActiveBundle')
  stored_cache = params.get('ModelRunnerTypeCache')

  if raw == _RUNNER_EVALUATED_RAW and _RUNNER_EVALUATED_RESULT is not None and stored_cache is not None:
    runner = _RUNNER_EVALUATED_RESULT
  else:
    runner = int(custom.ModelManagerSP.Runner.stock)
    if raw:
      bundle = get_active_bundle(params, raw_bundle_dict=raw)
      issues = _bundle_integrity_issues(bundle)
      if bundle is not None and not issues:
        runner = int(bundle.runner.raw)
      else:
        cloudlog.error('Custom model is selected but NOT runnable; using stock while preserving selection: ' + '; '.join(issues[:8]))
    _RUNNER_EVALUATED_RAW = raw
    _RUNNER_EVALUATED_RESULT = runner

  # Re-establish the cleared runner cache after an on-road transition. Do
  # not trust a stale cached tinygrad value for a different selected bundle.
  if stored_cache != runner:
    params.put('ModelRunnerTypeCache', runner)
  return runner


def _get_model():
  if bundle := get_active_bundle():
    return next((m for m in bundle.models if m.type == ModelManager.Model.Type.supercombo), None)
  return None


def load_metadata():
  model = _get_model()
  metadata_name = model.metadata.fileName if model and model.metadata.fileName else None
  metadata_path = os.path.join(CUSTOM_MODEL_PATH, metadata_name) if metadata_name else METADATA_PATH
  with open(metadata_path, 'rb') as stream:
    return pickle.load(stream)


def prepare_inputs(model_metadata: dict) -> dict[str, np.ndarray]:
  return {
    key: np.zeros(shape, dtype=np.float32).flatten()
    for key, shape in model_metadata['input_shapes'].items()
    if 'img' not in key
  }


def load_meta_constants(model_metadata: dict):
  if 'sim_pose' in model_metadata['input_shapes']:
    return MetaSimPose
  meta_slice = model_metadata['output_slices']['meta']
  if (meta_slice.start, meta_slice.stop, meta_slice.step) == (5868, 5921, None):
    return MetaTombRaider
  return Meta


def plan_x_idxs_helper(constants, plan, model_output) -> list[float]:
  LINE_T_IDXS = [np.nan] * constants.IDX_N
  LINE_T_IDXS[0] = 0.0
  plan_x = model_output['plan'][0, :, plan.POSITION][:, 0].tolist()
  for xidx in range(1, constants.IDX_N):
    tidx = 0
    while tidx < constants.IDX_N - 1 and model_output['plan'][0, tidx + 1, plan.POSITION][0] < constants.X_IDXS[xidx]:
      tidx += 1
    if tidx == constants.IDX_N - 1:
      LINE_T_IDXS[xidx] = constants.T_IDXS[constants.IDX_N - 1]
      break
    current_x_val = model_output['plan'][0, tidx, plan.POSITION][0]
    next_x_val = model_output['plan'][0, tidx + 1, plan.POSITION][0]
    p = (constants.X_IDXS[xidx] - current_x_val) / (next_x_val - current_x_val) if abs(next_x_val - current_x_val) > 1e-9 else float('nan')
    LINE_T_IDXS[xidx] = p * constants.T_IDXS[tidx + 1] + (1 - p) * constants.T_IDXS[tidx]
  return LINE_T_IDXS
