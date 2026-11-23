"""Read-only SunnyPilot saved-model catalog compatibility gate for 0.11.2.

The legacy selector persists ModelManager_ActiveBundle as the *user choice*.
Invalid, unsupported or missing bundles must never be silently removed.
0.11.2's native modeld does not load these custom bundles, and this module
NEVER selects or executes a runner: its effective runner remains stock.

Metadata checks here are NOT a substitute for actual file/chunk SHA256
verification or model-input ABI validation. It intentionally does no I/O.
"""
from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Any

SUPPORTED_SELECTOR_VERSIONS = frozenset({15, 20})
_SHA256 = re.compile(r"^[0-9a-fA-F]{64}$")
_MAX_MODELS = 16


@dataclass(frozen=True)
class ModelCatalogAudit:
  requested_name: str | None
  requested_ref: str | None
  preserved_selection: bool
  metadata_issues: tuple[str, ...]
  effective_runner: str = "stock"
  ready_for_runtime: bool = False


def _artifact_issues(artifact: Any, where: str) -> list[str]:
  issues: list[str] = []
  if not isinstance(artifact, dict):
    return [f"{where}: missing artifact metadata"]
  filename = artifact.get("fileName")
  if not isinstance(filename, str) or not filename or filename.startswith("/") or (
    "/" in filename or "\\" in filename or filename in (".", "..")):
    issues.append(f"{where}: unsafe or missing local artifact filename")
  chunks = artifact.get("chunks") or []
  if not isinstance(chunks, list):
    issues.append(f"{where}: chunks must be a list")
    return issues
  if len(chunks) > 9999:
    issues.append(f"{where}: too many chunks")
    return issues
  if chunks:
    for i, chunk in enumerate(chunks):
      if not isinstance(chunk, dict) or not _SHA256.fullmatch(str(chunk.get("sha256", ""))):
        issues.append(f"{where}: missing SHA256 for chunk {i}")
  else:
    uri = artifact.get("downloadUri") or {}
    if not isinstance(uri, dict) or not _SHA256.fullmatch(str(uri.get("sha256", ""))):
      issues.append(f"{where}: missing artifact SHA256")
  return issues


def audit_bundle(selected: Any) -> ModelCatalogAudit:
  if selected is None:
    return ModelCatalogAudit(None, None, False, ("No saved model selection",))
  if not isinstance(selected, dict):
    return ModelCatalogAudit(None, None, True, ("Saved selection is not a JSON object",))

  name = selected.get("displayName") if isinstance(selected.get("displayName"), str) else None
  ref = selected.get("ref") if isinstance(selected.get("ref"), str) else None
  issues: list[str] = []
  try:
    version = int(selected.get("minimumSelectorVersion", 0))
  except (ValueError, TypeError):
    version = None
  if version not in SUPPORTED_SELECTOR_VERSIONS:
    issues.append("Unsupported selector version")
  models = selected.get("models")
  if not isinstance(models, list) or not (1 <= len(models) <= _MAX_MODELS):
    issues.append("Expected between 1 and 16 declared models")
  else:
    for i, model in enumerate(models):
      if not isinstance(model, dict):
        issues.append(f"Model {i}: invalid model descriptor")
        continue
      issues.extend(_artifact_issues(model.get("artifact"), f"Model {i}"))
      metadata = model.get("metadata")
      if metadata and isinstance(metadata, dict) and metadata.get("fileName"):
        issues.extend(_artifact_issues(metadata, f"Model {i} metadata"))

  # An inspection can never claim the model's on-disk PKLs, warped pixels,
  # recurrent ABI, direct action head or runtime timing are safe.
  return ModelCatalogAudit(name, ref, True, tuple(issues))
