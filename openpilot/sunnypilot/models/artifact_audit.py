"""Read-only artifact verification for the native XNOR model migration.

Catalog checksums establish integrity, not provenance or pickle safety.
No pickle is opened or executed here. Native model selection is unchanged.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
from pathlib import Path

from openpilot.sunnypilot.models.catalog_guard import audit_bundle


@dataclass(frozen=True)
class ArtifactAudit:
  requested_name: str | None
  issues: tuple[str, ...]
  verified_files: int
  verified_bytes: int
  native_abi_verified: bool = False
  effective_runner: str = "stock"


def _digest(path: Path) -> tuple[str, int]:
  if path.is_symlink() or not path.is_file():
    raise ValueError(f"{path.name}: not a regular local file")
  h = hashlib.sha256()
  size = 0
  with path.open("rb") as stream:
    while block := stream.read(1024 * 1024):
      h.update(block)
      size += len(block)
  return h.hexdigest(), size


def verify_artifacts(selected: object, root: Path) -> ArtifactAudit:
  """Check every declared file/chunk, rejecting ambiguous native layouts."""
  catalog = audit_bundle(selected)
  if catalog.metadata_issues:
    return ArtifactAudit(catalog.requested_name, catalog.metadata_issues, 0, 0)
  root = Path(root)
  issues: list[str] = []
  files = total = 0
  seen: dict[str, str] = {}
  for model in selected["models"]:
    for artifact in (model["artifact"], model.get("metadata")):
      if not artifact or not artifact.get("fileName"):
        continue
      name = artifact["fileName"]
      path = root / name
      manifest = root / (name + ".chunkmanifest")
      chunks = artifact.get("chunks") or []
      try:
        if chunks:
          if path.exists() or path.is_symlink():
            raise ValueError(f"{name}: ambiguous direct and chunked layout")
          if manifest.is_symlink() or not manifest.is_file() or manifest.stat().st_size > 32:
            raise ValueError(f"{name}: missing or invalid chunk manifest")
          if manifest.read_text().strip() != str(len(chunks)):
            raise ValueError(f"{name}: incorrect chunk count")
          expected = [(f"{name}.chunk{i+1:02d}of{len(chunks):02d}", chunk["sha256"])
                      for i, chunk in enumerate(chunks)]
        else:
          if manifest.exists() or manifest.is_symlink():
            raise ValueError(f"{name}: undeclared chunk manifest")
          expected = [(name, artifact["downloadUri"]["sha256"])]
        for filename, digest in expected:
          digest = digest.lower()
          if filename in seen:
            if seen[filename] != digest:
              raise ValueError(f"{filename}: conflicting catalog checksums")
            continue
          actual, size = _digest(root / filename)
          if actual != digest:
            raise ValueError(f"{filename}: SHA256 mismatch")
          seen[filename] = digest
          files += 1
          total += size
      except (OSError, ValueError, UnicodeError) as exc:
        issues.append(str(exc))
  return ArtifactAudit(catalog.requested_name, tuple(issues), files, total)
