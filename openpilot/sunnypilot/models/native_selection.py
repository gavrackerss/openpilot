"""Native model startup selection, separate from legacy SunnyPilot cache formats.

The requested preference is always retained. Only a release-reviewed native
artifact may be selected; a download, local compile or synthetic smoke test
alone cannot qualify an artifact for vehicle execution.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any
import hashlib
import json
import re

from openpilot.sunnypilot.models.requested_model import REQUEST_KEY, STOCK, TSFDOM, requested_ref

# Release-controlled approvals. No runtime parameter, downloaded manifest or
# bench-generated JSON can add an entry. No QCOM candidate is approved yet.
APPROVED_NATIVE_ARTIFACTS: dict[str, tuple[str, str]] = {}

TSFDOM_ABI_PATH = (Path(__file__).resolve().parents[3] /
                  "migration/sunnypilot_legacy/tsfdom_native_abi_report.json")


@dataclass(frozen=True)
class NativeSelection:
  requested: str
  effective: str
  reason: str
  path: Path | None = None
  sha256: str | None = None


def select_native(saved: object, legacy: object, backend: str, root: Path) -> NativeSelection:
  ref = requested_ref(saved, legacy) or STOCK
  if ref == STOCK:
    return NativeSelection(ref, STOCK, "Native stock requested")
  if ref != TSFDOM:
    return NativeSelection(ref, STOCK, "Separate model architecture required")
  if backend != "QCOM":
    return NativeSelection(ref, STOCK, "Requested artifact is for QCOM only")
  approval = APPROVED_NATIVE_ARTIFACTS.get(ref)
  if approval is None:
    return NativeSelection(ref, STOCK, "QCOM release qualification pending")
  filename, digest = approval
  if (Path(filename).name != filename or not re.fullmatch(r"[a-f0-9]{64}", digest)
      or filename in ("", ".", "..")):
    return NativeSelection(ref, STOCK, "Invalid release artifact approval")
  path = root / filename
  if path.is_symlink() or not path.is_file():
    return NativeSelection(ref, STOCK, "Approved native artifact missing")
  return NativeSelection(ref, ref, "Approved QCOM artifact", path, digest)


def selection_from_params(params: Any, backend: str, root: Path) -> NativeSelection:
  return select_native(params.get(REQUEST_KEY), params.get("ModelManager_ActiveBundle"), backend, root)


def load_verified_bundle(selection: NativeSelection):
  """Only called for an exact source-approved artifact; never legacy PKLs."""
  from openpilot.selfdrive.modeld.helpers import load_oob
  from openpilot.sunnypilot.models.native_abi import compare_native_abi, describe_metadata
  from openpilot.sunnypilot.models.native_envelope import inspect_envelope
  if selection.effective != TSFDOM or selection.path is None or selection.sha256 is None:
    raise ValueError("No approved custom native selection")
  approval = APPROVED_NATIVE_ARTIFACTS.get(selection.effective)
  if approval != (selection.path.name, selection.sha256):
    raise ValueError("Artifact not approved by this release")
  with selection.path.open("rb") as f:
    digest = hashlib.sha256()
    while chunk := f.read(1024 * 1024):
      digest.update(chunk)
    if digest.hexdigest() != selection.sha256:
      raise ValueError("Compiled artifact SHA256 mismatch")
    f.seek(0)
    inspect_envelope(f)
    f.seek(0)
    bundle = load_oob(f)
  abi = json.loads(TSFDOM_ABI_PATH.read_text())["candidate_abi"]
  if compare_native_abi(abi, describe_metadata(bundle["metadata"])):
    raise ValueError("Native artifact ABI mismatch")
  return bundle
