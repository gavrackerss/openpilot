#!/usr/bin/env python3
"""Fail-closed, independent selection policy for CTM's production modeld path.

This module performs NO model loading, messaging, CAN, or file creation.
The arm file is a runtime kill-switch, not a safety-certification mechanism.
Evidence files and approval are operator-provided; hashes verify integrity,
NOT scientific validity. Production code never creates approval artifacts.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import re
from typing import Any

SOURCE_SHA = "e8d821733be15ebe9e27498bc27ad8bbbd741980ece37d77f377294010b8ff28"
ROOT = Path("/data/jetlink_xnor")
ARM_NAME = "CTM_AUTHORITY_ENABLED"
QUAL_NAME = "CTM_AUTHORITY_QUALIFIED"
KILL_NAME = "CTM_AUTHORITY_STOP"
CONFLICTS = (
    "CTM_AUTHORITY_NOT_QUALIFIED",
    "CTM_PARKED_TEST_ENABLED",
    "AUTHORITY_ENABLED",
    "LS2_AUTHORITY_QUALIFIED",
    "ONROAD_SHADOW_ENABLED",
)
REPORTS = ("numerical_parity", "onroad_shadow", "parked_fallback")
DIGEST = re.compile(r"^[0-9a-f]{64}$")


@dataclass(frozen=True)
class Gate:
    allowed: bool
    reason: str
    qualification_sha256: str | None = None


def requested(root: Path = ROOT) -> bool:
    """An explicit CTM arm request takes precedence over old LS2 selection.

    An invalid CTM request selects the stock local model rather than trying
    LS2 behind the operator's back.
    """
    return (root / ARM_NAME).exists()


def _read(path: Path, limit: int = 16_384) -> bytes:
    if not path.is_file() or path.stat().st_size > limit:
        raise ValueError(f"Missing or oversized CTM gate file: {path.name}")
    return path.read_bytes()


def _sha256(content: bytes) -> str:
    return hashlib.sha256(content).hexdigest()


def _qualify(raw: bytes, root: Path) -> None:
    try:
        manifest: dict[str, Any] = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeError) as exc:
        raise ValueError("Malformed CTM qualification JSON") from exc
    if not isinstance(manifest, dict):
        raise ValueError("CTM qualification must be an object")
    if manifest.get("schema") != 1 or manifest.get("source_sha256") != SOURCE_SHA:
        raise ValueError("CTM qualification schema or source identity mismatch")
    if manifest.get("approved_for_vehicle_control") is not True:
        raise ValueError("CTM vehicle-control approval not recorded")
    reports = manifest.get("reports")
    if not isinstance(reports, dict) or set(reports) != set(REPORTS):
        raise ValueError("CTM requires three independent evidence reports")
    for name in REPORTS:
        expected = reports[name]
        if not isinstance(expected, str) or not DIGEST.fullmatch(expected):
            raise ValueError(f"Invalid SHA256 for CTM {name}")
        # Fixed filenames, no caller-controlled paths, traversal or symlinks.
        evidence = root / "ctm_evidence" / f"{name}.json"
        if evidence.is_symlink():
            raise ValueError(f"Symlink forbidden for CTM {name}")
        payload = _read(evidence, 2_000_000)
        if _sha256(payload) != expected:
            raise ValueError(f"CTM {name} report SHA256 mismatch")
        try:
            report = json.loads(payload)
        except (ValueError, UnicodeError) as exc:
            raise ValueError(f"Invalid CTM {name} evidence JSON") from exc
        if (not isinstance(report, dict) or report.get("source_sha256") != SOURCE_SHA
                or report.get("passed") is not True):
            raise ValueError(f"CTM {name} evidence lacks matching verified pass")


def eligible(root: Path = ROOT) -> Gate:
    """Full startup check. No inference can join unless the gate is satisfied."""
    try:
        if not requested(root):
            return Gate(False, "ctm_not_requested")
        if (root / KILL_NAME).exists():
            return Gate(False, "operator_stop_marker")
        for name in CONFLICTS:
            if (root / name).exists():
                return Gate(False, f"conflicting_marker:{name}")
        if _read(root / ARM_NAME, 128).decode("ascii").strip() != SOURCE_SHA:
            return Gate(False, "arm_source_mismatch")
        raw = _read(root / QUAL_NAME)
        _qualify(raw, root)
        return Gate(True, "qualified_and_armed", _sha256(raw))
    except (OSError, ValueError, UnicodeError) as exc:
        return Gate(False, f"qualification_rejected:{exc}")


def lease_alive(original: Gate, root: Path = ROOT) -> bool:
    """Cheap per-frame revocation check: new evidence requires modeld restart.

    Removing either marker, altering the qualification, setting STOP or
    staging another conflicting Jetlink session forces local fallback.
    This cannot dynamically grant authority to an unqualified model.
    """
    if not original.allowed or not original.qualification_sha256:
        return False
    try:
        if (root / KILL_NAME).exists() or any((root / name).exists() for name in CONFLICTS):
            return False
        if _read(root / ARM_NAME, 128).decode("ascii").strip() != SOURCE_SHA:
            return False
        return _sha256(_read(root / QUAL_NAME)) == original.qualification_sha256
    except (OSError, ValueError, UnicodeError):
        return False
