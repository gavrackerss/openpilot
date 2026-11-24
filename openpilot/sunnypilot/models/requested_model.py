"""Persistent requested model identity for the native XNOR migration.

This is a selector, not an activation mechanism. The legacy SunnyPilot bundle
remains intact, and native modeld is not changed by a request.
"""
from __future__ import annotations

from dataclasses import dataclass


REQUEST_KEY = "XNORRequestedModelRef"
STOCK = "stock"
TSFDOM = "0394d7284cfc229f772ee4fa73ff724ba8245110"
OPM10V3 = "7d4c295c27bc3f72727c49ec73b98e45745b588d"


@dataclass(frozen=True)
class ModelChoice:
  ref: str
  label: str
  architecture: str
  status: str


CHOICES = (
  ModelChoice(STOCK, "XNOR stock", "native", "Native runtime"),
  ModelChoice(TSFDOM, "TSFDOM", "native supercombo", "Candidate; validation required"),
  ModelChoice(OPM10V3, "OPM10V3", "split policy", "Compatibility runner required"),
)
BY_REF = {c.ref: c for c in CHOICES}


def requested_ref(saved_ref: object, legacy_bundle: object) -> str | None:
  """An explicit preference wins; an unknown selection is never reset."""
  if isinstance(saved_ref, str) and saved_ref:
    return saved_ref
  if isinstance(legacy_bundle, dict):
    ref = legacy_bundle.get("ref")
    if isinstance(ref, str) and ref:
      return ref
  return None


def requested_label(saved_ref: object, legacy_bundle: object) -> str:
  ref = requested_ref(saved_ref, legacy_bundle)
  if ref in BY_REF:
    return BY_REF[ref].label
  if ref is not None:
    return "Other saved model"
  return "Not selected"


def save_request(params, ref: str, *, confirmed_offroad: bool) -> bool:
  """Persist an explicit offroad choice without touching legacy bundle or runtime."""
  if not confirmed_offroad or not params.get_bool("IsOffroad"):
    return False
  if ref not in BY_REF:
    return False
  params.put(REQUEST_KEY, ref, block=True)
  return True
