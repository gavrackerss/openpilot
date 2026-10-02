"""XNOR namespace bridge for the ported sunnypilot feature tree.

XNOR/openpilot keeps most implementation packages at repository root while
``openpilot.*`` is the canonical import namespace.  The V233 selector port
placed the Sunnypilot implementation at ``<repo>/sunnypilot`` but omitted the
corresponding namespace bridge, causing manager startup to fail on
``import openpilot.sunnypilot``.

Keep one implementation copy at repository root and extend this package's
search path to it.  This mirrors XNOR's existing openpilot/common,
openpilot/selfdrive, and openpilot/system namespace arrangement without
requiring a symlink to survive ZIP extraction.
"""
from __future__ import annotations

from enum import IntEnum
import hashlib
from pathlib import Path

_IMPL_ROOT = Path(__file__).resolve().parents[2] / "sunnypilot"
if not _IMPL_ROOT.is_dir():
  raise ModuleNotFoundError(f"XNOR Sunnypilot implementation not found at {_IMPL_ROOT}")

_impl_path = str(_IMPL_ROOT)
if _impl_path not in __path__:
  __path__.append(_impl_path)

# Keep the small public surface from sunnypilot/__init__.py available from the
# canonical openpilot.sunnypilot namespace as upstream Sunnypilot expects.
PARAMS_UPDATE_PERIOD = 3  # seconds


def get_file_hash(path: str) -> str:
  sha256_hash = hashlib.sha256()
  with open(path, "rb") as f:
    for byte_block in iter(lambda: f.read(4096), b""):
      sha256_hash.update(byte_block)
  return sha256_hash.hexdigest()


class IntEnumBase(IntEnum):
  @classmethod
  def min(cls):
    return min(cls)

  @classmethod
  def max(cls):
    return max(cls)


def get_sanitize_int_param(key: str, min_val: int, max_val: int, params) -> int:
  val: int = params.get(key, return_default=True)
  clipped_val = max(min_val, min(max_val, val))

  if clipped_val != val:
    params.put(key, clipped_val)

  return clipped_val
