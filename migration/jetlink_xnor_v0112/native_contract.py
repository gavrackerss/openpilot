"""Read-only XNOR-native 0.11.2 / Pixel LS2 compatibility contracts.

This is NOT a modeld hook or Jetlink joining adapter. It deliberately has
no socket, CAN, Panda, Tinygrad, Params or filesystem operations.

The archived joining implementation passes four positional run arguments,
whereas XNOR v0.11.2's resident ModelState.run has three. Do NOT attach
that joining implementation to XNOR modeld until its run and reset paths
are adapted and tested independently.

ModelV2 path colouring and a finite Pixel raw action head do NOT establish
a valid immediate longitudinal command or safe vehicle stopping.
"""
from __future__ import annotations

from dataclasses import dataclass
import inspect
import math
import struct
from typing import Any

PIXEL_REPLY_FLOATS = 2068
PIXEL_PREDICTION_FLOATS = 2066
PIXEL_ACTION_SLICE = (2062, 2066)
PIXEL_PADDING_SLICE = (2066, 2068)
XNOR_ACTION_WIDTH = 2
EXPECTED_FRAME_SKIP = 4
EXPECTED_VISION_INPUTS = frozenset({"img", "big_img"})
EXPECTED_MODEL_INPUTS = frozenset({"desire", "traffic_convention", "action_t"})


def validate_resident_xnor_model(small: Any) -> None:
  """Check the actual XNOR v0.11.2 ModelState ABI; never execute a frame."""
  if getattr(small, "usbgpu", None) is not False:
    raise ValueError("Requires the resident local QCOM model (usbgpu=False)")
  run = getattr(small, "run", None)
  if not callable(run):
    raise ValueError("Missing native ModelState.run")
  try:
    sig = inspect.signature(run)
  except (ValueError, TypeError) as exc:
    raise ValueError("Uninspectable native run method") from exc
  params = tuple(sig.parameters.values())
  if len(params) != 3 or tuple(p.name for p in params) != (
      "bufs", "transforms", "inputs") or any(
          p.kind is not inspect.Parameter.POSITIONAL_OR_KEYWORD for p in params):
    raise ValueError(f"XNOR requires run(bufs, transforms, inputs), got {sig}")
  if getattr(small, "frame_skip", None) != EXPECTED_FRAME_SKIP:
    raise ValueError("Unexpected XNOR model context stride")
  if set(getattr(small, "vision_input_names", ())) != EXPECTED_VISION_INPUTS:
    raise ValueError("Expected both img and big_img camera inputs")
  queues = getattr(small, "input_queues", None)
  arrays = getattr(small, "npy", None)
  if not isinstance(queues, dict) or not isinstance(arrays, dict):
    raise ValueError("XNOR requires input_queues and npy dictionaries")
  if not EXPECTED_MODEL_INPUTS <= set(arrays):
    raise ValueError("Missing resident XNOR host scalar inputs")
  if not callable(getattr(small, "run_policy", None)):
    raise ValueError("Missing XNOR policy JIT entry point")
  if not callable(getattr(small, "warp", None)):
    raise ValueError("Missing XNOR warp JIT entry point")
  prev = getattr(small, "prev_desire", None)
  if getattr(prev, "shape", None) != (8,):
    raise ValueError("Unexpected recurrent desire-edge shape")
  if hasattr(small, "packed_input") or hasattr(small, "state_pairs"):
    raise ValueError("Official-comma state ABI detected, not XNOR-native v0.11.2")


@dataclass(frozen=True)
class PixelActionObservation:
  """Uninterpreted Pixel head; MUST NOT be connected to the planner."""
  raw_head: tuple[float, float, float, float]
  tail_padding: tuple[float, float]
  pixel_reply_float_count: int = PIXEL_REPLY_FLOATS
  xnor_action_width: int = XNOR_ACTION_WIDTH
  native_action_semantics_proven: bool = False
  authority_allowed: bool = False


def inspect_pixel_reply(packet: bytes) -> PixelActionObservation:
  """Decode an exact 2068-float little-endian LS2 packet for offline audit.

  These four numbers are raw network values, not a 0.11.2 action vector.
  XNOR's modeld can use a 2-float action head when available, but the Pixel
  sends a 4-float raw head and the legacy decoder drops it entirely. We
  refuse to truncate or infer the semantic mapping automatically.
  """
  if not isinstance(packet, bytes) or len(packet) != PIXEL_REPLY_FLOATS * 4:
    raise ValueError("Expected an exact binary 2068-float Pixel reply")
  values = struct.unpack("<2068f", packet)
  if not all(math.isfinite(v) for v in values):
    raise ValueError("Non-finite value in LS2 reply")
  return PixelActionObservation(
    raw_head=tuple(values[slice(*PIXEL_ACTION_SLICE)]),
    tail_padding=tuple(values[slice(*PIXEL_PADDING_SLICE)]),
  )


def assert_authority_disabled() -> None:
  """No flag, configuration, or compatibility observation can activate LS2."""
  raise RuntimeError(
    "LS2 authority not supported by XNOR-native 0.11.2 staging: "
    "the action contract, frame timing, fallback and longitudinal replay "
    "are unqualified"
  )
