#!/usr/bin/env python3
"""Pinned original Cinque Terre queued-model Jetlink wire contract.

Pure Python, no cereal/tinygrad, so CI can reject bad manifests without a comma.
This defines what the EXISTING Zoompilot Parser expects. It does not assert
numerical parity, plausible trajectories, vehicle safety, or authority approval.
"""
from __future__ import annotations

import math

SOURCE_SHA = "e8d821733be15ebe9e27498bc27ad8bbbd741980ece37d77f377294010b8ff28"
SOURCE_NBYTES = 765950064
CTM_VERSION = "ctm-original-sept04-2026"
FRAME_SKIP = 4
OUTPUT_COUNT = 18452
WARPED_SHAPE = (2, 6, 128, 256)
PACKED_FLOATS = 12  # desire(8) + traffic_convention(2) + action_t(2)
REPLY_FLOATS = 2068  # 18452 minus the 16384-float server-retained history

# These six inputs are extracted from the *original CTM* ONNX graph, not LS2.
INPUT_SHAPES = {
    "img": (1, 12, 128, 256),
    "big_img": (1, 12, 128, 256),
    "desire_pulse": (1, 33, 8),
    "traffic_convention": (1, 2),
    "action_t": (1, 2),
    "features_buffer": (1, 32, 32, 512),
}
OUTPUT_SHAPES = {"outputs": (1, OUTPUT_COUNT)}

# The local stock Parser's *required* 18452-float layout. A live CTM ENGINE
# response must confirm this before the adapter can decode any prediction.
# This is a strict COMPATIBILITY ASSERTION, not proof of CTM output parity.
PARSER_SLICES = {
    "lane_lines": (0, 528),
    "lane_lines_prob": (528, 536),
    "road_edges": (536, 800),
    "meta": (800, 855),
    "desire_pred": (855, 887),
    "pose": (887, 899),
    "wide_from_device_euler": (899, 905),
    "road_transform": (905, 917),
    "plan": (917, 1907),
    "lead": (1907, 2051),
    "lead_prob": (2051, 2054),
    "desire_state": (2054, 2062),
    "action": (2062, 2066),
    "hidden_state": (2066, 18450),
    "pad": (18450, 18452),
}
PARSED_SHAPES = {
    "plan": (1, 33, 15),
    "plan_stds": (1, 33, 15),
    "lane_lines": (1, 4, 33, 2),
    "road_edges": (1, 2, 33, 2),
    "lead": (1, 3, 6, 4),
    "lead_prob": (1, 3),
    "desire_state": (1, 8),
}


class ContractError(ValueError):
    """A CTM reply cannot be safely decoded by the current local Parser."""


def _shape_dict(raw: object, label: str) -> dict[str, tuple[int, ...]]:
    if not isinstance(raw, dict):
        raise ContractError(f"{label} absent or not an object")
    out = {}
    for key, dimensions in raw.items():
        if not isinstance(key, str) or not isinstance(dimensions, (list, tuple)):
            raise ContractError(f"{label} contains invalid tensor description")
        if any(type(n) is not int or n <= 0 for n in dimensions):
            raise ContractError(f"{label}.{key} has invalid tensor dimensions")
        out[key] = tuple(dimensions)
    return out


def validate_spec(spec: dict) -> None:
    """Fail closed on model, shape, queue layout or output-slice mismatch.

    Protocol v3 server owns the feature_buffer and image/desire queues; only
    two newest camera images plus 12 packed scalars cross the USB link.
    """
    if not isinstance(spec, dict):
        raise ContractError("Missing CTM ENGINE_RESP model specification")
    if spec.get("sha256") != SOURCE_SHA:
        raise ContractError("CTM source SHA mismatch: refusing LS2 or another model")
    if spec.get("nbytes") is not None and int(spec["nbytes"]) != SOURCE_NBYTES:
        raise ContractError("CTM source bytes mismatch")
    if type(spec.get("frame_skip")) is not int or spec["frame_skip"] != FRAME_SKIP:
        raise ContractError("CTM recurrent frame stride changed")
    if _shape_dict(spec.get("input_shapes"), "input_shapes") != INPUT_SHAPES:
        raise ContractError("CTM six-input queued graph changed")
    if _shape_dict(spec.get("output_shapes"), "output_shapes") != OUTPUT_SHAPES:
        raise ContractError("CTM 18452-float output graph changed")
    raw_slices = spec.get("output_slices")
    if not isinstance(raw_slices, dict):
        raise ContractError("CTM server did not provide output slices")
    for key, pair in PARSER_SLICES.items():
        actual = raw_slices.get(key)
        if not isinstance(actual, (list, tuple)) or tuple(actual) != pair:
            raise ContractError(f"CTM output slice {key} does not match local Parser: {actual}")
    hidden_start, hidden_end = PARSER_SLICES["hidden_state"]
    if OUTPUT_COUNT - (hidden_end - hidden_start) != REPLY_FLOATS:
        raise ContractError("CTM reply count inconsistent with phone-retained feature history")
    if math.prod(WARPED_SHAPE) != 2 * 6 * 128 * 256 or PACKED_FLOATS != 8 + 2 + 2:
        raise ContractError("CTM transport framing inconsistent")


def validate_native_spec(spec: object) -> None:
    """Validate both Jetlink's parsed ModelSpec and its serialized description."""
    try:
        raw = spec.to_dict()
    except (AttributeError, TypeError) as exc:
        raise ContractError("Native ModelSpec cannot be serialized") from exc
    validate_spec(raw)
    if tuple(spec.model_hw) != (128, 256):
        raise ContractError("Unexpected native CTM camera geometry")
    if int(spec.packed_nelem) != PACKED_FLOATS:
        raise ContractError("Unexpected CTM scalar count")
    if int(spec.warped_nbytes) != math.prod(WARPED_SHAPE):
        raise ContractError("Unexpected CTM warped frame size")
    if int(spec.reply_nelem) != REPLY_FLOATS:
        raise ContractError("Unexpected CTM reply size")
    if bool(spec.stateful) or bool(spec.state_pairs):
        raise ContractError("CTM is a queued graph; native client reports state pairs")


def validate_parsed(outputs: dict) -> None:
    """Validate a decoded CTM prediction without manufacturing action values."""
    import numpy as np
    if not isinstance(outputs, dict):
        raise ContractError("CTM parser returned no output mapping")
    for key, shape in PARSED_SHAPES.items():
        arr = outputs.get(key)
        if not isinstance(arr, np.ndarray) or arr.shape != shape:
            raise ContractError(f"CTM parsed {key} wrong shape: {getattr(arr, 'shape', None)}")
        if not np.isfinite(arr).all():
            raise ContractError(f"CTM parsed {key} has NaN or Inf")
    for key, arr in outputs.items():
        if isinstance(arr, np.ndarray) and not np.isfinite(arr).all():
            raise ContractError(f"CTM parsed {key} has nonfinite values")
