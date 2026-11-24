#!/usr/bin/env python3
"""Native comma v0.11.2 compatibility bridge for XNOR Jetlink (OFFROAD ONLY).

This module fixes the v0.11.2 ModelState contract mismatch that exists in the
legacy copied XNOR _Small wrapper. It is deliberately not imported by
openpilot.selfdrive.modeld.modeld or by manager. It does NOT grant Pixel/LS2
vehicle authority. See V0112_NATIVE_PORT_STATUS.md.

Native 0.11.2:
  ModelState(cam_w, cam_h, chestnut)
  run(bufs, transforms, inputs, after_enqueue=None)
  state_pairs / input_queues / packed_input / prev_desire
Legacy XNOR _Small:
  run(..., prepare_only=False)
  img_queues / numpy_inputs / full_input_queues

Never use the old _Small or _reset_small on native 0.11.2 modeld.
"""
from __future__ import annotations

import inspect
from typing import Callable


def require_offroad_probe() -> None:
  """Never start a Pixel diagnostic session during an onroad lifecycle.

  This is a second boundary beyond JoiningModelState(in_control=True):
  no automatic attach, no remote output publication, and no connection
  attempts if the IsOffroad state is absent or false.
  """
  from openpilot.common.params import Params
  if not Params().get_bool("IsOffroad"):
    raise RuntimeError("XNOR v0.11.2 Pixel probe requires confirmed offroad state")


def validate_native_small_model(small) -> None:
  """Fail closed unless the small model has the *0.11.2* modeld ABI."""
  if getattr(small, "chestnut", None) is not False:
    raise ValueError("Expected a resident native small model with chestnut=False")
  if tuple(getattr(small, "vision_input_names", ())) != ("img", "big_img"):
    raise ValueError("Native v0.11.2 needs road and wide frame inputs")
  run = getattr(small, "run", None)
  if not callable(run):
    raise ValueError("Missing native ModelState.run")
  try:
    sig = inspect.signature(run)
  except (TypeError, ValueError) as exc:
    raise ValueError("Cannot verify native small-model run signature") from exc
  params = tuple(sig.parameters)
  if len(params) < 4 or params[:4] != ("bufs", "transforms", "inputs", "after_enqueue"):
    raise ValueError(f"Expected native run(..., after_enqueue), found {sig}")
  if sig.parameters["after_enqueue"].default is not None:
    raise ValueError("Native after_enqueue must default to None")
  for name in ("state_pairs", "input_queues", "packed_input", "prev_desire"):
    if not hasattr(small, name):
      raise ValueError(f"Missing native v0.11.2 model state: {name}")
  if not set(small.state_pairs).issubset(small.input_queues):
    raise ValueError("Missing one or more recurrent state tensors")


def prepare_native_small_reset(small) -> Callable[[], None]:
  """Prepare a GPU-state reset *before* any attempted handover.

  Stock.small_reset uses Jetlink's reset_for_stock and caches its kernel.
  This avoids reloading/compiling or reassigning the native JIT's buffers
  on the first fallback frame. Not hardware verified yet.
  """
  validate_native_small_model(small)
  from jetlink.openpilot.stock import Stock
  callback = Stock().small_reset(small)
  if not callable(callback):
    raise RuntimeError("Native reset preparation returned no callable")
  return callback


def verify_native_model_face() -> None:
  """Ensure the staged native face references official v0.11.2 modeld."""
  from jetlink.openpilot.stock import Stock
  face = Stock().model_face()
  if not callable(face.get_action_from_model):
    raise ValueError("Native Jetlink face has no modeld action extractor")
  if face.desire_len != 8:
    raise ValueError(f"Unexpected native v0.11.2 desire length: {face.desire_len}")
  if face.long_smooth_seconds != 0.3:
    raise ValueError("Unexpected native v0.11.2 longitudinal smoothing")


def attach_offroad_probe(small, cam_w: int, cam_h: int):
  """Build a parked connectivity/ABI probe, with authority permanently disabled.

  Never calls the large model from modeld: JoiningModelState's swap guard is
  permanently True. It may contact the Pixel to test connectivity and engine
  spec while parked, but cannot publish the large model's predictions.
  Does NOT run on vehicle or register with manager.
  """
  require_offroad_probe()
  validate_native_small_model(small)
  if cam_w <= 0 or cam_h <= 0:
    raise ValueError("Invalid camera dimensions")

  from openpilot.jetlink_authority import _Progress, discover_phone, PORT, MODEL_SHA, MODEL_NBYTES
  from openpilot.jetlink_native_authority import (
    _pixel_transport, _model_spec_checks, ExistingCameraWarp,
    PIXEL_SERVER_TIMEOUT_S, JETLINK_FRAME_FAILURE_S,
  )
  from jetlink.client import JetlinkClient
  from jetlink.openpilot.joining import JoiningModelState
  from jetlink.openpilot.model_state import JetlinkModelState
  from jetlink.openpilot.stock import Stock

  stock = Stock()
  face = stock.model_face()
  reset_small = prepare_native_small_reset(small)
  warp = ExistingCameraWarp(cam_w, cam_h, face.frame_size(cam_w, cam_h))

  def connect(should_stop=None):
    if should_stop is not None and should_stop():
      raise InterruptedError("Native probe stopped")
    # A parked test may cross an offroad/onroad transition after it starts.
    # Recheck for every retry; no Pixel connection should begin onroad.
    require_offroad_probe()
    transport = _pixel_transport(discover_phone(), PORT)
    client = JetlinkClient(transport, deadline=JETLINK_FRAME_FAILURE_S,
                           name="XNOR-v0.11.2-Pixel-offroad", want_hidden=False)
    try:
      client.hello(timeout=3.0)
      if not client.t.datagrams:
        raise RuntimeError("Pixel did not negotiate USB datagrams")
      spec = client.ensure_engine(MODEL_SHA, MODEL_NBYTES, frame_skip=4,
                                  build_timeout=PIXEL_SERVER_TIMEOUT_S,
                                  should_stop=should_stop)
      _model_spec_checks(spec)
      return client, spec
    except Exception:
      client.close()
      raise

  def build(client, spec):
    _model_spec_checks(spec)
    return JetlinkModelState(client, spec, warp, face=face,
                             log=stock.log, event=stock.event)

  return JoiningModelState(
    small, connect, build,
    reset_small=reset_small,
    progress=_Progress(), log=stock.log,
    in_control=lambda: True,  # deliberately prevents *all* model swaps
  )
