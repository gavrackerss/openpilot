#!/usr/bin/env python3
"""XNOR Jetlink V5.9.4: native JetlinkClient + JetlinkModelState, Pixel protocol-v3.

The native Jetlink frame lifecycle, hold accounting, parser, and Zoompilot
joining/demotion logic own timing. This module adds only the Pixel USB-tether
transport adapter and an adapter over the resident 0.11.2 modeld camera warp.
No vehicle control is written here; modeld remains the publisher and the
resident stock model remains the fallback.

Camera preprocessing deliberately stays on the comma and runs the exact
compiled ModelState.warp used by this checkout, matching Zoompilot/Clarity's
architecture. The old V5.8 CPU warp_indices implementation is not used for
authority.

Experimental integration. Real-road Pixel/reference parity must pass before
treating a model/runtime combination as qualified vehicle authority.
"""
from __future__ import annotations

import socket
from typing import Any

VERSION = '5.9.4-r1-native-modeld-warp-parity'
MODEL_W, MODEL_H = 512, 256
PIXEL_SERVER_TIMEOUT_S = 20.0
JETLINK_FRAME_FAILURE_S = 0.20  # Native Jetlink's existing peer-dead limit, NOT a hold budget.


def _model_spec_checks(spec: Any) -> None:
  """Use the existing V5.9.2 manifest validation, plus native client byte sizes."""
  from openpilot.jetlink_authority import validate_spec, MODEL_HW, REPLY_FLOATS
  validate_spec(spec.to_dict())
  if tuple(spec.model_hw) != MODEL_HW:
    raise ValueError(f'Pixel LS2 native model geometry changed: {spec.model_hw}')
  if spec.packed_nelem != 12:
    raise ValueError(f'Pixel LS2 packed scalars changed: {spec.packed_nelem}')
  if spec.reply_nelem != REPLY_FLOATS:
    raise ValueError(f'Pixel LS2 reply changed: {spec.reply_nelem}')
  if spec.warped_nbytes != 2 * 6 * MODEL_HW[0] * MODEL_HW[1]:
    raise ValueError(f'Pixel LS2 warped image byte count changed: {spec.warped_nbytes}')


def _pixel_transport(host: str, port: int):
  """Native TCP framing/receiver, with USB-tether detection and batched UDP.

  TcpTransport's default 'on cable' address is 192.168.60.1 (phone dials
  comma); our existing custom Android app is at 10.169.232.180 over usb0.
  Route verification is repeated by the connector BEFORE constructing this.
  """
  from jetlink import protocol as P
  from jetlink.transport.tcp import TcpTransport, datagrams
  from jetlink.transport.base import LinkError
  from openpilot.jetlink_shadow_bench import udp_sendmmsg

  class PixelUsbTransport(TcpTransport):
    def on_the_cable(self):
      # Caller checked the current route is usb0. This is a tether, not WiFi.
      return True

    def link_info(self):
      # Custom Android V5.8 expects kind=usb to advertise its UDP frame port.
      # Native Jetlink phone-dialled mode uses kind=cable instead; passing
      # that through here can silently suppress the phone's UDP offer.
      return {'kind': 'usb'}

    def send(self, msg_type: int, seq: int, parts=(), flags: int = 0,
             timeout: float | None = None) -> None:
      # The custom Pixel V5.8 server receives inference uploads over its
      # negotiated UDP frame channel. The generic Jetlink client normally
      # sends frame 1 over the control stream because there is no prior output
      # to hold yet; that ~393 KB TCP send can block until the 200 ms model
      # deadline and kill the first handover. Route every inference upload,
      # including frame 1, through the already-negotiated USB datagram path.
      if msg_type == P.Msg.INFER_REQ and self.datagrams:
        self.send_datagrams(msg_type, seq, parts, flags)
        return
      super().send(msg_type, seq, parts, flags, timeout)

    def send_datagrams(self, msg_type: int, seq: int, parts=(), flags: int = 0) -> None:
      if self._udp is None:
        raise LinkError('Pixel UDP not negotiated')
      udp, token = self._udp
      # Native tcp.datagrams reuses one mutable header; sendmmsg batches ALL
      # pieces, so copy each tiny header before yielding it to the native
      # syscall. Without this, every datagram could carry the LAST offset.
      pieces = []
      for group in datagrams(self._frame(msg_type, seq, parts, flags), token, seq):
        pieces.append([bytes(group[0]), *group[1:]])
      try:
        sent, _calls = udp_sendmmsg(udp, pieces)
      except (OSError, RuntimeError, ValueError) as exc:
        raise LinkError(f'Pixel USB datagram upload failed: {exc}') from exc
      if sent != len(pieces):
        raise LinkError(f'Partial Pixel datagram batch: {sent}/{len(pieces)}')

  return PixelUsbTransport.connect(host, port, timeout=3.0)




class NativeModeldWarp:
  """Run the exact compiled 0.11.2 modeld camera warp used by the resident model.

  Zoompilot/Clarity keep camera preprocessing on the comma and feed the remote
  model the byte output of openpilot's own driving warp. XNOR previously
  reimplemented that step with the V5.8 CPU warp_indices path. This adapter
  deliberately reuses ModelState.warp, its transform tensors, QCOM device and
  NV12 geometry so LS2 receives exactly the same current-frame tensor the
  local 0.11.2 modeld preprocessing would produce.
  """

  def __init__(self, model):
    import numpy as np

    required = ("warp", "WARP_DEV", "input_queues", "npy", "frame_buf_params")
    missing = [name for name in required if not hasattr(model, name)]
    if missing:
      raise RuntimeError(f"Resident ModelState cannot provide native camera warp: {missing}")
    for name in ("img", "big_img"):
      if name not in model.frame_buf_params:
        raise RuntimeError(f"Resident ModelState has no {name} NV12 geometry")
    self._model = model
    self._frame_size = {
      name: int(model.frame_buf_params[name][3]) for name in ("img", "big_img")
    }
    self._blob_cache = {}
    self.output = np.empty((2, 6, 128, 256), dtype=np.uint8)

  def _frame_tensor(self, name: str, address: int):
    from tinygrad.tensor import Tensor

    if not isinstance(address, int) or address <= 0:
      raise ValueError(f"Invalid {name} camera address")
    key = (name, address)
    tensor = self._blob_cache.get(key)
    if tensor is None:
      tensor = Tensor.from_blob(address, (self._frame_size[name],),
                                dtype="uint8", device=self._model.WARP_DEV)
      self._blob_cache[key] = tensor
    return tensor

  def start(self, frame: int, big_frame: int, tfm, big_tfm) -> None:
    import numpy as np
    from openpilot.selfdrive.modeld.compile_modeld import WARP_INPUTS

    tfm = np.asarray(tfm, dtype=np.float32)
    big_tfm = np.asarray(big_tfm, dtype=np.float32)
    if tfm.shape != (3, 3) or big_tfm.shape != (3, 3):
      raise ValueError("Invalid modeld camera transform shape")
    if not np.isfinite(tfm).all() or not np.isfinite(big_tfm).all():
      raise ValueError("Non-finite modeld camera transform")

    # These are the exact NPY-backed transform tensors ModelState.run uses.
    self._model.npy["tfm"][:, :] = tfm
    self._model.npy["big_tfm"][:, :] = big_tfm
    warped = self._model.warp(
      **{k: self._model.input_queues[k] for k in WARP_INPUTS},
      frame=self._frame_tensor("img", frame),
      big_frame=self._frame_tensor("big_img", big_frame),
    )

    # numpy() synchronizes the compiled QCOM warp before Jetlink sends it.
    # Keep a stable host buffer because the transport may retain views until
    # infer_begin returns.
    value = warped.numpy()
    if value.shape != self.output.shape or value.dtype != np.uint8:
      raise RuntimeError(f"Native modeld warp returned {value.shape} {value.dtype}")
    np.copyto(self.output, value)

  def wait(self) -> None:
    # start() synchronizes through warped.numpy().
    return None

def attach_native(small_model, cam_w: int, cam_h: int, *, runner: str = 'stock'):
  """Stage a native Jetlink model; Zoompilot owns all swaps and demotions."""
  from openpilot.jetlink_authority import (_Small, _Control, _Progress,
                                             _reset_small, discover_phone,
                                             PORT, MODEL_SHA, MODEL_NBYTES)
  from openpilot.common.swaglog import cloudlog
  from jetlink.client import JetlinkClient
  from jetlink.openpilot.joining import JoiningModelState
  from jetlink.openpilot.model_state import JetlinkModelState
  from jetlink.openpilot.stock import Stock

  # Stock is the only runner whose modeld interface has been checked here.
  if runner != 'stock':
    raise RuntimeError(f'Native LS2 adapter not qualified for local runner {runner!r}')
  stock = Stock()
  face = stock.model_face()
  # Match Zoompilot/Clarity's architecture: preprocessing stays on the comma
  # and uses the fork's own compiled modeld warp. No parallel V5.8 warp math.
  warp = NativeModeldWarp(small_model)

  def connect(should_stop=None):
    if should_stop is not None and should_stop():
      raise InterruptedError('Zoompilot join stopped')
    host = discover_phone()  # current phone USB-NCM gateway, verified usb0
    transport = _pixel_transport(host, PORT)
    client = JetlinkClient(transport, deadline=JETLINK_FRAME_FAILURE_S,
                           name='XNOR-TensorG6-native-LS2', want_hidden=False)
    try:
      hello = client.hello(timeout=3.0)
      if not client.t.datagrams:
        raise RuntimeError('Custom Pixel server did not negotiate USB UDP frames')
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
    # Native JetlinkModelState owns prepare->send->end, output parsing,
    # HOLD_FRAME measured from warp start, and _note_hold().
    return JetlinkModelState(client, spec, warp, face=face,
                             log=cloudlog, event=cloudlog.event)

  cloudlog.warning('xnor-jetlink: native Jetlink client + exact 0.11.2 modeld warp staged; stock fallback retained; parity validation required')
  return JoiningModelState(_Small(small_model, normalize_stock_views=True), connect, build,
                           reset_small=lambda: _reset_small(small_model),
                           progress=_Progress(), log=cloudlog,
                           in_control=_Control.in_control,
                           name=lambda sha: 'XNOR LS2 Tensor G6 (native Jetlink)')
