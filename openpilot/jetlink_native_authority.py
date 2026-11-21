#!/usr/bin/env python3
"""XNOR Jetlink V5.9.3: native JetlinkClient + JetlinkModelState, Pixel protocol-v3.

The native Jetlink frame lifecycle, hold accounting, parser, and Zoompilot
joining/demotion logic own timing. This module adds only the Pixel USB-tether
transport adapter and a verified precompiled driving warp. No vehicle control
is written here; modeld remains the publisher and stock the fallback.

Requires a prebuilt, validated native driving_warp_*_tinygrad.pkl. If missing,
attach() raises and modeld's existing catch retains the local stock model.
"""
from __future__ import annotations

import socket
from typing import Any

VERSION = '5.9.3-xnor-native-jetlink-ls2'
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


def attach_native(small_model, cam_w: int, cam_h: int, *, runner: str = 'stock'):
  """Stage a native Jetlink model; Zoompilot owns all swaps and demotions."""
  from openpilot.jetlink_authority import (_Small, _Control, _Progress,
                                             _reset_small, tether_route_valid,
                                             HOST, PORT, MODEL_SHA, MODEL_NBYTES)
  from openpilot.common.swaglog import cloudlog
  from jetlink.client import JetlinkClient
  from jetlink.openpilot.joining import JoiningModelState
  from jetlink.openpilot.model_state import JetlinkModelState
  from jetlink.openpilot.stock import Stock
  from jetlink.openpilot.warp import Warps, Warp

  # Only stock has been exercised with this adapter. Do not activate an
  # untested runner, and never substitute a CPU warp on the frame thread.
  if runner != 'stock':
    raise RuntimeError(f'Native LS2 adapter not qualified for local runner {runner!r}')

  stock = Stock()
  face = stock.model_face()
  # Demand the *native* GPU warp artifact before joining. No dynamic kernel
  # compile, and no silent fallback to the old synchronous CPU NumPy warp.
  cached = Warps(stock).load(int(cam_w), int(cam_h), MODEL_W, MODEL_H)
  warp = Warp(cached, face.frame_size(int(cam_w), int(cam_h)))

  def connect(should_stop=None):
    if should_stop is not None and should_stop():
      raise InterruptedError('Zoompilot join stopped')
    if not tether_route_valid():
      raise ConnectionError('Pixel must be reachable via verified usb0 route')
    transport = _pixel_transport(HOST, PORT)
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

  cloudlog.warning('xnor-jetlink: native Pixel LS2 adapter staged; stock fallback retained')
  return JoiningModelState(_Small(small_model, normalize_stock_views=True), connect, build,
                           reset_small=lambda: _reset_small(small_model),
                           progress=_Progress(), log=cloudlog,
                           in_control=_Control.in_control,
                           name=lambda sha: 'XNOR LS2 Tensor G6 (native Jetlink)')
