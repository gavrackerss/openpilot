#!/usr/bin/env python3
"""XNOR Jetlink V5.9.3: native JetlinkClient + JetlinkModelState, Pixel protocol-v3.

The native Jetlink frame lifecycle, hold accounting, parser, and Zoompilot
joining/demotion logic own timing. This module adds only the Pixel USB-tether
transport adapter and a verified precompiled driving warp. No vehicle control
is written here; modeld remains the publisher and stock the fallback.

This adapter reuses the established V5.8/V5.9.2 CPU NV12 warp. It requires
NO separately compiled driving_warp pickle, Tinygrad ABI adapter, or QCOM
warp builder. The native Zoompilot client, joining and demotion remain.

Experimental/offroad qualification only. Do not use as vehicle authority
without independent live camera/scheduling/fallback validation.
"""
from __future__ import annotations

import socket
from typing import Any

VERSION = '5.9.3-r4-native-jetlink-v58-camera-warp'
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




class ExistingCameraWarp:
  """Adapter for the V5.8/V5.9.2 NV12 preprocessor already used on real frames.

  JetlinkModelState needs start(address,address,matrix,matrix), wait(), output.
  The proven _CameraWarp instead accepts camera buffer views and matrices.
  This thin adapter copies neither full camera frames nor the warped output.
  It deliberately does NOT invoke any Tinygrad/JIT build or GPU buffer API.
  """

  def __init__(self, cam_w: int, cam_h: int, frame_size: int):
    from openpilot.jetlink_authority import _CameraWarp
    import numpy as np
    self._cam_w, self._cam_h = int(cam_w), int(cam_h)
    self._frame_size = int(frame_size)
    if self._cam_w <= 0 or self._cam_h <= 0 or self._frame_size <= 0:
      raise ValueError('Invalid camera dimensions or NV12 buffer size')
    self._camera = _CameraWarp(self._cam_w, self._cam_h)
    self.output = self._camera.output
    assert self.output.shape == (2, 6, 128, 256) and self.output.dtype == np.uint8

  def start(self, frame: int, big_frame: int, tfm, big_tfm) -> None:
    import ctypes
    from types import SimpleNamespace
    # modeld's native _address() returns the NV12 buffer address, which
    # remains owned and valid by VisionIPC during this synchronous operation.
    if not isinstance(frame, int) or not isinstance(big_frame, int) or frame <= 0 or big_frame <= 0:
      raise ValueError('Invalid native camera addresses')
    frame_data = (ctypes.c_uint8 * self._frame_size).from_address(frame)
    big_data = (ctypes.c_uint8 * self._frame_size).from_address(big_frame)
    self._camera.run(
      {'img': SimpleNamespace(data=frame_data, width=self._cam_w, height=self._cam_h),
       'big_img': SimpleNamespace(data=big_data, width=self._cam_w, height=self._cam_h)},
      {'img': tfm, 'big_img': big_tfm})

  def wait(self) -> None:
    # The legacy CPU warp is synchronous: output is ready on return of start.
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
  # Existing V5.8/V5.9.2 preprocessing from the real onroad shadow tests.
  # Deliberately avoid the incompatible custom QCOM warp and its pickle.
  warp = ExistingCameraWarp(int(cam_w), int(cam_h),
                            face.frame_size(int(cam_w), int(cam_h)))

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

  cloudlog.warning('xnor-jetlink: native Jetlink client + V5.8 camera warp staged; stock fallback retained; offroad-only validation pending')
  return JoiningModelState(_Small(small_model, normalize_stock_views=True), connect, build,
                           reset_small=lambda: _reset_small(small_model),
                           progress=_Progress(), log=cloudlog,
                           in_control=_Control.in_control,
                           name=lambda sha: 'XNOR LS2 Tensor G6 (native Jetlink)')
