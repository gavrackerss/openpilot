#!/usr/bin/env python3
"""Non-actuating native Jetlink Pixel protocol check. No camera or controls.

OFFROAD ONLY. Tests native JetlinkClient against custom protocol-v3 phone and
validates the actual compressed output layout. Does not touch authority flags
or send Openpilot messages.
"""
from __future__ import annotations

import os
from pathlib import Path
import statistics
import time


def main():
  if Path('/data/jetlink_xnor/AUTHORITY_ENABLED').exists() or Path('/data/jetlink_xnor/LS2_AUTHORITY_QUALIFIED').exists():
    raise RuntimeError('STOP: remove both authority markers before a parked protocol test')
  from cereal import messaging
  sm=messaging.SubMaster(['deviceState'])
  sm.update(1000)
  if not sm.seen['deviceState'] or not sm.alive['deviceState'] or sm['deviceState'].started:
    raise RuntimeError('STOP: live deviceState started=False is required')
  import numpy as np
  from jetlink.client import JetlinkClient
  from openpilot.jetlink_authority import MODEL_SHA, MODEL_NBYTES, HOST, PORT, tether_route_valid
  from openpilot.jetlink_native_authority import _pixel_transport, _model_spec_checks
  if not tether_route_valid():
    raise RuntimeError('STOP: Pixel not routed via usb0')
  t=_pixel_transport(HOST,PORT)
  client=JetlinkClient(t,deadline=0.20,name='XNOR-native-protocol-offroad-test',want_hidden=False)
  try:
    client.hello(timeout=3.0)
    if not client.t.datagrams:
      raise RuntimeError('STOP: UDP frame channel was not negotiated')
    spec=client.ensure_engine(MODEL_SHA,MODEL_NBYTES,frame_skip=4,build_timeout=20)
    _model_spec_checks(spec)
    image=np.zeros((2,6,*spec.model_hw),dtype=np.uint8)
    packed=np.array([0]*8+[1,0]+[0.25,0.25],dtype='<f4')
    timings=[]
    for n in range(1,41):
      start=time.perf_counter()
      seq=client.infer_begin(image,packed,n,reset=n==1,want_state=False)
      out=client.infer_end(seq)
      elapsed=(time.perf_counter()-start)*1000
      assert out.shape==(18452,), out.shape
      assert np.isfinite(out).all()
      timings.append(elapsed)
    timings.sort()
    print('PASS native Jetlink protocol-v3: 40/40 finite LS2 replies')
    print(f'Sequential request p50={statistics.median(timings):.2f}ms p95={timings[int(.95*(len(timings)-1))]:.2f}ms')
    print('This is a protocol check, not proof of authoritative frame cadence.')
  finally:
    client.close()


if __name__=='__main__':
  main()
