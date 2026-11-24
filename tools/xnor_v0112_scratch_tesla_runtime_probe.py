#!/usr/bin/env python3
"""Isolated workstation-only Tesla interface construction probe for XNOR 0.11.2.

Requires a fully SCons-built SCRATCH openpilot checkout with the reviewed
opendbc/Panda patches already applied. Does not send CAN, start pandad,
enable model authority, connect to a device, or flash firmware.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile


def main() -> None:
  if Path("/AGNOS").exists() or Path("/data/openpilot").exists():
    raise RuntimeError("Only a disposable development workstation is supported")
  if not (Path("XNOR_HYBRID_ASSEMBLY_MANIFEST.json")).is_file():
    raise RuntimeError("Run only in the isolated assembled candidate checkout")
  manifest = json.loads(Path("XNOR_HYBRID_ASSEMBLY_MANIFEST.json").read_text())
  if manifest.get("vehicle_authority_qualified") is not False:
    raise RuntimeError("Unexpected privileged Hybrid assembly manifest")

  # Construct real XNOR CarState, CarController, CAN parser definitions, and
  # their legacy support modules while confining persistent Params to a
  # temporary local directory. Never modify /data/params or user selection.
  from openpilot.common import params as params_module
  from opendbc.car.tesla import carcontroller, carstate
  from opendbc.car.tesla.interface import CarInterface
  from opendbc.car.tesla.values import CAR, TeslaSafetyFlags

  original_params = params_module.Params
  with tempfile.TemporaryDirectory(prefix="xnor-0112-runtime-probe-") as tmp:
    def temporary_params():
      return original_params(tmp)

    params_module.Params = temporary_params
    carcontroller.Params = temporary_params
    carstate.Params = temporary_params
    try:
      results = {}
      for model in (CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_Y):
        cp = CarInterface.get_non_essential_params(model)
        if model == CAR.TESLA_MODEL_S_HW2:
          # Default config must NOT claim Hybrid authority merely because
          # openpilotLongitudinalControl is configured for the platform.
          assert all((x.safetyParam & int(TeslaSafetyFlags.OP_STALK_ENABLE)) == 0
                     for x in cp.safetyConfigs)
        ci = CarInterface(cp)
        assert ci.CS is not None and ci.CC is not None
        assert ci.can_parsers and all(p.dbc_name for p in ci.can_parsers.values())
        results[str(model)] = {
          "carstate": type(ci.CS).__name__,
          "carcontroller": type(ci.CC).__name__,
          "can_parser_count": len(ci.can_parsers),
          "hybrid_owner_authorized": False,
        }
      print(json.dumps(results, indent=2, sort_keys=True))
    finally:
      params_module.Params = original_params
      carcontroller.Params = original_params
      carstate.Params = original_params


if __name__ == "__main__":
  main()
