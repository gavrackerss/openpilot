#!/usr/bin/env python3
"""CTM-specific native Zoompilot model adapter, staged for ISOLATED PARKED TESTS.

This is the *real* JetlinkClient -> JetlinkModelState -> JoiningModelState path.
The phone's PolicyQueues, not this adapter, owns CTM's img/big_img/
desire_pulse/features_buffer history. Reuses the LS2-tested USB transport,
frame warp, local-small reset and Zoompilot's fallback lifecycle.

NOT wired into production modeld. Not a modelV2 publisher. No Params/CAN/steering
writes. attach_parked() can only be called by the isolated parked test.
Authority deliberately has no entry point until output parity, fallback,
camera fidelity and on-road shadow qualification have been independently
validated. Never substitute CTM's hash into the LS2 authority module.
"""
from __future__ import annotations

import ipaddress
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

from openpilot.jetlink_ctm_contract import (
    CTM_VERSION, SOURCE_SHA, SOURCE_NBYTES, ContractError,
    validate_spec, validate_native_spec, validate_parsed,
)

VERSION = "ctm-v0.1-native-queued-parked-only"
PORT = 5599
PROBE_TARGET = "10.169.232.180"  # route probe ONLY, never a fixed phone endpoint
CONFIG = Path("/data/jetlink_xnor")
STAGED = CONFIG / "CTM_INTEGRATION_STAGED"
PARKED = CONFIG / "CTM_PARKED_TEST_ENABLED"
NOT_QUALIFIED = CONFIG / "CTM_AUTHORITY_NOT_QUALIFIED"
ARM = CONFIG / "AUTHORITY_ENABLED"
QUALIFIED = CONFIG / "LS2_AUTHORITY_QUALIFIED"
CTM_QUALIFIED = CONFIG / "CTM_AUTHORITY_QUALIFIED"
CTM_ARM = CONFIG / "CTM_AUTHORITY_ENABLED"
DEADLINE_S = 0.20
BUILD_TIMEOUT_S = 20.0


def _route(target: str) -> list[str]:
    """Require a kernel-confirmed usb0 route to the selected IPv4 endpoint."""
    try:
        output = subprocess.run(
            ["ip", "-4", "route", "get", target], check=True,
            capture_output=True, text=True, timeout=2.0,
        ).stdout.strip().split()
    except (OSError, subprocess.SubprocessError) as exc:
        raise ConnectionError(f"CTM cannot inspect USB route: {exc}") from exc
    if "dev" not in output or output[output.index("dev") + 1] != "usb0":
        raise ConnectionError(f"CTM endpoint is not on usb0: {' '.join(output)}")
    return output


def discover_phone() -> str:
    """Pick the current Android USB tether peer, never assume a stale 10.x IP.

    Override via JETLINK_CTM_HOST only if this address *also* routes over usb0.
    Without an override, choose the verified usb0 next hop. A network with
    no next-hop cannot be auto-discovered without an explicit phone address.
    """
    configured = os.environ.get("JETLINK_CTM_HOST", "").strip()
    if configured:
        try:
            peer = str(ipaddress.IPv4Address(configured))
        except ipaddress.AddressValueError as exc:
            raise ConnectionError("JETLINK_CTM_HOST must be an IPv4 address") from exc
        _route(peer)
        return peer
    tokens = _route(PROBE_TARGET)
    if "via" not in tokens:
        raise ConnectionError("No usb0 gateway; explicitly set JETLINK_CTM_HOST")
    peer = tokens[tokens.index("via") + 1]
    peer = str(ipaddress.IPv4Address(peer))
    _route(peer)
    return peer


def tether_route_valid() -> bool:
    try:
        discover_phone()
        return True
    except (ConnectionError, ValueError, IndexError):
        return False


def enabled() -> bool:
    """Production modeld must NEVER enable CTM through the parked adapter."""
    return False


def remote_active(model) -> bool:
    return bool(getattr(model, "chestnut", False))


def _check_parked_test_context() -> None:
    if os.environ.get("JETLINK_CTM_ISOLATED_PARKED_TEST") != "1":
        raise RuntimeError("CTM joining is allowed only from isolated non-publishing parked test")
    if not ("openpilot.jetlink_ctm_park_handover" in sys.argv[0]
            or Path(sys.argv[0]).name == "jetlink_ctm_park_handover.py"):
        raise RuntimeError("CTM staged adapter cannot be used from production modeld")
    if not STAGED.is_file() or STAGED.read_text().strip() != SOURCE_SHA:
        raise RuntimeError("Exact original CTM source not staged")
    if not PARKED.is_file() or not NOT_QUALIFIED.is_file():
        raise RuntimeError("CTM must be staged as parked-only and NOT qualified")
    for flag in (ARM, QUALIFIED, CTM_QUALIFIED, CTM_ARM):
        if flag.exists():
            raise RuntimeError(f"Remote authority marker present: {flag}; refusing parked CTM adapter")
    if not tether_route_valid():
        raise ConnectionError("CTM Pixel USB tether not reachable via usb0")


def attach_parked(small_model, cam_w: int, cam_h: int, *, runner: str = "stock"):
    """Build a REAL Zoompilot joiner in a *non-publishing* parked test process.

    The car/Panda state is checked on every frame by the calling parked test,
    which also overrides _ask_in_control with the same freshly verified state.
    """
    _check_parked_test_context()
    if runner != "stock":
        raise RuntimeError("CTM parked native adapter currently supports stock modeld only")
    if not (cam_w > 0 and cam_h > 0):
        raise ValueError("Invalid CTM camera dimensions")

    from openpilot.jetlink_authority import _Small, _Control, _Progress, _reset_small
    from openpilot.jetlink_native_authority import ExistingCameraWarp, _pixel_transport
    from openpilot.common.swaglog import cloudlog
    from jetlink.client import JetlinkClient
    from jetlink.openpilot.joining import JoiningModelState
    from jetlink.openpilot.model_state import JetlinkModelState
    from jetlink.openpilot.stock import Stock

    # The output decoder is the existing local modeld Parser, and shape checks
    # execute BEFORE a result can be used by the test's action calculation.
    class CTMModelState(JetlinkModelState):
        def _outputs(self, model_output):
            decoded = super()._outputs(model_output)
            validate_parsed(decoded)
            return decoded

    face = Stock().model_face()
    warp = ExistingCameraWarp(cam_w, cam_h, face.frame_size(cam_w, cam_h))

    def connect(should_stop=None):
        if should_stop is not None and should_stop():
            raise InterruptedError("CTM join thread stopped")
        _check_parked_test_context()
        host = discover_phone()
        transport = _pixel_transport(host, PORT)
        client = JetlinkClient(transport, deadline=DEADLINE_S,
                               name="XNOR-CTM-G6-native-parked", want_hidden=False)
        try:
            client.hello(timeout=3.0)
            if not client.t.datagrams:
                raise RuntimeError("Pixel did not offer USB UDP datagrams; use Jetlink v0.7")
            spec = client.ensure_engine(
                SOURCE_SHA, SOURCE_NBYTES, frame_skip=4,
                build_timeout=BUILD_TIMEOUT_S, should_stop=should_stop,
            )
            try:
                validate_native_spec(spec)
            except ContractError:
                # Exact CTM output_slices were not present in the original
                # preparation manifest. Preserve the *actual* live ENGINE_RESP
                # for diagnosis, instead of guessing or coercing LS2's layout.
                try:
                    import json
                    out = Path("/data/jetlink_ctm_park/actual_engine_spec.json")
                    out.parent.mkdir(parents=True, exist_ok=True)
                    out.write_text(json.dumps(spec.to_dict(), indent=2) + "\n")
                except (OSError, ValueError, TypeError):
                    pass
                raise
            return client, spec
        except Exception:
            client.close()
            raise

    def build(client, spec):
        validate_native_spec(spec)
        return CTMModelState(client, spec, warp, face=face,
                             log=cloudlog, event=cloudlog.event)

    cloudlog.warning("CTM native queued Jetlink parked adapter staged; local stock controls remain authoritative")
    return JoiningModelState(
        _Small(small_model, normalize_stock_views=True), connect, build,
        reset_small=lambda: _reset_small(small_model),
        progress=_Progress(), log=cloudlog, in_control=_Control.in_control,
        name=lambda sha: "Cinque Terre Original Tensor G6 [parked test only]",
    )


def attach_authority(*_args: Any, **_kwargs: Any):
    """Fail closed: this commit does not qualify or wire CTM steering authority."""
    raise RuntimeError(
        "CTM authority is NOT QUALIFIED. Adapter exists for isolated parked "
        "Zoompilot handover and fault-injection tests only; validate CTM "
        "output parity, trajectory semantics and fallback before modeld wiring."
    )
