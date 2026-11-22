#!/usr/bin/env python3
"""Original Cinque Terre native Zoompilot adapter for guarded stock modeld.

Production modeld may *select* the CTM joiner only with independently supplied
and verified, matching qualification evidence and a separate explicit arm.
No qualification/arm files are created here. Default is LOCAL STOCK.

The same Jetlink model state is used for isolated parked tests and production.
The Pixel's PolicyQueues (not comma) owns image, desire and feature history.
Zoompilot owns joining, late-frame holding, demotion and local fallback.
Never replace LS2 identity/output assumptions with CTM's SHA.
"""
from __future__ import annotations

import ipaddress
import os
from pathlib import Path
import subprocess
import sys
from typing import Any

from openpilot.jetlink_ctm_authority_gate import (
    Gate, eligible as authority_eligible, lease_alive, requested as authority_requested,
)
from openpilot.jetlink_ctm_contract import (
    CTM_VERSION, SOURCE_SHA, SOURCE_NBYTES, ContractError,
    validate_spec, validate_native_spec, validate_parsed,
)

VERSION = "ctm-v0.1-native-queued-parked-only"
PRODUCTION_VERSION = "ctm-v0.2-stock-modeld-guarded"
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


def requested() -> bool:
    """Operator specifically selected CTM. Invalid requests block LS2 fallback."""
    return authority_requested()


def authority_gate() -> Gate:
    """Full startup approval/evidence check, never creates a marker."""
    return authority_eligible()


def enabled() -> bool:
    """CTM is off by default; presence of an arm marker alone is insufficient."""
    return authority_gate().allowed


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


def _attach_joiner(small_model, cam_w: int, cam_h: int, *,
                   runner: str, production: bool, approval: Gate | None = None):
    """Shared pinned CTM transport/decoder with two independent call-site gates.

    Production is never available to isolated parked tests and vice versa.
    No Tensor G6 firmware, steering or Android changes are made here.
    """
    if production:
        if approval is None or not lease_alive(approval):
            raise RuntimeError("CTM authority lease missing or revoked")
    else:
        _check_parked_test_context()
    if runner != "stock":
        raise RuntimeError("CTM joiner currently supports stock Python modeld only")
    if not (cam_w > 0 and cam_h > 0):
        raise ValueError("Invalid CTM camera dimensions")

    from openpilot.jetlink_authority import _Small, _Control, _Progress, _reset_small
    from openpilot.jetlink_native_authority import ExistingCameraWarp, _pixel_transport
    from openpilot.common.swaglog import cloudlog
    from jetlink.client import JetlinkClient
    from jetlink.openpilot.joining import JoiningModelState, LOST
    from jetlink.openpilot.model_state import JetlinkModelState
    from jetlink.openpilot.stock import Stock

    class CTMModelState(JetlinkModelState):
        def _outputs(self, model_output):
            decoded = super()._outputs(model_output)
            validate_parsed(decoded)
            return decoded

    class GuardedCTMJoiner(JoiningModelState):
        """Arm revocation on the frame thread demotes before processing CTM.

        Do not wait for a background lease poll: losing arm/approval, adding
        STOP, or arming LS2 must immediately return LOCAL outputs. Cleanup
        of the joined client is deferred off modeld's frame thread.
        """
        _ctm_authority_selected = True
        _ctm_model_sha = SOURCE_SHA

        def run(self, bufs, transforms, inputs, after_enqueue=None):
            if getattr(self, "_ctm_revoked", False) or not lease_alive(approval):
                if not getattr(self, "_ctm_revoked", False):
                    self._ctm_revoked = True
                    self._log.error("CTM authority gate revoked: reverting to local stock")
                    if self._active is not self._small:
                        self._demote(LOST)
                    self._stop.set()
                    self._rejoin.set()
                    # A join may already hold a socket. Do not do blocking
                    # session cleanup on modeld's realtime inference thread.
                    import threading
                    threading.Thread(target=self.close, daemon=True,
                                     name="ctm-revoked-cleanup").start()
                self._small_frames += 1
                return self._small.run(bufs, transforms, inputs, after_enqueue)
            return super().run(bufs, transforms, inputs, after_enqueue)

    face = Stock().model_face()
    warp = ExistingCameraWarp(cam_w, cam_h, face.frame_size(cam_w, cam_h))

    def check_allowed():
        if production:
            if not lease_alive(approval):
                raise RuntimeError("CTM authority revoked before Jetlink connection")
        else:
            _check_parked_test_context()

    def connect(should_stop=None):
        if should_stop is not None and should_stop():
            raise InterruptedError("CTM join thread stopped")
        check_allowed()
        host = discover_phone()
        transport = _pixel_transport(host, PORT)
        client = JetlinkClient(
            transport, deadline=DEADLINE_S,
            name=("XNOR-CTM-G6-authority" if production else "XNOR-CTM-G6-native-parked"),
            want_hidden=False,
        )
        try:
            client.hello(timeout=3.0)
            if not client.t.datagrams:
                raise RuntimeError("Pixel did not offer USB UDP datagrams; install Jetlink v0.7")
            spec = client.ensure_engine(
                SOURCE_SHA, SOURCE_NBYTES, frame_skip=4,
                build_timeout=BUILD_TIMEOUT_S, should_stop=should_stop,
            )
            try:
                validate_native_spec(spec)
            except ContractError:
                # Save the phone's *actual* input/output graph when mismatch.
                # No LS2-compatible shape is invented on a failed contract.
                if not production:
                    try:
                        import json
                        out = Path("/data/jetlink_ctm_park/actual_engine_spec.json")
                        out.parent.mkdir(parents=True, exist_ok=True)
                        out.write_text(json.dumps(spec.to_dict(), indent=2) + "\n")
                    except (OSError, ValueError, TypeError):
                        pass
                raise
            check_allowed()
            return client, spec
        except Exception:
            client.close()
            raise

    def build(client, spec):
        check_allowed()
        validate_native_spec(spec)
        return CTMModelState(client, spec, warp, face=face,
                             log=cloudlog, event=cloudlog.event)

    joiner = (GuardedCTMJoiner if production else JoiningModelState)(
        _Small(small_model, normalize_stock_views=True), connect, build,
        reset_small=lambda: _reset_small(small_model),
        progress=_Progress(), log=cloudlog, in_control=_Control.in_control,
        name=lambda sha: (
            "Cinque Terre Original Tensor G6 [guarded stock modeld]"
            if production else "Cinque Terre Original Tensor G6 [parked test only]"
        ),
    )
    cloudlog.warning(
        "CTM native queued joiner attached in %s mode; local stock is fallback",
        "guarded_authority" if production else "isolated_parked",
    )
    return joiner


def attach_parked(small_model, cam_w: int, cam_h: int, *, runner: str = "stock"):
    """Same actual CTM engine, but disconnected from production modeld."""
    return _attach_joiner(small_model, cam_w, cam_h,
                          runner=runner, production=False)


def attach_authority(small_model=None, cam_w: int = 0, cam_h: int = 0, *,
                     runner: str = "stock"):
    """Production CTM modeld entry point; always fail closed without evidence.

    This is NOT an authorization grant. Missing parity, shadow or fallback
    qualification documents must leave local modeld authoritative.
    """
    if os.environ.get("JETLINK_CTM_ISOLATED_PARKED_TEST") == "1":
        raise RuntimeError("CTM authority unavailable in isolated parked test")
    approval = authority_gate()
    if not approval.allowed:
        raise RuntimeError(f"CTM authority NOT QUALIFIED: {approval.reason}")
    if small_model is None:
        raise RuntimeError("CTM authority requires a real resident local model")
    return _attach_joiner(small_model, cam_w, cam_h,
                          runner=runner, production=True, approval=approval)
