#!/usr/bin/env python3
"""Read-only LS2 authority readiness diagnosis; does NOT connect to Pixel.

This deliberately does not touch markers, model selection or manager. A TCP
probe can compete with the Jetlink app's live server and is not attempted.
"""
from pathlib import Path
import json
import subprocess

from openpilot.jetlink_usb_peer import discover_ls2_host, UsbPeerError

ROOT = Path("/data/jetlink_xnor")


def read(name: str) -> bool:
    return (ROOT / name).exists()


def main():
    arm = read("AUTHORITY_ENABLED")
    ls2 = read("LS2_AUTHORITY_QUALIFIED")
    blockers = [n for n in ("CTM_PARKED_TEST_ENABLED",
                            "CTM_AUTHORITY_ENABLED",
                            "ONROAD_SHADOW_ENABLED") if read(n)]
    info = {
        "authority_requested": arm,
        "ls2_qualified_marker": ls2,
        "old_ctm_parked_blocker": read("CTM_PARKED_TEST_ENABLED"),
        "other_session_markers": blockers,
        "ls2_selection_eligible": arm and ls2 and not read("CTM_PARKED_TEST_ENABLED") and not read("CTM_AUTHORITY_ENABLED"),
        "ls2_source_sha256": "909a140019050a9d7f00db22dd4f4085928d2af9474a82244672ddab964f95a8",
        "android_server": "requires LS2 Prepared and Enable Jetlink TCP",
    }
    try:
        info["verified_usb_pixel_host"] = discover_ls2_host()
        info["port"] = 5599
    except UsbPeerError as exc:
        info["usb_route_error"] = str(exc)
    try:
        from openpilot.common.params import Params
        raw = Params().get("ModelRunnerTypeCache")
        info["model_runner_cache"] = str(raw)
    except Exception as exc:
        info["model_runner_cache_error"] = f"{type(exc).__name__}: {exc}"
    print(json.dumps(info, indent=2))
    if not info["ls2_selection_eligible"]:
        print("BLOCKED: missing authority/LS2 qualification, stale CTM parked marker, or CTM authority request.")
    elif "verified_usb_pixel_host" not in info:
        print("BLOCKED: no verified USB tether route. Reconnect tether or configure JETLINK_LS2_HOST.")
    else:
        print("LS2 marker/route checks pass. If not connected, inspect modeld logs for "
              "'jetlink: not joined yet'; verify stock modeld, LS2 Prepared in Pixel, "
              "and TCP server listening on 5599.")


if __name__ == "__main__":
    main()
