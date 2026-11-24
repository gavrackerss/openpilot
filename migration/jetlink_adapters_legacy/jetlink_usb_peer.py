#!/usr/bin/env python3
"""Discover a Pixel USB-tether peer without trusting stale 10.x addresses.

Pure stdlib; never changes host networking, Panda, Jetlink markers or modeld.
A candidate MUST route over the comma's usb0 interface. An optional
JETLINK_LS2_HOST value may override discovery, subject to that same check.
"""
from __future__ import annotations

import ipaddress
import os
import subprocess


LEGACY_LS2_HOST = "10.169.232.180"
USB_INTERFACE = "usb0"


class UsbPeerError(ConnectionError):
    pass


def _route(*arguments: str) -> str:
    try:
        result = subprocess.run(
            ["ip", "-4", "route", *arguments],
            capture_output=True, text=True, check=True, timeout=2,
        )
        return result.stdout.strip()
    except (OSError, subprocess.SubprocessError) as exc:
        raise UsbPeerError(f"Cannot inspect USB-tether route: {exc}") from exc


def _ipv4(candidate: str) -> str:
    try:
        return str(ipaddress.IPv4Address(candidate))
    except ipaddress.AddressValueError as exc:
        raise UsbPeerError(f"Invalid IPv4 address in Jetlink USB route: {candidate!r}") from exc


def _interface(tokens: list[str]) -> str | None:
    return tokens[tokens.index("dev") + 1] if "dev" in tokens and len(tokens) > tokens.index("dev") + 1 else None


def _next_hop(output: str) -> str | None:
    for line in output.splitlines():
        tokens = line.split()
        if _interface(tokens) != USB_INTERFACE:
            continue
        if "via" in tokens and len(tokens) > tokens.index("via") + 1:
            return _ipv4(tokens[tokens.index("via") + 1])
    return None


def verify_usb_route(host: str) -> str:
    host = _ipv4(host)
    route = _route("get", host).split()
    if _interface(route) != USB_INTERFACE:
        raise UsbPeerError(f"Jetlink peer {host} is not routed via {USB_INTERFACE}: {' '.join(route)}")
    return host


def discover_ls2_host() -> str:
    """Prefer the existing tether gateway; never guess an IP from 10.0.0.0/8.

    The pre-v0.7 LS2 address is a routing *probe*, not the default destination.
    If it no longer has a usb0 route, inspect only usb0's own gateway entries.
    If neither reveals an unambiguous peer, require explicit JETLINK_LS2_HOST.
    """
    configured = os.environ.get("JETLINK_LS2_HOST", "").strip()
    if configured:
        return verify_usb_route(configured)

    try:
        route = _route("get", LEGACY_LS2_HOST)
        candidate = _next_hop(route)
        if candidate is not None:
            return verify_usb_route(candidate)
        tokens = route.split()
        if _interface(tokens) == USB_INTERFACE:
            return verify_usb_route(LEGACY_LS2_HOST)
    except UsbPeerError:
        pass

    # A newly tethered interface may have a different subnet entirely.
    # Only an explicit usb0 gateway qualifies; never select an arbitrary
    # ARP neighbour or guess .1/.2.
    for args in [("show", "default", "dev", USB_INTERFACE), ("show", "dev", USB_INTERFACE)]:
        try:
            candidate = _next_hop(_route(*args))
            if candidate is not None:
                return verify_usb_route(candidate)
        except UsbPeerError:
            continue

    raise UsbPeerError(
        "No verified Pixel gateway on usb0; reconnect USB tethering, "
        "or set JETLINK_LS2_HOST to its actual IPv4 address (usb0 route required)."
    )
