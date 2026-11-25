"""Live import surface for the final XNOR 0.11.1 USB-peer discovery logic."""
from migration.jetlink_adapters_legacy.jetlink_usb_peer import (
  LEGACY_LS2_HOST,
  USB_INTERFACE,
  UsbPeerError,
  discover_ls2_host,
  verify_usb_route,
)

__all__ = [
  "LEGACY_LS2_HOST",
  "USB_INTERFACE",
  "UsbPeerError",
  "discover_ls2_host",
  "verify_usb_route",
]
