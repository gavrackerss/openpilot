#!/usr/bin/env python3
"""LS2 USB-NCM gateway regression. Pure stdlib; no comma/Pixel required."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace

from openpilot.jetlink_usb_peer import UsbPeerError, discover_ls2_host, verify_usb_route


def route(*args, **kwargs):
    cmd = args[0]
    if cmd[-2:] == ["get", "10.169.232.180"]:
        return SimpleNamespace(stdout="10.169.232.180 via 10.162.48.109 dev usb0 src 10.162.48.71 uid 1000")
    if cmd[-2:] == ["get", "10.162.48.109"]:
        return SimpleNamespace(stdout="10.162.48.109 dev usb0 src 10.162.48.71 uid 1000")
    raise AssertionError(f"Unexpected routing command: {cmd}")


class USBDiscovery(unittest.TestCase):
    @patch.dict("os.environ", {"JETLINK_LS2_HOST": ""})
    def test_dynamic_pixel_gateway_from_actual_log(self):
        with patch("openpilot.jetlink_usb_peer.subprocess.run", side_effect=route):
            self.assertEqual(discover_ls2_host(), "10.162.48.109")

    @patch.dict("os.environ", {"JETLINK_LS2_HOST": "10.162.48.109"})
    def test_explicit_verified_usb_host(self):
        with patch("openpilot.jetlink_usb_peer.subprocess.run", side_effect=route):
            self.assertEqual(discover_ls2_host(), "10.162.48.109")

    @patch.dict("os.environ", {"JETLINK_LS2_HOST": "10.162.48.109"})
    def test_reject_non_usb_connection(self):
        with patch("openpilot.jetlink_usb_peer.subprocess.run", return_value=SimpleNamespace(
            stdout="10.162.48.109 via 192.168.1.1 dev wlan0 src 192.168.1.3"
        )):
            with self.assertRaisesRegex(UsbPeerError, "not routed via usb0"):
                discover_ls2_host()

    @patch.dict("os.environ", {"JETLINK_LS2_HOST": "999.999.1.1"})
    def test_reject_bad_override(self):
        with self.assertRaises(UsbPeerError):
            discover_ls2_host()

    @patch.dict("os.environ", {"JETLINK_LS2_HOST": ""})
    def test_refuse_guessed_phone_when_no_usb0_gateway(self):
        def no_gateway(*args, **kwargs):
            cmd = args[0]
            if "get" in cmd:
                return SimpleNamespace(stdout="10.169.232.180 via 192.168.0.1 dev wlan0 src 192.168.0.5")
            return SimpleNamespace(stdout="10.160.0.0/24 proto kernel scope link src 10.160.0.4 dev usb0")
        with patch("openpilot.jetlink_usb_peer.subprocess.run", side_effect=no_gateway):
            with self.assertRaisesRegex(UsbPeerError, "No verified Pixel gateway"):
                discover_ls2_host()

    @patch.dict("os.environ", {"JETLINK_LS2_HOST": ""})
    def test_fallback_to_usb0_route_table(self):
        def fallback(*args, **kwargs):
            cmd = args[0]
            if cmd[-2:] == ["get", "10.169.232.180"]:
                return SimpleNamespace(stdout="10.169.232.180 dev wlan0 src 192.168.0.3")
            if cmd[-2:] == ["get", "10.40.1.1"]:
                return SimpleNamespace(stdout="10.40.1.1 dev usb0 src 10.40.1.17")
            if cmd[-4:] == ["show", "default", "dev", "usb0"]:
                return SimpleNamespace(stdout="default via 10.40.1.1 dev usb0 metric 50")
            raise AssertionError(cmd)
        with patch("openpilot.jetlink_usb_peer.subprocess.run", side_effect=fallback):
            self.assertEqual(discover_ls2_host(), "10.40.1.1")


if __name__ == "__main__":
    unittest.main()
