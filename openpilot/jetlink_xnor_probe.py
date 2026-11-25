"""Compatibility entry point for the final XNOR 0.11.1 Jetlink link probe."""
from migration.jetlink_adapters_legacy import jetlink_xnor_probe as _legacy

CONFIG = _legacy.CONFIG
ENABLED = _legacy.ENABLED
STATUS = _legacy.STATUS
LinkError = _legacy.LinkError
probe = _legacy.probe
run_once = _legacy.run_once
usb_tether_candidates = _legacy.usb_tether_candidates


def main() -> None:
  _legacy.main()


if __name__ == "__main__":
  main()
