"""Compatibility surface for the final XNOR 0.11.1 Jetlink benchmark helpers.

The full proven diagnostic implementation remains byte-preserved under
migration/jetlink_adapters_legacy. The live LS2 path only needs these helpers;
keeping the legacy module out of openpilot's current lint/type surface avoids
rewriting diagnostic code that is not part of model authority.
"""
from migration.jetlink_adapters_legacy.jetlink_shadow_bench import (
  PING,
  PONG,
  PipelinedInference,
  udp_sendmmsg,
  warp_indices,
)

__all__ = ["PING", "PONG", "PipelinedInference", "udp_sendmmsg", "warp_indices"]
