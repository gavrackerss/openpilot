"""Integrated 0.11.2 Hybrid behavior port for offline review.

This module models the two cooperating legacy seams (selfdrived pedal event
classification and controlsd longitudinal latch) against a single snapshot.
It deliberately keeps native event observations separate from the reference
event stream and never modifies a live event list or CarControl. No runtime,
CAN, Params, messaging or hardware imports. Do not register with manager.
"""
from __future__ import annotations

from dataclasses import dataclass
from migration.tesla_v0112.hybrid_long_owner import (
    HybridLongInputs, HybridLongObservation, HybridLongState,
    step_hybrid_long_owner,
)


@dataclass(frozen=True)
class ReviewFrame:
  timestamp_ms: int
  brand: str = "tesla"
  pcm_cruise: bool = True
  op_long_available: bool = True
  enabled: bool = True
  lateral_eligible: bool = True
  brake: bool = False
  regen: bool = False
  gas: bool = False
  standstill: bool = False
  acc_fault: bool = False
  stock_aeb: bool = False
  can_valid: bool = True
  can_timeout: bool = False
  drive_gear: bool = True
  external_long_override: bool = False
  disengage_on_accelerator: bool = True
  physical_main_res: bool = False
  auto_resume_preference: bool = True
  experimental: bool = True


@dataclass(frozen=True)
class ReviewState:
  latch: HybridLongState = HybridLongState()
  previous_brake: bool = False
  previous_regen: bool = False
  previous_gas: bool = False
  previous_timestamp_ms: int = -1


@dataclass(frozen=True)
class ReviewObservation:
  state: ReviewState
  native_pedal_events: tuple[str, ...]
  legacy_reference_pedal_events: tuple[str, ...]
  reference_latch: HybridLongObservation
  native_enabled_observed: bool
  lateral_eligibility_observed: bool
  independent_owner_blockers: tuple[str, ...]
  runtime_activated: bool = False
  vehicle_qualified: bool = False


def observe_frame(state: ReviewState, frame: ReviewFrame) -> ReviewObservation:
  """Port the combined legacy behavior into an observation-only state machine.

  Caller supplies decoded, authenticated physical input observations, not
  synthetic UI buttons. The native engagement and lateral state are preserved
  as observations, never rewritten. Legacy events describe what the old
  implementation would have emitted, not events to publish.
  """
  if frame.timestamp_ms <= state.previous_timestamp_ms:
    raise ValueError("Review timestamps must strictly increase")
  gas_edge = frame.gas and not state.previous_gas and frame.disengage_on_accelerator
  brake_event = frame.brake and (not state.previous_brake or not frame.standstill)
  regen_event = frame.regen and (not state.previous_regen or not frame.standstill)
  native = ("pedalPressed",) if gas_edge or brake_event or regen_event else ()
  reference = ["pedalPressed"] if gas_edge else []
  if frame.brand == "tesla":
    if frame.brake or frame.regen:
      reference.append("gasPressedOverride")
  elif brake_event or regen_event:
    reference.append("pedalPressed")

  blockers = tuple(name for name, blocked in (
      ("invalid_can", not frame.can_valid), ("can_timeout", frame.can_timeout),
      ("not_drive", not frame.drive_gear), ("stock_aeb", frame.stock_aeb),
      ("acc_fault", frame.acc_fault), ("gas_pressed", frame.gas),
      ("external_long_override", frame.external_long_override),
  ) if blocked)
  # Preserve the exact old reference logic for parity, and report independent
  # safety blockers separately. Neither result grants any ownership.
  latch = step_hybrid_long_owner(state.latch, HybridLongInputs(
      now_ms=frame.timestamp_ms, is_tesla=frame.brand == "tesla",
      pcm_cruise=frame.pcm_cruise, openpilot_long_available=frame.op_long_available,
      enabled=frame.enabled, brake_pressed=frame.brake, regen_braking=frame.regen,
      gas_pressed=frame.gas, acc_faulted=frame.acc_fault,
      physical_main_res_pressed=frame.physical_main_res,
      auto_resume_acc_enabled=frame.auto_resume_preference,
      experimental_mode=frame.experimental,
      override_longitudinal=frame.external_long_override or "gasPressedOverride" in reference,
  ))
  next_state = ReviewState(latch.state, frame.brake, frame.regen, frame.gas, frame.timestamp_ms)
  return ReviewObservation(next_state, native, tuple(reference), latch,
                           frame.enabled, frame.lateral_eligible, blockers)
