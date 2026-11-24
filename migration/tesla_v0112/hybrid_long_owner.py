"""Staged Tesla Hybrid LONG brake-owner state machine for XNOR 0.11.2.

A pure functional extraction of the installed 24d304b controls V207/V234
gate. It does NOT import opendbc, read CAN, write CarControl, send Panda
frames, or attach to the native controlsd.

This is a behavioural *reference* for offline replay only. It is NOT
qualified to resume longitudinal driving. Any future runtime integration
requires the original user/car/Panda safety ownership to be reconciled
and hardware independent tests before activation.
"""
from __future__ import annotations

from dataclasses import dataclass

AUTO_RESUME_DELAY_MS = 1_000


@dataclass(frozen=True)
class HybridLongInputs:
  now_ms: int
  is_tesla: bool
  pcm_cruise: bool
  openpilot_long_available: bool
  enabled: bool
  brake_pressed: bool
  regen_braking: bool
  gas_pressed: bool
  acc_faulted: bool
  physical_main_res_pressed: bool
  auto_resume_acc_enabled: bool
  experimental_mode: bool
  override_longitudinal: bool


@dataclass(frozen=True)
class HybridLongState:
  cancelled_by_brake: bool = False
  last_brake_or_regen_ms: int = 0


@dataclass(frozen=True)
class HybridLongObservation:
  state: HybridLongState
  would_allow_long: bool
  transition: str
  # This is only a model of desired ownership; never an ECU acknowledgement.
  ecu_acceptance_verified: bool = False
  vehicle_authority_qualified: bool = False


def step_hybrid_long_owner(state: HybridLongState, data: HybridLongInputs) -> HybridLongObservation:
  """Calculate the 24d304b logical latch for replay, with no side effects.

  Caller must supply an authenticated *physical* MAIN/RES observation.
  A UI speed-up, virtual cruise button or model action must never be
  substituted for physical_main_res_pressed.
  """
  if not isinstance(data.now_ms, int) or data.now_ms < 0:
    raise ValueError("Expected a nonnegative monotonic timestamp in milliseconds")
  hybrid = bool(data.is_tesla and data.pcm_cruise and data.openpilot_long_available)
  next_state = state
  event = "hold"

  if not hybrid:
    next_state = HybridLongState()
    event = "non_hybrid"
  elif not data.enabled:
    next_state = HybridLongState()
    event = "disengaged"
  elif data.brake_pressed:
    next_state = HybridLongState(True, data.now_ms)
    event = "brake_cancel"
  elif state.cancelled_by_brake:
    last_brake = state.last_brake_or_regen_ms
    if data.regen_braking:
      last_brake = data.now_ms

    if not data.regen_braking and data.physical_main_res_pressed:
      next_state = HybridLongState(False, last_brake)
      event = "manual_resume"
    elif (not data.regen_braking and data.auto_resume_acc_enabled and
          data.experimental_mode and not data.gas_pressed and
          not data.acc_faulted and last_brake > 0 and
          data.now_ms - last_brake >= AUTO_RESUME_DELAY_MS):
      next_state = HybridLongState(False, last_brake)
      event = "auto_resume"
    else:
      next_state = HybridLongState(True, last_brake)
      event = "latched"

  would_allow = bool(
    data.enabled and data.openpilot_long_available and
    not data.override_longitudinal and
    not data.brake_pressed and not data.regen_braking and
    not next_state.cancelled_by_brake
  )
  return HybridLongObservation(next_state, would_allow, event)
