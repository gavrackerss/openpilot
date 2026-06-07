#!/usr/bin/env python3
# /data/openpilot/tools/clear_path_watch.py
from __future__ import annotations

import argparse
import json
import math
import re
import signal
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from cereal import messaging

RUNNING = True

MPH_PER_MPS = 2.2369362920544
STEER_ANGLE_SATURATION_THRESHOLD_DEG = 2.5
TESLA_LEGACY_STEER_RATIO = 16.5
TESLA_LEGACY_WHEELBASE_M = 2.96
TESLA_LEGACY_SLIP_FACTOR = -0.00075
TESLA_LEGACY_STEER_COMMAND_HZ = 50.0
PANDA_VM_MAX_LATERAL_ACCEL = 3.0 + (9.81 * 0.06)
PANDA_VM_MAX_LATERAL_JERK = 3.0 + (9.81 * 0.06)

CRUISE_SYNC_RE = re.compile(
  r"\[XNOR_CRUISE_SYNC\]\s+"
  r"src=(?P<src>.*?)\s+"
  r"uom=(?P<uom>\S+)\s+"
  r"tgt=(?P<tgt>-?\d+(?:\.\d+)?)\s+"
  r"cur=(?P<cur>-?\d+(?:\.\d+)?)\s+"
  r"est=(?P<est>-?\d+(?:\.\d+)?)\s+"
  r"btn=(?P<btn>-?\d+)\s+"
  r"reason=(?P<reason>.*)$"
)
CC_DIAG_RE = re.compile(r"\[XNOR_CC_DIAG\]\s+gate=(?P<gate>\S+)(?:\s+(?P<rest>.*))?$")
KEYVAL_RE = re.compile(r"(?P<key>[a-zA-Z_][a-zA-Z0-9_]*)=(?P<value>[^\s]+)")


def _sig_handler(signum, frame) -> None:
  del signum, frame
  global RUNNING
  RUNNING = False


def _now_ms() -> int:
  return time.monotonic_ns() // 1_000_000


def _wall_iso() -> str:
  return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def _safe_float(value: Any, default: float = 0.0) -> float:
  try:
    v = float(value)
  except Exception:
    return float(default)
  return float(v) if math.isfinite(v) else float(default)


def _safe_int(value: Any, default: int = 0) -> int:
  try:
    return int(value)
  except Exception:
    return int(default)


def _safe_bool(value: Any) -> bool:
  try:
    return bool(value)
  except Exception:
    return False


def _safe_str(value: Any, default: str = "") -> str:
  if value is None:
    return default
  try:
    return str(value)
  except Exception:
    return default


def _take_numeric(seq: Any, limit: int) -> list[float]:
  out: list[float] = []
  try:
    items = list(seq)[:limit] if seq is not None else []
  except Exception:
    return out
  for item in items:
    v = _safe_float(item, float("nan"))
    if math.isfinite(v):
      out.append(float(v))
  return out


def _maybe_attr(obj: Any, name: str, default: Any = None) -> Any:
  try:
    return getattr(obj, name)
  except Exception:
    return default


def _enum_name(value: Any) -> str:
  try:
    name = getattr(value, "name")
    if name is not None:
      return str(name)
  except Exception:
    pass
  try:
    return str(value).split(".")[-1]
  except Exception:
    return ""


def _button_events_summary(cs: Any) -> list[dict[str, Any]]:
  out: list[dict[str, Any]] = []
  try:
    events = list(_maybe_attr(cs, "buttonEvents", []) or [])
  except Exception:
    return out
  for event in events[-12:]:
    event_type = _maybe_attr(event, "type", None)
    out.append({
      "type": _enum_name(event_type),
      "pressed": _safe_bool(_maybe_attr(event, "pressed", False)),
      "rawType": _safe_str(event_type),
    })
  return out


def _interesting_named_fields(obj: Any, tokens: tuple[str, ...]) -> dict[str, Any]:
  out: dict[str, Any] = {}
  if obj is None:
    return out

  try:
    raw = obj.to_dict()
  except Exception:
    raw = None
  if isinstance(raw, dict):
    for key, value in raw.items():
      key_s = str(key)
      if any(token in key_s.lower() for token in tokens):
        if isinstance(value, (bool, int, float, str)):
          out[key_s] = value
    return out

  for name in dir(obj):
    if name.startswith("_"):
      continue
    name_l = name.lower()
    if not any(token in name_l for token in tokens):
      continue
    try:
      value = getattr(obj, name)
    except Exception:
      continue
    if callable(value):
      continue
    if isinstance(value, (bool, int, float, str)):
      out[str(name)] = value
  return out


def _lead_summary(lead: Any) -> dict[str, Any]:
  if lead is None:
    return {"status": False}
  return {
    "status": _safe_bool(_maybe_attr(lead, "status", False)),
    "dRel": _safe_float(_maybe_attr(lead, "dRel", 0.0)),
    "yRel": _safe_float(_maybe_attr(lead, "yRel", 0.0)),
    "vRel": _safe_float(_maybe_attr(lead, "vRel", 0.0)),
    "aRel": _safe_float(_maybe_attr(lead, "aRel", 0.0)),
    "vLead": _safe_float(_maybe_attr(lead, "vLead", 0.0)),
    "vLeadK": _safe_float(_maybe_attr(lead, "vLeadK", 0.0)),
    "aLeadK": _safe_float(_maybe_attr(lead, "aLeadK", 0.0)),
    "dPath": _safe_float(_maybe_attr(lead, "dPath", 0.0)),
    "vLat": _safe_float(_maybe_attr(lead, "vLat", 0.0)),
    "aLeadTau": _safe_float(_maybe_attr(lead, "aLeadTau", 0.0)),
    "modelProb": _safe_float(_maybe_attr(lead, "modelProb", 0.0)),
    "prob": _safe_float(_maybe_attr(lead, "prob", 0.0)),
    "radar": _safe_bool(_maybe_attr(lead, "radar", False)),
    "fcw": _safe_bool(_maybe_attr(lead, "fcw", False)),
    "source": _safe_str(_maybe_attr(lead, "source", "")),
    "trackId": _safe_int(_maybe_attr(lead, "trackId", -1), -1),
  }


def _actuators_summary(actuators: Any) -> dict[str, Any]:
  if actuators is None:
    return {}
  return {
    "steeringAngleDeg": _safe_float(_maybe_attr(actuators, "steeringAngleDeg", 0.0)),
    "curvature": _safe_float(_maybe_attr(actuators, "curvature", 0.0)),
    "torque": _safe_float(_maybe_attr(actuators, "torque", 0.0)),
    "accel": _safe_float(_maybe_attr(actuators, "accel", 0.0)),
    "gas": _safe_float(_maybe_attr(actuators, "gas", 0.0)),
    "brake": _safe_float(_maybe_attr(actuators, "brake", 0.0)),
    "torqueOutputCan": _safe_float(_maybe_attr(actuators, "torqueOutputCan", 0.0)),
    "speed": _safe_float(_maybe_attr(actuators, "speed", 0.0)),
    "longControlState": _safe_str(_maybe_attr(actuators, "longControlState", "")),
  }


def _car_control_summary(cc: Any) -> dict[str, Any]:
  if cc is None:
    return {}
  return {
    "enabled": _safe_bool(_maybe_attr(cc, "enabled", False)),
    "latActive": _safe_bool(_maybe_attr(cc, "latActive", False)),
    "longActive": _safe_bool(_maybe_attr(cc, "longActive", False)),
    "currentCurvature": _safe_float(_maybe_attr(cc, "currentCurvature", 0.0)),
    "actuators": _actuators_summary(_maybe_attr(cc, "actuators", None)),
  }


def _car_output_summary(co: Any) -> dict[str, Any]:
  if co is None:
    return {}
  return {
    "actuatorsOutput": _actuators_summary(_maybe_attr(co, "actuatorsOutput", None)),
  }


def _live_parameters_summary(lp: Any) -> dict[str, Any]:
  if lp is None:
    return {}
  return {
    "valid": _safe_bool(_maybe_attr(lp, "valid", False)),
    "steerRatio": _safe_float(_maybe_attr(lp, "steerRatio", 0.0)),
    "stiffnessFactor": _safe_float(_maybe_attr(lp, "stiffnessFactor", 0.0)),
    "angleOffsetDeg": _safe_float(_maybe_attr(lp, "angleOffsetDeg", 0.0)),
    "angleOffsetAverageDeg": _safe_float(_maybe_attr(lp, "angleOffsetAverageDeg", 0.0)),
    "roll": _safe_float(_maybe_attr(lp, "roll", 0.0)),
  }


def _tesla_legacy_panda_vm_limits(v_ego: float) -> dict[str, float]:
  speed = max(_safe_float(v_ego, 0.0) - 1.0, 1.0)
  curvature_factor = 1.0 / (1.0 - (TESLA_LEGACY_SLIP_FACTOR * speed * speed)) / TESLA_LEGACY_WHEELBASE_M

  max_curvature = PANDA_VM_MAX_LATERAL_ACCEL / (speed * speed)
  max_angle = math.degrees(max_curvature * TESLA_LEGACY_STEER_RATIO / curvature_factor)

  max_curvature_rate_sec = PANDA_VM_MAX_LATERAL_JERK / (speed * speed)
  max_angle_rate_sec = math.degrees(max_curvature_rate_sec * TESLA_LEGACY_STEER_RATIO / curvature_factor)
  max_angle_delta = max_angle_rate_sec / TESLA_LEGACY_STEER_COMMAND_HZ

  return {
    "speedForLimit": speed,
    "maxAngleDeg": float(max_angle),
    "maxDeltaDeg": float(max_angle_delta),
    "maxLateralAccel": PANDA_VM_MAX_LATERAL_ACCEL,
    "maxLateralJerk": PANDA_VM_MAX_LATERAL_JERK,
  }


def _steer_diag_summary(
  car: dict[str, Any],
  controls: dict[str, Any],
  car_control: dict[str, Any],
  car_output: dict[str, Any],
  live_parameters: dict[str, Any],
  previous_output_angle: float | None,
) -> dict[str, Any]:
  actuators = car_control.get("actuators", {}) if isinstance(car_control.get("actuators"), dict) else {}
  output_actuators = car_output.get("actuatorsOutput", {}) if isinstance(car_output.get("actuatorsOutput"), dict) else {}
  lateral = controls.get("lateralState", {}) if isinstance(controls.get("lateralState"), dict) else {}

  v_ego = _safe_float(car.get("vEgo"), 0.0)
  desired_angle = _safe_float(actuators.get("steeringAngleDeg"), 0.0)
  output_angle = _safe_float(output_actuators.get("steeringAngleDeg"), 0.0)
  actual_angle = _safe_float(car.get("steeringAngleDeg"), 0.0)
  previous_angle = output_angle if previous_output_angle is None else float(previous_output_angle)

  desired_curvature = _first_nonzero_float(
    controls.get("desiredCurvature"),
    lateral.get("desiredCurvature"),
    actuators.get("curvature"),
  )
  actual_curvature = _first_nonzero_float(
    controls.get("curvature"),
    lateral.get("actualCurvature"),
    car_control.get("currentCurvature"),
  )

  clipped_speed = max(v_ego, 0.3)
  desired_lateral_accel = desired_curvature * clipped_speed * clipped_speed
  actual_lateral_accel = actual_curvature * clipped_speed * clipped_speed
  lateral_accel_ratio = abs(desired_lateral_accel) / max(abs(actual_lateral_accel), 1e-3)

  desired_minus_output = desired_angle - output_angle
  output_minus_actual = output_angle - actual_angle
  desired_minus_actual = desired_angle - actual_angle
  output_step = output_angle - previous_angle

  panda = _tesla_legacy_panda_vm_limits(v_ego)
  steer_limited_by_safety = abs(desired_minus_output) > STEER_ANGLE_SATURATION_THRESHOLD_DEG
  angle_state_saturated = _safe_bool(lateral.get("saturated", False))
  turning = abs(desired_lateral_accel) > 1.0
  undershooting = lateral_accel_ratio > 1.2

  reasons: list[str] = []
  if steer_limited_by_safety:
    reasons.append("desired_output_mismatch")
  if angle_state_saturated:
    reasons.append("lat_saturated")
  if undershooting and turning:
    reasons.append("lat_accel_undershoot")
  if abs(output_step) > (_safe_float(panda.get("maxDeltaDeg"), 0.0) + 0.2):
    reasons.append("panda_vm_delta_risk")
  if abs(output_angle) > (_safe_float(panda.get("maxAngleDeg"), 0.0) + 0.5):
    reasons.append("panda_vm_angle_risk")
  if abs(output_minus_actual) > STEER_ANGLE_SATURATION_THRESHOLD_DEG:
    reasons.append("actual_angle_lag")

  return {
    "active": _safe_bool(car_control.get("latActive", False)) or _safe_bool(lateral.get("active", False)),
    "vEgo": v_ego,
    "mph": v_ego * MPH_PER_MPS,
    "desiredAngleDeg": desired_angle,
    "outputAngleDeg": output_angle,
    "actualAngleDeg": actual_angle,
    "previousOutputAngleDeg": previous_angle,
    "desiredMinusOutputDeg": desired_minus_output,
    "outputMinusActualDeg": output_minus_actual,
    "desiredMinusActualDeg": desired_minus_actual,
    "outputStepDeg": output_step,
    "actualSteeringRateDeg": _safe_float(car.get("steeringRateDeg"), 0.0),
    "desiredCurvature": desired_curvature,
    "actualCurvature": actual_curvature,
    "desiredLateralAccel": desired_lateral_accel,
    "actualLateralAccel": actual_lateral_accel,
    "lateralAccelRatioAbs": lateral_accel_ratio,
    "turning": turning,
    "undershooting": undershooting,
    "steerLimitedBySafety": steer_limited_by_safety,
    "lateralSaturated": angle_state_saturated,
    "pandaLegacyVm": panda,
    "liveParameters": live_parameters,
    "reason": ",".join(reasons) if reasons else "-",
  }


def _car_state_summary(cs: Any) -> dict[str, Any]:
  cruise = _maybe_attr(cs, "cruiseState", None)
  out = {
    "vEgo": _safe_float(_maybe_attr(cs, "vEgo", 0.0)),
    "aEgo": _safe_float(_maybe_attr(cs, "aEgo", 0.0)),
    "standstill": _safe_bool(_maybe_attr(cs, "standstill", False)),
    "gasPressed": _safe_bool(_maybe_attr(cs, "gasPressed", False)),
    "brakePressed": _safe_bool(_maybe_attr(cs, "brakePressed", False)),
    "pedalOverride": _safe_bool(_maybe_attr(cs, "pedalOverride", False)),
    "gas": _safe_float(_maybe_attr(cs, "gas", 0.0)),
    "brake": _safe_float(_maybe_attr(cs, "brake", 0.0)),
    "steeringAngleDeg": _safe_float(_maybe_attr(cs, "steeringAngleDeg", 0.0)),
    "steeringRateDeg": _safe_float(_maybe_attr(cs, "steeringRateDeg", 0.0)),
    "steeringTorque": _safe_float(_maybe_attr(cs, "steeringTorque", 0.0)),
    "steeringTorqueEps": _safe_float(_maybe_attr(cs, "steeringTorqueEps", 0.0)),
    "steeringPressed": _safe_bool(_maybe_attr(cs, "steeringPressed", False)),
    "yawRate": _safe_float(_maybe_attr(cs, "yawRate", 0.0)),
    "leftBlinker": _safe_bool(_maybe_attr(cs, "leftBlinker", False)),
    "rightBlinker": _safe_bool(_maybe_attr(cs, "rightBlinker", False)),
    "speedLimit": _safe_float(_maybe_attr(cs, "speedLimit", 0.0)),
    "speedLimitOffset": _safe_float(_maybe_attr(cs, "speedLimitOffset", 0.0)),
    "cruiseButtons": _safe_int(_maybe_attr(cs, "cruiseButtons", 0), 0),
    "cruiseButtonsCounter": _safe_int(_maybe_attr(cs, "cruiseButtonsCounter", 0), 0),
    "stockCruiseState": _safe_str(_maybe_attr(cs, "stockCruiseState", _maybe_attr(cs, "stock_cruise_state", ""))),
    "stockCruiseEnabled": _safe_bool(_maybe_attr(cs, "stockCruiseEnabled", False)),
    "stockCruiseAvailable": _safe_bool(_maybe_attr(cs, "stockCruiseAvailable", False)),
    "followDistanceS": _safe_int(_maybe_attr(cs, "followDistanceS", 255), 255),
    "followDistance": _safe_float(_maybe_attr(cs, "followDistance", 0.0)),
    "cruiseGap": _safe_float(_maybe_attr(cs, "cruiseGap", 0.0)),
    "distanceSetting": _safe_float(_maybe_attr(cs, "distanceSetting", 0.0)),
    "accDistance": _safe_float(_maybe_attr(cs, "accDistance", 0.0)),
    "stockFollowDistance": _safe_float(_maybe_attr(cs, "stockFollowDistance", 0.0)),
    "teslaFollowDistance": _safe_float(_maybe_attr(cs, "teslaFollowDistance", 0.0)),
    "gapAdjustCruiseTr": _safe_float(_maybe_attr(cs, "gapAdjustCruiseTr", 0.0)),
    "accFollowDistance": _safe_float(_maybe_attr(cs, "accFollowDistance", 0.0)),
    "followTime": _safe_float(_maybe_attr(cs, "followTime", 0.0)),
    "followTimeGap": _safe_float(_maybe_attr(cs, "followTimeGap", 0.0)),
    "modeSel": _safe_float(_maybe_attr(cs, "modeSel", 0.0)),
    "accMode": _safe_float(_maybe_attr(cs, "accMode", 0.0)),
    "gapSetting": _safe_float(_maybe_attr(cs, "gapSetting", 0.0)),
    "timeGap": _safe_float(_maybe_attr(cs, "timeGap", 0.0)),
    "apFollowDistance": _safe_float(_maybe_attr(cs, "apFollowDistance", 0.0)),
    "apFollowTime": _safe_float(_maybe_attr(cs, "apFollowTime", 0.0)),
    "followDistanceStock": _safe_float(_maybe_attr(cs, "followDistanceStock", 0.0)),
    "stockGap": _safe_float(_maybe_attr(cs, "stockGap", 0.0)),
    "teslaGap": _safe_float(_maybe_attr(cs, "teslaGap", 0.0)),
    "dasFollowDistance": _safe_float(_maybe_attr(cs, "dasFollowDistance", 0.0)),
    "dasFollowTime": _safe_float(_maybe_attr(cs, "dasFollowTime", 0.0)),
    "DAS_followDistance": _safe_float(_maybe_attr(cs, "DAS_followDistance", 0.0)),
    "DAS_timeGap": _safe_float(_maybe_attr(cs, "DAS_timeGap", 0.0)),
    "cruiseFollowDistance": _safe_float(_maybe_attr(cs, "cruiseFollowDistance", 0.0)),
    "cruiseTimeGap": _safe_float(_maybe_attr(cs, "cruiseTimeGap", 0.0)),
    "longitudinalControlGap": _safe_float(_maybe_attr(cs, "longitudinalControlGap", 0.0)),
    "buttonEvents": _button_events_summary(cs),
    "rawGapFields": _interesting_named_fields(cs, ("gap", "follow", "distance", "timegap", "modesel")),
  }
  if cruise is not None:
    out["cruiseState"] = {
      "enabled": _safe_bool(_maybe_attr(cruise, "enabled", False)),
      "available": _safe_bool(_maybe_attr(cruise, "available", False)),
      "standstill": _safe_bool(_maybe_attr(cruise, "standstill", False)),
      "speed": _safe_float(_maybe_attr(cruise, "speed", 0.0)),
      "speedCluster": _safe_float(_maybe_attr(cruise, "speedCluster", 0.0)),
      "modeSel": _safe_float(_maybe_attr(cruise, "modeSel", 0.0)),
      "gap": _safe_float(_maybe_attr(cruise, "gap", 0.0)),
      "followDistance": _safe_float(_maybe_attr(cruise, "followDistance", 0.0)),
      "distanceSetting": _safe_float(_maybe_attr(cruise, "distanceSetting", 0.0)),
      "timeGap": _safe_float(_maybe_attr(cruise, "timeGap", 0.0)),
      "followTime": _safe_float(_maybe_attr(cruise, "followTime", 0.0)),
      "cruiseGap": _safe_float(_maybe_attr(cruise, "cruiseGap", 0.0)),
      "gapSetting": _safe_float(_maybe_attr(cruise, "gapSetting", 0.0)),
      "accDistance": _safe_float(_maybe_attr(cruise, "accDistance", 0.0)),
      "rawGapFields": _interesting_named_fields(cruise, ("gap", "follow", "distance", "timegap", "modesel")),
    }
  return out


def _controls_state_summary(cs: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  for name in (
    "enabled",
    "active",
    "vCruise",
    "vCruiseCluster",
    "longControlState",
    "forceDecel",
    "alertSize",
    "alertStatus",
    "curvature",
    "desiredCurvature",
  ):
    value = _maybe_attr(cs, name, None)
    if isinstance(value, (bool, int, str)):
      out[name] = value
    elif value is not None:
      out[name] = _safe_float(value, 0.0)

  lateral_state = _maybe_attr(cs, "lateralControlState", None)
  if lateral_state is not None:
    state_name = ""
    try:
      state_name = lateral_state.which()
    except Exception:
      state_name = ""
    out["lateralStateName"] = _safe_str(state_name)
    if state_name:
      lat_state = _maybe_attr(lateral_state, state_name, None)
      if lat_state is not None:
        out["lateralState"] = {
          "saturated": _safe_bool(_maybe_attr(lat_state, "saturated", False)),
          "steeringAngleDeg": _safe_float(_maybe_attr(lat_state, "steeringAngleDeg", 0.0)),
          "output": _safe_float(_maybe_attr(lat_state, "output", 0.0)),
          "desiredCurvature": _safe_float(_maybe_attr(lat_state, "desiredCurvature", 0.0)),
          "actualCurvature": _safe_float(_maybe_attr(lat_state, "actualCurvature", 0.0)),
        }
  return out


def _plan_summary(lp: Any) -> dict[str, Any]:
  speeds = _take_numeric(_maybe_attr(lp, "speeds", None), 33)
  accels = _take_numeric(_maybe_attr(lp, "accels", None), 16)
  jerks = _take_numeric(_maybe_attr(lp, "jerks", None), 16)

  p_last = float(speeds[-1]) if speeds else None
  near_window = max(3, int(math.ceil(len(speeds) * 0.35))) if speeds else 0
  preview_window = max(5, int(math.ceil(len(speeds) * 0.70))) if speeds else 0
  p_near = min(speeds[:near_window]) if near_window > 0 else None
  p_preview = min(speeds[:preview_window]) if preview_window > 0 else None

  return {
    "hasLead": _safe_bool(_maybe_attr(lp, "hasLead", False)),
    "aTarget": _safe_float(_maybe_attr(lp, "aTarget", 0.0)),
    "vTarget": _safe_float(_maybe_attr(lp, "vTarget", 0.0)),
    "vCruise": _safe_float(_maybe_attr(lp, "vCruise", 0.0)),
    "vCruiseCluster": _safe_float(_maybe_attr(lp, "vCruiseCluster", 0.0)),
    "desiredTF": _safe_float(_maybe_attr(lp, "desiredFollowDistance", 0.0)),
    "longitudinalPlanSource": _safe_str(_maybe_attr(lp, "longitudinalPlanSource", "")),
    "xState": _safe_str(_maybe_attr(lp, "xState", "")),
    "trafficState": _safe_str(_maybe_attr(lp, "trafficState", "")),
    "fcw": _safe_bool(_maybe_attr(lp, "fcw", False)),
    "speeds": speeds,
    "accels": accels,
    "jerks": jerks,
    "p_last": p_last,
    "p_near": p_near,
    "p_preview": p_preview,
  }


def _mapd_summary(mo: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  for name in (
    "suggestedSpeed",
    "speedLimit",
    "nextSpeedLimit",
    "distToSpeedLimit",
    "curveSpeed",
    "visionCurveSpeed",
    "mapCurveSpeed",
  ):
    value = _maybe_attr(mo, name, None)
    if value is None:
      continue
    out[name] = _safe_float(value, 0.0)

  for name in ("speedLimitControlState", "source"):
    value = _maybe_attr(mo, name, None)
    if value is not None:
      out[name] = _safe_str(value)

  out["rawRoadFields"] = _interesting_named_fields(
    mo,
    ("road", "route", "way", "junction", "roundabout", "intersection", "turn", "maneuver", "type", "class"),
  )
  return out


def _first_nonzero_float(*values: Any) -> float:
  for value in values:
    out = _safe_float(value, 0.0)
    if abs(out) > 0.001:
      return float(out)
  return 0.0


def _follow_gap_summary(car: dict[str, Any], plan: dict[str, Any], lead1: dict[str, Any], lead2: dict[str, Any]) -> dict[str, Any]:
  cruise = car.get("cruiseState", {}) if isinstance(car.get("cruiseState"), dict) else {}
  primary = lead1 if bool(lead1.get("status")) else lead2
  v_ego = _safe_float(car.get("vEgo"), 0.0)
  d_rel = _safe_float(primary.get("dRel"), 0.0)
  actual_gap_s = d_rel / max(v_ego, 0.1) if d_rel > 0.1 and v_ego > 0.1 else 0.0

  raw_candidates = {
    "followDistanceS": float(_safe_int(car.get("followDistanceS"), 255)) if 0 <= _safe_int(car.get("followDistanceS"), 255) <= 6 else 0.0,
    "followDistance": _safe_float(car.get("followDistance"), 0.0),
    "cruiseGap": _safe_float(car.get("cruiseGap"), 0.0),
    "distanceSetting": _safe_float(car.get("distanceSetting"), 0.0),
    "accDistance": _safe_float(car.get("accDistance"), 0.0),
    "stockFollowDistance": _safe_float(car.get("stockFollowDistance"), 0.0),
    "teslaFollowDistance": _safe_float(car.get("teslaFollowDistance"), 0.0),
    "gapAdjustCruiseTr": _safe_float(car.get("gapAdjustCruiseTr"), 0.0),
    "accFollowDistance": _safe_float(car.get("accFollowDistance"), 0.0),
    "followTime": _safe_float(car.get("followTime"), 0.0),
    "followTimeGap": _safe_float(car.get("followTimeGap"), 0.0),
    "modeSel": _safe_float(car.get("modeSel"), 0.0),
    "accMode": _safe_float(car.get("accMode"), 0.0),
    "gapSetting": _safe_float(car.get("gapSetting"), 0.0),
    "timeGap": _safe_float(car.get("timeGap"), 0.0),
    "apFollowDistance": _safe_float(car.get("apFollowDistance"), 0.0),
    "apFollowTime": _safe_float(car.get("apFollowTime"), 0.0),
    "followDistanceStock": _safe_float(car.get("followDistanceStock"), 0.0),
    "stockGap": _safe_float(car.get("stockGap"), 0.0),
    "teslaGap": _safe_float(car.get("teslaGap"), 0.0),
    "dasFollowDistance": _safe_float(car.get("dasFollowDistance"), 0.0),
    "dasFollowTime": _safe_float(car.get("dasFollowTime"), 0.0),
    "DAS_followDistance": _safe_float(car.get("DAS_followDistance"), 0.0),
    "DAS_timeGap": _safe_float(car.get("DAS_timeGap"), 0.0),
    "cruiseFollowDistance": _safe_float(car.get("cruiseFollowDistance"), 0.0),
    "cruiseTimeGap": _safe_float(car.get("cruiseTimeGap"), 0.0),
    "longitudinalControlGap": _safe_float(car.get("longitudinalControlGap"), 0.0),
    "cruiseState.gap": _safe_float(cruise.get("gap") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.followDistance": _safe_float(cruise.get("followDistance") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.distanceSetting": _safe_float(cruise.get("distanceSetting") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.modeSel": _safe_float(cruise.get("modeSel") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.timeGap": _safe_float(cruise.get("timeGap") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.followTime": _safe_float(cruise.get("followTime") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.cruiseGap": _safe_float(cruise.get("cruiseGap") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.gapSetting": _safe_float(cruise.get("gapSetting") if isinstance(cruise, dict) else 0.0, 0.0),
    "cruiseState.accDistance": _safe_float(cruise.get("accDistance") if isinstance(cruise, dict) else 0.0, 0.0),
  }
  for key, value in (car.get("rawGapFields") or {}).items():
    raw_candidates[f"raw.{key}"] = _safe_float(value, 0.0)
  if isinstance(cruise, dict):
    for key, value in (cruise.get("rawGapFields") or {}).items():
      raw_candidates[f"cruiseState.raw.{key}"] = _safe_float(value, 0.0)
  follow_s = _safe_int(car.get("followDistanceS"), 255)
  active_name = ""
  stalk_gap = 0.0
  if 0 <= follow_s <= 6:
    active_name = "followDistanceS"
    stalk_gap = float(follow_s)
  else:
    for name, value in raw_candidates.items():
      if abs(float(value)) > 0.001:
        active_name = str(name)
        stalk_gap = float(value)
        break

  button_events = car.get("buttonEvents", []) if isinstance(car.get("buttonEvents"), list) else []
  gap_button_events = [
    event for event in button_events
    if any(token in _safe_str(event.get("type", "")).lower() for token in ("gap", "distance", "follow"))
  ]

  return {
    "desiredTF": _safe_float(plan.get("desiredTF"), 0.0),
    "actualGapS": actual_gap_s,
    "dRel": d_rel,
    "followS": follow_s,
    "stalkGap": stalk_gap,
    "stalkGapField": active_name,
    "rawCandidates": raw_candidates,
    "gapButtonEvents": gap_button_events[-6:],
    "buttonEvents": button_events[-8:],
    "cruiseButtons": _safe_int(car.get("cruiseButtons"), 0),
    "cruiseButtonsCounter": _safe_int(car.get("cruiseButtonsCounter"), 0),
  }



def _model_summary(model: Any) -> dict[str, Any]:
  out: dict[str, Any] = {}
  leads_out: list[dict[str, Any]] = []
  try:
    leads = list(_maybe_attr(model, "leadsV3", []) or [])[:2]
  except Exception:
    leads = []

  for lead in leads:
    probs = _take_numeric(_maybe_attr(lead, "prob", None), 3)
    xs = _take_numeric(_maybe_attr(lead, "x", None), 3)
    ys = _take_numeric(_maybe_attr(lead, "y", None), 3)
    vs = _take_numeric(_maybe_attr(lead, "v", None), 3)
    leads_out.append({
      "prob": probs[0] if probs else None,
      "x": xs[0] if xs else None,
      "y": ys[0] if ys else None,
      "v": vs[0] if vs else None,
    })

  lane_lines: list[dict[str, Any]] = []
  try:
    lines = list(_maybe_attr(model, "laneLines", []) or [])[:4]
  except Exception:
    lines = []
  try:
    line_probs = list(_maybe_attr(model, "laneLineProbs", []) or [])[:4]
  except Exception:
    line_probs = []

  for i, line in enumerate(lines):
    xs = _take_numeric(_maybe_attr(line, "x", None), 3)
    ys = _take_numeric(_maybe_attr(line, "y", None), 3)
    lane_lines.append({
      "prob": _safe_float(line_probs[i], 0.0) if i < len(line_probs) else 0.0,
      "x0": xs[0] if xs else None,
      "y0": ys[0] if ys else None,
    })

  position = _maybe_attr(model, "position", None)
  out["position"] = {
    "x0": (_take_numeric(_maybe_attr(position, "x", None), 1) or [None])[0],
    "y0": (_take_numeric(_maybe_attr(position, "y", None), 1) or [None])[0],
  }

  # Vision path curvature from the comma model (candidate cross-check vs CSA c2 / mapd).
  # kappa(i) = yawRate(i) / max(v(i), 1) [1/m] along the predicted path. Report: the immediate
  # value (mean of the first few near points), the tightest |kappa| within ~130 m ahead, and how
  # far ahead that tightest point sits -- directly comparable to csaC2 + csaRange.
  out["mdlCurvNow"] = None
  out["mdlCurvMax"] = None
  out["mdlCurvMaxDist"] = None
  try:
    xs = _take_numeric(_maybe_attr(position, "x", None), 33)
    yaw_rate = _take_numeric(_maybe_attr(_maybe_attr(model, "orientationRate", None), "z", None), 33)
    vx = _take_numeric(_maybe_attr(_maybe_attr(model, "velocity", None), "x", None), 33)
    n = min(len(xs), len(yaw_rate), len(vx))
    if n > 0:
      kappa = [yaw_rate[i] / (vx[i] if vx[i] > 1.0 else 1.0) for i in range(n)]
      near = [abs(k) for k in kappa[:4]]
      if near:
        out["mdlCurvNow"] = sum(near) / len(near)
      horizon = [(xs[i], kappa[i]) for i in range(n) if xs[i] <= 130.0]
      if horizon:
        x_at, k_at = max(horizon, key=lambda p: abs(p[1]))
        out["mdlCurvMax"] = k_at
        out["mdlCurvMaxDist"] = x_at
  except Exception:
    pass

  out["leadsV3"] = leads_out
  out["laneLines"] = lane_lines
  return out


def _derive_flags(*, car: dict[str, Any], plan: dict[str, Any], lead1: dict[str, Any], lead2: dict[str, Any], mapd: dict[str, Any], long_log: dict[str, Any] | None, steer_diag: dict[str, Any] | None = None) -> dict[str, Any]:
  cruise = car.get("cruiseState", {}) if isinstance(car.get("cruiseState"), dict) else {}
  current_set = _safe_float(cruise.get("speed"), 0.0)
  v_ego = _safe_float(car.get("vEgo"), 0.0)
  p_near = plan.get("p_near")
  p_preview = plan.get("p_preview")
  a_target = _safe_float(plan.get("aTarget"), 0.0)
  plan_drop = False
  if isinstance(p_near, (float, int)) and current_set > 0.0:
    plan_drop = float(p_near) < (current_set - 1.0)
  no_actual_lead = not bool(lead1.get("status")) and not bool(lead2.get("status"))
  long_src = _safe_str((long_log or {}).get("src", ""))
  vrels = []
  for val in (_safe_float(lead1.get("vRel"), 0.0), _safe_float(lead2.get("vRel"), 0.0)):
    if val != 0.0:
      vrels.append(val)
  primary_vrel = min(vrels) if vrels else 0.0
  follow_gap = _follow_gap_summary(car, plan, lead1, lead2)
  return {
    "clearRoadCandidate": bool(no_actual_lead and not _safe_bool(car.get("brakePressed", False)) and v_ego > 4.0),
    "plannerDropVsSet": bool(plan_drop),
    "longHasPlannerOwner": ("planner" in long_src),
    "longHasCurveOwner": any(
      token in long_src
      for token in (
        "curve_hold",
        "hard_entry",
        "curve_fused",
        "roundabout_fused",
        "mapd_roundabout",
        "mapd_roundabout_soft",
        "roundabout_cap",
        "lat_sat_hard_cap",
        "steer_busy_hard",
        "low_speed_lat_trim",
        "low_speed_lat_release",
        "roundabout_early_cap",
        "roundabout_planner_early_cap",
        "roundabout_release_guard",
            "planner_rescue_block",
        "planner_rescue",
        "weak_curve_trim",
        "state[CURVE",
      )
    ),
    "longSource": long_src,
    "currentSet": current_set,
    "vEgo": v_ego,
    "pNear": p_near,
    "pPreview": p_preview,
    "mapd": mapd if isinstance(mapd, dict) else {},
    "followGap": follow_gap,
    "closingLead": bool((lead1.get("status") or lead2.get("status")) and (primary_vrel < -0.5 or a_target < -0.15)),
    "steeringBusy": bool(abs(_safe_float(car.get("steeringAngleDeg"), 0.0)) > 8.0 or abs(_safe_float(car.get("steeringRateDeg"), 0.0)) > 25.0),
    "steerLimited": _safe_bool((steer_diag or {}).get("steerLimitedBySafety", False)),
    "latSaturated": _safe_bool((steer_diag or {}).get("lateralSaturated", False)),
    "latUndershooting": _safe_bool((steer_diag or {}).get("undershooting", False)) and _safe_bool((steer_diag or {}).get("turning", False)),
    "steerDiagReason": _safe_str((steer_diag or {}).get("reason", "-")),
    "gasPressed": _safe_bool(car.get("gasPressed", False)),
    "brakePressed": _safe_bool(car.get("brakePressed", False)),
    "pedalOverride": _safe_bool(car.get("pedalOverride", False)) or _safe_bool(car.get("gasPressed", False)) or _safe_bool(car.get("brakePressed", False)),
    "cruiseEnabled": _safe_bool(cruise.get("enabled", False)) if isinstance(cruise, dict) else False,
    "cruiseAvailable": _safe_bool(cruise.get("available", False)) if isinstance(cruise, dict) else False,
    "stockCruiseState": _safe_str(car.get("stockCruiseState", "")),
    "stockCruiseEnabled": _safe_bool(car.get("stockCruiseEnabled", False)),
    "stockCruiseAvailable": _safe_bool(car.get("stockCruiseAvailable", False)),
  }


class FlapTracker:
  def __init__(self) -> None:
    self.last_primary = "none"
    self.last_present = False
    self.last_status_change_ms: int | None = None
    self.primary_switches = 0
    self.presence_toggles = 0
    self.dropout_short = 0
    self.last_dropout_ms: int | None = None

  def update(self, mono_ms: int, lead1: dict[str, Any], lead2: dict[str, Any]) -> dict[str, Any]:
    primary = "lead1" if lead1.get("status") else ("lead2" if lead2.get("status") else "none")
    present = primary != "none"

    if primary != self.last_primary:
      self.primary_switches += 1
      if self.last_primary != "none" and primary == "none":
        self.last_dropout_ms = mono_ms
      elif self.last_primary == "none" and primary != "none" and self.last_dropout_ms is not None:
        gap_ms = mono_ms - self.last_dropout_ms
        if gap_ms <= 1200:
          self.dropout_short += 1
      self.last_primary = primary

    if present != self.last_present:
      self.presence_toggles += 1
      self.last_status_change_ms = mono_ms
      self.last_present = present

    return {
      "primary": primary,
      "present": present,
      "primarySwitches": self.primary_switches,
      "presenceToggles": self.presence_toggles,
      "shortDropouts": self.dropout_short,
      "sinceStatusChangeMs": 0 if self.last_status_change_ms is None else max(0, mono_ms - self.last_status_change_ms),
    }


class SwaglogTail:
  KEYWORDS = (
    "[XNOR_CRUISE_SYNC]",
    "[XNOR_CC_DIAG]",
    "TESLA_STEER_DIAG",
    "STEER_SAT_DIAG",
    "turn exceeds",
    "steerSaturated",
    "lead_guard",
    "lead_stuck_cancel",
    "lead_approach_force",
    "autoengage_mismatch_reset",
    "autoengage_stale_block",
    "lead_flap_block_resume",
    "lead_takeover_after_autoengage",
    "mapd_cap",
    "mapd_comfort",
    "curve_fused",
    "roundabout_fused",
    "mapd_roundabout",
    "roundabout_cap",
    "lat_sat_hard_cap",
    "steer_busy_hard",
    "lead_far_release",
    "lead_soft_release",
    "lead_observer",
    "curve_clear(dynamic)",
    "curve_reentry_block",
    "curve_clear(vision)",
    "CURVE_EXIT_RELEASE",
    "mapd_vision_clear",
    "mapd_roundabout_soft",
    "low_speed_floor",
    "low_speed_lat_trim",
    "low_speed_lat_release",
    "roundabout_early_cap",
    "roundabout_release_guard",
    "weak_curve_advisory",
    "weak_curve_release",
    "weak_curve_trim",
    "high_speed_drop_clamp",
    "lead_straight_reject",
    "blank_planner_reject",
    "stale_cancel_reject",
    "invalid_curve_target_recover",
    "min_hold_invalid_recover",
    "curve_zero_target_recover",
    "curve_fused_invalid_recover",
    "junction_lead_guard_soften",
    "low_speed_curve_rescue",
    "lead_low_speed_reject",
    "lat_trim_reject",
    "roundabout_reject_vision_clear",
    "map_only_unconfirmed",
  )

  def __init__(self) -> None:
    self._lock = threading.Lock()
    self._recent: deque[dict[str, Any]] = deque(maxlen=80)
    self._positions: dict[str, int] = {}
    self._thread = threading.Thread(target=self._run, daemon=True)

  def start(self) -> None:
    self._thread.start()

  def recent(self) -> list[dict[str, Any]]:
    with self._lock:
      return list(self._recent)

  def latest(self, kind: str | None = None) -> dict[str, Any] | None:
    with self._lock:
      if kind is None:
        return self._recent[-1] if self._recent else None
      for record in reversed(self._recent):
        if _safe_str(record.get("kind")) == kind:
          return record
      return None

  def _log_files(self) -> list[Path]:
    out: list[Path] = []
    folder = Path("/data/log")
    if not folder.exists():
      return out
    try:
      for path in sorted(folder.glob("swaglog*")):
        if path.is_file():
          out.append(path)
    except Exception:
      pass
    return out

  def _run(self) -> None:
    while RUNNING:
      try:
        for path in self._log_files():
          self._consume_file(path)
      except Exception:
        pass
      time.sleep(0.2)

  def _consume_file(self, path: Path) -> None:
    key = str(path)
    try:
      current_size = path.stat().st_size
    except Exception:
      return

    pos = self._positions.get(key, 0)
    if pos > current_size:
      pos = 0

    try:
      with path.open("r", encoding="utf-8", errors="ignore") as f:
        f.seek(pos)
        while True:
          line = f.readline()
          if not line:
            break
          pos = f.tell()
          self._process_line(line.rstrip("\n"))
    except Exception:
      return

    self._positions[key] = pos

  def _process_line(self, line: str) -> None:
    if not any(keyword in line for keyword in self.KEYWORDS):
      return

    raw_message = line
    created = None
    filename = ""
    module = ""
    msg_s = line

    try:
      payload = json.loads(line)
      raw_message = line
      created = _safe_float(payload.get("created", 0.0), 0.0)
      filename = _safe_str(payload.get("filename", ""))
      module = _safe_str(payload.get("module", ""))
      msg_s = _safe_str(payload.get("msg$s", line))
    except Exception:
      pass

    if "[XNOR_CRUISE_SYNC]" in msg_s:
      kind = "long_decision"
    elif "[XNOR_CC_DIAG]" in msg_s:
      kind = "cc_diag"
    elif "TESLA_STEER_DIAG" in msg_s or "STEER_SAT_DIAG" in msg_s:
      kind = "steer_diag"
    else:
      kind = "event"
    record = {
      "created": created,
      "filename": filename,
      "module": module,
      "kind": kind,
      "message": msg_s,
      "raw": raw_message,
    }

    msg = msg_s
    if "[XNOR_CC_DIAG]" in msg:
      m = CC_DIAG_RE.search(msg)
      if m:
        rest = m.group("rest") or ""
        record["gate"] = m.group("gate")
        record["extra"] = {km.group("key"): km.group("value") for km in KEYVAL_RE.finditer(rest)}
        if "detail=" in rest:
          record["detail"] = rest.split("detail=", 1)[1].strip()
          msg = record["detail"]

    sync_match = CRUISE_SYNC_RE.search(msg)
    if sync_match:
      gd = sync_match.groupdict()
      record["src"] = gd.get("src", "")
      record["uom"] = gd.get("uom", "")
      record["tgt"] = _safe_float(gd.get("tgt"), 0.0)
      record["cur"] = _safe_float(gd.get("cur"), 0.0)
      record["est"] = _safe_float(gd.get("est"), 0.0)
      record["btn"] = _safe_int(gd.get("btn"), 0)
      record["reason"] = _safe_str(gd.get("reason"), "")
    else:
      for token in msg.split():
        if "=" not in token:
          continue
        k, v = token.split("=", 1)
        if k in ("src", "uom", "reason", "mode", "state", "gate", "detail"):
          record[k] = v
        elif k in ("tgt", "cur", "est", "gap", "delta", "cooldown", "stuck", "vrel", "drel"):
          record[k] = _safe_float(v, 0.0)
        elif k == "btn":
          record[k] = _safe_int(v, 0)

    with self._lock:
      self._recent.append(record)



def _diag_classification(record: dict[str, Any]) -> str:
  cc = record.get("cc_diag_log") if isinstance(record.get("cc_diag_log"), dict) else {}
  long_log = record.get("long_log") if isinstance(record.get("long_log"), dict) else {}

  gate = _safe_str(cc.get("gate", ""))
  reason = _safe_str(long_log.get("reason", "")) or _safe_str(cc.get("reason", ""))
  extra = cc.get("extra") if isinstance(cc.get("extra"), dict) else {}

  if gate == "pre":
    if _safe_str(extra.get("ap_disabled")) == "0":
      return "blocked_before_long:ap_disabled_param_false"
    if _safe_str(extra.get("enabled")) == "0":
      return "blocked_before_long:control_not_enabled"
    return "blocked_before_long:pre"
  if gate == "pending_release":
    return "blocked_before_long:pending_stalk_release"
  if gate == "turn_hold":
    return "blocked_before_long:virtual_turn_hold"
  if gate == "no_decision":
    if reason.startswith("gated:"):
      return f"long_returned_gate:{reason}"
    if reason.startswith("standby:"):
      return f"long_returned_standby:{reason}"
    if reason.startswith("no-op"):
      return f"long_returned_noop:{reason}"
    if "cooldown" in reason:
      return "long_returned_cooldown"
    return "long_returned_no_button"
  if _safe_int(long_log.get("btn"), 0) != 0:
    return "long_sent_button"
  if record.get("derived", {}).get("pedalOverride"):
    return "driver_pedal_active"
  if not record.get("derived", {}).get("cruiseEnabled"):
    return "cruise_disabled"
  return "no_recent_long_or_cc_log"



def _write_summary_line(txt_f, record: dict[str, Any]) -> None:
  flags = record["derived"]
  long_log = record.get("long_log") or {}
  cc_diag = record.get("cc_diag_log") or {}
  cc_extra = cc_diag.get("extra") if isinstance(cc_diag.get("extra"), dict) else {}
  flap = record.get("leadState", {})
  follow = flags.get("followGap", {}) if isinstance(flags.get("followGap"), dict) else {}
  mapd = flags.get("mapd", {}) if isinstance(flags.get("mapd"), dict) else {}
  steer = record.get("steerDiag", {}) if isinstance(record.get("steerDiag"), dict) else {}
  panda = steer.get("pandaLegacyVm", {}) if isinstance(steer.get("pandaLegacyVm"), dict) else {}
  line = (
    f"{record['wall_time']} "
    f"vEgo={flags['vEgo']:.2f} "
    f"set={flags['currentSet']:.2f} "
    f"clear={int(flags['clearRoadCandidate'])} "
    f"plannerDrop={int(flags['plannerDropVsSet'])} "
    f"closing={int(flags['closingLead'])} "
    f"steerBusy={int(flags['steeringBusy'])} "
    f"steerLim={int(flags.get('steerLimited', False))} "
    f"latSat={int(flags.get('latSaturated', False))} "
    f"latUnder={int(flags.get('latUndershooting', False))} "
    f"steerReason={_safe_str(flags.get('steerDiagReason'), '-') or '-'} "
    f"desAng={_safe_float(steer.get('desiredAngleDeg'), 0.0):.2f} "
    f"outAng={_safe_float(steer.get('outputAngleDeg'), 0.0):.2f} "
    f"actAng={_safe_float(steer.get('actualAngleDeg'), 0.0):.2f} "
    f"dOut={_safe_float(steer.get('desiredMinusOutputDeg'), 0.0):.2f} "
    f"oAct={_safe_float(steer.get('outputMinusActualDeg'), 0.0):.2f} "
    f"step={_safe_float(steer.get('outputStepDeg'), 0.0):.2f} "
    f"pMax={_safe_float(panda.get('maxAngleDeg'), 0.0):.1f} "
    f"pDelta={_safe_float(panda.get('maxDeltaDeg'), 0.0):.2f} "
    f"latA={_safe_float(steer.get('actualLateralAccel'), 0.0):.2f} "
    f"desLatA={_safe_float(steer.get('desiredLateralAccel'), 0.0):.2f} "
    f"gas={int(flags.get('gasPressed', False))} "
    f"brake={int(flags.get('brakePressed', False))} "
    f"pedal={int(flags.get('pedalOverride', False))} "
    f"cruiseEn={int(flags.get('cruiseEnabled', False))} "
    f"cruiseAvail={int(flags.get('cruiseAvailable', False))} "
    f"stockState={_safe_str(flags.get('stockCruiseState', '-')) or '-'} "
    f"stockEn={int(flags.get('stockCruiseEnabled', False))} "
    f"lead1={int(record['radarState']['leadOne'].get('status', False))} "
    f"lead2={int(record['radarState']['leadTwo'].get('status', False))} "
    f"leadSel={_safe_str(flap.get('primary', '-'))} "
    f"toggles={_safe_int(flap.get('presenceToggles', 0))} "
    f"gapS={_safe_float(follow.get('actualGapS'), 0.0):.2f} "
    f"dRel={_safe_float(follow.get('dRel'), 0.0):.1f} "
    f"vRel={_safe_float(follow.get('vRel'), 0.0):.2f} "
    f"yRel={_safe_float(record['radarState']['leadOne'].get('yRel'), 0.0):.1f} "
    f"aLeadK={_safe_float(record['radarState']['leadOne'].get('aLeadK'), 0.0):.2f} "
    f"desiredTF={_safe_float(follow.get('desiredTF'), 0.0):.2f} "
    f"pNear={_safe_float(flags.get('pNear'), 0.0):.2f} "
    f"mapAlive={int(_safe_bool(mapd.get('alive', False)))} "
    f"mapValid={int(_safe_bool(mapd.get('valid', False)))} "
    f"mapAge={_safe_float(mapd.get('ageMs'), 0.0):.0f} "
    f"mapHas={int(_safe_bool(mapd.get('hasCurveInputs', False)))} "
    f"map={_safe_float(mapd.get('mapCurveSpeed'), 0.0):.2f} "
    f"vis={_safe_float(mapd.get('visionCurveSpeed'), 0.0):.2f} "
    f"csaOk={int(_safe_bool((record.get('csa') or {}).get('ok', False)))} "
    f"csaBus={_safe_int((record.get('csa') or {}).get('bus'), -1)} "
    f"csaState={_safe_float((record.get('csa') or {}).get('csaState'), 0.0):.0f} "
    f"csaC2={_safe_float((record.get('csa') or {}).get('roadC2'), 0.0):.6f} "
    f"csaRange={_safe_float((record.get('csa') or {}).get('roadRange'), 0.0):.0f} "
    f"csaCtr={_safe_float((record.get('csa') or {}).get('roadCtr'), 0.0):.0f} "
    f"mdlCurvNow={_safe_float((record.get('modelV2') or {}).get('mdlCurvNow'), 0.0):.6f} "
    f"mdlCurvMax={_safe_float((record.get('modelV2') or {}).get('mdlCurvMax'), 0.0):.6f} "
    f"mdlCurvDist={_safe_float((record.get('modelV2') or {}).get('mdlCurvMaxDist'), 0.0):.0f} "
    f"csaOffRange={_safe_float((record.get('csa') or {}).get('offrampRange'), 0.0):.0f} "
    f"navExp={_safe_float((record.get('csa') or {}).get('navExpSpeed'), 0.0):.0f} "
    f"navAct={_safe_float((record.get('csa') or {}).get('navRouteActive'), 0.0):.0f} "
    f"laneC2={_safe_float((record.get('csa') or {}).get('laneC2'), 0.0):.6f} "
    f"laneRng={_safe_float((record.get('csa') or {}).get('laneRange'), 0.0):.0f} "
    f"laneCtr={_safe_float((record.get('csa') or {}).get('laneCtr'), 0.0):.0f} "
    f"mppSL={_safe_float((record.get('csa') or {}).get('mppSpeedLimit'), 0.0):.0f} "
    f"mapSL={_safe_float((record.get('csa') or {}).get('mapSpeedLimit'), 0.0):.0f} "
    f"roadSign={_safe_float((record.get('csa') or {}).get('roadSign'), 0.0):.0f} "
    f"redLight={_safe_float((record.get('csa') or {}).get('redLightStopSign'), 0.0):.0f} "
    f"road={_safe_str((mapd.get('rawRoadFields') or {}).get('roadName', (mapd.get('rawRoadFields') or {}).get('wayName', '-'))).replace(' ', '_')[:32] or '-'} "
    f"ctx={_safe_str((mapd.get('rawRoadFields') or {}).get('roadContext', '-')) or '-'} "
    f"sel={_safe_str((mapd.get('rawRoadFields') or {}).get('waySelectionType', '-')) or '-'} "
    f"followS={_safe_int(follow.get('followS'), 255)} "
    f"stalkGap={_safe_float(follow.get('stalkGap'), 0.0):.1f} "
    f"stalkField={_safe_str(follow.get('stalkGapField'), '-') or '-'} "
    f"gapBtn={','.join(_safe_str(e.get('type'), '') for e in (follow.get('gapButtonEvents') or [])[-2:]) or '-'} "
    f"btnEvt={','.join(_safe_str(e.get('type'), '') for e in (follow.get('buttonEvents') or [])[-3:]) or '-'} "
    f"longAge={_safe_float(record.get('longLogAgeMs'), 0.0):.0f} "
    f"longStale={int(_safe_bool(record.get('longLogStale', False)))} "
    f"ccGate={_safe_str(cc_diag.get('gate'), '-') or '-'} "
    f"ccAge={_safe_float(record.get('ccDiagAgeMs'), -1.0):.0f} "
    f"ccClass={_safe_str(record.get('decisionClassification'), '-') or '-'} "
    f"src={flags['longSource'] or '-'} "
    f"tgt={_safe_float(long_log.get('tgt'), 0.0):.2f} "
    f"cur={_safe_float(long_log.get('cur'), 0.0):.2f} "
    f"btn={_safe_int(long_log.get('btn'), 0)} "
    f"reason={_safe_str(long_log.get('reason'), '-')}"
  )
  txt_f.write(line + "\n")
  txt_f.flush()


class CsaCanReader:
  """Decode Tesla CSA / nav curvature signals from raw CAN, alongside mapd.

  CSA (Curve Speed Adaptation) on legacy Tesla (tesla_can.dbc) is route-independent
  (map-matched GPS), so it is the strongest pre-emptive roundabout/curve signal we
  have. None of it is on cereal, so we decode it straight off the 'can' socket with
  a CANParser. We don't know which bus carries it on this car, so we run one parser
  per candidate bus and report whichever is live (Counter advancing).

  Fail-safe by design: if cereal/CANParser/the DBC/the 'can' socket is unavailable,
  self.ok stays False, every field reads as missing, and the rest of the watcher is
  completely unaffected. tesla_can has no checksum/counter state registered, so the
  parser never gates rows on checksum/counter -- the *Counter signals come through
  as plain incrementing values, which is exactly what we want for liveness.
  """
  DBC_NAME = "tesla_can"
  BUSES = (0, 1, 2)
  MESSAGES = [
    ("UI_csaRoadCurvature", 0),     # 744: road-ahead curvature + lookahead Range (m)
    ("UI_csaOfframpCurvature", 0),  # 728: offramp curvature + Range
    ("AutopilotStatus", 0),         # 921: DAS_csaState gate (0 unavail/1 avail/2 enable/3 hold)
    ("MCU_locationStatus2", 0),     # 104: MCU_navigonExpectedSpeed (route-tied, mph)
    ("UI_driverAssistMapData", 0),  # 968: UI_navRouteActive, UI_mapSpeedLimit, UI_mapSpeedLimitType
    # Camera / AP-derived candidates (evaluate as replacements for the CSA/mapd cross-checks)
    ("DAS_lanes", 0),               # 569: DAS_virtualLaneC2/C3 (camera lane-path curvature), view range, lane-exists
    ("UI_gpsVehicleSpeed", 0),      # 760: UI_mppSpeedLimit (posted limit, kph/mph)
    ("UI_driverAssistRoadSign", 0), # 568: UI_roadSign (camera TSR), UI_roadSignCounter
    ("MCU_chassisControl", 0),      # 536: MCU_redLightStopSignEnable (red-light / stop-sign awareness)
  ]

  def __init__(self) -> None:
    self.ok = False
    self.err = ""
    self.buses_built: list[int] = []
    self._parsers: dict[int, Any] = {}
    self._sock = None
    self._capnp_to_list = None
    self._last_counter: dict[int, int] = {}
    try:
      from opendbc.can.parser import CANParser
      try:
        from openpilot.selfdrive.pandad import can_capnp_to_list
      except Exception:
        from selfdrive.pandad import can_capnp_to_list  # type: ignore
      self._capnp_to_list = can_capnp_to_list
      for bus in self.BUSES:
        try:
          self._parsers[bus] = CANParser(self.DBC_NAME, list(self.MESSAGES), bus)
          self.buses_built.append(bus)
        except Exception as exc:
          self.err = f"{self.err} bus{bus}:{exc!r}".strip()
      self._sock = messaging.sub_sock("can", conflate=False)
      self.ok = bool(self._parsers) and self._sock is not None
    except Exception as exc:
      self.err = repr(exc)
      self.ok = False

  def update(self) -> None:
    if not self.ok or self._sock is None or self._capnp_to_list is None:
      return
    try:
      raw = messaging.drain_sock_raw(self._sock)
      if not raw:
        return
      can_list = self._capnp_to_list(raw)
      for cp in self._parsers.values():
        try:
          cp.update(can_list)
        except Exception:
          pass
    except Exception:
      pass

  def _read(self, cp: Any, msg: str, sig: str) -> float | None:
    try:
      v = float(cp.vl[msg][sig])
    except Exception:
      return None
    return v if math.isfinite(v) else None

  def _read_any(self, msg: str, sig: str) -> float | None:
    # The camera/AP messages may ride a different bus than CSA, so try every built parser
    # and return the first finite read instead of assuming the CSA-live bus.
    for b in self.buses_built:
      cp = self._parsers.get(b)
      if cp is None:
        continue
      v = self._read(cp, msg, sig)
      if v is not None:
        return v
    return None

  def _bus_counter(self, bus: int) -> int:
    cp = self._parsers.get(bus)
    if cp is None:
      return -1
    v = self._read(cp, "UI_csaRoadCurvature", "UI_csaRoadCurvCounter")
    return int(v) if v is not None else -1

  def _live_bus(self) -> int:
    advancing = -1
    nonzero = -1
    for bus in self.BUSES:
      ctr = self._bus_counter(bus)
      last = self._last_counter.get(bus, -1)
      if ctr >= 0:
        if last >= 0 and ctr != last and advancing < 0:
          advancing = bus
        if ctr > 0 and nonzero < 0:
          nonzero = bus
        self._last_counter[bus] = ctr
    return advancing if advancing >= 0 else nonzero

  def summary(self) -> dict[str, Any]:
    out: dict[str, Any] = {
      "ok": bool(self.ok),
      "err": self.err,
      "busesBuilt": list(self.buses_built),
      "busCtr": {str(b): self._bus_counter(b) for b in self.BUSES},
    }
    if not self.ok:
      out["bus"] = -1
      return out
    bus = self._live_bus()
    out["bus"] = bus
    cp = self._parsers.get(bus) if bus >= 0 else self._parsers.get(self.buses_built[0] if self.buses_built else -1)
    if cp is None:
      return out
    out["csaState"] = self._read(cp, "AutopilotStatus", "DAS_csaState")
    out["roadC2"] = self._read(cp, "UI_csaRoadCurvature", "UI_csaRoadCurvC2")
    out["roadC3"] = self._read(cp, "UI_csaRoadCurvature", "UI_csaRoadCurvC3")
    out["roadRange"] = self._read(cp, "UI_csaRoadCurvature", "UI_csaRoadCurvRange")
    out["roadCtr"] = self._read(cp, "UI_csaRoadCurvature", "UI_csaRoadCurvCounter")
    out["offrampC2"] = self._read(cp, "UI_csaOfframpCurvature", "UI_csaOfframpCurvC2")
    out["offrampRange"] = self._read(cp, "UI_csaOfframpCurvature", "UI_csaOfframpCurvRange")
    out["offrampCtr"] = self._read(cp, "UI_csaOfframpCurvature", "UI_csaOfframpCurvCounter")
    out["navExpSpeed"] = self._read(cp, "MCU_locationStatus2", "MCU_navigonExpectedSpeed")
    out["navRouteActive"] = self._read(cp, "UI_driverAssistMapData", "UI_navRouteActive")
    # Camera / AP-derived candidate signals (read across all buses; null => not on this car's bus)
    out["laneC2"] = self._read_any("DAS_lanes", "DAS_virtualLaneC2")            # camera lane-path curvature (1/m)
    out["laneC3"] = self._read_any("DAS_lanes", "DAS_virtualLaneC3")
    out["laneRange"] = self._read_any("DAS_lanes", "DAS_virtualLaneViewRange")  # m
    out["laneL"] = self._read_any("DAS_lanes", "DAS_leftLaneExists")
    out["laneR"] = self._read_any("DAS_lanes", "DAS_rightLaneExists")
    out["laneCtr"] = self._read_any("DAS_lanes", "DAS_lanesCounter")
    out["mppSpeedLimit"] = self._read_any("UI_gpsVehicleSpeed", "UI_mppSpeedLimit")        # posted limit (kph/mph)
    out["mapSpeedLimit"] = self._read_any("UI_driverAssistMapData", "UI_mapSpeedLimit")    # enum (LESS_OR_EQ_*)
    out["mapSpeedLimitType"] = self._read_any("UI_driverAssistMapData", "UI_mapSpeedLimitType")
    out["roadSign"] = self._read_any("UI_driverAssistRoadSign", "UI_roadSign")             # camera TSR
    out["roadSignCtr"] = self._read_any("UI_driverAssistRoadSign", "UI_roadSignCounter")
    out["redLightStopSign"] = self._read_any("MCU_chassisControl", "MCU_redLightStopSignEnable")
    return out


def main() -> int:
  parser = argparse.ArgumentParser(description="Watch clear-road handoff, lead stability, and LONG owner output.")
  parser.add_argument("duration_s", nargs="?", type=int, default=300, help="Run duration in seconds")
  parser.add_argument("--interval-ms", type=int, default=100, help="Sampling interval in milliseconds")
  args = parser.parse_args()

  signal.signal(signal.SIGINT, _sig_handler)
  signal.signal(signal.SIGTERM, _sig_handler)

  sm = messaging.SubMaster([
    "carState",
    "carControl",
    "carOutput",
    "controlsState",
    "radarState",
    "longitudinalPlan",
    "modelV2",
    "mapdOut",
    "liveParameters",
  ])

  tail = SwaglogTail()
  tail.start()
  flap_tracker = FlapTracker()
  csa_reader = CsaCanReader()

  ts = datetime.now().strftime("%Y%m%d_%H%M%S")
  out_dir = Path("/data")
  jsonl_path = out_dir / f"clear_path_watch_{ts}.jsonl"
  txt_path = out_dir / f"clear_path_watch_{ts}.txt"

  started = _now_ms()
  next_sample_ms = started
  previous_output_angle: float | None = None

  with jsonl_path.open("w", encoding="utf-8") as jsonl_f, txt_path.open("w", encoding="utf-8") as txt_f:
    txt_f.write("# clear_path_watch\n")
    txt_f.write(f"# started { _wall_iso() }\n")
    txt_f.write(
      f"# csa_reader ok={int(bool(csa_reader.ok))} "
      f"buses={','.join(str(b) for b in csa_reader.buses_built) or '-'} "
      f"err={(csa_reader.err or '-')[:160]}\n"
    )

    while RUNNING and (_now_ms() - started) < (int(args.duration_s) * 1000):
      sm.update(0)
      csa_reader.update()
      now_ms = _now_ms()
      if now_ms < next_sample_ms:
        time.sleep(0.01)
        continue
      next_sample_ms = now_ms + int(args.interval_ms)

      car = _car_state_summary(sm["carState"]) if sm.seen["carState"] else {}
      car_control = _car_control_summary(sm["carControl"]) if sm.seen["carControl"] else {}
      car_output = _car_output_summary(sm["carOutput"]) if sm.seen["carOutput"] else {}
      controls = _controls_state_summary(sm["controlsState"]) if sm.seen["controlsState"] else {}
      live_parameters = _live_parameters_summary(sm["liveParameters"]) if sm.seen["liveParameters"] else {}
      steer_diag = _steer_diag_summary(car, controls, car_control, car_output, live_parameters, previous_output_angle)
      previous_output_angle = _safe_float(steer_diag.get("outputAngleDeg"), previous_output_angle or 0.0)
      radar = {
        "leadOne": _lead_summary(_maybe_attr(sm["radarState"], "leadOne", None)) if sm.seen["radarState"] else {"status": False},
        "leadTwo": _lead_summary(_maybe_attr(sm["radarState"], "leadTwo", None)) if sm.seen["radarState"] else {"status": False},
      }
      plan = _plan_summary(sm["longitudinalPlan"]) if sm.seen["longitudinalPlan"] else {}
      model = _model_summary(sm["modelV2"]) if sm.seen["modelV2"] else {}
      mapd = _mapd_summary(sm["mapdOut"]) if sm.seen["mapdOut"] else {}
      mapd_mono_ns = _safe_int(sm.logMonoTime.get("mapdOut", 0), 0)
      mapd_age_ms = ((time.monotonic_ns() - mapd_mono_ns) / 1_000_000.0) if mapd_mono_ns > 0 else 0.0
      mapd["seen"] = _safe_bool(sm.seen["mapdOut"])
      mapd["alive"] = _safe_bool(sm.alive["mapdOut"])
      mapd["valid"] = _safe_bool(sm.valid["mapdOut"])
      mapd["ageMs"] = float(mapd_age_ms)
      mapd["hasCurveInputs"] = bool(
        _safe_float(mapd.get("mapCurveSpeed"), 0.0) > 0.1
        or _safe_float(mapd.get("visionCurveSpeed"), 0.0) > 0.1
      )
      long_decision_raw = tail.latest(kind="long_decision")
      cc_diag_raw = tail.latest(kind="cc_diag")

      long_log_age_ms = -1.0
      if isinstance(long_decision_raw, dict) and long_decision_raw.get("created") is not None:
        try:
          long_log_age_ms = max(0.0, (time.time() - float(long_decision_raw.get("created"))) * 1000.0)
        except Exception:
          long_log_age_ms = -1.0
      long_log_stale = bool(long_log_age_ms < 0.0 or long_log_age_ms > 1800.0)

      cc_diag_age_ms = -1.0
      if isinstance(cc_diag_raw, dict) and cc_diag_raw.get("created") is not None:
        try:
          cc_diag_age_ms = max(0.0, (time.time() - float(cc_diag_raw.get("created"))) * 1000.0)
        except Exception:
          cc_diag_age_ms = -1.0
      cc_diag_stale = bool(cc_diag_age_ms < 0.0 or cc_diag_age_ms > 1800.0)

      # Prefer the LONG module's actual decision (src/tgt/btn/reason). The carcontroller's
      # high-rate cc_diag 'no_decision' heartbeat (the 5Hz send-gate "gated: 5Hz(frame)") must
      # NOT override a fresh real command -- doing so blanked ~95% of summary rows and hid every
      # press/cancel. Only fall back to cc_diag when there is NO fresh long_decision, so the
      # summary still surfaces no_decision/pre/turn_hold context when the module isn't commanding.
      # (Revert: drop the `long_log_stale and` guard to restore the old no_decision-wins behaviour.)
      decision_raw = long_decision_raw
      if (
        long_log_stale
        and isinstance(cc_diag_raw, dict)
        and not cc_diag_stale
        and _safe_str(cc_diag_raw.get("gate")) == "no_decision"
      ):
        decision_raw = cc_diag_raw

      decision_age_ms = cc_diag_age_ms if decision_raw is cc_diag_raw else long_log_age_ms
      decision_stale = cc_diag_stale if decision_raw is cc_diag_raw else long_log_stale
      long_log = {} if decision_stale else decision_raw
      lead_state = flap_tracker.update(now_ms, radar["leadOne"], radar["leadTwo"])
      csa = csa_reader.summary()

      record = {
        "wall_time": _wall_iso(),
        "mono_ms": now_ms,
        "valid": {k: _safe_bool(v) for k, v in sm.valid.items()},
        "alive": {k: _safe_bool(v) for k, v in sm.alive.items()},
        "csa": csa,
        "carState": car,
        "carControl": car_control,
        "carOutput": car_output,
        "controlsState": controls,
        "liveParameters": live_parameters,
        "steerDiag": steer_diag,
        "radarState": radar,
        "longitudinalPlan": plan,
        "modelV2": model,
        "mapdOut": mapd,
        "leadState": lead_state,
        "followGap": _follow_gap_summary(car, plan, radar["leadOne"], radar["leadTwo"]),
        "long_log": long_log,
        "long_decision_log": {} if long_log_stale else long_decision_raw,
        "cc_diag_log": {} if cc_diag_stale else cc_diag_raw,
        "long_log_raw": decision_raw,
        "longLogAgeMs": decision_age_ms,
        "longLogStale": decision_stale,
        "ccDiagAgeMs": cc_diag_age_ms,
        "ccDiagStale": cc_diag_stale,
        "recent_long_logs": tail.recent()[-20:],
      }
      record["derived"] = _derive_flags(
        car=car,
        plan=plan,
        lead1=radar["leadOne"],
        lead2=radar["leadTwo"],
        mapd=mapd,
        long_log=long_log,
        steer_diag=steer_diag,
      )
      record["decisionClassification"] = _diag_classification(record)

      jsonl_f.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
      jsonl_f.flush()

      d = record["derived"]
      interesting = False
      if d["clearRoadCandidate"] and d["plannerDropVsSet"]:
        interesting = True
      if d["longHasPlannerOwner"] or d["longHasCurveOwner"]:
        interesting = True
      if lead_state["present"] and (lead_state["sinceStatusChangeMs"] < 1500 or lead_state["shortDropouts"] > 0):
        interesting = True
      if d["closingLead"] and d["plannerDropVsSet"]:
        interesting = True
      if _safe_str(record.get("decisionClassification"), "no_recent_long_or_cc_log") != "no_recent_long_or_cc_log":
        interesting = True
      if d.get("steerLimited") or d.get("latSaturated") or d.get("latUndershooting"):
        interesting = True
      if abs(_safe_float(steer_diag.get("desiredMinusOutputDeg"), 0.0)) > STEER_ANGLE_SATURATION_THRESHOLD_DEG:
        interesting = True
      _csa = record.get("csa") or {}
      if _safe_int(_csa.get("bus"), -1) >= 0 or _safe_float(_csa.get("csaState"), 0.0) > 0.0:
        interesting = True
      if any(
        marker in d["longSource"]
        for marker in (
          "autoengage",
          "lead_flap",
          "lead_takeover",
          "curve_fused",
          "roundabout_fused",
          "roundabout_planner_early_cap",
          "roundabout_release_guard",
          "planner_rescue_block",
          "planner_rescue",
          "mapd_roundabout",
          "lat_sat_hard_cap",
          "steer_busy_hard",
          "curve_clear(vision)",
          "mapd_vision_clear",
          "planner_lead_ignored",
          "lead_far_release",
          "lead_soft_release",
          "cruise_inactive_guard",
          "pedal_override_guard",
          "mapd_blank_reject",
          "cruise_inactive_auto_engage",
          "cruise_state_disagree",
          "roundabout_fused[name",
          "mapd_town_false_positive",
        )
      ):
        interesting = True

      if interesting:
        _write_summary_line(txt_f, record)

      time.sleep(0.001)

    txt_f.write(f"# finished { _wall_iso() }\n")

  print(str(jsonl_path))
  print(str(txt_path))
  return 0


if __name__ == "__main__":
  raise SystemExit(main())
