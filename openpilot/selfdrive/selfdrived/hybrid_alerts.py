"""Pure Tesla Hybrid alert predicates.

Keep these helpers dependency-free so they can be tested before the OpenPilot
runtime and Cap'n Proto bindings are installed.
"""


def hybrid_lane_change_steer_handover(brand: str, pcm_cruise: bool,
                                      stock_lkas: bool, handover_ready: bool,
                                      lane_change_active: bool) -> bool:
  """True while Hybrid intentionally leaves lane-change steering outside OP authority."""
  return bool(
    brand == "tesla"
    and pcm_cruise
    and lane_change_active
    and (stock_lkas or not handover_ready)
  )


def tesla_op_long_brake_preserves_lateral(brand: str, openpilot_longitudinal_control: bool) -> bool:
  """True when a Tesla brake press should cancel longitudinal only, not OP lateral.

  This intentionally covers both Hybrid (pcmCruise) and the normal Tesla
  openpilot-longitudinal path. Cancel/steering/system faults remain separate
  disengagement mechanisms.
  """
  return bool(brand == "tesla" and openpilot_longitudinal_control)
