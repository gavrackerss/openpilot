"""V229 source-only, startup-latched Tesla Model S HW2 PCM-off test selection.

pcmCruise is CarParams (static): an Experimental UI toggle cannot safely mutate it
inside an active drive. Re-evaluate only after a full userspace restart.
"""
from __future__ import annotations
import os
from openpilot.common.params import Params

TEST_ARM = '/data/xnor_enable_experimental_pcmfalse_test'
DIRECT_ARM = '/data/xnor_enable_experimental_direct_bench'
DIRECT_DENY = '/data/xnor_disable_experimental_direct_bench'
OWNER_ARM = '/data/xnor_enable_v221_hil_owner'
OWNER_DENY = '/data/xnor_disable_v221_hil_owner'


def pcmfalse_test_requested(params: Params | None = None) -> bool:
  p = params if params is not None else Params()
  return bool(
    p.get_bool('TinklaHybridNativeAP') and not p.get_bool('TinklaAutopilotDisabled')
    and p.get_bool('ExperimentalMode')
    and os.path.isfile(TEST_ARM)
    and os.path.isfile(DIRECT_ARM) and not os.path.exists(DIRECT_DENY)
    and os.path.isfile(OWNER_ARM) and not os.path.exists(OWNER_DENY)
  )


def pcmfalse_test_active(CP, params: Params | None = None) -> bool:
  """Identify a *latched* test CarParams; don't depend on live UI/arm file here."""
  p = params if params is not None else Params()
  return bool(
    getattr(CP, 'brand', '') == 'tesla'
    and CP.openpilotLongitudinalControl and not CP.pcmCruise
    and p.get_bool('TinklaHybridNativeAP') and not p.get_bool('TinklaAutopilotDisabled')
  )
