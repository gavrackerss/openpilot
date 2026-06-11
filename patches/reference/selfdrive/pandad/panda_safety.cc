#include "selfdrive/pandad/pandad.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"

namespace {
static volatile const char xnor_v123_persistent_carparams_parse_fix_marker_blob[] __attribute__((used)) = "XNOR_V123_PERSISTENT_CARPARAMS_PARSE_FIX";

static const char *xnorV123PersistentCarParamsParseFixMarker() {
  return const_cast<const char *>(xnor_v123_persistent_carparams_parse_fix_marker_blob);
}
}

// XNOR_V123_PERSISTENT_CARPARAMS_PARSE_FIX:
// Use a persistent copy of the previous route's Tesla CarParams so pandad can
// set teslaLegacy before ControlsReady and avoid the early ELM327 forwarding
// window. v123 fixes the installer-side Python capnp parse bug from v122.
bool PandaSafety::carParamsHaveTeslaLegacySafety(const std::string &params_string) {
  if (params_string.empty()) {
    return false;
  }

  try {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
    cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

    auto safety_configs = car_params.getSafetyConfigs();
    for (uint32_t i = 0; i < safety_configs.size(); ++i) {
      if (safety_configs[i].getSafetyModel() == cereal::CarParams::SafetyModel::TESLA_LEGACY) {
        return true;
      }
    }
  } catch (...) {
    return false;
  }

  return false;
}

bool PandaSafety::trySetEarlyTeslaSafety() {
  static const char *early_param_keys[] = {
    "CarParamsPersistent",
    "CarParamsPrevRoute",
    "CarParamsCache",
    "CarParams",
  };

  for (const char *key : early_param_keys) {
    const std::string params_string = params_.get(key);
    if (carParamsHaveTeslaLegacySafety(params_string)) {
      LOGW("%s: applying early Tesla safety from %s before ControlsReady", xnorV123PersistentCarParamsParseFixMarker(), key);
      setSafetyMode(params_string);
      return true;
    }
  }

  return false;
}

void PandaSafety::configureSafetyMode(bool is_onroad) {
  if (is_onroad && !safety_configured_) {
    if (!early_tesla_safety_configured_) {
      early_tesla_safety_configured_ = trySetEarlyTeslaSafety();
      if (!early_tesla_safety_configured_) {
        updateMultiplexingMode();
      }
    }

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    initialized_ = false;
    early_tesla_safety_configured_ = false;
    safety_configured_ = false;
    log_once_ = false;
  }
}
