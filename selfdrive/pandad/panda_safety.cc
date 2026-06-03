#include "selfdrive/pandad/pandad.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"

namespace {
static const char xnor_v146_early_tesla_diag_safe_marker[] __attribute__((used)) =
    "XNOR_V146_EARLY_TESLA_DIAGNOSTIC_SAFE";

const char *xnorV146EarlyTeslaDiagSafeMarker() {
  return xnor_v146_early_tesla_diag_safe_marker;
}
}  // namespace

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

bool PandaSafety::trySetEarlyPersistentTeslaSafety() {
  static const char *early_param_keys[] = {
    "CarParamsPersistent",
    "CarParamsPrevRoute",
    "CarParamsCache",
  };

  for (const char *key : early_param_keys) {
    const std::string params_string = params_.get(key);
    if (carParamsHaveTeslaLegacySafety(params_string)) {
      LOGW("%s: applying cached Tesla safety from %s before ControlsReady",
           xnorV146EarlyTeslaDiagSafeMarker(), key);
      setSafetyMode(params_string);
      initialized_ = true;
      early_tesla_safety_configured_ = true;
      return true;
    }
  }

  return false;
}

void PandaSafety::configureSafetyMode(bool is_onroad) {
  if (is_onroad && !safety_configured_) {
    if (!early_tesla_safety_configured_) {
      trySetEarlyPersistentTeslaSafety();
    }

    updateMultiplexingMode();

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

void PandaSafety::setObdMultiplexing(bool enabled) {
  for (int i = 0; i < pandas_.size(); ++i) {
    pandas_[i]->set_obd_multiplexing(enabled);
  }
}

void PandaSafety::updateMultiplexingMode() {
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    for (int i = 0; i < pandas_.size(); ++i) {
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    }
    initialized_ = true;
  }

  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    if (early_tesla_safety_configured_) {
      LOGW("%s: setting OBD multiplexing to %d without leaving Tesla safety",
           xnorV146EarlyTeslaDiagSafeMarker(), (int)obd_multiplexing_requested);
      setObdMultiplexing(obd_multiplexing_requested);
    } else {
      for (int i = 0; i < pandas_.size(); ++i) {
        const uint16_t safety_param = (i > 0 || !obd_multiplexing_requested) ? 1U : 0U;
        pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
      }
    }

    prev_obd_multiplexing_ = obd_multiplexing_requested;
    params_.putBool("ObdMultiplexingChanged", true);
  }
}

std::string PandaSafety::fetchCarParams() {
  if (!params_.getBool("FirmwareQueryDone")) {
    return {};
  }

  if (!log_once_) {
    LOGW("Finished FW query, Waiting for params to set safety model");
    log_once_ = true;
  }

  if (!params_.getBool("ControlsReady")) {
    return {};
  }
  return params_.get("CarParams");
}

void PandaSafety::setSafetyMode(const std::string &params_string) {
  AlignedBuffer aligned_buf;
  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  auto safety_configs = car_params.getSafetyConfigs();
  uint16_t alternative_experience = car_params.getAlternativeExperience();

  for (int i = 0; i < pandas_.size(); ++i) {
    cereal::CarParams::SafetyModel safety_model = cereal::CarParams::SafetyModel::SILENT;
    uint16_t safety_param = 0U;
    if (i < safety_configs.size()) {
      safety_model = safety_configs[i].getSafetyModel();
      safety_param = safety_configs[i].getSafetyParam();
    }

    LOGW("Panda %d: setting safety model: %d, param: %d, alternative experience: %d", i, (int)safety_model, safety_param, alternative_experience);
    pandas_[i]->set_alternative_experience(alternative_experience);
    pandas_[i]->set_safety_model(safety_model, safety_param);
  }
}
