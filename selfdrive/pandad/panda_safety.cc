#include "selfdrive/pandad/pandad.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"

namespace {
// XNOR passive-init helper. Returns true if a cached CarParams blob already carries FW
// versions. When it does, the next get_car reuses the cache and runs NO live FW query
// (opendbc car_helpers.fingerprint), so the panda never needs to TX during fingerprinting --
// which means we can init it passive (NO_OUTPUT) instead of ELM327 and avoid the CAN2 storm
// (interruptRateCan2 -> latched faultTemp -> controls mismatch + stuck AEB, clears only on a
// power cycle). Self-contained: no PandaSafety members, so pandad.h is unchanged.
bool xnorCachedFingerprintPresent(const std::string &params_string) {
  if (params_string.empty()) {
    return false;
  }
  try {
    AlignedBuffer aligned_buf;
    capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
    cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();
    return car_params.getCarFw().size() > 0;
  } catch (...) {
    return false;
  }
}
}  // namespace

void PandaSafety::configureSafetyMode(bool is_onroad) {
  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
  }
}

void PandaSafety::updateMultiplexingMode() {
  if (!initialized_) {
    prev_obd_multiplexing_ = false;

    // XNOR passive init: ELM327 reconfigures CAN on the live AP bus and can storm CAN2
    // (interruptRateCan2 -> latched faultTemp -> controls mismatch + stuck AEB, clears only
    // on a power cycle). ELM327 is only actually needed for a first-ever FW fingerprint.
    // When a cached CarParams with FW versions exists, get_car reuses it with NO live FW
    // query, so the panda never needs to TX -- init it PASSIVE (NO_OUTPUT) and never touch
    // CAN2. Fall back to ELM327 only with no cache (true first-time setup).
    const bool have_cache = xnorCachedFingerprintPresent(params_.get("CarParamsPersistent")) ||
                            xnorCachedFingerprintPresent(params_.get("CarParamsCache"));
    const cereal::CarParams::SafetyModel init_model =
        have_cache ? cereal::CarParams::SafetyModel::NO_OUTPUT
                   : cereal::CarParams::SafetyModel::ELM327;
    if (have_cache) {
      LOGW("XNOR: cached fingerprint present -> passive (NO_OUTPUT) init, skipping ELM327 CAN2 window");
    }
    for (int i = 0; i < pandas_.size(); ++i) {
      pandas_[i]->set_safety_model(init_model, 1U);
    }
    initialized_ = true;
  }

  // Switch between multiplexing modes based on the OBD multiplexing request.
  // On a cached boot get_car never requests OBD multiplexing, so this stays dormant; it is
  // kept intact so a true first-time fingerprint (ELM327 branch above) still works.
  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    for (int i = 0; i < pandas_.size(); ++i) {
      const uint16_t safety_param = (i > 0 || !obd_multiplexing_requested) ? 1U : 0U;
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
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
    // Default to SILENT safety model if not specified
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
