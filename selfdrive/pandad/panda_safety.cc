#include "selfdrive/pandad/pandad.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"

// --- XNOR early-teslaLegacy ---------------------------------------------------------------
// Cold-boot AEB flash root cause: the stock AP asserts fcw=3 during its own boot and reaches the
// IC through the closed harness relay BEFORE the panda is in teslaLegacy. The IC latches it for
// the whole ignition cycle. The fix is to bring teslaLegacy up (relay open + forward-scrub live)
// BEFORE the stock AP's fcw=3 -- i.e. set the safety model from cached CarParams the moment we go
// onroad, instead of waiting for the FW-query/ControlsReady handshake.
//
// The previous early-safety attempt dropped controls mid-drive because it re-ran setSafetyMode on
// an is_onroad transient and that resets controls_allowed to 0. This version is structurally
// guarded against that:
//   - setSafetyMode is only ever called while NOT engaged (see configureSafetyMode's `engaged`
//     guard), so it can never reset controls out from under an active drive;
//   - the offroad re-arm requires a SUSTAINED offroad (debounced) AND not engaged, so a brief
//     is_onroad glitch cannot trigger a re-apply.

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
  };
  for (const char *key : early_param_keys) {
    const std::string params_string = params_.get(key);
    if (carParamsHaveTeslaLegacySafety(params_string)) {
      LOGW("XNOR early Tesla safety: applying teslaLegacy from %s before ControlsReady", key);
      setSafetyMode(params_string);  // sets teslaLegacy (+ is_tesla_legacy_), relay opens, scrub live
      initialized_ = true;           // skip the ELM327 init block in updateMultiplexingMode
      return true;
    }
  }
  return false;
}

void PandaSafety::configureSafetyMode(bool is_onroad, bool engaged) {
  if (is_onroad && !safety_configured_) {
    // Never apply/replace the safety mode while engaged -- setSafetyMode resets controls_allowed.
    // At boot this branch runs pre-engage; the guard is defensive against any odd ordering.
    if (engaged) {
      return;
    }

    offroad_debounce_ = 0;

    // Early path: bring teslaLegacy up from cached CarParams now, so the scrub beats the stock
    // boot fcw=3. Falls back to the stock ELM327 fingerprint init only with no cached Tesla.
    if (!early_applied_) {
      early_applied_ = trySetEarlyTeslaSafety();
    }
    if (!early_applied_) {
      updateMultiplexingMode();
    }

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    // Re-arm for a new drive ONLY on a sustained offroad and only while not engaged. A transient
    // is_onroad glitch must not reset our flags (that is what re-ran setSafetyMode mid-drive and
    // dropped controls last time).
    if (!engaged && (++offroad_debounce_ >= 30)) {  // ~3 s sustained offroad at 10 Hz
      initialized_ = false;
      safety_configured_ = false;
      early_applied_ = false;
      log_once_ = false;
      offroad_debounce_ = 0;
    }
  } else {
    // onroad and already configured: nothing to do; keep the offroad debounce cleared.
    offroad_debounce_ = 0;
  }
}

// XNOR boot relay-reset (belt-and-braces for any latch the early path doesn't beat).
// Reproduces the user's power-cycle/offroad->onroad clear: a one-shot safety re-cycle that
// toggles the intercept relay (NO_OUTPUT closes it, teslaLegacy opens it) so the IC re-syncs to
// the recovered stock fcw=0. Fires once per drive, pre-engage only, holding the relay closed for
// ~5 s (a 0.5 s blip did not give the IC time to re-sync).
void PandaSafety::maybeBootRelayReset(bool is_onroad, bool engaged) {
  if (!is_onroad) {
    relay_reset_phase_ = 0;
    relay_reset_counter_ = 0;
    return;
  }
  if (!safety_configured_ || !is_tesla_legacy_) {
    return;
  }

  switch (relay_reset_phase_) {
    case 0:  // armed once teslaLegacy is up
      relay_reset_counter_ = 0;
      relay_reset_phase_ = 1;
      break;

    case 1:  // pre-engage wait (~8 s). If OP engages first, skip -- never disrupt a live drive.
      if (engaged) {
        relay_reset_phase_ = 99;
        break;
      }
      if (++relay_reset_counter_ >= 80) {  // ~8 s at 10 Hz
        LOGW("XNOR boot relay-reset: cycling relay to clear stale IC AEB latch (pre-engage)");
        for (int i = 0; i < pandas_.size(); ++i) {
          pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::NO_OUTPUT);
        }
        relay_reset_counter_ = 0;
        relay_reset_phase_ = 2;
      }
      break;

    case 2:  // relay closed (~5 s), then re-apply the real safety to re-open it
      if (engaged) {  // extremely unlikely while NO_OUTPUT, but be safe
        relay_reset_phase_ = 99;
        break;
      }
      if (++relay_reset_counter_ >= 50) {  // ~5 s at 10 Hz
        const std::string car_params = params_.get("CarParams");
        if (!car_params.empty()) {
          setSafetyMode(car_params);  // restores teslaLegacy + alternativeExperience, relay re-opens
        }
        relay_reset_phase_ = 99;  // done for this drive
      }
      break;

    default:  // 99 = done
      break;
  }
}

void PandaSafety::updateMultiplexingMode() {
  // Initialize to ELM327 without OBD multiplexing for initial fingerprinting
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    for (int i = 0; i < pandas_.size(); ++i) {
      pandas_[i]->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    }
    initialized_ = true;
  }

  // Switch between multiplexing modes based on the OBD multiplexing request
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

  is_tesla_legacy_ = false;
  for (int i = 0; i < pandas_.size(); ++i) {
    // Default to SILENT safety model if not specified
    cereal::CarParams::SafetyModel safety_model = cereal::CarParams::SafetyModel::SILENT;
    uint16_t safety_param = 0U;
    if (i < safety_configs.size()) {
      safety_model = safety_configs[i].getSafetyModel();
      safety_param = safety_configs[i].getSafetyParam();
    }
    if (safety_model == cereal::CarParams::SafetyModel::TESLA_LEGACY) {
      is_tesla_legacy_ = true;
    }

    LOGW("Panda %d: setting safety model: %d, param: %d, alternative experience: %d", i, (int)safety_model, safety_param, alternative_experience);
    pandas_[i]->set_alternative_experience(alternative_experience);
    pandas_[i]->set_safety_model(safety_model, safety_param);
  }
}
