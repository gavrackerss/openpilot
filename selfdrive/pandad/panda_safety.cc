#include "selfdrive/pandad/pandad.h"
#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"

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

// XNOR boot relay-reset.
// The cold-boot AEB flash is the stock AP asserting fcw=3 during its own boot, reaching the IC
// through the closed harness relay BEFORE the panda is in teslaLegacy. The IC latches it for the
// whole ignition cycle; nothing the panda does after teslaLegacy clears it. The user's manual
// fix -- power off/on, i.e. OP offroad->onroad -- works because that cycles the safety mode and
// thus toggles the intercept relay (set_intercept_relay: car modes open it, NO_OUTPUT closes it),
// which makes the IC re-sync. By then the stock AP has recovered to fcw=0, so the IC comes up
// clean. This reproduces that ONCE, automatically, in the safe pre-engage window:
//   - only while onroad and teslaLegacy is configured,
//   - only while NOT engaged (controls can't be reset out from under an active drive),
//   - ~8 s after teslaLegacy (past the stock AP's fcw=3 -> fcw=0 recovery),
//   - exactly once per drive (re-arms when offroad).
// Phase timers count 10 Hz calls (this is invoked once per main-loop state tick).
void PandaSafety::maybeBootRelayReset(bool is_onroad, bool engaged) {
  if (!is_onroad) {
    // Re-arm for the next drive.
    relay_reset_phase_ = 0;
    relay_reset_counter_ = 0;
    return;
  }
  // Only for a configured teslaLegacy car.
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

    case 2:  // relay closed (~0.5 s), then re-apply the real safety to re-open it
      if (engaged) {  // extremely unlikely while NO_OUTPUT, but be safe
        relay_reset_phase_ = 99;
        break;
      }
      if (++relay_reset_counter_ >= 5) {  // ~0.5 s at 10 Hz
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
