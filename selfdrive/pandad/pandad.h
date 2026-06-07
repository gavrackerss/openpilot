#pragma once

#include <string>
#include <vector>

#include "common/params.h"
#include "selfdrive/pandad/panda.h"

void pandad_main_thread(std::vector<std::string> serials);

class PandaSafety {
public:
  PandaSafety(const std::vector<Panda *> &pandas) : pandas_(pandas) {}
  void configureSafetyMode(bool is_onroad);
  // XNOR boot relay-reset: one-shot safety re-cycle that toggles the intercept relay shortly
  // after teslaLegacy comes up, so the IC re-syncs and drops the cold-boot AEB latch. Gated to
  // pre-engage + once per drive (see panda_safety.cc). Call at 10 Hz from the main loop with the
  // current engaged state.
  void maybeBootRelayReset(bool is_onroad, bool engaged);

private:
  void updateMultiplexingMode();
  std::string fetchCarParams();
  void setSafetyMode(const std::string &params_string);

  bool initialized_ = false;
  bool log_once_ = false;
  bool safety_configured_ = false;
  bool prev_obd_multiplexing_ = false;
  // XNOR relay-reset state
  bool is_tesla_legacy_ = false;       // set in setSafetyMode when the model is teslaLegacy
  int relay_reset_phase_ = 0;          // 0=arm, 1=wait pre-engage, 2=relay closed, 99=done
  int relay_reset_counter_ = 0;        // 10 Hz call counter for the phase timers
  std::vector<Panda *> pandas_;
  Params params_;
};
