#pragma once

#include <mc_control/fsm/State.h>

/**
 * Robot-agnostic single RL execution state.
 *
 * The historical name "Initial" is preserved so this template applies cleanly
 * to the original single-state controller. Rename it later if desired.
 */
struct NewRLQPController_Initial : mc_control::fsm::State
{
  void configure(const mc_rtc::Configuration & config) override;
  void start(mc_control::fsm::Controller & ctl) override;
  bool run(mc_control::fsm::Controller & ctl) override;
  void teardown(mc_control::fsm::Controller & ctl) override;
};
