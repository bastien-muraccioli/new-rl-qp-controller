#include "policy/RLStateRunner.h"

#include "H1RLQPController.h"

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

void RLStateRunner::start(mc_control::fsm::Controller & ctl_, const std::string & stateName)
{
  H1RLQPController & ctl = static_cast<H1RLQPController &>(ctl_);

  stateName_ = stateName;

  mc_rtc::log::info("[RLStateRunner] {} state started", stateName_);

  if(!ctl.rlRuntime().policyLoaded())
  {
    mc_rtc::log::error("[RLStateRunner] RL policy is not loaded in state '{}'", stateName_);
    return;
  }

  ctl.gui()->addElement(
    {"H1RLQPController", stateName_},
    mc_rtc::gui::Label("Policy loaded", [&ctl]() { return ctl.rlRuntime().policyLoaded() ? "Yes" : "No"; }),
    mc_rtc::gui::Label("Observation size", [&ctl]() { return std::to_string(ctl.rlRuntime().observationSize()); }),
    mc_rtc::gui::Label("Action size", [&ctl]() { return std::to_string(ctl.rlRuntime().actionSize()); }),
    mc_rtc::gui::Label("Policy step size", [&ctl]() { return std::to_string(ctl.rlRuntime().policyStepSize()); }),
    mc_rtc::gui::Label("Current policy", [&ctl]() { return ctl.rlRuntime().currentPolicyName(); }));

  ctl.rlRuntime().reset(ctl);

  mc_rtc::log::success("[RLStateRunner] {} state initialization completed", stateName_);
}

void RLStateRunner::run(mc_control::fsm::Controller & ctl_)
{
  H1RLQPController & ctl = static_cast<H1RLQPController &>(ctl_);

  try
  {
    ctl.rlRuntime().runPolicyStepIfNeeded(ctl, ctl.timeStep);
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("[RLStateRunner] Error during RL state run: {}", e.what());
  }
}
