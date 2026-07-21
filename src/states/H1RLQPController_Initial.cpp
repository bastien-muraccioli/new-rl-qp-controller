#include "H1RLQPController_Initial.h"

#include <H1RLQPController.h>

void H1RLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void H1RLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<H1RLQPController &>(ctl_);
  ctl.rlStateRunner.start(ctl, "RL_State");
}

bool H1RLQPController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<H1RLQPController &>(ctl_);
  ctl.rlStateRunner.run(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.rlRuntime().q_rl());
  return false;
}

void H1RLQPController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<H1RLQPController &>(ctl_);
}

EXPORT_SINGLE_STATE("H1RLQPController_Initial", H1RLQPController_Initial)
