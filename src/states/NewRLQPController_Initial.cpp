#include "NewRLQPController_Initial.h"

#include <NewRLQPController.h>

void NewRLQPController_Initial::configure(const mc_rtc::Configuration & config) {}

void NewRLQPController_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<NewRLQPController &>(ctl_);
  ctl.rlStateRunner.start(ctl, "RL_State");
}

bool NewRLQPController_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<NewRLQPController &>(ctl_);
  ctl.rlStateRunner.run(ctl);
  ctl.torqueJointTask->setPosTarget(ctl.rlRuntime().q_rl());
  return false;
}

void NewRLQPController_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<NewRLQPController &>(ctl_);
  ctl.rlStateRunner.teardown(ctl);
}

EXPORT_SINGLE_STATE("NewRLQPController_Initial", NewRLQPController_Initial)
