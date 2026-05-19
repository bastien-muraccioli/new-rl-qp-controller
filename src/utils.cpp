#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <mc_rtc/logging.h>

#include "NewRLQPController.h"

void utils::start_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name)
{
  auto & ctl = static_cast<NewRLQPController&>(ctl_);
  state_name_ = state_name;
  mc_rtc::log::info("{} state started", state_name);

  syncTime_ = ctl.policyStepSize;
    
  if(!ctl.rlPolicy || !ctl.rlPolicy->isLoaded())
  {
    mc_rtc::log::error("RL policy not loaded in {} state", state_name);
    return;
  }

  ctl.gui()->addElement(
    {"NewRLQPController", state_name},
    mc_rtc::gui::Label("Policy Loaded", [&ctl]() { 
      return ctl.rlPolicy->isLoaded() ? "Yes" : "No"; 
    }),
    mc_rtc::gui::Label("Observation Size", [&ctl]() { 
      return std::to_string(ctl.rlPolicy->getObservationSize()); 
    }),
    mc_rtc::gui::Label("Action Size", [&ctl]() { 
      return std::to_string(ctl.rlPolicy->getActionSize()); 
    })
  );

  mc_rtc::log::success("{} state initialization completed", state_name);
}

void utils::run_rl_state(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<NewRLQPController&>(ctl_);
  try
  {
    syncTime_ += ctl.timeStep;
    if(syncTime_ >= ctl.policyStepSize)
    {
      ctl.currentObservation = getCurrentObservation(ctl);
      ctl.currentAction = ctl.rlPolicy->predict(ctl.currentObservation)*ctl.actionScale;
      // Run new inference and update target position, scaled by action scale
      ctl.q_rl = ctl.q_zero + ctl.currentAction;
      syncTime_ = 0.0;
    }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("Error during RL state run: {}", e.what());
  }
}

void utils::teardown_rl_state(mc_control::fsm::Controller & ctl_)
{
  ctl_.gui()->removeCategory({"NewRLQPController", state_name_});
}

Eigen::VectorXd utils::getCurrentObservation(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<NewRLQPController&>(ctl_);
  Eigen::VectorXd obs(ctl.rlPolicy->getObservationSize());
  obs = Eigen::VectorXd::Zero(ctl.rlPolicy->getObservationSize());

  // Observation examples - these should be adapted to match the expected observation of the loaded policy
  // auto & real_robot = ctl.realRobot(ctl.robots()[0].name());
  // auto q_map = real_robot.encoderValues();
  // auto q_dot_map = real_robot.encoderVelocities();
  // ctl.jointPos = Eigen::VectorXd::Map(q_map.data(), int(q_map.size())).tail(ctl.dofNumber);
  // ctl.jointVel = Eigen::VectorXd::Map(q_dot_map.data(), int(q_dot_map.size())).tail(ctl.dofNumber);

  // switch (ctl.currentPolicyIndex) {
  //   case 0:
  //   {
  //     obs.segment(0, 7) = ctl.jointPos;
  //     obs.segment(7, 7) = ctl.jointVel;
  //     obs.segment(14, 3) = ctl.jointAction;
  //     ...
  //     break;
  //   }
  //   case 1:
  //   {
  //     obs.segment(0, 7) = ctl.jointPos;
  //     obs.segment(7, 7) = ctl.jointVel;
  //     ...
  //     break;
  //   }
  //   default:
  //   {
  //     mc_rtc::log::error("Unknown policy index: {}", ctl.currentPolicyIndex);
  //     break;
  //   }
  // }
  
  return obs;
}
