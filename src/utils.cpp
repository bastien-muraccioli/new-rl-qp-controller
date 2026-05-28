#include "utils.h"
#include <Eigen/src/Core/Matrix.h>
#include <mc_rtc/logging.h>

#include "NewRLQPController.h"

void utils::start_rl_state(mc_control::fsm::Controller & ctl_, std::string state_name)
{
  auto & ctl = static_cast<NewRLQPController&>(ctl_);
  state_name_ = state_name;
  mc_rtc::log::info("[NewRLQPController::utils] {} state started", state_name);

  syncTime_ = ctl.policyStepSize;
    
  if(!ctl.rlPolicy || !ctl.rlPolicy->isLoaded())
  {
    mc_rtc::log::error("[NewRLQPController::utils] RL policy not loaded in {} state", state_name);
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

  ctl.initializeRLObservation();

  mc_rtc::log::success("[NewRLQPController::utils] {} state initialization completed", state_name);
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
      ctl.currentAction = ctl.rlPolicy->predict(ctl.currentObservation);
      for (int j = 0; j < ctl.currentAction.size(); ++j) {
          int i = ctl.actionToDofMap[j];
          ctl.currentActionScaled(i) = ctl.actionScale(i) * ctl.currentAction(j);
          ctl.q_rl(i) = ctl.currentActionScaled(i) + ctl.q_zero(i);
      }
      syncTime_ = 0.0;
    }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("[NewRLQPController::utils] Error during RL state run: {}", e.what());
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
  int offset = 0;
  auto appendToObs = [&](const Eigen::VectorXd& v) {
    obs.segment(offset, v.size()) = v;
    offset += v.size();
  };

  switch (ctl.currentPolicyIndex) {
    case 0: // PolicyName1.onnx observation example
    {
      // // shift history: t-2 <- t-1 <- t
      // for (int i = ctl.HISTORY_SIZE - 1; i > 0; --i) {
      //     ctl.linVel[i] = ctl.linVel[i - 1];
      //     ctl.angVel[i] = ctl.angVel[i - 1];
      //     ctl.projectedGravity[i] = ctl.projectedGravity[i - 1];
      //     ctl.velCmd[i] = ctl.velCmd[i - 1];
      //     ctl.jointPos[i] = ctl.jointPos[i - 1];
      //     ctl.jointVel[i] = ctl.jointVel[i - 1];
      //     ctl.jointAction[i] = ctl.jointAction[i - 1];
      // }

      // ctl.initializeRLObservation(); // update t with current observation

      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.linVel[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.angVel[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.projectedGravity[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointPos[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointVel[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointAction[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.velCmd[i]);
      break;
    }
    case 1: // PolicyName2.onnx observation example 
    {
      // // shift history: t-2 <- t-1 <- t
      // for (int i = ctl.HISTORY_SIZE - 1; i > 0; --i) {
      //     ctl.linVel[i] = ctl.linVel[i - 1];
      //     ctl.angVel[i] = ctl.angVel[i - 1];
      //     ctl.projectedGravity[i] = ctl.projectedGravity[i - 1];
      //     ctl.velCmd[i] = ctl.velCmd[i - 1];
      //     ctl.jointPos[i] = ctl.jointPos[i - 1];
      //     ctl.jointVel[i] = ctl.jointVel[i - 1];
      //     ctl.jointAction[i] = ctl.jointAction[i - 1];
      // }

      // ctl.initializeRLObservation(); // update t with current observation

      // // Older observation first (t-2, t-1, t) --> mjlab order
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.linVel[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.angVel[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.projectedGravity[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointPos[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointVel[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.footContactForces[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.jointAction[i]);
      // for (int i = ctl.HISTORY_SIZE - 1; i >= 0; --i) appendToObs(ctl.velCmd[i]);

      // // Newer observation first (t, t-1, t-2)
      // // for (int i = 0; i < ctl.HISTORY_SIZE; ++i) appendToObs(ctl.linVel[i]);
      // // for (int i = 0; i < ctl.HISTORY_SIZE; ++i) appendToObs(ctl.angVel[i]);
      // // for (int i = 0; i < ctl.HISTORY_SIZE; ++i) appendToObs(ctl.projectedGravity[i]);
      // // for (int i = 0; i < ctl.HISTORY_SIZE; ++i) appendToObs(ctl.jointPos[i]);
      // // for (int i = 0; i < ctl.HISTORY_SIZE; ++i) appendToObs(ctl.jointVel[i]);
      // // for (int i = 0; i < ctl.HISTORY_SIZE; ++i) appendToObs(ctl.footContactForces[i]);
      // // for (int i = 0; i < ctl.HISTORY_SIZE; ++i) appendToObs(ctl.jointAction[i]);
      // // for (int i = 0; i < ctl.HISTORY_SIZE; ++i) appendToObs(ctl.velCmd[i]);
      break;
    }
    default:
    {
      mc_rtc::log::error("[NewRLQPController::utils] Unknown policy index: {}", ctl.currentPolicyIndex);
      break;
    }
  }

  assert(offset == obs.size() && "[NewRLQPController::utils] Observation size mismatch: written bytes != allocated size");
  return obs;
}