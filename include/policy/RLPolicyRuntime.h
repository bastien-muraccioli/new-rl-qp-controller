#pragma once

#include "observation/Observation.h"
#include "observation/ObservationManager.h"
#include "policy/PolicyConfig.h"
#include "policy/RLPolicyInterface.h"

#include <Eigen/Core>

#include <mc_rtc/Configuration.h>
#include <mc_rbdyn/Robot.h>
#include <mc_tasks/TorqueJointTask.h>

#include <memory>
#include <string>
#include <vector>

struct NewRLQPController;

namespace rlqp
{

class RLPolicyRuntime
{
public:
  RLPolicyRuntime();

  void configure(const mc_rtc::Configuration & controllerConfig,
                 NewRLQPController & ctl,
                 const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void reset(NewRLQPController & ctl);
  void runPolicyStepIfNeeded(NewRLQPController & ctl, double dt);

  void reloadCurrentPolicy(NewRLQPController & ctl,
                           const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void loadNextPolicy(NewRLQPController & ctl,
                      const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  bool policyLoaded() const { return policy_ && policy_->isLoaded(); }

  const std::string & currentPolicyName() const { return policyManager_.currentName(); }
  const std::string & currentPolicyFolder() const { return policyManager_.current().folder; }
  const std::string & conventionName() const { return observationManager_.conventionName(); }

  int observationSize() const;
  int actionSize() const;

  double policyStepSize() const { return policyStepSize_; }

  bool useQP() const { return useQP_; }
  void setUseQP(bool useQP) { useQP_ = useQP; }

  const Eigen::VectorXd & q_rl() const { return q_rl_; }
  const Eigen::VectorXd & q_zero() const { return q_zero_; }
  const Eigen::VectorXd & currentObservation() const { return currentObservation_; }
  const Eigen::VectorXd & currentAction() const { return currentAction_; }
  const Eigen::VectorXd & currentActionScaled() const { return currentActionScaled_; }
  const Eigen::VectorXd & actionScale() const { return actionScale_; }

  const Eigen::VectorXd & kp() const { return kp_; }
  const Eigen::VectorXd & kd() const { return kd_; }
  const Eigen::VectorXd & kpBase() const { return kpBase_; }
  const Eigen::VectorXd & kdBase() const { return kdBase_; }

  double pdGainsRatio() const { return pdGainsRatio_; }

  void setPDGainsRatio(double ratio,
                       const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  Eigen::Vector3d & command() { return command_; }
  const Eigen::Vector3d & command() const { return command_; }

private:
  void loadPolicy(const std::string & policyName,
                  NewRLQPController & ctl,
                  const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void validatePolicyAgainstRobot(const PolicyConfig & policy, NewRLQPController & ctl) const;

  void configureControl(const PolicyConfig & policy,
                        NewRLQPController & ctl,
                        const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void configureAction(const PolicyConfig & policy, NewRLQPController & ctl);
  void configureNetwork(const PolicyConfig & policy);
  void configureObservations(const PolicyConfig & policy, NewRLQPController & ctl);

  void resetObservationHistory(NewRLQPController & ctl);
  Eigen::VectorXd computeObservation(NewRLQPController & ctl);

  void validateObservationAgainstNetwork() const;
  ObservationContext makeObservationContext(NewRLQPController & ctl);
  mc_rbdyn::Robot & selectedObservationRobot(NewRLQPController & ctl);

  int jointIndexInOrder(const std::vector<std::string> & order, const std::string & joint) const;

  double mapValueOrThrow(const std::map<std::string, double> & values,
                         const std::string & key,
                         const std::string & mapName,
                         const std::string & policyName) const;

private:
  mc_rtc::Configuration controllerConfig_;

  PolicyManager policyManager_;
  ObservationRegistry observationRegistry_;
  ObservationManager observationManager_;
  ObservationConvention activeConvention_;

  std::unique_ptr<RLPolicyInterface> policy_;

  std::string robotName_;
  std::string baseBody_ = "base_link";
  std::string observationSource_ = "realRobot";

  std::vector<std::string> controllerJointOrder_;
  std::vector<std::string> policyJointOrder_;
  std::vector<int> actionToDofMap_;

  Eigen::VectorXd q_rl_;
  Eigen::VectorXd q_zero_;
  Eigen::VectorXd currentObservation_;
  Eigen::VectorXd currentAction_;
  Eigen::VectorXd currentActionScaled_;
  Eigen::VectorXd actionScale_;

  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd kpBase_;
  Eigen::VectorXd kdBase_;

  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();

  bool useQP_ = true;
  double policyStepSize_ = 0.01;
  double policyTimer_ = 0.0;
  double pdGainsRatio_ = 1.0;
};

} // namespace rlqp
