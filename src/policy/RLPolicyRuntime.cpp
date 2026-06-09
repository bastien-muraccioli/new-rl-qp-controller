#include "policy/RLPolicyRuntime.h"

#include "NewRLQPController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace rlqp
{

RLPolicyRuntime::RLPolicyRuntime()
{
}

void RLPolicyRuntime::configure(const mc_rtc::Configuration & controllerConfig,
                                NewRLQPController & ctl,
                                const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  controllerConfig_ = controllerConfig;
  robotName_ = ctl.robot().name();
  controllerJointOrder_ = ctl.robot().refJointOrder();

  if(controllerConfig_.has("robot"))
  {
    baseBody_ = controllerConfig_("robot")("base_body", std::string("base_link"));
    observationSource_ = controllerConfig_("robot")("observation_source", std::string("realRobot"));
  }

  if(observationSource_ != "realRobot" && observationSource_ != "robot")
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime] Invalid robot.observation_source '{}'. Expected 'realRobot' or 'robot'.",
      observationSource_);
  }

  const int nbActuatedJoints = static_cast<int>(controllerJointOrder_.size());

  q_rl_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  q_zero_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  currentActionScaled_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  actionScale_ = Eigen::VectorXd::Zero(nbActuatedJoints);

  kp_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kd_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kpBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kdBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);

  observationRegistry_ = makeDefaultObservationRegistry();

  policyManager_.load(controllerConfig_);
  loadPolicy(policyManager_.currentName(), ctl, torqueTask);
}

void RLPolicyRuntime::reset(NewRLQPController & ctl)
{
  phaseElapsedTime_ = 0.0;
  phaseNormalized_ = 0.0;
  resetObservationHistory(ctl);
  policyTimer_ = policyStepSize_;
}

void RLPolicyRuntime::runPolicyStepIfNeeded(NewRLQPController & ctl, double dt)
{
  if(!policyLoaded())
  {
    mc_rtc::log::error("[RLPolicyRuntime] Cannot run policy: no ONNX policy loaded");
    return;
  }

  policyTimer_ += dt;
  phaseElapsedTime_ += dt;

  if(phasePeriod_ > 0.0)
  {
    phaseNormalized_ = std::fmod(phaseElapsedTime_ / phasePeriod_, 1.0);
  }

  if(policyTimer_ < policyStepSize_) { return; }

  currentObservation_ = computeObservation(ctl);

  if(currentObservation_.size() != policy_->getObservationSize())
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime] Observation size mismatch. ObservationManager produced {}, ONNX expects {}.",
      currentObservation_.size(),
      policy_->getObservationSize());
  }

  currentAction_ = policy_->predict(currentObservation_);

  if(currentAction_.size() != static_cast<int>(policyJointOrder_.size()))
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime] Action size mismatch. ONNX produced {}, policy action.joints has {} joints.",
      currentAction_.size(),
      policyJointOrder_.size());
  }

  currentActionScaled_.setZero();

  for(int actionIndex = 0; actionIndex < currentAction_.size(); ++actionIndex)
  {
    const int dofIndex = actionToDofMap_[static_cast<size_t>(actionIndex)];

    currentActionScaled_(dofIndex) = actionScale_(dofIndex) * currentAction_(actionIndex);
    q_rl_(dofIndex) = currentActionScaled_(dofIndex) + q_zero_(dofIndex);
  }

  policyTimer_ = 0.0;
}

void RLPolicyRuntime::reloadCurrentPolicy(NewRLQPController & ctl,
                                          const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  loadPolicy(policyManager_.currentName(), ctl, torqueTask);
}

void RLPolicyRuntime::loadNextPolicy(NewRLQPController & ctl,
                                     const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  policyManager_.selectNext();
  loadPolicy(policyManager_.currentName(), ctl, torqueTask);
}

int RLPolicyRuntime::observationSize() const
{
  if(policyLoaded()) { return policy_->getObservationSize(); }
  return static_cast<int>(currentObservation_.size());
}

int RLPolicyRuntime::actionSize() const
{
  if(policyLoaded()) { return policy_->getActionSize(); }
  return static_cast<int>(currentAction_.size());
}

void RLPolicyRuntime::setPDGainsRatio(double ratio,
                                      const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  if(ratio < 0.0)
  {
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] PD gains ratio must be non-negative, got {}", ratio);
  }

  pdGainsRatio_ = ratio;
  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = std::sqrt(pdGainsRatio_) * kdBase_;

  if(torqueTask)
  {
    torqueTask->setStiffness(kp_);
    torqueTask->setDamping(kd_);
  }
}

void RLPolicyRuntime::loadPolicy(const std::string & policyName,
                                 NewRLQPController & ctl,
                                 const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  policyManager_.select(policyName);

  const PolicyConfig & policy = policyManager_.current();

  mc_rtc::log::info("[RLPolicyRuntime] Loading policy '{}'", policy.name);

  validatePolicyAgainstRobot(policy, ctl);

  configureControl(policy, ctl, torqueTask);
  configureAction(policy, ctl);
  configureNetwork(policy);
  configureObservations(policy, ctl);

  phaseElapsedTime_ = 0.0;
  phaseNormalized_ = 0.0;
  resetObservationHistory(ctl);
  validateObservationAgainstNetwork();

  policyTimer_ = policyStepSize_;

  mc_rtc::log::success(
    "[RLPolicyRuntime] Policy '{}' loaded. Observation size: {}, action size: {}",
    policy.name,
    currentObservation_.size(),
    currentAction_.size());
}

void RLPolicyRuntime::validatePolicyAgainstRobot(const PolicyConfig & policy,
                                                 NewRLQPController & ctl) const
{
  for(size_t i = 0; i < policy.actionJoints.size(); ++i)
  {
    const std::string & joint = policy.actionJoints[i];

    if(!ctl.robot().hasJoint(joint))
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] action.joints contains unknown robot joint '{}'",
        policy.name,
        joint);
    }

    if(jointIndexInOrder(controllerJointOrder_, joint) < 0)
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] action joint '{}' is not in controllerJointOrder",
        policy.name,
        joint);
    }
  }
}

void RLPolicyRuntime::configureControl(const PolicyConfig & policy,
                                       NewRLQPController & ctl,
                                       const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  useQP_ = policy.useQP;
  policyStepSize_ = policy.policyStepSize;
  pdGainsRatio_ = policy.kpScale;

  phasePeriod_ = policy.rawObservations("phase_period", 1.0);
  if(policy.rawPolicy.has("control"))
  {
    phasePeriod_ = policy.rawPolicy("control")("phase_period", phasePeriod_);
  }

  if(phasePeriod_ <= 0.0)
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] phase_period must be positive, got {}",
      policy.name,
      phasePeriod_);
  }

  if(policy.physicsStepSize - ctl.timeStep > 1e-6)
  {
    mc_rtc::log::warning(
      "[RLPolicyRuntime:{}] physics_step_size ({}) is larger than controller timestep ({}).",
      policy.name,
      policy.physicsStepSize,
      ctl.timeStep);
  }

  kpBase_.setZero();
  kdBase_.setZero();

  for(size_t i = 0; i < controllerJointOrder_.size(); ++i)
  {
    const std::string & joint = controllerJointOrder_[i];

    if(policy.kp.find(joint) != policy.kp.end())
    {
      kpBase_(static_cast<int>(i)) = mapValueOrThrow(policy.kp, joint, "kp", policy.name);
    }

    if(policy.kd.find(joint) != policy.kd.end())
    {
      kdBase_(static_cast<int>(i)) = mapValueOrThrow(policy.kd, joint, "kd", policy.name);
    }
  }

  kp_ = policy.kpScale * kpBase_;
  kd_ = policy.kdScale * kdBase_;

  if(torqueTask)
  {
    torqueTask->setStiffness(kp_);
    torqueTask->setDamping(kd_);
  }
}

void RLPolicyRuntime::configureAction(const PolicyConfig & policy,
                                      NewRLQPController &)
{
  policyJointOrder_ = policy.actionJoints;

  actionToDofMap_.clear();
  actionToDofMap_.resize(policyJointOrder_.size(), -1);

  q_zero_.setZero();
  q_rl_.setZero();
  actionScale_.setZero();
  currentActionScaled_.setZero();

  for(size_t actionIndex = 0; actionIndex < policyJointOrder_.size(); ++actionIndex)
  {
    const std::string & joint = policyJointOrder_[actionIndex];
    const int dofIndex = jointIndexInOrder(controllerJointOrder_, joint);

    if(dofIndex < 0)
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] Could not map action joint '{}' to controllerJointOrder",
        policy.name,
        joint);
    }

    actionToDofMap_[actionIndex] = dofIndex;

    actionScale_(dofIndex) = mapValueOrThrow(policy.actionScale, joint, "action.scale", policy.name);
    q_zero_(dofIndex) = mapValueOrThrow(policy.defaultPosition, joint, "action.default_position", policy.name);
  }

  q_rl_ = q_zero_;
}

void RLPolicyRuntime::configureNetwork(const PolicyConfig & policy)
{
  try
  {
    policy_.reset(new RLPolicyInterface(policy.onnxPath));

    if(!policy_ || !policy_->isLoaded())
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] RL policy creation failed for '{}'",
        policy.name,
        policy.onnxPath);
    }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] Failed to load ONNX policy '{}': {}",
      policy.name,
      policy.onnxPath,
      e.what());
  }

  currentAction_ = Eigen::VectorXd::Zero(policy_->getActionSize());

  if(static_cast<int>(policyJointOrder_.size()) != policy_->getActionSize())
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] action.joints size ({}) does not match ONNX action size ({})",
      policy.name,
      policyJointOrder_.size(),
      policy_->getActionSize());
  }
}

void RLPolicyRuntime::configureObservations(const PolicyConfig & policy,
                                            NewRLQPController & ctl)
{
  const std::string conventionName =
    policy.rawObservations("training_convention", std::string("mjlab"));

  activeConvention_ = ObservationConvention::fromConfig(controllerConfig_, conventionName);

  observationManager_.load(policy.rawObservations, controllerConfig_, observationRegistry_);
  observationManager_.configure(makeObservationContext(ctl));

  currentObservation_ = Eigen::VectorXd::Zero(policy_->getObservationSize());
}

void RLPolicyRuntime::resetObservationHistory(NewRLQPController & ctl)
{
  if(!policyLoaded())
  {
    mc_rtc::log::error("[RLPolicyRuntime] Cannot reset observation history: no policy loaded");
    return;
  }

  ObservationContext context = makeObservationContext(ctl);

  observationManager_.updateHistory(context);
  currentObservation_ = observationManager_.compute(context);
}

Eigen::VectorXd RLPolicyRuntime::computeObservation(NewRLQPController & ctl)
{
  ObservationContext context = makeObservationContext(ctl);
  return observationManager_.compute(context);
}

void RLPolicyRuntime::validateObservationAgainstNetwork() const
{
  if(!policyLoaded())
  {
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Cannot validate observation: no policy loaded");
  }

  if(currentObservation_.size() != policy_->getObservationSize())
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime] ObservationManager dimension ({}) does not match ONNX input size ({})",
      currentObservation_.size(),
      policy_->getObservationSize());
  }
}

ObservationContext RLPolicyRuntime::makeObservationContext(NewRLQPController & ctl)
{
  return ObservationContext{
    selectedObservationRobot(ctl),
    baseBody_,
    controllerJointOrder_,
    policyJointOrder_,
    q_zero_,
    currentAction_,
    command_,
    phaseNormalized_,
    activeConvention_};
}

mc_rbdyn::Robot & RLPolicyRuntime::selectedObservationRobot(NewRLQPController & ctl)
{
  if(observationSource_ == "robot")
  {
    return ctl.robot();
  }

  return ctl.realRobot(robotName_);
}

int RLPolicyRuntime::jointIndexInOrder(const std::vector<std::string> & order,
                                       const std::string & joint) const
{
  std::vector<std::string>::const_iterator it = std::find(order.begin(), order.end(), joint);

  if(it == order.end()) { return -1; }

  return static_cast<int>(std::distance(order.begin(), it));
}

double RLPolicyRuntime::mapValueOrThrow(const std::map<std::string, double> & values,
                                        const std::string & key,
                                        const std::string & mapName,
                                        const std::string & policyName) const
{
  std::map<std::string, double>::const_iterator it = values.find(key);

  if(it == values.end())
  {
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] Missing '{}' value for joint '{}'",
      policyName,
      mapName,
      key);
  }

  return it->second;
}

} // namespace rlqp
