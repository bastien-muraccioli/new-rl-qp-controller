#include "policy/RLPolicyRuntime.h"

#include "H1RLQPController.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace rlqp
{

RLPolicyRuntime::RLPolicyRuntime()
{
}

void RLPolicyRuntime::configure(const mc_rtc::Configuration & controllerConfig,
                                H1RLQPController & ctl,
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
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Invalid robot.observation_source '{}'. Expected 'realRobot' or 'robot'.", observationSource_);

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

  policyManager_.load(controllerConfig_, ctl.jointNames);
  loadPolicy(policyManager_.currentName(), ctl, torqueTask);
}

void RLPolicyRuntime::reset(H1RLQPController & ctl)
{
  phaseElapsedTime_ = 0.0;
  phaseNormalized_ = 0.0;
  resetObservationHistory(ctl);
  policyTimer_ = policyStepSize_;
  policyUpdateCount_ = 0;
}

void RLPolicyRuntime::runPolicyStepIfNeeded(H1RLQPController & ctl, double dt)
{
  if(!policyLoaded())
  {
    mc_rtc::log::error("[RLPolicyRuntime] Cannot run policy: no ONNX policy loaded");
    return;
  }

  policyTimer_ += dt;
  phaseElapsedTime_ += dt;

  if(phasePeriod_ > 0.0)
    phaseNormalized_ = std::fmod(phaseElapsedTime_ / phasePeriod_, 1.0);

  if(policyTimer_ < policyStepSize_) { return; }

  currentObservation_ = computeObservation(ctl);

  if(currentObservation_.size() != policy_->getObservationSize())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Observation size mismatch. ObservationManager produced {}, ONNX expects {}.",
      currentObservation_.size(), policy_->getObservationSize());

  currentAction_ = policy_->predict(currentObservation_);
  ++policyUpdateCount_;
  // mc_rtc::log::warning("TEST {}", currentAction_);

  if(currentAction_.size() != static_cast<int>(actionToControllerMap_.size()))
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Action size mismatch. ONNX produced {}, active action mapping expects {} joints.",
      currentAction_.size(), actionToControllerMap_.size());

  currentActionScaled_.setZero();
  q_rl_ = q_zero_;

  for(int actionIndex = 0; actionIndex < currentAction_.size(); ++actionIndex)
  {
    const int dofIndex = actionToControllerMap_[static_cast<size_t>(actionIndex)];

    currentActionScaled_(dofIndex) = actionScale_(dofIndex) * currentAction_(actionIndex);
    if(std::find(controlledActionControllerIndices_.begin(),
                 controlledActionControllerIndices_.end(),
                 dofIndex) != controlledActionControllerIndices_.end())
    {
      q_rl_(dofIndex) = currentActionScaled_(dofIndex) + q_zero_(dofIndex);
    }
  }

  policyTimer_ = std::fmod(policyTimer_, policyStepSize_);
}

void RLPolicyRuntime::reloadCurrentPolicy(H1RLQPController & ctl,
                                          const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  loadPolicy(policyManager_.currentName(), ctl, torqueTask);
}

void RLPolicyRuntime::loadPolicyByName(const std::string & policyName,
                                       H1RLQPController & ctl,
                                       const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  if(policyName == policyManager_.currentName()) { return; }

  loadPolicy(policyName, ctl, torqueTask);
}

void RLPolicyRuntime::loadNextPolicy(H1RLQPController & ctl,
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
                                 H1RLQPController & ctl,
                                 const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  policyManager_.select(policyName);

  const PolicyConfig & policy = policyManager_.current();

  mc_rtc::log::info("[RLPolicyRuntime] Loading policy '{}'", policy.name);

  configureControl(policy, ctl, torqueTask);

  const std::string conventionName =
    policy.observationsConfiguration("training_convention", std::string("mjlab"));
  activeConvention_ = ObservationConvention::fromConfig(controllerConfig_, conventionName);

  configureAction(policy, ctl);
  configureNetwork(policy);
  configureObservations(policy, ctl);

  phaseElapsedTime_ = 0.0;
  phaseNormalized_ = 0.0;
  resetObservationHistory(ctl);
  validateObservationAgainstNetwork();

  policyTimer_ = policyStepSize_;
  policyUpdateCount_ = 0;

  mc_rtc::log::success(
    "[RLPolicyRuntime] Policy '{}' loaded. Observation size: {}, action size: {}",
    policy.name, currentObservation_.size(), currentAction_.size());
}

void RLPolicyRuntime::configureControl(const PolicyConfig & policy,
                                       H1RLQPController & ctl,
                                       const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  useQP_ = policy.useQP;
  policyStepSize_ = policy.policyStepSize;
  pdGainsRatio_ = policy.pdGainsRatio;

  if(policyStepSize_ + 1e-8 < ctl.timeStep)
  {
    mc_rtc::log::warning(
        "[RLPolicyRuntime:{}] Policy period ({:.3f} ms) is shorter than the controller step ({:.3f} ms); "
        "the requested policy rate cannot be reached.",
        policy.name, 1000.0 * policyStepSize_, 1000.0 * ctl.timeStep);
  }

  phasePeriod_ = policy.observationsConfiguration("phase_period", 1.0);
  
  if(policy.policyConfiguration.has("control"))
  {
    const mc_rtc::Configuration control = policy.policyConfiguration("control");

    if(control.has("phase_period"))
      control("phase_period", phasePeriod_);
  }
  if(phasePeriod_ <= 0.0)
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] phase_period must be positive, got {}",
      policy.name, phasePeriod_);

  kpBase_.setZero();
  kdBase_.setZero();

  kpBase_ = Eigen::Map<const Eigen::VectorXd>(policy.kp.data(), policy.kp.size());
  kdBase_ = Eigen::Map<const Eigen::VectorXd>(policy.kd.data(), policy.kp.size());

  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = std::sqrt(pdGainsRatio_) * kdBase_;

  if(torqueTask)
  {
    torqueTask->setStiffness(kp_);
    torqueTask->setDamping(kd_);
  }
}

void RLPolicyRuntime::configureAction(const PolicyConfig & policy,
                                      H1RLQPController & ctl)
{
  // Resolve action.joints to controller indices.
  // This is the ONNX action vector layout/size. Do not use controlled_joints here,
  // otherwise policies that output all joints but only apply legs will get a size mismatch.
  mc_rtc::Configuration actionSelector;
  
  if(!policy.actionJointGroup.empty())
    actionSelector.add("joints", policy.actionJointGroup);

  // Fallback: all controller joints in controller order
  std::vector<int> fullFallback(controllerJointOrder_.size());
  std::iota(fullFallback.begin(), fullFallback.end(), 0);

  actionToControllerMap_ = activeConvention_.resolveJointControllerIndices(
    actionSelector, controllerJointOrder_, fullFallback);

  // Resolve action.controlled_joints to controller indices.
  // This is the subset of the ONNX outputs that is actually applied to q_rl.
  mc_rtc::Configuration controlledSelector;
  controlledSelector.add("joints", policy.controlledJointGroup);
  controlledActionControllerIndices_ = activeConvention_.resolveJointControllerIndices(
    controlledSelector, controllerJointOrder_, actionToControllerMap_);

  // controlled_joints must be a subset of action.joints: every controlled joint must
  // correspond to one output in the ONNX action vector.
  for(const int controlledIndex : controlledActionControllerIndices_)
  {
    if(std::find(actionToControllerMap_.begin(), actionToControllerMap_.end(), controlledIndex)
       == actionToControllerMap_.end())
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] action.controlled_joints contains controller joint index {}, "
        "but this joint is not present in action.joints",
        policy.name,
        controlledIndex);
    }
  }

  mc_rtc::log::info("[RLPolicyRuntime:{}] action.joints='{}' -> controller indices {}",
                    policy.name, policy.actionJointGroup, actionToControllerMap_);
  mc_rtc::log::info("[RLPolicyRuntime:{}] action.controlled_joints='{}' -> controller indices {}",
                    policy.name, policy.controlledJointGroup, controlledActionControllerIndices_);

  q_zero_.setZero();
  q_rl_.setZero();
  actionScale_.setOnes();
  currentActionScaled_.setZero();

  actionScale_ = Eigen::Map<const Eigen::VectorXd>(policy.actionScale.data(), policy.actionScale.size());

  if (!policy.defaultPosition.empty())
  {
    q_zero_ = Eigen::Map<const Eigen::VectorXd>(policy.defaultPosition.data(), policy.defaultPosition.size());
  }
  else
  {
    size_t i = 0;
    std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = ctl.getPostureTask(ctl.robot().name());
    auto posture = FSMPostureTask->posture();
    for (const auto& j : ctl.robot().mb().joints()) {
      const std::string& joint_name = j.name();
      if (j.type() == rbd::Joint::Type::Rev) {
        if (const auto& t = posture[ctl.robot().jointIndexByName(joint_name)]; !t.empty()) {
          q_zero_(i) =t[0];
          i++;
        }
      }
    }
  }

  for(size_t actionIndex = 0; actionIndex < actionToControllerMap_.size(); ++actionIndex)
  {
    const int dofIndex = actionToControllerMap_[actionIndex];
    const std::string & joint = controllerJointOrder_[static_cast<size_t>(dofIndex)];

    if(!ctl.robot().hasJoint(joint))
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] Resolved action joint '{}' does not exist on robot",
        policy.name,
        joint);
    }
  }

  q_rl_ = q_zero_;
}

void RLPolicyRuntime::configureNetwork(const PolicyConfig & policy)
{
  try
  {
    policy_.reset(new RLPolicyInterface(policy.onnxPath));

    if(!policy_ || !policy_->isLoaded())
      mc_rtc::log::error_and_throw("[RLPolicyRuntime:{}] RL policy creation failed for '{}'",
        policy.name, policy.onnxPath);
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error_and_throw("[RLPolicyRuntime:{}] Failed to load ONNX policy '{}': {}",
      policy.name, policy.onnxPath, e.what()); 
  }

  currentAction_ = Eigen::VectorXd::Zero(policy_->getActionSize());

  if(static_cast<int>(actionToControllerMap_.size()) != policy_->getActionSize())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime:{}] Resolved action joint count ({}) does not match ONNX action size ({})",
      policy.name, actionToControllerMap_.size(), policy_->getActionSize());
}

void RLPolicyRuntime::configureObservations(const PolicyConfig & policy,
                                            H1RLQPController & ctl)
{
  observationManager_.load(policy.observationsConfiguration, controllerConfig_, observationRegistry_);
  observationManager_.configure(makeObservationContext(ctl));
  currentObservation_ = Eigen::VectorXd::Zero(policy_->getObservationSize());
}

void RLPolicyRuntime::resetObservationHistory(H1RLQPController & ctl)
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

Eigen::VectorXd RLPolicyRuntime::computeObservation(H1RLQPController & ctl)
{
  ObservationContext context = makeObservationContext(ctl);
  return observationManager_.compute(context);
}

void RLPolicyRuntime::validateObservationAgainstNetwork() const
{
  if(!policyLoaded())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Cannot validate observation: no policy loaded");

  if(currentObservation_.size() != policy_->getObservationSize())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] ObservationManager dimension ({}) does not match ONNX input size ({})",
      currentObservation_.size(), policy_->getObservationSize());
}

ObservationContext RLPolicyRuntime::makeObservationContext(H1RLQPController & ctl)
{
  return ObservationContext{
    selectedObservationRobot(ctl),
    baseBody_,
    controllerJointOrder_,
    actionToControllerMap_,
    q_zero_,
    currentAction_,
    command_,
    phaseNormalized_,
    activeConvention_};
}

mc_rbdyn::Robot & RLPolicyRuntime::selectedObservationRobot(H1RLQPController & ctl)
{
  if(observationSource_ == "robot")
    return ctl.robot();
  return ctl.realRobot(robotName_);
}


void RLPolicyRuntime::addLogObs(H1RLQPController & ctl)
{
  for(const auto & entry : observationManager_.entries())
  {
    const std::string baseName = "H1RLQPController_Observations_" + entry.observation->name();
    const size_t size = entry.historyBuffer.size();
    if(size == 1)
    {
      ctl.logger().addLogEntry(baseName, [entry]() { return entry.historyBuffer[0]; });
    }
    else if(size > 1)
    {
      for(size_t i = 0; i < size; ++i)
      {
        const size_t bufferIndex = observationManager_.newest_first() ? i : size - 1 - i;
        const std::string suffix = i == 0 ? "_t" : "_t-" + std::to_string(i);
        ctl.logger().addLogEntry(baseName + suffix,
                                 [entry, bufferIndex]() { return entry.historyBuffer[bufferIndex]; });
      }
    }
  }
}

} // namespace rlqp
