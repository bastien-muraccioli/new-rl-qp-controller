#include "policy/PolicyConfig.h"
#include "common/ConfigurationHelpers.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

namespace rlqp
{

namespace
{

void validateMapContains(const std::map<std::string, double> & values,
                         const std::string & mapName,
                         const std::vector<std::string> & joints,
                         const std::string & policyName)
{
  for(size_t i = 0; i < joints.size(); ++i)
  {
    const std::string & joint = joints[i];

    if(values.find(joint) == values.end())
    {
      mc_rtc::log::error_and_throw(
        "[PolicyConfig:{}] Map '{}' is missing required joint '{}'",
        policyName,
        mapName,
        joint);
    }
  }
}

} // namespace

PolicyConfig PolicyConfig::load(const std::string & policyFolder)
{
  PolicyConfig out;

  out.folder = policyFolder;
  out.policyYamlPath = config::joinPath(policyFolder, "policy.yaml");
  out.observationsYamlPath = config::joinPath(policyFolder, "observations.yaml");

  out.rawPolicy.load(out.policyYamlPath);
  out.rawObservations.load(out.observationsYamlPath);

  out.name = out.rawPolicy("name", config::basenameWithoutExtension(policyFolder));

  const std::string defaultOnnxName = out.name + ".onnx";
  const std::string onnxFile = out.rawPolicy("onnx", defaultOnnxName);
  out.onnxPath = config::joinPath(policyFolder, onnxFile);

  if(out.rawPolicy.has("control"))
  {
    const mc_rtc::Configuration control = out.rawPolicy("control");

    out.useQP = control("use_QP", out.rawPolicy("use_QP", true));
    out.policyStepSize = control("policy_step_size", out.rawPolicy("policy_step_size", 0.02));
    out.physicsStepSize = control("physics_step_size", out.rawPolicy("physics_step_size", 0.001));
    out.kpScale = control("kp_scale", out.rawPolicy("pd_gains_ratio", 1.0));
    out.kdScale = control("kd_scale", std::sqrt(out.kpScale));

    out.kp = config::readOr<std::map<std::string, double> >(control, "kp", std::map<std::string, double>());
    out.kd = config::readOr<std::map<std::string, double> >(control, "kd", std::map<std::string, double>());
  }
  else
  {
    out.useQP = out.rawPolicy("use_QP", true);
    out.policyStepSize = out.rawPolicy("policy_step_size", 0.02);
    out.physicsStepSize = out.rawPolicy("physics_step_size", 0.001);
    out.kpScale = out.rawPolicy("pd_gains_ratio", 1.0);
    out.kdScale = std::sqrt(out.kpScale);
  }

  if(!out.rawPolicy.has("action"))
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] policy.yaml is missing required section 'action'", out.name);
  }

  const mc_rtc::Configuration action = out.rawPolicy("action");

  out.actionJoints = config::readRequired<std::vector<std::string> >(action, "joints", "PolicyConfig:" + out.name);
  out.actionScale = config::readRequired<std::map<std::string, double> >(action, "scale", "PolicyConfig:" + out.name);
  out.defaultPosition = config::readRequired<std::map<std::string, double> >(action, "default_position", "PolicyConfig:" + out.name);

  if(out.kp.empty())
  {
    out.kp = config::readOr<std::map<std::string, double> >(out.rawPolicy, "kp", std::map<std::string, double>());
  }

  if(out.kd.empty())
  {
    out.kd = config::readOr<std::map<std::string, double> >(out.rawPolicy, "kd", std::map<std::string, double>());
  }

  out.validate();
  return out;
}

void PolicyConfig::validate() const
{
  if(name.empty())
  {
    mc_rtc::log::error_and_throw("[PolicyConfig] Policy name cannot be empty");
  }

  if(folder.empty())
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] Policy folder cannot be empty", name);
  }

  if(onnxPath.empty())
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] ONNX path cannot be empty", name);
  }

  if(actionJoints.empty())
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] action.joints cannot be empty", name);
  }

  validateMapContains(actionScale, "action.scale", actionJoints, name);
  validateMapContains(defaultPosition, "action.default_position", actionJoints, name);

  if(!kp.empty())
  {
    validateMapContains(kp, "kp", actionJoints, name);
  }

  if(!kd.empty())
  {
    validateMapContains(kd, "kd", actionJoints, name);
  }

  if(policyStepSize <= 0.0)
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.policy_step_size must be positive", name);
  }

  if(physicsStepSize <= 0.0)
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.physics_step_size must be positive", name);
  }

  if(kpScale <= 0.0)
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.kp_scale must be positive", name);
  }

  if(kdScale <= 0.0)
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.kd_scale must be positive", name);
  }
}

void PolicyManager::load(const mc_rtc::Configuration & controllerConfig)
{
  orderedNames_.clear();
  policies_.clear();
  currentName_.clear();

  const std::string policiesRoot = controllerConfig("policies_root", std::string("policies"));
  const std::vector<std::string> folders =
    config::listPolicies(policiesRoot, {"policy.yaml", "observations.yaml"});

  if(folders.empty())
  {
    mc_rtc::log::error_and_throw(
      "[PolicyManager] No policy folders found in '{}'. Expected subdirectories containing policy.yaml and observations.yaml.",
      policiesRoot);
  }

  for(size_t i = 0; i < folders.size(); ++i)
  {
    PolicyConfig policy = PolicyConfig::load(folders[i]);

    if(policies_.find(policy.name) != policies_.end())
    {
      mc_rtc::log::error_and_throw("[PolicyManager] Duplicate policy name '{}'", policy.name);
    }

    orderedNames_.push_back(policy.name);
    policies_[policy.name] = policy;
  }

  const std::string defaultPolicy = controllerConfig("default_policy", orderedNames_.front());
  select(defaultPolicy);

  mc_rtc::log::success("[PolicyManager] Loaded {} policies from '{}'. Active policy: {}",
                       policies_.size(),
                       policiesRoot,
                       currentName_);
}

bool PolicyManager::empty() const { return policies_.empty(); }

size_t PolicyManager::size() const { return policies_.size(); }

const PolicyConfig & PolicyManager::current() const { return get(currentName_); }

const std::string & PolicyManager::currentName() const { return currentName_; }

const PolicyConfig & PolicyManager::get(const std::string & name) const
{
  std::map<std::string, PolicyConfig>::const_iterator it = policies_.find(name);

  if(it == policies_.end())
  {
    mc_rtc::log::error_and_throw("[PolicyManager] Unknown policy '{}'", name);
  }

  return it->second;
}

void PolicyManager::select(const std::string & name)
{
  get(name);
  currentName_ = name;
}

void PolicyManager::selectNext()
{
  if(orderedNames_.empty())
  {
    mc_rtc::log::error_and_throw("[PolicyManager] Cannot select next policy: no policies loaded");
  }

  std::vector<std::string>::const_iterator it =
    std::find(orderedNames_.begin(), orderedNames_.end(), currentName_);

  if(it == orderedNames_.end())
  {
    currentName_ = orderedNames_.front();
    return;
  }

  ++it;

  if(it == orderedNames_.end())
  {
    it = orderedNames_.begin();
  }

  currentName_ = *it;
}

std::vector<std::string> PolicyManager::names() const { return orderedNames_; }

} // namespace rlqp
