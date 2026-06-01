#include "policy/PolicyConfig.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

namespace rlqp
{

namespace
{

std::string joinPath(const std::string & lhs, const std::string & rhs)
{
  if(lhs.empty())
  {
    return rhs;
  }

  if(lhs[lhs.size() - 1] == '/')
  {
    return lhs + rhs;
  }

  return lhs + "/" + rhs;
}

std::string basenameWithoutExtension(const std::string & path)
{
  std::string base = path;

  const size_t slash = base.find_last_of('/');
  if(slash != std::string::npos)
  {
    base = base.substr(slash + 1);
  }

  const size_t dot = base.find_last_of('.');
  if(dot != std::string::npos)
  {
    base = base.substr(0, dot);
  }

  return base;
}

std::map<std::string, double> readRequiredMap(const mc_rtc::Configuration & config,
                                              const std::string & key,
                                              const std::string & policyName)
{
  if(!config.has(key))
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] Missing required map '{}'", policyName, key);
  }

  return config(key, std::map<std::string, double>());
}

std::map<std::string, double> readOptionalMap(const mc_rtc::Configuration & config,
                                              const std::string & key)
{
  if(!config.has(key))
  {
    return std::map<std::string, double>();
  }

  return config(key, std::map<std::string, double>());
}

std::vector<std::string> readRequiredStringVector(const mc_rtc::Configuration & config,
                                                  const std::string & key,
                                                  const std::string & policyName)
{
  if(!config.has(key))
  {
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] Missing required list '{}'", policyName, key);
  }

  return config(key, std::vector<std::string>());
}

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

std::vector<std::string> readPolicyFolders(const mc_rtc::Configuration & controllerConfig)
{
  if(controllerConfig.has("policy_folders"))
  {
    return controllerConfig("policy_folders", std::vector<std::string>());
  }

  if(controllerConfig.has("policies_root") && controllerConfig.has("policy_names"))
  {
    const std::string root = controllerConfig("policies_root", std::string("policy"));
    const std::vector<std::string> names =
      controllerConfig("policy_names", std::vector<std::string>());

    std::vector<std::string> folders;
    for(size_t i = 0; i < names.size(); ++i)
    {
      folders.push_back(joinPath(root, names[i]));
    }

    return folders;
  }

  return std::vector<std::string>();
}

} // namespace

PolicyConfig PolicyConfig::load(const std::string & policyFolder)
{
  PolicyConfig out;

  out.folder = policyFolder;
  out.policyYamlPath = joinPath(policyFolder, "policy.yaml");
  out.observationsYamlPath = joinPath(policyFolder, "observations.yaml");

  out.rawPolicy.load(out.policyYamlPath);
  out.rawObservations.load(out.observationsYamlPath);

  out.name = out.rawPolicy("name", basenameWithoutExtension(policyFolder));

  const std::string defaultOnnxName = out.name + ".onnx";
  const std::string onnxFile = out.rawPolicy("onnx", defaultOnnxName);
  out.onnxPath = joinPath(policyFolder, onnxFile);

  if(out.rawPolicy.has("control"))
  {
    const mc_rtc::Configuration control = out.rawPolicy("control");

    out.useQP = control("use_QP", out.rawPolicy("use_QP", true));
    out.policyStepSize = control("policy_step_size", out.rawPolicy("policy_step_size", 0.02));
    out.physicsStepSize = control("physics_step_size", out.rawPolicy("physics_step_size", 0.001));
    out.kpScale = control("kp_scale", out.rawPolicy("pd_gains_ratio", 1.0));
    out.kdScale = control("kd_scale", std::sqrt(out.kpScale));

    out.kp = readOptionalMap(control, "kp");
    out.kd = readOptionalMap(control, "kd");
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

  out.actionJoints = readRequiredStringVector(action, "joints", out.name);
  out.actionScale = readRequiredMap(action, "scale", out.name);
  out.defaultPosition = readRequiredMap(action, "default_position", out.name);

  if(out.kp.empty())
  {
    out.kp = readOptionalMap(out.rawPolicy, "kp");
  }

  if(out.kd.empty())
  {
    out.kd = readOptionalMap(out.rawPolicy, "kd");
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

  const std::vector<std::string> folders = readPolicyFolders(controllerConfig);

  if(folders.empty())
  {
    mc_rtc::log::error_and_throw(
      "[PolicyManager] No policies configured. Expected either 'policy_folders' or 'policies_root' + 'policy_names'.");
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

  const std::string defaultPolicy =
    controllerConfig("default_policy", orderedNames_.front());

  select(defaultPolicy);

  mc_rtc::log::success("[PolicyManager] Loaded {} policies. Active policy: {}",
                       policies_.size(),
                       currentName_);
}

bool PolicyManager::empty() const
{
  return policies_.empty();
}

size_t PolicyManager::size() const
{
  return policies_.size();
}

const PolicyConfig & PolicyManager::current() const
{
  return get(currentName_);
}

const std::string & PolicyManager::currentName() const
{
  return currentName_;
}

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

std::vector<std::string> PolicyManager::names() const
{
  return orderedNames_;
}

} // namespace rlqp
