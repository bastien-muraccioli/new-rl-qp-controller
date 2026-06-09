#pragma once

#include <mc_rtc/Configuration.h>

#include <map>
#include <string>
#include <vector>

namespace rlqp
{

struct PolicyConfig
{
  std::string name;
  std::string folder;
  std::string onnxPath;
  std::string policyYamlPath;
  std::string observationsYamlPath;

  bool useQP = true;
  double policyStepSize = 0.02;
  double physicsStepSize = 0.001;
  double kpScale = 1.0;
  double kdScale = 1.0;

  /**
   * @brief Optional joint-group selector from policy.yaml/action/joints.
   *
   * Filled only when action.joints is written as a string, e.g. ``joints: legs``.
   * The group is resolved by RLPolicyRuntime using the active convention.
   */
  std::string actionJointGroup;

  /** @brief Explicit ordered action joints when action.joints is a YAML list. */
  std::vector<std::string> actionJoints;

  /** @brief Per-joint action scale overrides. Missing entries use scale 1.0. */
  std::map<std::string, double> actionScale;

  /** @brief Per-joint default target pose overrides. Missing entries use 0.0. */
  std::map<std::string, double> defaultPosition;
  std::map<std::string, double> kp;
  std::map<std::string, double> kd;

  mc_rtc::Configuration rawPolicy;
  mc_rtc::Configuration rawObservations;

  static PolicyConfig load(const std::string & policyFolder);

  void validate() const;
};

class PolicyManager
{
public:
  void load(const mc_rtc::Configuration & controllerConfig);

  bool empty() const;
  size_t size() const;

  const PolicyConfig & current() const;
  const std::string & currentName() const;

  const PolicyConfig & get(const std::string & name) const;

  void select(const std::string & name);
  void selectNext();

  std::vector<std::string> names() const;

private:
  std::vector<std::string> orderedNames_;
  std::map<std::string, PolicyConfig> policies_;
  std::string currentName_;
};

} // namespace rlqp
