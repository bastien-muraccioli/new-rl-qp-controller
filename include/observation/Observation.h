#pragma once

#include <Eigen/Core>

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/Configuration.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

struct NewRLQPController;

namespace rlqp
{

/**
 * @brief Convention defaults for one training environment.
 *
 * This is not robot geometry. It only describes training-environment naming,
 * aliases, default joint groups/order and default observation parameters.
 */
struct ObservationConvention
{
  std::string name = "mjlab";

  std::vector<std::string> defaultJointOrder;
  std::map<std::string, std::vector<std::string> > jointGroups;

  /** Defaults keyed by exact external observation name first, e.g. joint_pos_rel. */
  std::map<std::string, mc_rtc::Configuration> defaultParameters;

  /** Maps external observation names to internal generic implementations. */
  std::map<std::string, std::string> typeAliases;

  static ObservationConvention fromConfig(const mc_rtc::Configuration & controllerConfig,
                                          const std::string & conventionName);

  std::string resolveType(const std::string & requestedType) const;

  std::vector<std::string> resolveJoints(const mc_rtc::Configuration & parameters,
                                         const std::vector<std::string> & fallback) const;

  mc_rtc::Configuration parametersFor(const std::string & requestedType,
                                      const std::string & internalType,
                                      const mc_rtc::Configuration & localParameters) const;
};

/**
 * @brief Runtime data available to observation implementations.
 */
struct ObservationContext
{
  NewRLQPController & ctl;
  mc_rbdyn::Robot & robot;
  mc_rbdyn::Robot & realRobot;

  std::string baseBody;

  /** mc_rtc controller order: robot().refJointOrder(). */
  const std::vector<std::string> & controllerJointOrder;

  /** Policy action order from policy.yaml/action/joints. */
  const std::vector<std::string> & policyJointOrder;

  /** Default pose expanded in controllerJointOrder. */
  const Eigen::VectorXd & qZeroControllerOrder;

  /** Last raw action in policyJointOrder. */
  const Eigen::VectorXd & lastActionPolicyOrder;

  const Eigen::Vector3d & command;

  ObservationConvention convention;
};

struct ObservationConfig
{
  std::string type;
  std::string requestedType;
  std::string name;
  int history = 1;
  mc_rtc::Configuration parameters;
  mc_rtc::Configuration raw;
};

class Observation
{
public:
  Observation(const ObservationConfig & config, const ObservationConvention & convention);
  virtual ~Observation();

  const std::string & type() const { return config_.type; }
  const std::string & requestedType() const { return config_.requestedType; }
  const std::string & name() const { return config_.name; }
  int history() const { return config_.history; }

  virtual void configure(const ObservationContext & context) = 0;
  virtual int size() const = 0;
  virtual void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const = 0;

protected:
  template<typename T>
  T readParameter(const mc_rtc::Configuration & parameters,
                  const std::string & key,
                  const T & fallback) const
  {
    if(!parameters.has(key))
    {
      return fallback;
    }
    return parameters(key, fallback);
  }

  std::vector<int> resolveJointIndices(const ObservationContext & context,
                                       const std::vector<std::string> & joints) const;

  Eigen::VectorXd readScaleVector(const mc_rtc::Configuration & parameters,
                                  const std::string & key,
                                  int size,
                                  double fallback) const;

protected:
  ObservationConfig config_;
  ObservationConvention convention_;
};

class ObservationRegistry
{
public:
  using Creator = std::function<std::shared_ptr<Observation>(const ObservationConfig &, const ObservationConvention &)>;

  void registerType(const std::string & type, Creator creator);
  std::shared_ptr<Observation> create(const ObservationConfig & config,
                                      const ObservationConvention & convention) const;

  std::vector<std::string> knownTypes() const;

private:
  std::map<std::string, Creator> creators_;
};

ObservationRegistry makeDefaultObservationRegistry();

} // namespace rlqp
