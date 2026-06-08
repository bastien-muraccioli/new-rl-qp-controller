#pragma once

#include <Eigen/Core>

#include <mc_rbdyn/Robot.h>
#include <mc_rtc/Configuration.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rlqp
{

/**
 * @brief Training-environment convention defaults.
 *
 * A convention describes how an external training environment names
 * observations and which default parameters it expects. It should not contain
 * robot-specific information such as the base body name.
 *
 * Examples:
 * - mjlab exposes joint_pos_rel, implemented internally by joint_pos.
 * - mjlab exposes generated_commands, implemented internally by command.
 */
struct ObservationConvention
{
  /** @brief Convention name, e.g. "mjlab" or "isaaclab". */
  std::string name = "mjlab";

  /** @brief Named joint groups, e.g. all, legs, arms. */
  std::map<std::string, std::vector<std::string> > jointGroups;

  /**
   * @brief Default parameters keyed by exact external observation name first.
   *
   * Example: mjlab::joint_pos_rel can have defaults that differ from a generic
   * joint_pos implementation.
   */
  std::map<std::string, mc_rtc::Configuration> defaultParameters;

  /** @brief Maps external observation names to internal generic implementations. */
  std::map<std::string, std::string> typeAliases;

  /**
   * @brief Load one convention block.
   *
   * The preferred location is policies/conventions.yaml, found from
   * controllerConfig["policies_root"]. For compatibility, this also accepts a
   * controllerConfig that directly contains a "conventions" block.
   */
  static ObservationConvention fromConfig(const mc_rtc::Configuration & controllerConfig,
                                          const std::string & conventionName);

  /** @brief Resolve an external YAML type to an internal registered observation type. */
  std::string resolveType(const std::string & requestedType) const;

  /**
   * @brief Resolve an observation joint selector to explicit joint names.
   *
   * The joints parameter may be absent, a joint-group name, or an explicit
   * list. If absent, the fallback is used. In this controller, the fallback is
   * normally the policy action joint list.
   */
  std::vector<std::string> resolveJoints(const mc_rtc::Configuration & parameters,
                                         const std::vector<std::string> & fallback) const;

  /**
   * @brief Resolve final parameters for one observation.
   *
   * Convention defaults are first searched with the exact requested type, then
   * with the internal type. Local parameters from observations.yaml always
   * override convention defaults.
   */
  mc_rtc::Configuration resolveObservationParameters(const std::string & requestedType,
                                                     const std::string & internalType,
                                                     const mc_rtc::Configuration & localParameters) const;
};

/**
 * @brief Runtime data passed to observations during configure() and compute().
 *
 * The observationRobot is selected once by RLPolicyRuntime according to
 * robot.observation_source in the controller configuration:
 * - realRobot: use ctl.realRobot(...), default
 * - robot: use ctl.robot()
 *
 * Joint-order convention:
 * - controllerJointOrder is mc_rtc's robot().refJointOrder().
 * - policyJointOrder is policy.yaml/action/joints.
 * - qZeroControllerOrder is stored in controllerJointOrder.
 * - lastActionPolicyOrder is stored in policyJointOrder.
 */
struct ObservationContext
{
  /** @brief Selected robot state used to compute observations. */
  mc_rbdyn::Robot & observationRobot;

  /** @brief Robot base body used by base velocity and projected gravity observations. */
  std::string baseBody;

  /** @brief mc_rtc controller order: robot().refJointOrder(). */
  const std::vector<std::string> & controllerJointOrder;

  /** @brief Policy action order from policy.yaml/action/joints. */
  const std::vector<std::string> & policyJointOrder;

  /** @brief Default pose expanded in controllerJointOrder. */
  const Eigen::VectorXd & qZeroControllerOrder;

  /** @brief Last raw policy action in policyJointOrder. */
  const Eigen::VectorXd & lastActionPolicyOrder;

  /** @brief High-level command vector, e.g. vx, vy, yaw_rate. */
  const Eigen::Vector3d & command;

  /** @brief Active training-environment convention. */
  ObservationConvention convention;
};

/**
 * @brief Parsed declaration of one observation entry from observations.yaml.
 */
struct ObservationConfig
{
  /** @brief Internal implementation type after convention alias resolution. */
  std::string type;

  /** @brief Exact type requested in YAML, e.g. joint_pos_rel. */
  std::string requestedType;

  /**
   * @brief Optional user/debug label.
   *
   * Useful when the same observation type appears multiple times with
   * different parameters. If absent, it defaults to requestedType.
   */
  std::string name;

  /**
   * @brief Number of samples stored by ObservationManager for this observation.
   *
   * This is intentionally not inside parameters: history is managed by
   * ObservationManager, not by the concrete observation class.
   */
  int history = 1;

  /** @brief Observation-specific parameter block. */
  mc_rtc::Configuration parameters;
};

/**
 * @brief Base class for one observation term.
 *
 * Subclasses own observation-specific state such as resolved joint indices,
 * body index, default pose and scale.
 */
class Observation
{
public:
  Observation(const ObservationConfig & config, const ObservationConvention & convention);
  virtual ~Observation();

  const std::string & type() const { return config_.type; }
  const std::string & requestedType() const { return config_.requestedType; }
  const std::string & name() const { return config_.name; }
  int history() const { return config_.history; }

  /** @brief Resolve configuration-time data into runtime indices/state. */
  virtual void configure(const ObservationContext & context) = 0;

  /** @brief Size of one sample of this observation, excluding history. */
  virtual int size() const = 0;

  /** @brief Compute one sample into out. */
  virtual void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const = 0;

protected:
  /** @brief Read a typed parameter with a fallback value. */
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

  /** @brief Convert joint names to indices in ObservationContext::controllerJointOrder. */
  std::vector<int> resolveControllerJointIndices(const ObservationContext & context,
                                                 const std::vector<std::string> & joints) const;

  /** @brief Read a scalar or vector scale parameter. */
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
  using ObservationFactory =
    std::function<std::shared_ptr<Observation>(const ObservationConfig &, const ObservationConvention &)>;

  void registerType(const std::string & type, ObservationFactory factory);
  std::shared_ptr<Observation> create(const ObservationConfig & config,
                                      const ObservationConvention & convention) const;

  std::vector<std::string> knownTypes() const;

private:
  std::map<std::string, ObservationFactory> factories_;
};

ObservationRegistry makeDefaultObservationRegistry();

} // namespace rlqp
