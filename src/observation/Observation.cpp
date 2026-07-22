#include "observation/Observation.h"
#include "observation/Observations.h"

#include <algorithm>
#include <set>
#include <sstream>

#include <mc_rtc/logging.h>

namespace rlqp
{

//============================================================================//
// Helper function
//============================================================================//
namespace
{
template<typename T>
std::shared_ptr<Observation> makeObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
{
  return std::shared_ptr<Observation>(new T(config, convention));
}

mc_rtc::Configuration loadConventionRoot(const mc_rtc::Configuration & controllerConfig)
{
  if(controllerConfig.has("conventions"))
    return controllerConfig;
  std::string policiesRoot = "policies/";
  controllerConfig("policies_root", policiesRoot);
  const std::string conventionsPath = policiesRoot + "/conventions.yaml";

  mc_rtc::Configuration conventionsConfig;
  conventionsConfig.load(conventionsPath);
  return conventionsConfig;
}
} // namespace

//============================================================================//
// ObservationConvention
//============================================================================//
ObservationConvention ObservationConvention::fromConfig(const mc_rtc::Configuration & controllerConfig,
                                                        const std::string & conventionName)
{
  ObservationConvention out;
  out.name = conventionName;

  mc_rtc::Configuration conventionRoot = loadConventionRoot(controllerConfig);
  mc_rtc::Configuration conventions = conventionRoot("conventions");

  if(!conventions.has(conventionName))
    mc_rtc::log::error_and_throw(
      "[ObservationConvention] Requested convention '{}' does not exist",
      conventionName);

  mc_rtc::Configuration cfg = conventions(conventionName);

  auto loadJointGroups = [&out](const mc_rtc::Configuration & source)
  {
    if(!source.has("joint_groups"))
      return;

    mc_rtc::Configuration groups = source("joint_groups");
    std::vector<std::string> keys = groups.keys();

    for(size_t i = 0; i < keys.size(); ++i)
      out.jointGroups[keys[i]] = groups(keys[i], std::vector<std::string>());
  };

  auto loadObservationDefaults = [&out](const mc_rtc::Configuration & source)
  {
    if(!source.has("observation_defaults"))
      return;

    mc_rtc::Configuration defaults = source("observation_defaults");
    std::vector<std::string> keys = defaults.keys();

    for(size_t i = 0; i < keys.size(); ++i)
      out.defaultParameters[keys[i]] = defaults(keys[i]);
  };

  auto loadTypeAliases = [&out](const mc_rtc::Configuration & source)
  {
    if(!source.has("type_aliases"))
      return;

    mc_rtc::Configuration aliases = source("type_aliases");
    std::vector<std::string> keys = aliases.keys();

    for(size_t i = 0; i < keys.size(); ++i)
    {
      std::string alias = keys[i];
      aliases(keys[i], alias);
      out.typeAliases[keys[i]] = alias;
    }
  };

  if(conventions.has("general") && conventionName != "general")
  {
    mc_rtc::Configuration general = conventions("general");
    loadObservationDefaults(general);
    loadTypeAliases(general);
  }

  loadJointGroups(cfg);
  loadObservationDefaults(cfg);
  loadTypeAliases(cfg);

  return out;
}

std::string ObservationConvention::resolveType(const std::string & requestedType) const
{
  std::map<std::string, std::string>::const_iterator it = typeAliases.find(requestedType);
  if(it == typeAliases.end())
    return requestedType;

  return it->second;
}

std::vector<int> ObservationConvention::resolveJointControllerIndices(
  const mc_rtc::Configuration & parameters,
  const std::vector<std::string> & controllerJointOrder,
  const std::vector<int> & fallbackIndices) const
{
  // Helper: map a joint name to its index in controllerJointOrder
  auto nameToIdx = [&](const std::string & name) -> int
  {
    std::vector<std::string>::const_iterator it =
      std::find(controllerJointOrder.begin(), controllerJointOrder.end(), name);
    if(it == controllerJointOrder.end())
      mc_rtc::log::error_and_throw(
        "[ObservationConvention] Joint '{}' not found in controllerJointOrder", name);
    return static_cast<int>(std::distance(controllerJointOrder.begin(), it));
  };

  // Parse as a group name (string)
  const std::string groupName = parameters("joints", std::string(""));
  if(!groupName.empty())
  {
    // Check RL convention groups (e.g. mjlab.joint_groups.legs)
    std::map<std::string, std::vector<std::string> >::const_iterator rlIt =
      jointGroups.find(groupName);
    if(rlIt != jointGroups.end())
    {
      std::vector<int> out;
      out.reserve(rlIt->second.size());
      for(size_t i = 0; i < rlIt->second.size(); ++i)
        out.push_back(nameToIdx(rlIt->second[i]));
      return out;
    }
  }

  // Check an explicit list of joint names
  const std::vector<std::string> names = parameters("joints", std::vector<std::string>());
  if(!names.empty())
  {
    std::vector<int> out;
    out.reserve(names.size());
    for(size_t i = 0; i < names.size(); ++i)
      out.push_back(nameToIdx(names[i]));
    return out;
  }

  return fallbackIndices;
}

mc_rtc::Configuration ObservationConvention::resolveObservationParameters(
  const std::string & requestedType,
  const std::string & internalType,
  const mc_rtc::Configuration & localParameters) const
{
  mc_rtc::Configuration out;

  std::map<std::string, mc_rtc::Configuration>::const_iterator exactIt = defaultParameters.find(requestedType);
  if(exactIt != defaultParameters.end())
    out = exactIt->second;
  else
  {
    std::map<std::string, mc_rtc::Configuration>::const_iterator internalIt = defaultParameters.find(internalType);
    if(internalIt != defaultParameters.end())
      out = internalIt->second;
  }

  std::vector<std::string> keys = localParameters.keys();

  for(size_t i = 0; i < keys.size(); ++i)
    out.add(keys[i], localParameters(keys[i]));

  return out;
}

//============================================================================//
// Observation
//============================================================================//
Observation::Observation(const ObservationConfig & config, const ObservationConvention & convention)
: config_(config), convention_(convention)
{
  if(config_.name.empty())
  {
    config_.name = config_.requestedType.empty() ? config_.type : config_.requestedType;
  }
}

Observation::~Observation()
{
}

Eigen::VectorXd Observation::readScale(const mc_rtc::Configuration & parameters,
                                             const std::string & key,
                                             int size,
                                             double fallback) const
{
  if(!parameters.has(key))
    return Eigen::VectorXd::Constant(size, fallback);

  const std::vector<double> values = parameters(key, std::vector<double>());
  if (values.size() != 0)
  {
    if (values.size() != static_cast<size_t>(size))
      mc_rtc::log::error("[Observation:{}] Parameter '{}' has size {}, expected {}",
        name(), key, values.size(),size);
        
    Eigen::VectorXd out(size);
    return Eigen::Map<const Eigen::VectorXd>(values.data(), values.size());
  }

  double scale = fallback;
  parameters(key, scale);
  return Eigen::VectorXd::Constant(size, scale);
}

//============================================================================//
// ObservationRegistry
//============================================================================//
void ObservationRegistry::registerType(const std::string & type, ObservationFactory factory)
{
  if(type.empty())
    mc_rtc::log::error_and_throw("[ObservationRegistry] Cannot register an empty observation type");

  if(!factory)
    mc_rtc::log::error_and_throw("[ObservationRegistry] Cannot register null factory for '{}'", type);

  if(factories_.find(type) != factories_.end())
    mc_rtc::log::error_and_throw("[ObservationRegistry] Observation type '{}' is already registered", type);

  factories_[type] = factory;
}

std::shared_ptr<Observation> ObservationRegistry::create(const ObservationConfig & config,
                                                         const ObservationConvention & convention) const
{
  std::map<std::string, ObservationFactory>::const_iterator it = factories_.find(config.type);

  if(it == factories_.end())
  {
    std::ostringstream os;
    for(size_t i = 0; i < knownTypes().size(); ++i)
    {
      os << knownTypes()[i];
      if(i + 1 < knownTypes().size())
        os << ", ";
    }
    mc_rtc::log::error_and_throw(
      "[ObservationRegistry] Unknown observation type '{}'. Known types are: {}",
      config.type,
      os.str());
  }

  return it->second(config, convention);
}

std::vector<std::string> ObservationRegistry::knownTypes() const
{
  std::vector<std::string> out;

  for(std::map<std::string, ObservationFactory>::const_iterator it = factories_.begin();
      it != factories_.end();
      ++it)
    out.push_back(it->first);

  return out;
}

ObservationRegistry makeDefaultObservationRegistry()
{
  ObservationRegistry registry;

  registry.registerType("joint_pos", &makeObservation<JointPosObservation>);
  registry.registerType("joint_vel", &makeObservation<JointVelObservation>);
  registry.registerType("projected_gravity", &makeObservation<ProjectedGravityObservation>);
  registry.registerType("base_ang_vel", &makeObservation<BaseAngVelObservation>);
  registry.registerType("base_lin_vel", &makeObservation<BaseLinVelObservation>);
  registry.registerType("base_orientation", &makeObservation<BaseOrientationObservation>);
  registry.registerType("phase", &makeObservation<PhaseObservation>);
  registry.registerType("last_action", &makeObservation<LastActionObservation>);
  registry.registerType("command", &makeObservation<CommandObservation>);
  registry.registerType("log_force_sensor", &makeObservation<LogForceSensorObservation>);
  registry.registerType("force_sensor", &makeObservation<ForceSensorObservation>);

  return registry;
}

} // namespace rlqp
