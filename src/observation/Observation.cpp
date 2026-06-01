#include "observation/Observation.h"

#include "NewRLQPController.h"

#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <algorithm>
#include <sstream>
#include <stdexcept>

#include <mc_rtc/logging.h>

namespace rlqp
{

namespace
{

std::vector<std::string> readStringVector(const mc_rtc::Configuration & config,
                                          const std::string & key,
                                          const std::vector<std::string> & fallback)
{
  if(!config.has(key)) { return fallback; }
  return config(key, fallback);
}

int jointIndexInOrder(const std::vector<std::string> & order, const std::string & joint)
{
  auto it = std::find(order.begin(), order.end(), joint);
  if(it == order.end()) { return -1; }
  return static_cast<int>(std::distance(order.begin(), it));
}

std::string joinStrings(const std::vector<std::string> & values)
{
  std::ostringstream os;
  for(size_t i = 0; i < values.size(); ++i)
  {
    os << values[i];
    if(i + 1 < values.size()) { os << ", "; }
  }
  return os.str();
}

Eigen::VectorXd constantVector(int size, double value)
{
  Eigen::VectorXd out(size);
  out.setConstant(value);
  return out;
}

Eigen::VectorXd vector3ToVectorXd(const Eigen::Vector3d & value)
{
  Eigen::VectorXd out(3);
  out << value(0), value(1), value(2);
  return out;
}

template<typename T>
std::shared_ptr<Observation> makeObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
{
  return std::shared_ptr<Observation>(new T(config, convention));
}

} // namespace

ObservationConvention ObservationConvention::fromConfig(const mc_rtc::Configuration & controllerConfig,
                                                        const std::string & conventionName)
{
  ObservationConvention out;
  out.name = conventionName;

  if(!controllerConfig.has("conventions")) { return out; }

  mc_rtc::Configuration conventions = controllerConfig("conventions");

  if(!conventions.has(conventionName))
  {
    mc_rtc::log::error_and_throw(
      "[ObservationConvention] Requested convention '{}' does not exist in controller config",
      conventionName);
  }

  mc_rtc::Configuration cfg = conventions(conventionName);

  out.defaultJointOrder = readStringVector(cfg, "joint_order", std::vector<std::string>());

  if(cfg.has("joint_groups"))
  {
    mc_rtc::Configuration groups = cfg("joint_groups");
    std::vector<std::string> keys = groups.keys();

    for(size_t i = 0; i < keys.size(); ++i)
    {
      out.jointGroups[keys[i]] = groups(keys[i], std::vector<std::string>());
    }
  }

  if(cfg.has("observation_defaults"))
  {
    mc_rtc::Configuration defaults = cfg("observation_defaults");
    std::vector<std::string> keys = defaults.keys();

    for(size_t i = 0; i < keys.size(); ++i)
    {
      out.defaultParameters[keys[i]] = defaults(keys[i]);
    }
  }

  if(cfg.has("type_aliases"))
  {
    mc_rtc::Configuration aliases = cfg("type_aliases");
    std::vector<std::string> keys = aliases.keys();

    for(size_t i = 0; i < keys.size(); ++i)
    {
      out.typeAliases[keys[i]] = aliases(keys[i], keys[i]);
    }
  }

  return out;
}

std::string ObservationConvention::resolveType(const std::string & requestedType) const
{
  std::map<std::string, std::string>::const_iterator it = typeAliases.find(requestedType);
  if(it == typeAliases.end()) { return requestedType; }
  return it->second;
}

std::vector<std::string> ObservationConvention::resolveJoints(const mc_rtc::Configuration & parameters,
                                                              const std::vector<std::string> & fallback) const
{
  if(!parameters.has("joints"))
  {
    if(!defaultJointOrder.empty()) { return defaultJointOrder; }
    return fallback;
  }

  try
  {
    const std::string groupName = parameters("joints", std::string(""));
    if(!groupName.empty())
    {
      std::map<std::string, std::vector<std::string> >::const_iterator groupIt = jointGroups.find(groupName);
      if(groupIt != jointGroups.end()) { return groupIt->second; }
    }
  }
  catch(...)
  {
  }

  return parameters("joints", fallback);
}

mc_rtc::Configuration ObservationConvention::parametersFor(const std::string & requestedType,
                                                           const std::string & internalType,
                                                           const mc_rtc::Configuration & localParameters) const
{
  mc_rtc::Configuration out;

  std::map<std::string, mc_rtc::Configuration>::const_iterator exactIt = defaultParameters.find(requestedType);
  if(exactIt != defaultParameters.end())
  {
    out = exactIt->second;
  }
  else
  {
    std::map<std::string, mc_rtc::Configuration>::const_iterator internalIt = defaultParameters.find(internalType);
    if(internalIt != defaultParameters.end()) { out = internalIt->second; }
  }

  std::vector<std::string> keys = localParameters.keys();

  for(size_t i = 0; i < keys.size(); ++i)
  {
    out.add(keys[i], localParameters(keys[i]));
  }

  return out;
}

Observation::Observation(const ObservationConfig & config, const ObservationConvention & convention)
: config_(config), convention_(convention)
{
  if(config_.name.empty())
  {
    config_.name = config_.requestedType.empty() ? config_.type : config_.requestedType;
  }
}

Observation::~Observation() {}

std::vector<int> Observation::resolveJointIndices(const ObservationContext & context,
                                                  const std::vector<std::string> & joints) const
{
  std::vector<int> indices;
  indices.reserve(joints.size());

  for(size_t i = 0; i < joints.size(); ++i)
  {
    const std::string & joint = joints[i];

    if(!context.realRobot.hasJoint(joint))
    {
      mc_rtc::log::error_and_throw(
        "[Observation:{}] Requested joint '{}' does not exist on robot '{}'",
        name(),
        joint,
        context.realRobot.name());
    }

    const int controllerIndex = jointIndexInOrder(context.controllerJointOrder, joint);

    if(controllerIndex < 0)
    {
      mc_rtc::log::error_and_throw(
        "[Observation:{}] Joint '{}' is not in controllerJointOrder",
        name(),
        joint);
    }

    indices.push_back(controllerIndex);
  }

  return indices;
}

Eigen::VectorXd Observation::readScaleVector(const mc_rtc::Configuration & parameters,
                                             const std::string & key,
                                             int size,
                                             double fallback) const
{
  if(!parameters.has(key)) { return constantVector(size, fallback); }

  try
  {
    const std::vector<double> values = parameters(key, std::vector<double>());
    if(values.size() == static_cast<size_t>(size))
    {
      Eigen::VectorXd out(size);
      for(int i = 0; i < size; ++i) { out(i) = values[static_cast<size_t>(i)]; }
      return out;
    }

    if(!values.empty())
    {
      mc_rtc::log::error_and_throw(
        "[Observation:{}] Parameter '{}' has size {}, expected {}",
        name(),
        key,
        values.size(),
        size);
    }
  }
  catch(...)
  {
  }

  const double scalar = parameters(key, fallback);
  return constantVector(size, scalar);
}

void ObservationRegistry::registerType(const std::string & type, Creator creator)
{
  if(type.empty())
  {
    mc_rtc::log::error_and_throw("[ObservationRegistry] Cannot register an empty observation type");
  }

  if(!creator)
  {
    mc_rtc::log::error_and_throw("[ObservationRegistry] Cannot register null creator for '{}'", type);
  }

  if(creators_.find(type) != creators_.end())
  {
    mc_rtc::log::error_and_throw("[ObservationRegistry] Observation type '{}' is already registered", type);
  }

  creators_[type] = creator;
}

std::shared_ptr<Observation> ObservationRegistry::create(const ObservationConfig & config,
                                                         const ObservationConvention & convention) const
{
  std::map<std::string, Creator>::const_iterator it = creators_.find(config.type);

  if(it == creators_.end())
  {
    mc_rtc::log::error_and_throw(
      "[ObservationRegistry] Unknown observation type '{}'. Known types are: {}",
      config.type,
      joinStrings(knownTypes()));
  }

  return it->second(config, convention);
}

std::vector<std::string> ObservationRegistry::knownTypes() const
{
  std::vector<std::string> out;

  for(std::map<std::string, Creator>::const_iterator it = creators_.begin(); it != creators_.end(); ++it)
  {
    out.push_back(it->first);
  }

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
  registry.registerType("last_action", &makeObservation<LastActionObservation>);
  registry.registerType("command", &makeObservation<CommandObservation>);

  return registry;
}

} // namespace rlqp
