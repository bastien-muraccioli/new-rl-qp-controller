#include "observation/Observations.h"


#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rbdyn/rpy_utils.h>
#include <numeric>

#include <mc_rtc/logging.h>

#include <cmath>

namespace rlqp
{

//============================================================================//
// JointPosObservation
//============================================================================//

JointPosObservation::JointPosObservation(const ObservationConfig & config,
                                         const ObservationConvention & convention)
: Observation(config, convention)
{
}

void JointPosObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
      context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  const std::vector<int> controllerIndices = context.convention.resolveJointControllerIndices(
      parameters, context.controllerJointOrder, context.policyJointControllerIndices);

  const int n = static_cast<int>(controllerIndices.size());
  controllerIndices_ = controllerIndices;
  mbcIndices_.resize(static_cast<size_t>(n));
  defaultPose_ = Eigen::VectorXd::Zero(n);

  for(int i = 0; i < n; ++i)
  {
    const int ctrlIdx = controllerIndices[static_cast<size_t>(i)];
    const std::string & jointName = context.controllerJointOrder[static_cast<size_t>(ctrlIdx)];
    mbcIndices_[static_cast<size_t>(i)] = context.observationRobot.jointIndexByName(jointName);
    defaultPose_(i) = context.qZeroControllerOrder(ctrlIdx);
  }

  relativeToDefaultPose_ = readParameter<bool>(parameters, "relative_to_default_pose", true);
  scale_ = readScale(parameters, "scale", n, 1.0);
}

void JointPosObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & currentPos = context.observationRobot.encoderValues();
  const bool useEncoders = currentPos.size() == context.controllerJointOrder.size();

  if(!currentPos.empty() && !useEncoders)
  {
    mc_rtc::log::error_and_throw("[Observation:{}] encoderValues has size {}, expected {} or 0", name(),
                                 currentPos.size(), context.controllerJointOrder.size());
  }

  for(size_t i = 0; i < controllerIndices_.size(); ++i)
  {
    double value = 0.0;
    if(useEncoders)
    {
      value = currentPos[static_cast<size_t>(controllerIndices_[i])];
    }
    else
    {
      const auto & q = context.observationRobot.mbc().q[static_cast<size_t>(mbcIndices_[i])];
      if(q.size() != 1)
      {
        mc_rtc::log::error_and_throw("[Observation:{}] Joint '{}' has {} position DoFs, expected 1", name(),
                                     context.controllerJointOrder[static_cast<size_t>(controllerIndices_[i])],
                                     q.size());
      }
      value = q[0];
    }

    if(relativeToDefaultPose_) value -= defaultPose_(static_cast<int>(i));

    out(static_cast<int>(i)) = scale_(static_cast<int>(i)) * value;
  }
}

//============================================================================//
// JointVelObservation
//============================================================================//

JointVelObservation::JointVelObservation(const ObservationConfig & config,
                                         const ObservationConvention & convention)
: Observation(config, convention)
{
}

void JointVelObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
      context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  const std::vector<int> controllerIndices = context.convention.resolveJointControllerIndices(
      parameters, context.controllerJointOrder, context.policyJointControllerIndices);

  const int n = static_cast<int>(controllerIndices.size());
  controllerIndices_ = controllerIndices;
  mbcIndices_.resize(static_cast<size_t>(n));
  defaultVelocity_ = Eigen::VectorXd::Zero(n);

  for(int i = 0; i < n; ++i)
  {
    const int ctrlIdx = controllerIndices[static_cast<size_t>(i)];
    const std::string & jointName = context.controllerJointOrder[static_cast<size_t>(ctrlIdx)];
    mbcIndices_[static_cast<size_t>(i)] = context.observationRobot.jointIndexByName(jointName);
  }

  relativeToDefaultVelocity_ = readParameter<bool>(parameters, "relative_to_default_velocity", true);
  scale_ = readScale(parameters, "scale", n, 1.0);
}

void JointVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & currentVel = context.observationRobot.encoderVelocities();
  const bool useEncoders = currentVel.size() == context.controllerJointOrder.size();

  if(!currentVel.empty() && !useEncoders)
  {
    mc_rtc::log::error_and_throw("[Observation:{}] encoderVelocities has size {}, expected {} or 0", name(),
                                 currentVel.size(), context.controllerJointOrder.size());
  }

  for(size_t i = 0; i < controllerIndices_.size(); ++i)
  {
    double value = 0.0;
    if(useEncoders)
    {
      value = currentVel[static_cast<size_t>(controllerIndices_[i])];
    }
    else
    {
      const auto & alpha = context.observationRobot.mbc().alpha[static_cast<size_t>(mbcIndices_[i])];
      if(alpha.size() != 1)
      {
        mc_rtc::log::error_and_throw("[Observation:{}] Joint '{}' has {} velocity DoFs, expected 1", name(),
                                     context.controllerJointOrder[static_cast<size_t>(controllerIndices_[i])],
                                     alpha.size());
      }
      value = alpha[0];
    }

    if(relativeToDefaultVelocity_) value -= defaultVelocity_(static_cast<int>(i));

    out(static_cast<int>(i)) = scale_(static_cast<int>(i)) * value;
  }
}

//============================================================================//
// ProjectedGravityObservation
//============================================================================//

ProjectedGravityObservation::ProjectedGravityObservation(const ObservationConfig & config,
                                                         const ObservationConvention & convention)
: Observation(config, convention)
{
}

void ProjectedGravityObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  const std::string body = readParameter<std::string>(parameters, "body", context.baseBody);

  if(!context.observationRobot.hasBody(body))
  {
    mc_rtc::log::error_and_throw(
      "[Observation:{}] Body '{}' does not exist on robot '{}'",
      name(),
      body,
      context.observationRobot.name());
  }

  bodyIndex_ = context.observationRobot.mb().bodyIndexByName(body);
  scale_ = readScale(parameters, "scale", 3, 1.0);
}

void ProjectedGravityObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const sva::PTransformd & X_0_body =
    context.observationRobot.mbc().bodyPosW[static_cast<size_t>(bodyIndex_)];

  const Eigen::Vector3d gravityWorld(0.0, 0.0, -1.0);
  const Eigen::Vector3d gravityBody = X_0_body.rotation() * gravityWorld;

  out = gravityBody.cwiseProduct(scale_);
}

//============================================================================//
// BaseAngVelObservation
//============================================================================//

BaseAngVelObservation::BaseAngVelObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
: Observation(config, convention)
{
}

void BaseAngVelObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  sensorName_ = readParameter<std::string>(parameters, "sensor", std::string("Accelerometer"));

  if(!context.observationRobot.hasBodySensor(sensorName_))
  {
    mc_rtc::log::error_and_throw(
      "[Observation:{}] Body sensor '{}' does not exist on robot '{}'",
      name(),
      sensorName_,
      context.observationRobot.name());
  }

  scale_ = readScale(parameters, "scale", 3, 1.0);
}

void BaseAngVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & imu = context.observationRobot.bodySensor(sensorName_);
  const Eigen::Vector3d value = imu.angularVelocity();
  out = value.cwiseProduct(scale_);
}

//============================================================================//
// BaseLinVelObservation
//============================================================================//

BaseLinVelObservation::BaseLinVelObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
: Observation(config, convention)
{
}

void BaseLinVelObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  sensorName_ = readParameter<std::string>(parameters, "sensor", std::string("Accelerometer"));

  if(!context.observationRobot.hasBodySensor(sensorName_))
  {
    mc_rtc::log::error_and_throw(
      "[Observation:{}] Body sensor '{}' does not exist on robot '{}'",
      name(),
      sensorName_,
      context.observationRobot.name());
  }

  scale_ = readScale(parameters, "scale", 3, 1.0);
}

void BaseLinVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & imu = context.observationRobot.bodySensor(sensorName_);
  const Eigen::Vector3d value = imu.linearVelocity();
  out = value.cwiseProduct(scale_);
}

//============================================================================//
// LastActionObservation
//============================================================================//

LastActionObservation::LastActionObservation(const ObservationConfig & config,
                                             const ObservationConvention & convention)
: Observation(config, convention)
{
}

void LastActionObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  indexes_ = context.convention.resolveJointControllerIndices(parameters, context.observationRobot.refJointOrder(), context.policyJointControllerIndices);
  size_ = static_cast<int>(indexes_.size());
  scale_ = readScale(parameters, "scale", size_, 1.0);
}

void LastActionObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  out = Eigen::VectorXd::Zero(size_);
  for(size_t i = 0; i < indexes_.size(); ++i)
  {
    const int observedControllerIndex = indexes_[i];

    auto it = std::find(
      context.policyJointControllerIndices.begin(),
      context.policyJointControllerIndices.end(),
      observedControllerIndex);

    if(it == context.policyJointControllerIndices.end())
    {
      continue;
    }

    const Eigen::Index src =
      static_cast<Eigen::Index>(std::distance(context.policyJointControllerIndices.begin(), it));

    if(src >= 0 && src < context.lastActionPolicyOrder.size())
    {
      out(static_cast<Eigen::Index>(i)) =
        context.lastActionPolicyOrder(src) * scale_[i];
    }
  }
}

//============================================================================//
// CommandObservation
//============================================================================//

CommandObservation::CommandObservation(const ObservationConfig & config,
                                       const ObservationConvention & convention)
: Observation(config, convention)
{
}

void CommandObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  size_ = readParameter<int>(parameters, "size", 3);

  if(size_ <= 0 || size_ > 3)
  {
    mc_rtc::log::error_and_throw(
      "[Observation:{}] Command observation supports size in [1, 3], got {}",
      name(),
      size_);
  }

  scale_ = readScale(parameters, "scale", size_, 1.0);
}

void CommandObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  for(int i = 0; i < size_; ++i)
    out(i) = scale_(i) * context.command(i);
}

//============================================================================//
// BaseOrientationObservation
//============================================================================//

BaseOrientationObservation::BaseOrientationObservation(const ObservationConfig & config,
                                                       const ObservationConvention & convention)
: Observation(config, convention)
{
}

void BaseOrientationObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  sensorName_ = readParameter<std::string>(parameters, "sensor", std::string("Accelerometer"));

  if(!context.observationRobot.hasBodySensor(sensorName_))
  {
    mc_rtc::log::error_and_throw(
      "[Observation:{}] Body sensor '{}' does not exist on robot '{}'",
      name(),
      sensorName_,
      context.observationRobot.name());
  }

  indexes_ = readParameter<std::vector<int>>(parameters, "index", {0,1,2});

  scale_ = readScale(parameters, "scale", 3, 1.0);
}

void BaseOrientationObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & imu = context.observationRobot.bodySensor(sensorName_);

  Eigen::Matrix3d baseRot = imu.orientation().toRotationMatrix().normalized();
  const Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(baseRot);
  Eigen::VectorXd rpy_scaled = rpy.cwiseProduct(scale_);

  if (indexes_.size() == 3)
    out = rpy_scaled;
  else
    out = rpy_scaled(Eigen::Map<const Eigen::VectorXi>(indexes_.data(), indexes_.size()));
}

//============================================================================//
// PhaseObservation
//============================================================================//

PhaseObservation::PhaseObservation(const ObservationConfig & config,
                                   const ObservationConvention & convention)
: Observation(config, convention)
{
}

void PhaseObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
    context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  offset_ = readParameter<double>(parameters, "offset", 0.0);
  scale_ = readScale(parameters, "scale", 2, 1.0);
  cos_first_ = readParameter<bool>(parameters, "cos_first", true);
}

void PhaseObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const double phase = context.phaseNormalized + offset_;
  const double angle = 2.0 * M_PI * phase;
  
  out(1-cos_first_) = scale_(0) * std::cos(angle);
  out(cos_first_) = scale_(1) * std::sin(angle);
}

//============================================================================//
// LogForceSensorObservation
//============================================================================//

LogForceSensorObservation::LogForceSensorObservation(const ObservationConfig & config,
                                                     const ObservationConvention & convention)
: Observation(config, convention)
{
}

void LogForceSensorObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
      context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  sensor_names_ = readParameter<std::vector<std::string>>(parameters, "sensor_names", {});
  if(sensor_names_.size() == 0)
    mc_rtc::log::error_and_throw("[ForceSensorObservation] Please specify at least one force sensor");

  size_ = sensor_names_.size() * 3;
}

void LogForceSensorObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  auto log1p_compress = [](const Eigen::Vector3d & f) -> Eigen::Vector3d
  {
    return Eigen::Vector3d(std::copysign(std::log1p(std::abs(f.x())), f.x()),
                           std::copysign(std::log1p(std::abs(f.y())), f.y()),
                           std::copysign(std::log1p(std::abs(f.z())), f.z()));
  };
  out = Eigen::VectorXd::Zero(size_);
  int i = 0;
  for(const auto & sensor_name : sensor_names_)
  {
    const auto & forceSensor = context.observationRobot.forceSensor(sensor_name);
    out.segment(i, 3) = log1p_compress(forceSensor.worldWrench(context.observationRobot).force());
    ;
    i += 3;
  }
}

//============================================================================//
// ForceSensorObservation
//============================================================================//

ForceSensorObservation::ForceSensorObservation(const ObservationConfig & config,
                                               const ObservationConvention & convention)
: Observation(config, convention)
{
}

void ForceSensorObservation::configure(const ObservationContext & context)
{
  mc_rtc::Configuration parameters =
      context.convention.resolveObservationParameters(requestedType(), type(), config_.parameters);

  sensor_names_ = readParameter<std::vector<std::string>>(parameters, "sensor_names", {});
  if(sensor_names_.size() == 0)
    mc_rtc::log::error_and_throw("[ForceSensorObservation] Please specify at least one force sensor");

  size_ = sensor_names_.size() * 3;
}

void ForceSensorObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  out = Eigen::VectorXd::Zero(size_);
  int i = 0;
  for(const auto & sensor_name : sensor_names_)
  {
    const auto & forceSensor = context.observationRobot.forceSensor(sensor_name);
    out.segment(i, 3) = forceSensor.worldWrench(context.observationRobot).force();
    ;
    i += 3;
  }
}

} // namespace rlqp
