#include "observation/Observations.h"


#include <RBDyn/MultiBodyConfig.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <mc_rbdyn/rpy_utils.h>

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

  const std::vector<std::string> joints =
    context.convention.resolveJoints(parameters, context.policyJointOrder);

  indices_ = resolveControllerJointIndices(context, joints);
  relativeToDefaultPose_ = readParameter<bool>(parameters, "relative_to_default_pose", true);
  scale_ = readScaleVector(parameters, "scale", static_cast<int>(indices_.size()), 1.0);

  defaultPose_ = Eigen::VectorXd::Zero(static_cast<int>(indices_.size()));

  for(size_t i = 0; i < indices_.size(); ++i)
  {
    defaultPose_(static_cast<int>(i)) = context.qZeroControllerOrder(indices_[i]);
  }
}

void JointPosObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  for(size_t i = 0; i < indices_.size(); ++i)
  {
    const std::string & joint = context.controllerJointOrder[static_cast<size_t>(indices_[i])];
    const int mbcIndex = context.observationRobot.jointIndexByName(joint);

    double value = context.observationRobot.mbc().q[static_cast<size_t>(mbcIndex)][0];

    if(relativeToDefaultPose_)
    {
      value -= defaultPose_(static_cast<int>(i));
    }

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

  const std::vector<std::string> joints =
    context.convention.resolveJoints(parameters, context.policyJointOrder);

  indices_ = resolveControllerJointIndices(context, joints);
  relativeToDefaultVelocity_ = readParameter<bool>(parameters, "relative_to_default_velocity", true);
  scale_ = readScaleVector(parameters, "scale", static_cast<int>(indices_.size()), 1.0);

  defaultVelocity_ = Eigen::VectorXd::Zero(static_cast<int>(indices_.size()));
}

void JointVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  for(size_t i = 0; i < indices_.size(); ++i)
  {
    const std::string & joint = context.controllerJointOrder[static_cast<size_t>(indices_[i])];
    const int mbcIndex = context.observationRobot.jointIndexByName(joint);

    double value = context.observationRobot.mbc().alpha[static_cast<size_t>(mbcIndex)][0];

    if(relativeToDefaultVelocity_)
    {
      value -= defaultVelocity_(static_cast<int>(i));
    }

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
  scale_ = readScaleVector(parameters, "scale", 3, 1.0);
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
  scale_ = readScaleVector(parameters, "scale", 3, 1.0);
}

void BaseAngVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const Eigen::Vector3d value = context.observationRobot.mbc().bodyVelB[static_cast<size_t>(bodyIndex_)].angular();
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
  scale_ = readScaleVector(parameters, "scale", 3, 1.0);
}

void BaseLinVelObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const Eigen::Vector3d value = context.observationRobot.mbc().bodyVelB[static_cast<size_t>(bodyIndex_)].linear();
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

  size_ = readParameter<int>(parameters, "size", static_cast<int>(context.lastActionPolicyOrder.size()));

  if(size_ != context.lastActionPolicyOrder.size())
  {
    mc_rtc::log::error_and_throw(
      "[Observation:{}] Configured size ({}) does not match current action size ({})",
      name(),
      size_,
      context.lastActionPolicyOrder.size());
  }

  scale_ = readScaleVector(parameters, "scale", size_, 1.0);
}

void LastActionObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  out = scale_.cwiseProduct(context.lastActionPolicyOrder);
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

  scale_ = readScaleVector(parameters, "scale", size_, 1.0);
}

void CommandObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  for(int i = 0; i < size_; ++i)
  {
    out(i) = scale_(i) * context.command(i);
  }
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

  scale_ = readScaleVector(parameters, "scale", 3, 1.0);
}

void BaseOrientationObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const auto & imu = context.observationRobot.bodySensor(sensorName_);

  Eigen::Matrix3d baseRot = imu.orientation().toRotationMatrix().normalized();
  const Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(baseRot);

  out = rpy.cwiseProduct(scale_);
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
  scale_ = readScaleVector(parameters, "scale", 2, 1.0);
}

void PhaseObservation::compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const
{
  const double phase = context.phaseNormalized + offset_;
  const double angle = 2.0 * M_PI * phase;

  out(0) = scale_(0) * std::cos(angle);
  out(1) = scale_(1) * std::sin(angle);
}


} // namespace rlqp
