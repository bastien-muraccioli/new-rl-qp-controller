#pragma once

#include "Observation.h"

#include <Eigen/Core>

#include <mc_rtc/Configuration.h>

#include <string>
#include <vector>

namespace rlqp
{

/** @brief Joint position observation, optionally relative to q_zero. */
class JointPosObservation : public Observation
{
public:
  JointPosObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override
  {
    return static_cast<int>(controllerIndices_.size());
  }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::vector<int> controllerIndices_;
  std::vector<int> mbcIndices_;
  bool relativeToDefaultPose_ = true;
  Eigen::VectorXd defaultPose_;
  Eigen::VectorXd scale_;
};

/** @brief Joint velocity observation, optionally relative to a default velocity. */
class JointVelObservation : public Observation
{
public:
  JointVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override
  {
    return static_cast<int>(controllerIndices_.size());
  }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::vector<int> controllerIndices_;
  std::vector<int> mbcIndices_;
  bool relativeToDefaultVelocity_ = true;
  Eigen::VectorXd defaultVelocity_;
  Eigen::VectorXd scale_;
};

/** @brief Projected gravity in the configured base body frame. */
class ProjectedGravityObservation : public Observation
{
public:
  ProjectedGravityObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return 3; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  int bodyIndex_ = -1;
  Eigen::VectorXd scale_;
};

/** @brief Base angular velocity in the configured base body frame. */
class BaseAngVelObservation : public Observation
{
public:
  BaseAngVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return 3; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::string sensorName_ = "Accelerometer";
  Eigen::VectorXd scale_;
};

/** @brief Base linear velocity in the configured base body frame. */
class BaseLinVelObservation : public Observation
{
public:
  BaseLinVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return 3; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::string sensorName_ = "Accelerometer";
  Eigen::VectorXd scale_;
};

/** @brief Last raw policy action. */
class LastActionObservation : public Observation
{
public:
  LastActionObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return size_; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;
  Eigen::VectorXd makeFullLastActionPolicyOrder(
    const Eigen::VectorXd & rawAction,
    const std::vector<int> & actionJointIndicesPolicyOrder,
    int fullPolicyActionSize,
    double actionScale);

private:
  int size_ = 0;
  std::vector<int> indexes_ = {};
  Eigen::VectorXd scale_;
};

/** @brief Command observation, usually vx/vy/yaw_rate. */
class CommandObservation : public Observation
{
public:
  CommandObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return size_; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  int size_ = 3;
  Eigen::VectorXd scale_;
};


/**
 * @brief Base roll/pitch/yaw observation computed from a body sensor orientation.
 *
 * The default sensor name is "Accelerometer". It can be overridden with:
 *
 * parameters:
 *   sensor: ...
 */
class BaseOrientationObservation : public Observation
{
public:
  BaseOrientationObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return indexes_.size(); }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::string sensorName_ = "Accelerometer";
  std::vector<int> indexes_ = {0,1,2};
  Eigen::VectorXd scale_;
};

/**
 * @brief Phase observation represented as [cos(phase), sin(phase)].
 *
 * The runtime owns the normalized phase in [0, 1). This observation only
 * converts it to the trigonometric representation expected by many locomotion
 * policies.
 */
class PhaseObservation : public Observation
{
public:
  PhaseObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override { return 2; }
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  double offset_ = 0.0;
  Eigen::VectorXd scale_;
  bool cos_first_ = true;
};

/**
 * @brief Observation from a force sensor by name - log compressed data
 *
 * Handles an array of sensor, their data will be concatenated
 */
class LogForceSensorObservation : public Observation
{
public:
  LogForceSensorObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override
  {
    return size_;
  };
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::vector<std::string> sensor_names_ = {};
  int size_ = 0;
};

/**
 * @brief Observation from a force sensor by name - raw data
 *
 * Handles an array of sensor, their data will be concatenated
 */
class ForceSensorObservation : public Observation
{
public:
  ForceSensorObservation(const ObservationConfig & config, const ObservationConvention & convention);

  void configure(const ObservationContext & context) override;
  int size() const override
  {
    return size_;
  };
  void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

private:
  std::vector<std::string> sensor_names_ = {};
  int size_ = 0;
};

} // namespace rlqp
