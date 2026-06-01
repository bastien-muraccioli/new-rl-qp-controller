#pragma once

#include "Observation.h"

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
    class JointPosObservation : public Observation
    {
    public:
    JointPosObservation(const ObservationConfig & config, const ObservationConvention & convention);

    void configure(const ObservationContext & context) override;
    int size() const override { return static_cast<int>(indices_.size()); }
    void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

    private:
    std::vector<int> indices_;
    bool relativeToDefaultPose_ = true;
    bool biased_ = false;
    Eigen::VectorXd defaultPose_;
    Eigen::VectorXd scale_;
    };

    class JointVelObservation : public Observation
    {
    public:
    JointVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

    void configure(const ObservationContext & context) override;
    int size() const override { return static_cast<int>(indices_.size()); }
    void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

    private:
    std::vector<int> indices_;
    bool relativeToDefaultVelocity_ = true;
    Eigen::VectorXd defaultVelocity_;
    Eigen::VectorXd scale_;
    };

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

    class BaseAngVelObservation : public Observation
    {
    public:
    BaseAngVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

    void configure(const ObservationContext & context) override;
    int size() const override { return 3; }
    void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

    private:
    int bodyIndex_ = -1;
    Eigen::VectorXd scale_;
    };

    class BaseLinVelObservation : public Observation
    {
    public:
    BaseLinVelObservation(const ObservationConfig & config, const ObservationConvention & convention);

    void configure(const ObservationContext & context) override;
    int size() const override { return 3; }
    void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

    private:
    int bodyIndex_ = -1;
    Eigen::VectorXd scale_;
    };

    class LastActionObservation : public Observation
    {
    public:
    LastActionObservation(const ObservationConfig & config, const ObservationConvention & convention);

    void configure(const ObservationContext & context) override;
    int size() const override { return size_; }
    void compute(const ObservationContext & context, Eigen::Ref<Eigen::VectorXd> out) const override;

    private:
    int size_ = 0;
    Eigen::VectorXd scale_;
    };

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
} // namespace rlqp