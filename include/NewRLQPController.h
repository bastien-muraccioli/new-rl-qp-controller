#pragma once

#include "policy/RLPolicyRuntime.h"
#include "policy/RLStateRunner.h"
#include "api.h"

#include <Eigen/Core>

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <mc_tasks/TorqueJointTask.h>

#include <memory>
#include <string>
#include <vector>

struct NewRLQPController_DLLAPI NewRLQPController : public mc_control::fsm::Controller
{
  NewRLQPController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  void activateQPControl(bool activate);

  rlqp::RLPolicyRuntime & rlRuntime();
  const rlqp::RLPolicyRuntime & rlRuntime() const;

  std::shared_ptr<mc_tasks::TorqueJointTask> torqueJointTask;

  int nbActuatedJoints = 0;
  std::vector<std::string> jointNames;

  RLStateRunner rlStateRunner;

private:
  mc_rtc::Configuration config_;

  void addLog();
  void addGui();

  void initializeRobotBasics();

  bool byPassQPControl();
  void computeLimits();

private:
  bool printLimits_ = true;

  std::string robotName_;

  double velPercent_ = 0.95;
  double dsPercent_ = 0.01;
  double diPercent_ = 0.1;

  double zeta_jointLimit_ = 1.2;
  double lambda_jointLimit_ = 100.0;
  double zeta_selfCollision_ = 1.2;
  double lambda_selfCollision_ = 10.0;

  rlqp::RLPolicyRuntime rlRuntime_;
};
