#include "NewRLQPController.h"

#include <RBDyn/MultiBodyConfig.h>

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <cmath>

NewRLQPController::NewRLQPController(mc_rbdyn::RobotModulePtr rm,
                                     double dt,
                                     const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;

  //Initialize Constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {zeta_selfCollision_, lambda_selfCollision_});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, zeta_jointLimit_, lambda_jointLimit_}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));
  // Initialize Task
  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(
      solver(), robot().robotIndex(), 100.0, 1);
  solver().addTask(torqueJointTask);
  initializeRobotBasics();

  rlRuntime_.configure(config_, *this, torqueJointTask);

  addGui();
  addLog();

  mc_rtc::log::success("[NewRLQPController] init done");
}

bool NewRLQPController::run()
{
  if(printLimits_) computeLimits();
  bool run = mc_control::fsm::Controller::run(
          mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  if(byPassQPControl()) // Run RL without taking the QP into account
  {
    return true;
  }
  return run; // Return false if QP fails
}

void NewRLQPController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  rlRuntime_.reset(*this);
}

void NewRLQPController::activateQPControl(bool activate)
{
  rlRuntime_.setUseQP(activate);
}

rlqp::RLPolicyRuntime & NewRLQPController::rlRuntime() { return rlRuntime_; }

const rlqp::RLPolicyRuntime & NewRLQPController::rlRuntime() const { return rlRuntime_; }

void NewRLQPController::initializeRobotBasics()
{
  mc_rtc::log::info("[NewRLQPController] Using torque control mode");

  if(!datastore().has("ControlMode"))
  {
    datastore().make<std::string>("ControlMode", "Torque");
  }
  else
  {
    datastore().assign<std::string>("ControlMode", "Torque");
  }

  robotName_ = robot().name();
  jointNames = robot().refJointOrder();
  nbActuatedJoints = static_cast<int>(jointNames.size());
}

bool NewRLQPController::byPassQPControl()
{
  if(rlRuntime_.useQP()) return false; // QP is not bypassed, do nothing

  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  Eigen::VectorXd tau_rl = Eigen::VectorXd::Zero(nbActuatedJoints);
  const std::vector<std::vector<double> > & q_mbc = robot().mbc().q;
  const std::vector<std::vector<double> > & q_dot_mbc = robot().mbc().alpha;

  int i = 0;
  for(const auto &joint_name : jointNames)
  {
    const double q = q_mbc[robot().jointIndexByName(joint_name)][0];
    const double q_dot = q_dot_mbc[robot().jointIndexByName(joint_name)][0];
    tau_rl(i) = kp_(i) * (q_rl(i) - q) - kd_(i) * q_dot;
    robot().mbc().jointTorque[robot().jointIndexByName(joint_name)][0] = tau_rl(i);
    i++;
  }

  return true;
}

void NewRLQPController::addLog()
{
  // Robot State variables
  logger().addLogEntry("NewRLQPController_kp_base", [this]() { return rlRuntime_.kpBase(); });
  logger().addLogEntry("NewRLQPController_kd_base", [this]() { return rlRuntime_.kdBase(); });
  logger().addLogEntry("NewRLQPController_kp_current", [this]() { return rlRuntime_.kp(); });
  logger().addLogEntry("NewRLQPController_kd_current", [this]() { return rlRuntime_.kd(); });
  logger().addLogEntry("NewRLQPController_pd_gains_ratio", [this]() { return rlRuntime_.pdGainsRatio(); });

  // RL variables
  logger().addLogEntry("NewRLQPController_RL_q", [this]() { return rlRuntime_.q_rl(); });
  logger().addLogEntry("NewRLQPController_RL_qZero", [this]() { return rlRuntime_.q_zero(); });
  logger().addLogEntry("NewRLQPController_RL_currentObservation", [this]() { return rlRuntime_.currentObservation(); });
  logger().addLogEntry("NewRLQPController_RL_currentAction", [this]() { return rlRuntime_.currentAction(); });
  logger().addLogEntry("NewRLQPController_RL_currentActionScaled", [this]() { return rlRuntime_.currentActionScaled(); });
  logger().addLogEntry("NewRLQPController_RL_actionScale", [this]() { return rlRuntime_.actionScale(); });
  logger().addLogEntry("NewRLQPController_RL_command", [this]() { return rlRuntime_.command(); });

  // Controller state variables
  logger().addLogEntry("NewRLQPController_useQP", [this]() { return rlRuntime_.useQP(); });

  // Log current policy (name and convention)
  logger().addLogEntry("NewRLQPController_currentPolicy", [this]() { return rlRuntime_.currentPolicyName(); });
  logger().addLogEntry("NewRLQPController_observationConvention", [this]() { return rlRuntime_.conventionName(); });
}

void NewRLQPController::addGui()
{
  gui()->addElement(
    {"NewRLQPController", "Policy"},
    mc_rtc::gui::Label("Current policy", [this]() { return rlRuntime_.currentPolicyName(); }),
    mc_rtc::gui::Label("Current policy folder", [this]() { return rlRuntime_.currentPolicyFolder(); }),
    mc_rtc::gui::Label("Observation convention", [this]() { return rlRuntime_.conventionName(); }),
    mc_rtc::gui::Button("Reload current policy", [this]() {
      rlRuntime_.reloadCurrentPolicy(*this, torqueJointTask);
    }),
    mc_rtc::gui::Button("Load next policy", [this]() {
      rlRuntime_.loadNextPolicy(*this, torqueJointTask);
    }));

  gui()->addElement(
    {"NewRLQPController", "PD Gains"},
    mc_rtc::gui::NumberSlider(
      "PD Gains Ratio",
      [this]() { return rlRuntime_.pdGainsRatio(); },
      [this](double v) { rlRuntime_.setPDGainsRatio(v, torqueJointTask); },
      0.0,
      2.0),
    mc_rtc::gui::Label("Current kp", [this]() { return rlRuntime_.kp(); }),
    mc_rtc::gui::Label("Current kd", [this]() { return rlRuntime_.kd(); }));

  gui()->addElement(
    {"NewRLQPController", "Control"},
    mc_rtc::gui::Button("Toggle QP Control", [this]() {
      rlRuntime_.setUseQP(!rlRuntime_.useQP());
    }),
    mc_rtc::gui::Label("QP Control", [this]() {
      return rlRuntime_.useQP() ? "Enforced" : "Bypassed";
    }),
    mc_rtc::gui::Button("Toggle print joint limits", [this]() {
      printLimits_ = !printLimits_;
    }),
    mc_rtc::gui::Label("Print joint limits", [this]() {
      return printLimits_ ? "Enabled" : "Disabled";
    }));

  gui()->addElement(
    {"NewRLQPController", "Command"},
    mc_rtc::gui::NumberInput(
      "vx",
      [this]() { return rlRuntime_.command()(0); },
      [this](double v) { rlRuntime_.command()(0) = v; }),
    mc_rtc::gui::NumberInput(
      "vy",
      [this]() { return rlRuntime_.command()(1); },
      [this](double v) { rlRuntime_.command()(1) = v; }),
    mc_rtc::gui::NumberInput(
      "yaw_rate",
      [this]() { return rlRuntime_.command()(2); },
      [this](double v) { rlRuntime_.command()(2) = v; }));
}

void NewRLQPController::computeLimits()
{
  const double epsilon = 1e-5;

  mc_rbdyn::Robot & real_robot = realRobot(robotName_);

  const std::vector<std::vector<double> > & currentPos = real_robot.q();
  const std::vector<std::vector<double> > & currentVel = real_robot.alpha();
  const std::vector<std::vector<double> > & currentTau = real_robot.jointTorque();

  const std::vector<std::vector<double> > & qLimLower = real_robot.ql();
  const std::vector<std::vector<double> > & qLimUpper = real_robot.qu();

  const std::vector<std::vector<double> > & qDotLimLower = real_robot.vl();
  const std::vector<std::vector<double> > & qDotLimUpper = real_robot.vu();

  const std::vector<std::vector<double> > & tauLimLower = real_robot.tl();
  const std::vector<std::vector<double> > & tauLimUpper = real_robot.tu();

  for(size_t j = 0; j < jointNames.size(); ++j)
  {
    const std::string & joint = jointNames[j];
    const int i = real_robot.jointIndexByName(joint);
    const size_t idx = static_cast<size_t>(i);

    const double ds = dsPercent_ * (qLimUpper[idx][0] - qLimLower[idx][0]);

    const double posLimitUp = qLimUpper[idx][0] - ds;
    const double posLimitLow = qLimLower[idx][0] + ds;

    const double velLimitUp = velPercent_ * qDotLimUpper[idx][0];
    const double velLimitLow = velPercent_ * qDotLimLower[idx][0];

    const double tauLimitUp = tauLimUpper[idx][0];
    const double tauLimitLow = tauLimLower[idx][0];

    if(currentPos[idx][0] > posLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[NewRLQPController] Joint {} position upper limit breached: currentPos = {}, limit = {}",
        joint,
        currentPos[idx][0],
        posLimitUp);
    }

    if(currentPos[idx][0] < posLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[NewRLQPController] Joint {} position lower limit breached: currentPos = {}, limit = {}",
        joint,
        currentPos[idx][0],
        posLimitLow);
    }

    if(currentVel[idx][0] > velLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[NewRLQPController] Joint {} velocity upper limit breached: currentVel = {}, limit = {}",
        joint,
        currentVel[idx][0],
        velLimitUp);
    }

    if(currentVel[idx][0] < velLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[NewRLQPController] Joint {} velocity lower limit breached: currentVel = {}, limit = {}",
        joint,
        currentVel[idx][0],
        velLimitLow);
    }

    if(currentTau[idx][0] > tauLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[NewRLQPController] Joint {} torque upper limit breached: currentTau = {}, limit = {}",
        joint,
        currentTau[idx][0],
        tauLimitUp);
    }

    if(currentTau[idx][0] < tauLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[NewRLQPController] Joint {} torque lower limit breached: currentTau = {}, limit = {}",
        joint,
        currentTau[idx][0],
        tauLimitLow);
    }
  }
}
