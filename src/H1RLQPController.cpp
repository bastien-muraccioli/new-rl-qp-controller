#include "H1RLQPController.h"

#include <RBDyn/MultiBodyConfig.h>

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <cmath>

H1RLQPController::H1RLQPController(mc_rbdyn::RobotModulePtr rm,
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

  mc_rtc::log::success("[H1RLQPController] init done");
}

bool H1RLQPController::run()
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

void H1RLQPController::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
  rlRuntime_.reset(*this);
}

void H1RLQPController::activateQPControl(bool activate)
{
  rlRuntime_.setUseQP(activate);
}

rlqp::RLPolicyRuntime & H1RLQPController::rlRuntime() { return rlRuntime_; }

const rlqp::RLPolicyRuntime & H1RLQPController::rlRuntime() const { return rlRuntime_; }

void H1RLQPController::initializeRobotBasics()
{
  mc_rtc::log::info("[H1RLQPController] Using torque control mode");

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
  if(!datastore().has("anchorFrameFunction"))
  {
    datastore().make_call("anchorFrameFunction", [this](const mc_rbdyn::Robot & real_robot) {return createContactAnchor(real_robot);});
  }
}

bool H1RLQPController::byPassQPControl()
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
    tau_rl(i) = rlRuntime_.kp()(i) * (rlRuntime_.q_rl()(i) - q) - rlRuntime_.kd()(i) * q_dot;
    robot().mbc().jointTorque[robot().jointIndexByName(joint_name)][0] = tau_rl(i);
    i++;
  }

  return true;
}

void H1RLQPController::addLog()
{
  // Robot State variables
  logger().addLogEntry("H1RLQPController_kp_base", [this]() { return rlRuntime_.kpBase(); });
  logger().addLogEntry("H1RLQPController_kd_base", [this]() { return rlRuntime_.kdBase(); });
  logger().addLogEntry("H1RLQPController_kp_current", [this]() { return rlRuntime_.kp(); });
  logger().addLogEntry("H1RLQPController_kd_current", [this]() { return rlRuntime_.kd(); });
  logger().addLogEntry("H1RLQPController_pd_gains_ratio", [this]() { return rlRuntime_.pdGainsRatio(); });

  // RL variables
  logger().addLogEntry("H1RLQPController_RL_q", [this]() { return rlRuntime_.q_rl(); });
  logger().addLogEntry("H1RLQPController_RL_qZero", [this]() { return rlRuntime_.q_zero(); });
  logger().addLogEntry("H1RLQPController_RL_currentObservation", [this]() { return rlRuntime_.currentObservation(); });
  logger().addLogEntry("H1RLQPController_RL_currentAction", [this]() { return rlRuntime_.currentAction(); });
  logger().addLogEntry("H1RLQPController_RL_currentActionScaled", [this]() { return rlRuntime_.currentActionScaled(); });
  logger().addLogEntry("H1RLQPController_RL_actionScale", [this]() { return rlRuntime_.actionScale(); });
  logger().addLogEntry("H1RLQPController_RL_command", [this]() { return rlRuntime_.command(); });

  // Controller state variables
  logger().addLogEntry("H1RLQPController_useQP", [this]() { return rlRuntime_.useQP(); });

  // Log current policy (name and convention)
  logger().addLogEntry("H1RLQPController_currentPolicy", [this]() { return rlRuntime_.currentPolicyName(); });
  logger().addLogEntry("H1RLQPController_observationConvention", [this]() { return rlRuntime_.conventionName(); });
  logger().addLogEntry("H1RLQPController_RL_phase", [this]() { return rlRuntime_.phase(); });
  logger().addLogEntry("H1RLQPController_RL_policy_period_s", [this]() { return rlRuntime_.policyStepSize(); });
  logger().addLogEntry("H1RLQPController_RL_update_count", [this]() { return rlRuntime_.policyUpdateCount(); });

  rlRuntime_.addLogObs(*this);
}

void H1RLQPController::addGui()
{
  gui()->addElement(
    {"H1RLQPController", "Policy"},
    mc_rtc::gui::Label("Current policy", [this]() { return rlRuntime_.currentPolicyName(); }),
    mc_rtc::gui::Label("Current policy folder", [this]() { return rlRuntime_.currentPolicyFolder(); }),
    mc_rtc::gui::Label("Observation convention", [this]() { return rlRuntime_.conventionName(); }),
    mc_rtc::gui::Label("Observation source", [this]() { return rlRuntime_.observationSource(); }),
    mc_rtc::gui::Label("Base body", [this]() { return rlRuntime_.baseBody(); }),
    mc_rtc::gui::Label("Observation size", [this]() { return rlRuntime_.observationSize(); }),
    mc_rtc::gui::Label("Action size", [this]() { return rlRuntime_.actionSize(); }),
    mc_rtc::gui::Label("Controlled action size", [this]() { return rlRuntime_.controlledActionSize(); }),
    mc_rtc::gui::ComboInput(
      "Select policy",
      rlRuntime_.availablePolicyNames(),
      [this]() { return rlRuntime_.currentPolicyName(); },
      [this](const std::string & policyName) {
        rlRuntime_.loadPolicyByName(policyName, *this, torqueJointTask);
      }),
    mc_rtc::gui::Button("Reload current policy", [this]() {
      rlRuntime_.reloadCurrentPolicy(*this, torqueJointTask);
    }));

  gui()->addElement(
    {"H1RLQPController", "PD Gains"},
    mc_rtc::gui::NumberSlider(
      "PD Gains Ratio",
      [this]() { return rlRuntime_.pdGainsRatio(); },
      [this](double v) { rlRuntime_.setPDGainsRatio(v, torqueJointTask); },
      0.0,
      2.0),
    mc_rtc::gui::Label("Current kp", [this]() { return rlRuntime_.kp(); }),
    mc_rtc::gui::Label("Current kd", [this]() { return rlRuntime_.kd(); }));

  gui()->addElement(
    {"H1RLQPController", "Control"},
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
    {"H1RLQPController", "Runtime"},
    mc_rtc::gui::Label("Controller period [s]", [this]() { return timeStep; }),
    mc_rtc::gui::Label("Controller rate [Hz]", [this]() { return 1.0 / timeStep; }),
    mc_rtc::gui::Label("Policy period [s]", [this]() { return rlRuntime_.policyStepSize(); }),
    mc_rtc::gui::Label("Policy rate [Hz]", [this]() { return rlRuntime_.policyRate(); }),
    mc_rtc::gui::Label("Policy updates", [this]() { return rlRuntime_.policyUpdateCount(); }),
    mc_rtc::gui::Label("Phase", [this]() { return rlRuntime_.phase(); }));

  gui()->addElement(
    {"H1RLQPController", "Command"},
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

void H1RLQPController::computeLimits()
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
        "[H1RLQPController] Joint {} position upper limit breached: currentPos = {}, limit = {}",
        joint,
        currentPos[idx][0],
        posLimitUp);
    }

    if(currentPos[idx][0] < posLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[H1RLQPController] Joint {} position lower limit breached: currentPos = {}, limit = {}",
        joint,
        currentPos[idx][0],
        posLimitLow);
    }

    if(currentVel[idx][0] > velLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[H1RLQPController] Joint {} velocity upper limit breached: currentVel = {}, limit = {}",
        joint,
        currentVel[idx][0],
        velLimitUp);
    }

    if(currentVel[idx][0] < velLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[H1RLQPController] Joint {} velocity lower limit breached: currentVel = {}, limit = {}",
        joint,
        currentVel[idx][0],
        velLimitLow);
    }

    if(currentTau[idx][0] > tauLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[H1RLQPController] Joint {} torque upper limit breached: currentTau = {}, limit = {}",
        joint,
        currentTau[idx][0],
        tauLimitUp);
    }

    if(currentTau[idx][0] < tauLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[H1RLQPController] Joint {} torque lower limit breached: currentTau = {}, limit = {}",
        joint,
        currentTau[idx][0],
        tauLimitLow);
    }
  }
}

std::pair<sva::PTransformd, Eigen::Vector3d> H1RLQPController::createContactAnchor(const mc_rbdyn::Robot & anchorRobot)
{
  sva::PTransformd X_foot_r = anchorRobot.bodyPosW("right_ankle_link");
  sva::PTransformd X_foot_l = anchorRobot.bodyPosW("left_ankle_link");

  sva::MotionVecd v_foot_r = anchorRobot.bodyVelW("right_ankle_link");
  sva::MotionVecd v_foot_l = anchorRobot.bodyVelW("left_ankle_link");

  int right_knee_index = int(robot().jointIndexByName("right_knee_joint")) + 5;
  int left_knee_index = int(robot().jointIndexByName("left_knee_joint")) + 5;
  double tau_ext_knee_r =  abs(robot().externalTorques()[right_knee_index]);
  double tau_ext_knee_l =  abs(robot().externalTorques()[left_knee_index]);
  double leftFootRatio = tau_ext_knee_l/(tau_ext_knee_r+tau_ext_knee_l);
  if(tau_ext_knee_r + tau_ext_knee_l < 0.02)
  {
    leftFootRatio = 0.5;
  }
         
  Eigen::VectorXd w_r = X_foot_r.translation();
  Eigen::VectorXd w_l = X_foot_l.translation();
  Eigen::VectorXd contact_anchor = (w_r * (1 - leftFootRatio) + w_l * leftFootRatio)  ;
  Eigen::VectorXd anchor_vel = (v_foot_r.linear() * (1 - leftFootRatio) + v_foot_l.linear() * leftFootRatio);
  contactAnchorTf_ = sva::PTransformd(Eigen::Matrix3d::Identity(), contact_anchor); 

  return {contactAnchorTf_, anchor_vel};
}