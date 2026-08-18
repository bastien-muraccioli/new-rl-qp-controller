#include "NewRLQPController.h"

#include <RBDyn/MultiBodyConfig.h>

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <fcntl.h>
#include <mc_joystick_plugin/joystick_inputs.h>
#include <termios.h>

#include <cmath>

NewRLQPController::NewRLQPController(mc_rbdyn::RobotModulePtr rm,
                                     double dt,
                                     const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;

  //Initialize Constraints
  // TODO(robot): review which constraints are valid for the selected robot.
  // These defaults provide joint-limit and self-collision protection
  selfCollisionConstraint->setCollisionsDampers(solver(), {zeta_selfCollision_, lambda_selfCollision_});
  solver().removeConstraintSet(dynamicsConstraint);
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {diPercent_, dsPercent_, 0.0, zeta_jointLimit_, lambda_jointLimit_}, velPercent_, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Remove the default posture task created by the FSM
  solver().removeTask(getPostureTask(robot().name()));
  // Initialize Task
  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(
      solver(), robot().robotIndex(), 100.0, 1); // TODO(robot): tune stiffness/weight
  solver().addTask(torqueJointTask);
  initializeRobotBasics();

  rlRuntime_.configure(config_, *this, torqueJointTask);

  addGui();
  addLog();

  mc_rtc::log::success("[NewRLQPController] init done");
}

bool NewRLQPController::run()
{
  // Use joystick plugin if present else jeyboard inputs
  controllerAvailable = datastore().has("Joystick::connected") && datastore().get<bool>("Joystick::connected");
  if(controllerAvailable && toggleJoystick)
    RLuseJoyStickInputs();
  else if(toggleKeyboard)
    RLuseKeyboardInputs();

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

void NewRLQPController::RLuseJoyStickInputs()
{
  // Get joystick functions
  auto & stickFunc = datastore().get<std::function<Eigen::Vector2d(joystickAnalogicInputs)>>("Joystick::Stick");

  // Read sticks values
  leftStick = stickFunc(joystickAnalogicInputs::L_STICK);
  // Apply dead zone
  double vel_x = 0.0;
  if(std::abs(leftStick(0) - 0.5) > joystickDeadZone)
  {
    vel_x = (leftStick(0) - 0.5) * 2.0 * maxVelCmd;
  }
  double vel_y = 0.0;
  if(std::abs(leftStick(1) - 0.5) > joystickDeadZone)
  {
    vel_y = (leftStick(1) - 0.5) * 2.0 * maxVelCmd;
  }

  rightStick = stickFunc(joystickAnalogicInputs::R_STICK);
  double yaw_cmd = 0.0;
  if(std::abs(rightStick(1) - 0.5) > joystickDeadZone)
  {
    yaw_cmd = (rightStick(1) - 0.5) * 2.0 * maxYawCmd;
  }

  // Read D-pad buttons
  DirectionButtons = {datastore().get<bool>("Joystick::UpPad"), datastore().get<bool>("Joystick::DownPad"),
                      datastore().get<bool>("Joystick::LeftPad"), datastore().get<bool>("Joystick::RightPad")};

  for(size_t i = 0; i < DirectionButtons.size(); ++i)
  {
    if(DirectionButtons[i])
    {
      switch(i)
      {
        case 0: // Up
          vel_x += 1.0 * maxVelCmd;
          break;
        case 1: // Down
          vel_x -= 1.0 * maxVelCmd;
          break;
        case 2: // Left
          vel_y += 1.0 * maxVelCmd;
          break;
        case 3: // Right
          vel_y -= 1.0 * maxVelCmd;
          break;
        default:
          break;
      }
    }
  }
  rlRuntime_.setCommand({vel_x, vel_y, yaw_cmd});
}

void NewRLQPController::RLuseKeyboardInputs()
{
  struct Ctx
  {
    bool ready = false;
    termios old{};
    bool seen[4] = {};
    std::chrono::steady_clock::time_point ts[4];
    std::array<char, 64> buf{};
    size_t sz = 0;
  };
  static Ctx k;

  if(!k.ready)
  {
    if(::isatty(STDIN_FILENO) != 1) return;
    k.ready = true;
    ::tcgetattr(STDIN_FILENO, &k.old);
    termios raw = k.old;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = raw.c_cc[VTIME] = 0;
    ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    ::fcntl(STDIN_FILENO, F_SETFL, ::fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
  }

  char tmp[32];
  ssize_t n = ::read(STDIN_FILENO, tmp, sizeof(tmp));
  if(n > 0 && k.sz + n < 64) std::copy(tmp, tmp + n, k.buf.begin() + k.sz), k.sz += n;

  auto now = std::chrono::steady_clock::now();
  for(size_t i = 0; i + 2 < k.sz; ++i)
    if(k.buf[i] == 27 && k.buf[i + 1] == '[')
    {
      int idx = k.buf[i + 2] == 'A'   ? 0
                : k.buf[i + 2] == 'B' ? 1
                : k.buf[i + 2] == 'D' ? 2
                : k.buf[i + 2] == 'C' ? 3
                                      : -1;
      if(idx >= 0) k.seen[idx] = true, k.ts[idx] = now;
      i += 2;
    }
  std::copy(k.buf.begin() + (k.sz > 2 ? k.sz - 2 : 0), k.buf.begin() + k.sz, k.buf.begin());
  k.sz = k.sz > 2 ? 2 : 0;

  const auto active = [&](int i)
  { return k.seen[i] && std::chrono::duration_cast<std::chrono::milliseconds>(now - k.ts[i]).count() < 500; };
  rlRuntime_.setCommand({(active(0) ? maxVelCmd : 0.0) - (active(1) ? maxVelCmd : 0.0),
                         (active(2) ? maxVelCmd : 0.0) - (active(3) ? maxVelCmd : 0.0), rlRuntime_.command()(2)});
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
    tau_rl(i) = rlRuntime_.kp()(i) * (rlRuntime_.q_rl()(i) - q) - rlRuntime_.kd()(i) * q_dot;
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
  logger().addLogEntry("NewRLQPController_RL_phase", [this]() { return rlRuntime_.phase(); });
  logger().addLogEntry("NewRLQPController_RL_policy_period_s", [this]() { return rlRuntime_.policyStepSize(); });
  logger().addLogEntry("NewRLQPController_RL_update_count", [this]() { return rlRuntime_.policyUpdateCount(); });

  rlRuntime_.addLogObs(*this);
}

void NewRLQPController::addGui()
{
  gui()->addElement(
    {"NewRLQPController", "Policy"},
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
    {"NewRLQPController", "Runtime"},
    mc_rtc::gui::Label("Controller period [s]", [this]() { return timeStep; }),
    mc_rtc::gui::Label("Controller rate [Hz]", [this]() { return 1.0 / timeStep; }),
    mc_rtc::gui::Label("Policy period [s]", [this]() { return rlRuntime_.policyStepSize(); }),
    mc_rtc::gui::Label("Policy rate [Hz]", [this]() { return rlRuntime_.policyRate(); }),
    mc_rtc::gui::Label("Policy updates", [this]() { return rlRuntime_.policyUpdateCount(); }),
    mc_rtc::gui::Label("Phase", [this]() { return rlRuntime_.phase(); }));

  gui()->addElement(
    {"NewRLQPController", "Command"},
    mc_rtc::gui::Button(
        "Toggle Joystick Plugin",
        [this]()
        {
          toggleJoystick = !toggleJoystick;
          if(toggleJoystick)
            toggleKeyboard = false;
        }),
    mc_rtc::gui::Label(
        "Current velcity control mode",
        [this]()
        {
          if(toggleKeyboard)
            return std::string{"Keyboard"};
          if(toggleJoystick && controllerAvailable)
            return std::string{"mc_joystick_plugin"};
          return std::string{"GUI"};
        }),
    mc_rtc::gui::Button(
        "Toggle Keyboard",
        [this]()
        {
          toggleKeyboard = !toggleKeyboard;
          if(toggleKeyboard)
            toggleJoystick = false;
        }),
    mc_rtc::gui::Label(
        "Joystick plugin available",
        [this]() { return controllerAvailable ? "Yes" : "No"; }),
    mc_rtc::gui::Label("Command values", []() { return std::string(" "); }),
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
      [this](double v) { rlRuntime_.command()(2) = v; }),
    mc_rtc::gui::Label("Max values", []() { return std::string(" "); }),
    mc_rtc::gui::NumberInput(
        "max_vel_cmd",
        [this]() { return maxVelCmd; }, [this](double v) { maxVelCmd = v; }),
    mc_rtc::gui::NumberInput(
        "max_yaw_cmd",
        [this]() { return maxYawCmd; }, [this](double v) { maxYawCmd = v; }));
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
