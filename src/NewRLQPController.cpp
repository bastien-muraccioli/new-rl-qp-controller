#include "NewRLQPController.h"

#include <RBDyn/MultiBodyConfig.h>

NewRLQPController::NewRLQPController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  config_ = config;
  currentPolicyIndex = size_t(config_("default_policy_index", 0));
  
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

  initializeRobot();
  initializeRLPolicy();

  addGui();
  addLog();
  mc_rtc::log::success("NewRLQPController init done");
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
}

void NewRLQPController::initializeRobot()
{
  useQP_ = config_("policies")[currentPolicyIndex]("use_QP", true);
  mc_rtc::log::info("[NewRLQPController] Using Torque Control mode");
  datastore().make<std::string>("ControlMode", "Torque");

  // get the joints order (urdf) depending on the robot used
  robotName_ = robot().name();
  jointNames = robot().refJointOrder(); // Get the joint names in the order used by the robot state
  nbActuatedJoints = jointNames.size();

  q_rl = Eigen::VectorXd::Zero(nbActuatedJoints);
  q_zero = Eigen::VectorXd::Zero(nbActuatedJoints);
  actionScale = Eigen::VectorXd::Zero(nbActuatedJoints);
  currentActionScaled = Eigen::VectorXd::Zero(nbActuatedJoints);
  kp_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kd_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kpBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kdBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  
  // Get the gains from the configuration or set default values
  pdGainsRatio_ = config_("policies")[currentPolicyIndex]("pd_gains_ratio", 1.0);
  std::map<std::string, double> actionScale_map = config_("policies")[currentPolicyIndex]("action_scale");
  std::map<std::string, double> kp_map = config_("policies")[currentPolicyIndex]("kp");
  std::map<std::string, double> kd_map = config_("policies")[currentPolicyIndex]("kd");
  q0_map_ = config_("policies")[currentPolicyIndex]("q0");
  
  // Fill the gain vectors based on the joint names and the configuration maps. If a joint is not found in the map, the joint is skipped.
  // Used for the action scale, useful if the policy doesn't control all the joints.
  auto updateIfExists =
    [&](auto& target,
        const auto& map,
        const std::string& joint_name)
  {
      if (auto it = map.find(joint_name);
          it != map.end())
      {
          target = it->second;
      }
  };
  
  int i = 0;
  for (const auto &joint_name : jointNames)
  {
    kpBase_[i] = kp_map.at(joint_name);
    kdBase_[i] = kd_map.at(joint_name);
    q_zero[i] = q0_map_.at(joint_name);
    updateIfExists(actionScale[i], actionScale_map, joint_name);
    i++;
  }

  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = sqrt(pdGainsRatio_) * kdBase_;
  torqueJointTask->setStiffness(kp_);
  torqueJointTask->setDamping(kd_);
}

void NewRLQPController::initializeRLPolicy()
{
  // load policy specific configuration
  policyPaths_ = config_("policy_path", std::vector<std::string>{"walking_better_h1.onnx"});
  configRL();

  currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
  currentAction = Eigen::VectorXd::Zero(rlPolicy->getActionSize());

  initializeRLObservation();

  // Initialize all history slots (example for this set of observations)
  // for (int i = 0; i < HISTORY_SIZE; ++i) {
  //     linVel[i] = linVel[0];
  //     angVel[i] = angVel[0];
  //     projectedGravity[i] = projectedGravity[0];
  //     velCmd[i] = velCmd[0];
  //     jointPos[i] = jointPos[0];
  //     jointVel[i] = jointVel[0];
  //     jointAction[i] = jointAction[0];
  // }
}

void NewRLQPController::initializeRLObservation()
{
  // Observation
  auto & robot = realRobot(robots()[0].name());

  // Bellow an example of how to fill the observation vector with the robot state, this should be adapted to the specific policy observation used.

  // // ---------------- Joint positions and velocities ---------------------------------------
  // const auto & q_mbc = robot.mbc().q; // MBC order
  // const auto & q_dot_mbc = robot.mbc().alpha; // MBC order
  // Eigen::VectorXd q_rlFrameworkOrdered, q_0_rlFrameworkOrdered, q_dot_rlFrameworkOrdered;

  // if (currentPolicyIndex < 2) // Use all joints as observation
  // {
  //   q_rlFrameworkOrdered = Eigen::VectorXd::Zero(nbActuatedJoints);
  //   q_0_rlFrameworkOrdered = Eigen::VectorXd::Zero(nbActuatedJoints);
  //   q_dot_rlFrameworkOrdered = Eigen::VectorXd::Zero(nbActuatedJoints);

  //   for(size_t i = 0; i < jointNames.size(); ++i)
  //   {
  //     const auto & joint_name = jointNames[i];

  //     // Fill mc_rtc ordered vectors
  //     const double q = q_mbc[robot.jointIndexByName(joint_name)][0];
  //     const double q_dot = q_dot_mbc[robot.jointIndexByName(joint_name)][0];

  //     // RL remapping
  //     int rl_index = mcRtcToRLFrameworkJointMap[i];
  //     q_rlFrameworkOrdered(rl_index) = q;
  //     q_0_rlFrameworkOrdered(rl_index) = q_zero(i);
  //     q_dot_rlFrameworkOrdered(rl_index) = q_dot;
  //   }
  // }
  // else // Use only the joints that are in the action space as observation 
  // {
  //   const int policyObsJointSize = rlPolicy->getActionSize();
  //   q_rlFrameworkOrdered = Eigen::VectorXd::Zero(policyObsJointSize);
  //   q_0_rlFrameworkOrdered = Eigen::VectorXd::Zero(policyObsJointSize);
  //   q_dot_rlFrameworkOrdered = Eigen::VectorXd::Zero(policyObsJointSize);
  //   int i = 0;
  //   for (const auto &joint_name : refJointOrderRLAction)
  //   {
  //     q_rlFrameworkOrdered[i] = q_mbc[robot.jointIndexByName(joint_name)][0];
  //     q_dot_rlFrameworkOrdered[i] = q_dot_mbc[robot.jointIndexByName(joint_name)][0];
  //     q_0_rlFrameworkOrdered[i] = q0_map_.at(joint_name);
  //     i++;
  //   } 
  // }

  // // gravity, fb linear and angular velocity in floating base frame -------------------------------- 
  // const auto & X_0_body = robot.mbc().bodyPosW[robot.mb().bodyIndexByName("Body")];
  // const auto & bodyVel = robot.mbc().bodyVelB[robot.mb().bodyIndexByName("Body")];
  // Eigen::Matrix3d R_world_to_body = X_0_body.rotation();
  // Eigen::Vector3d gravity_b = R_world_to_body * Eigen::Vector3d(0.0, 0.0, -1.0);
  // Eigen::Vector3d angVel_b = bodyVel.angular();
  // Eigen::Vector3d linVel_b = bodyVel.linear();

  // projectedGravity[0] = gravity_b;
  // angVel[0] = angVel_b;
  // linVel[0] = linVel_b;
  // velCmd[0] = currentVelCmd;
  // jointPos[0] = q_rlFrameworkOrdered - q_0_rlFrameworkOrdered; // Start with current joint positions
  // jointVel[0] = q_dot_rlFrameworkOrdered;
  // jointAction[0] = currentAction;
}

bool NewRLQPController::byPassQPControl()
{
  if(useQP_) return false; // QP is not bypassed, do nothing

  robot().forwardKinematics();
  robot().forwardVelocity();
  robot().forwardAcceleration();

  Eigen::VectorXd tau_rl_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  const auto & q_mbc = robot().mbc().q;
  const auto & q_dot_mbc = robot().mbc().alpha;

  int i = 0;
  for(const auto &joint_name : jointNames)
  {
      const double q = q_mbc[robot().jointIndexByName(joint_name)][0];
      const double q_dot = q_dot_mbc[robot().jointIndexByName(joint_name)][0];
      tau_rl_(i) = kp_(i) * (q_rl(i) - q) - kd_(i) * q_dot;
      robot().mbc().jointTorque[robot().jointIndexByName(joint_name)][0] = tau_rl_(i);
      i++;
  }
  return true;
}

void NewRLQPController::addLog()
{
  // Robot State variables
  logger().addLogEntry("NewRLQPController_kp_base", [this]() { return kpBase_; });
  logger().addLogEntry("NewRLQPController_kd_base", [this]() { return kdBase_; });
  logger().addLogEntry("NewRLQPController_kp_current", [this]() { return kp_; });
  logger().addLogEntry("NewRLQPController_kd_current", [this]() { return kd_; });
  logger().addLogEntry("NewRLQPController_pd_gains_ratio", [this]() { return pdGainsRatio_; });

  // RL variables
  logger().addLogEntry("NewRLQPController_RL_q", [this]() { return q_rl; });
  logger().addLogEntry("NewRLQPController_RL_qZero", [this]() { return q_zero; });
  logger().addLogEntry("NewRLQPController_RL_currentObservation", [this]() { return currentObservation; });
  logger().addLogEntry("NewRLQPController_RL_currentAction", [this]() { return currentAction; });
  logger().addLogEntry("NewRLQPController_RL_currentActionScaled", [this]() { return currentActionScaled; });
  logger().addLogEntry("NewRLQPController_RL_actionScale", [this]() { return actionScale; });
  
  // Controller state variables
  logger().addLogEntry("NewRLQPController_useQP", [this]() { return useQP_; });

  // Log current policy (combined index and path)
  logger().addLogEntry("NewRLQPController_currentPolicy", [this]() { 
    return std::to_string(currentPolicyIndex) + ": " + policyPaths_[currentPolicyIndex]; 
  });

  // Log observation
  // for (int i = 0; i < HISTORY_SIZE; ++i) {
  //   logger().addLogEntry("NewRLQPController_obs_linVel_" + std::to_string(i), [this, i]() { return linVel[i]; });
  //   logger().addLogEntry("NewRLQPController_obs_angVel_" + std::to_string(i), [this, i]() { return angVel[i]; });
  //   logger().addLogEntry("NewRLQPController_obs_projectedGravity_" + std::to_string(i), [this, i]() { return projectedGravity[i]; });
  //   logger().addLogEntry("NewRLQPController_obs_velCmd_" + std::to_string(i), [this, i]() { return velCmd[i]; });
  //   logger().addLogEntry("NewRLQPController_obs_jointPos_" + std::to_string(i), [this, i]() { return jointPos[i]; });
  //   logger().addLogEntry("NewRLQPController_obs_jointVel_" + std::to_string(i), [this, i]() { return jointVel[i]; });
  //   logger().addLogEntry("NewRLQPController_obs_jointAction_" + std::to_string(i), [this, i]() { return jointAction[i]; });
  //   logger().addLogEntry("NewRLQPController_obs_footContactForces_" + std::to_string(i), [this, i]() { return footContactForces[i]; });
  // }
}

void NewRLQPController::addGui()
{
  gui()->addElement({"NewRLQPController", "Policy"},
  mc_rtc::gui::Label("Current policy", [this]() -> const std::string & 
    { 
      return policyPaths_[currentPolicyIndex]; 
    })
  );

  // Add PD gains ratio slider
  gui()->addElement({"NewRLQPController", "PD Gains"},
    mc_rtc::gui::NumberSlider(
      "PD Gains Ratio", [this]() { return pdGainsRatio_; },
      [this](double v) { 
        pdGainsRatio_ = v;
        kp_ = pdGainsRatio_ * kpBase_;
        kd_ = sqrt(pdGainsRatio_) * kdBase_;
        torqueJointTask->setStiffness(kp_);
        torqueJointTask->setDamping(kd_);
      }, 0.0, 2.0),
    mc_rtc::gui::Label("Current kp", kp_),
    mc_rtc::gui::Label("Current kd", kd_)
  );

  gui()->addElement({"ControlMode"}, 
    mc_rtc::gui::Button("Toggle QP Control", [this]()
        {
          useQP_ = !useQP_;
        }),
    mc_rtc::gui::Label("QP Control", [this]()
      {
        return useQP_ ? "Enforced" : "Bypassed";
      }),
    mc_rtc::gui::Button("Toggle print joint limits", [this]()
      {
        printLimits_ = !printLimits_;
      }),
    mc_rtc::gui::Label("Print joint limits", [this]()
      {
        return printLimits_ ? "Enabled" : "Disabled";
      })
    );

  // Example of changing the velocity command from the GUI, this should be adapted to the specific policy and observation used.
  // gui()->addElement({"NewRLQPController", "Velocity Command"},
  //     mc_rtc::gui::ArrayInput("Current Velocity Command", {"vx", "vy", "yaw_rate"}, currentVelCmd));
}

void NewRLQPController::configRL()
{
  mc_rtc::log::info("[NewRLQPController] Loading RL policy [{}]: {}", currentPolicyIndex, policyPaths_[currentPolicyIndex]);
  try {
    rlPolicy = std::make_unique<RLPolicyInterface>(policyPaths_[currentPolicyIndex]);
    if(rlPolicy) {
      mc_rtc::log::success("[NewRLQPController] RL policy loaded successfully");
      // Initialize observation vector with the correct size from the loaded policy
      currentObservation = Eigen::VectorXd::Zero(rlPolicy->getObservationSize());
      mc_rtc::log::info("[NewRLQPController] Initialized observation vector with size: {}", rlPolicy->getObservationSize());
      currentAction = Eigen::VectorXd::Zero(rlPolicy->getActionSize());
      mc_rtc::log::info("[NewRLQPController] Initialized action vector with size: {}", rlPolicy->getActionSize());
    } else {
      mc_rtc::log::error_and_throw("[NewRLQPController] RL policy creation failed - policy is null");
    }
  } catch(const std::exception& e) {
    mc_rtc::log::error_and_throw("[NewRLQPController] Failed to load RL policy: {}", e.what());
  }

  policyStepSize = config_("policies")[currentPolicyIndex]("policy_step_size", 0.01);
  const double physicsStepSize = config_("policies")[currentPolicyIndex]("physics_step_size", 0.001);
  if(physicsStepSize - timeStep > 1e-6) {
    mc_rtc::log::warning("[NewRLQPController] Physics step size ({:.3f} s) is larger than controller time step ({:.3f} s). This may cause issues with the policy. Consider fixing the controller time step.", physicsStepSize, timeStep);
  }

  refJointOrderRLAction = config_("policies")[currentPolicyIndex]("ref_joint_order", std::vector<std::string>{});
  if(refJointOrderRLAction.size() != size_t(rlPolicy->getActionSize())) {
    mc_rtc::log::error_and_throw("[NewRLQPController] Reference joint order size ({}) does not match policy action size ({}). Please check the configuration.", refJointOrderRLAction.size(), rlPolicy->getActionSize());
  }

  // Create mapping from action indices to robot joint indices based on the reference joint order
  actionToDofMap.resize(refJointOrderRLAction.size(), -1); // Initialize with -1 to indicate unmapped actions
  for (size_t j = 0; j < refJointOrderRLAction.size(); ++j) {
    for (int i = 0; i < nbActuatedJoints; ++i) {
      if (jointNames[i] == refJointOrderRLAction[j]) {
        actionToDofMap[j] = i;
        break;
      }
    }
  }

  // Create mapping between RL framework joint order and mc_rtc joint indices, this is useful for correctly ordering the observation and action vectors according to the robot's joint order in mc_rtc.
  auto q0_map_cfg = config_("policies")[currentPolicyIndex]("q0");
  std::vector<std::string> keys = q0_map_cfg.keys(); // this preserves order

  if (keys.size() != static_cast<size_t>(nbActuatedJoints)) {
    mc_rtc::log::error_and_throw("[NewRLQPController] The number of joints in q0 config ({}) does not match the robot's dof number ({}). Please check the configuration.", keys.size(), nbActuatedJoints);
  }

  // Compare if q0 order matches mc_rtc joint order, just to log it since the mapping will be created anyway.
  bool orderMatches = true;
  for (size_t i = 0; i < keys.size(); ++i) {
      if (keys[i] != jointNames[i]) {
          orderMatches = false;
          break;
      }
  }
  if(orderMatches) mc_rtc::log::info("[NewRLQPController] The order of joints in q0 config matches the robot's joint order in mc_rtc.");

  // rlFrameworkToMcRtcJointMap.resize(dofNumber, -1); // Initialize with -1 to indicate unmapped joints
  mcRtcToRLFrameworkJointMap.resize(nbActuatedJoints, -1);
  int j = 0;
  for (const auto & key : keys) { 
    for (int i = 0; i < nbActuatedJoints; ++i) {
      if (jointNames[i] == key) {
        // rlFrameworkToMcRtcJointMap[j] = i;
        mcRtcToRLFrameworkJointMap[i] = j;
        break;
      }
    }
    j++;
  }

  for (int i = 0; i < nbActuatedJoints; ++i) {
    if (mcRtcToRLFrameworkJointMap[i] == -1) {
      mc_rtc::log::error_and_throw("[NewRLQPController] Joint '{}' was not properly mapped!", jointNames[i]);
    }
  }
}

void NewRLQPController::computeLimits()
{
  const double epsilon = 1e-5;

  auto & real_robot = realRobot(robots()[0].name());
  const auto & currentPos = real_robot.q();
  const auto & currentVel = real_robot.alpha();
  const auto & currentTau = real_robot.jointTorque();

  const auto & qLimLower = real_robot.ql();
  const auto & qLimUpper = real_robot.qu();

  const auto & qDotLimLower = real_robot.vl();
  const auto & qDotLimUpper = real_robot.vu();

  const auto & tauLimLower = real_robot.tl();
  const auto & tauLimUpper = real_robot.tu();

  for (std::string joint : robot().refJointOrder())
  {
    int i = robot().jointIndexByName(joint);

    const double ds = dsPercent_ * (qLimUpper[i][0] - qLimLower[i][0]);
    const double posLimitUp = qLimUpper[i][0] - ds;
    const double posLimitLow = qLimLower[i][0] + ds;
    const double velLimitUp = velPercent_ * qDotLimUpper[i][0];
    const double velLimitLow = velPercent_ * qDotLimLower[i][0];
    const double tauLimitUp = tauLimUpper[i][0];
    const double tauLimitLow = tauLimLower[i][0];

    if (currentPos[i][0] > posLimitUp + epsilon)
    {
      mc_rtc::log::warning("[NewRLQPController] Joint {} position upper limit breached: currentPos = {}, limit = {}", joint, currentPos[i][0], posLimitUp);
    }
    if (currentPos[i][0] < posLimitLow - epsilon)
    {
      mc_rtc::log::warning("[NewRLQPController] Joint {} position lower limit breached: currentPos = {}, limit = {}", joint, currentPos[i][0], posLimitLow);
    }
    if (currentVel[i][0] > velLimitUp + epsilon)
    {
      mc_rtc::log::warning("[NewRLQPController] Joint {} velocity upper limit breached: currentVel = {}, limit = {}", joint, currentVel[i][0], velLimitUp);
    }
    if (currentVel[i][0] < velLimitLow - epsilon)
    {
      mc_rtc::log::warning("[NewRLQPController] Joint {} velocity lower limit breached: currentVel = {}, limit = {}", joint, currentVel[i][0], velLimitLow);
    }
    if (currentTau[i][0] > tauLimitUp + epsilon)    {
      mc_rtc::log::warning("[NewRLQPController] Joint {} torque upper limit breached: currentTau = {}, limit = {}", joint, currentTau[i][0], tauLimitUp);
    }
    if (currentTau[i][0] < tauLimitLow - epsilon)    {
      mc_rtc::log::warning("[NewRLQPController] Joint {} torque lower limit breached: currentTau = {}, limit = {}", joint, currentTau[i][0], tauLimitLow);
    }
  }
}