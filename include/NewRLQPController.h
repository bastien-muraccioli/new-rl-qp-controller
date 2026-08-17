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

/**
 * @brief RL-QP Controller for deploying reinforcement learning policies on real robots.
 *
 * This controller bridges a trained RL policy (exported as ONNX) with mc_rtc's
 * whole-body QP framework augmented with Control Barrier Functions (CBFs).
 *
 * ## Control Pipeline
 *
 * At each policy step (typically 20ms):
 *   1. Build the observation vector from robot state (joint positions, velocities,
 *      IMU data, contact forces, velocity commands, etc.)
 *   2. Run inference: action = policy(observation)
 *   3. Compute position target: q* = action * action_scale + q_zero
 *   4. Compute desired torque via PD control: τ = Kp*(q* - q) - Kd*q̇
 *   5. Either:
 *      - (useQP=true)  Feed τ as input to TorqueJointTask inside the CBF-QP solver,
 *        which enforces joint limits, velocity limits, and self-collision constraints.
 *      - (useQP=false) Apply τ directly to the robot joints, bypassing the QP.
 *
 * ## Torque Equation
 *
 *   τ_i = clip(Kp_i * (q*_i - q_i) - Kd_i * q̇_i,  ±effort_limit_i)
 *
 * where:
 *   - q*_i  = action_i * action_scale_i + q_zero_i   (position target)
 *   - Kp_i  = pd_gains_ratio * kp_base_i
 *   - Kd_i  = sqrt(pd_gains_ratio) * kd_base_i
 *
 * ## Configuration
 *
 * All parameters are loaded from YAML config files :
 * etc/NewRLQPController.in.yaml :
 *  - policies root directory     Path where policy directories will be search for
 *  - default policy              Name of the first policy to run
 * {policy_root}/{policy_name} :
 * policy.yaml :
 *  - action_scale:      Per-joint scale applied to raw policy output (map: joint -> scale)
 *  - q0:                Reference joint positions (default pose), in radians
 *  - kp / kd:           PD gains per joint
 *  - use_QP:            Whether to route torques through the CBF-QP (true) or apply directly (false)
 *  - pd_gains_ratio:    Runtime gain scaling factor (1.0 = nominal gains)
 *  - action_joints:     Joints on which to apply the action. Can be a pre-registered name such as "legs"
 *  - period_s:          Policy inference period in seconds
 *  - frequency_hz:      Alternative to period_s; policy inference frequency in hertz
 *  observations.yaml :
 *  - training_convention     Convention to load default values and aliases from
 *  - list of observations and their parameters
 * {policy_root}/conventions.yaml : stores all known conventions (mjlab, isaaclab)
 *  - joint groups in training order
 *  - type aliases to correspond precisely to the name in the training environment.
 *  - observations defaults
 *
 * Contact forces must be log-compressed before insertion:
 *   f_obs = sign(f) * log(1 + |f|)
 *
 * @see utils.h for the observation/action state machine helpers.
 * @see RLPolicyInterface.h for ONNX inference wrapper.
 */

struct NewRLQPController_DLLAPI NewRLQPController : public mc_control::fsm::Controller
{
  NewRLQPController(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;
  void reset(const mc_control::ControllerResetData & reset_data) override;

  void RLuseJoyStickInputs();
  void RLuseKeyboardInputs();

  /** @brief Enable or disable the CBF-QP layer at runtime. */
  void activateQPControl(bool activate);

  rlqp::RLPolicyRuntime & rlRuntime();
  const rlqp::RLPolicyRuntime & rlRuntime() const;

  void setMaxVelCmd(double new_max_vel_cmd)
  {
    maxVelCmd = new_max_vel_cmd;
  };
  void setMaxYawCmd(double new_max_yaw_cmd)
  {
    maxYawCmd = new_max_yaw_cmd;
  };

  /** @brief Torque-space whole-body task fed into the CBF-QP solver. */
  std::shared_ptr<mc_tasks::TorqueJointTask> torqueJointTask;

  /** @brief Total number of actuated joints (from robot().refJointOrder()). */
  int nbActuatedJoints = 0;
 
  /**
   * @brief Joint names in mc_rtc's reference order (robot().refJointOrder()).
   *
   * This order is used as the canonical ordering for all Eigen vectors
   * in this controller (kp_, kd_, q_zero, q_rl, etc.).
   */
  std::vector<std::string> jointNames;

  RLStateRunner rlStateRunner;

private:
  mc_rtc::Configuration config_;

  /** @brief Register data entries visible in mc_log_ui. */
  void addLog();
  /** @brief Register GUI elements visible in RViz and mc_mujoco. */
  void addGui();
  /** @brief Load robot parameters from config. */
  void initializeRobotBasics();

  /**
   * @brief Apply RL torques directly, bypassing the QP (useQP=false mode).
   *
   * Computes τ = Kp*(q_rl - q) - Kd*q̇ and writes it to robot().mbc().jointTorque.
   * @return true if bypass was applied, false if QP should run instead.
   */
  bool byPassQPControl();
  /** @brief Log warnings when joint position/velocity/torque limits are exceeded. */
  void computeLimits();

private:
  bool printLimits_ = true;

  std::string robotName_;

  // --- CBF-QP constraint parameters ---
  double velPercent_ = 0.9; // Percentage of the max velocity taking account in the joint velocity constraint.
  double dsPercent_ = 0.01; // Percentage of the max joint range taking account in the joint position limit constraint.
  double diPercent_ = 0.1; // Doesn't matter since di > ds. This variable is not used in the constraint dynamics.

  // --- CBF Gains ---
  // More details are explained in the paper cf. Readme.md. 
  // TODO(robot) : These vealues must be tuned depending on the robot.
  double zeta_jointLimit_ = 1.2;
  double lambda_jointLimit_ = 200.0; // Same gain for joint position limits and velocity limits.
  double zeta_selfCollision_ = 1.2;
  double lambda_selfCollision_ = 200.0;

  rlqp::RLPolicyRuntime rlRuntime_;

  // --- Velocity Cmd Params ---
  std::vector<bool> DirectionButtons = std::vector<bool>(4, false); // Up, Down, Left, Right
  double joystickDeadZone = 0.02; // Dead zone for joystick inputs
  Eigen::Vector2d leftStick = Eigen::Vector2d(0.5, 0.5); // x (UP), y (LEFT)
  Eigen::Vector2d rightStick = Eigen::Vector2d(0.5, 0.5); // x (UP), y (LEFT)

  double maxVelCmd;
  double maxYawCmd;
};
