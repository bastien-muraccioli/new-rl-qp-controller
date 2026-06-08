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
 * All parameters are loaded from the YAML config file. See etc/NewRLQPController.in.yaml
 * for a fully documented example. Key sections per policy entry:
 *   - policy_path:       Path(s) to ONNX policy file(s)
 *   - action_scale:      Per-joint scale applied to raw policy output (map: joint -> scale)
 *   - q0:                Reference joint positions (default pose), in radians
 *   - kp / kd:           PD gains per joint
 *   - ref_joint_order:   Ordered list of joints controlled by the policy (action vector order)
 *   - use_QP:            Whether to route torques through the CBF-QP (true) or apply directly (false)
 *   - pd_gains_ratio:    Runtime gain scaling factor (1.0 = nominal gains)
 *   - policy_step_size:  Policy inference period in seconds (e.g. 0.02)
 *   - physics_step_size: Simulation/control timestep in seconds (e.g. 0.0025)
 *
 * ## Observation Vector
 *
 * The observation is built in utils::getCurrentObservation() and must exactly match
 * what the policy was trained on. Typical terms (history_length stacked, oldest first):
 *   - base_lin_vel:          Linear velocity of the floating base in body frame   [3]
 *   - base_ang_vel:          Angular velocity of the floating base in body frame   [3]
 *   - projected_gravity:     Unit gravity vector in body frame (from R_world_to_body * [0,0,-1]) [3]
 *   - joint_pos:             Joint positions relative to default pose: q - q_zero  [N_joints]
 *   - joint_vel:             Joint velocities                                      [N_joints]
 *   - foot_contact_forces:   Log-compressed world-frame contact forces             [N_feet * 3]
 *   - last_action:           Previous raw policy output (before scaling)           [N_action]
 *   - command:               Velocity command [vx, vy, yaw_rate]                  [3]
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

  /** @brief Enable or disable the CBF-QP layer at runtime. */
  void activateQPControl(bool activate);

  rlqp::RLPolicyRuntime & rlRuntime();
  const rlqp::RLPolicyRuntime & rlRuntime() const;

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
  double velPercent_ = 0.95; // Percentage of the max velocity taking account in the joint velocity constraint.
  double dsPercent_ = 0.01; // Percentage of the max joint range taking account in the joint position limit constraint.
  double diPercent_ = 0.1; // Doesn't matter since di > ds. This variable is not used in the constraint dynamics.

  // --- CBF Gains ---
  // More details are explained in the paper cf. Readme.md. 
  // Must be tuned depending on the robot.
  double zeta_jointLimit_ = 1.2;
  double lambda_jointLimit_ = 100.0; // Same gain for joint position limits and velocity limits.
  double zeta_selfCollision_ = 1.2;
  double lambda_selfCollision_ = 10.0;

  rlqp::RLPolicyRuntime rlRuntime_;
};
