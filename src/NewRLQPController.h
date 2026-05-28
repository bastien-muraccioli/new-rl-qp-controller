#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_tasks/TorqueJointTask.h>

#include "api.h"

#include "RLPolicyInterface.h"
#include "utils.h"


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

  /**
   * @brief Populate the history buffers with the current robot state.
   *
   * Called once at startup and at the beginning of each RL FSM state.
   * Must be overridden (filled in) by the user to match the specific
   * observation structure expected by the loaded policy.
   *
   * Typical implementation:
   * @code
   *   auto & robot = realRobot(robots()[0].name());
   *   // ... fill linVel[0], angVel[0], jointPos[0], etc. ...
   * @endcode
   */
  void initializeRLObservation();

  // =========================================================================
  // QP Task
  // =========================================================================

  /** @brief Torque-space whole-body task fed into the CBF-QP solver. */
  std::shared_ptr<mc_tasks::TorqueJointTask> torqueJointTask;
  
  // =========================================================================
  // Robot state
  // =========================================================================

  /** @brief Total number of actuated joints (from robot().refJointOrder()). */
  int nbActuatedJoints = 0;

  /**
   * @brief Joint names in mc_rtc's reference order (robot().refJointOrder()).
   *
   * This order is used as the canonical ordering for all Eigen vectors
   * in this controller (kp_, kd_, q_zero, q_rl, etc.).
   */
  std::vector<std::string> jointNames;

  // =========================================================================
  // RL action
  // =========================================================================

  /**
   * @brief Current joint position targets sent to the PD controller.
   *
   * Computed as: q_rl = currentActionScaled + q_zero
   * Size: nbActuatedJoints (uncontrolled joints keep their q_zero value).
   */
  Eigen::VectorXd q_rl;

   /**
   * @brief Default/reference joint positions loaded from config (q0).
   *
   * Corresponds to the robot's nominal standing pose used during RL training.
   * A zero policy output produces q_rl = q_zero.
   * Units: radians. Size: nbActuatedJoints.
   */
  Eigen::VectorXd q_zero;                      // Reference joint positions

  /**
   * @brief Raw output from the RL policy (before scaling).
   *
   * Directly returned by rlPolicy->predict(). Typically in [-1, 1] if the
   * policy uses a tanh output activation. Size: action space dimension.
   */
  Eigen::VectorXd currentAction;

  /**
   * @brief Scaled policy output: currentActionScaled = currentAction * actionScale.
   *
   * Has the same size as q_rl (nbActuatedJoints). Joints not controlled by
   * the policy have value 0.
   */
  Eigen::VectorXd currentActionScaled;

  /**
   * @brief Per-joint action scaling factors loaded from config.
   *
   * Typically computed as effort_limit / Kp to keep the policy output
   * in a physically meaningful range. Size: nbActuatedJoints.
   */
  Eigen::VectorXd actionScale;

  // =========================================================================
  // Observation
  // =========================================================================

  /** @brief Full observation vector fed to the policy at each inference step. */
  Eigen::VectorXd currentObservation;

  // Observation history buffers (uncomment and adapt to your policy)
  // History buffers store the last HISTORY_SIZE policy steps of each observation
  // term. Index 0 = most recent, index HISTORY_SIZE-1 = oldest.
  // Stacked oldest-first into the observation vector to match mjlab ordering.
  //
  // static constexpr int HISTORY_SIZE = 5;
  // std::array<Eigen::Vector3d, HISTORY_SIZE> linVel;         ///< Base linear velocity in body frame
  // std::array<Eigen::Vector3d, HISTORY_SIZE> angVel;         ///< Base angular velocity in body frame
  // std::array<Eigen::Vector3d, HISTORY_SIZE> projectedGravity; ///< Gravity unit vector in body frame
  // std::array<Eigen::VectorXd, HISTORY_SIZE> jointPos;       ///< q - q_zero, size = action dim
  // std::array<Eigen::VectorXd, HISTORY_SIZE> jointVel;       ///< Joint velocities, size = action dim
  // std::array<Eigen::VectorXd, HISTORY_SIZE> jointAction;    ///< Raw policy output (currentAction)
  // std::array<Eigen::Vector3d, HISTORY_SIZE> velCmd;         ///< Velocity command [vx, vy, yaw_rate]

  // =========================================================================
  // Policy / timing
  // =========================================================================

  /**
   * @brief Policy inference period in seconds.
   *
   * The policy runs once every policyStepSize seconds. Between inference steps,
   * q_rl is held constant while the PD torque is recomputed at every controller
   * timestep using fresh joint state (equivalent to a real onboard PD loop).
   */
  double policyStepSize;

  /**
   * @brief Ordered joint names that correspond to the policy's action vector.
   *
   * Loaded from config ref_joint_order. May be a subset of jointNames if the
   * policy does not control all joints (e.g. fingers excluded).
   */
  std::vector<std::string> refJointOrderRLAction;

  /**
   * @brief Maps action vector index → mc_rtc joint index (jointNames).
   *
   * actionToDofMap[j] = i means policy output j controls jointNames[i].
   * Size: action space dimension.
   */
  std::vector<int> actionToDofMap;

  /**
   * @brief Maps mc_rtc joint index → RL framework joint index.
   *
   * mcRtcToRLFrameworkJointMap[i] = j means jointNames[i] corresponds to
   * position j in the RL observation/action joint ordering (from q0 config keys).
   * Size: nbActuatedJoints.
   */
  std::vector<int> mcRtcToRLFrameworkJointMap;

  // =========================================================================
  // Policy management
  // =========================================================================

  /** @brief Index of the currently active policy (indexes into policy_path list). */
  size_t currentPolicyIndex = 0;

  /** @brief ONNX runtime wrapper for running policy inference. */
  std::unique_ptr<RLPolicyInterface> rlPolicy;

  /** @brief Utility helpers for FSM state lifecycle and observation building. */
  utils utilsClass;

private:
  mc_rtc::Configuration config_;

  /** @brief Register data entries visible in mc_log_ui. */
  void addLog();
  /** @brief Register GUI elements visible in RViz and mc_mujoco. */
  void addGui();

  /** @brief Load robot parameters (gains, action scale, q0) from config. */
  void initializeRobot();

  /** @brief Load and validate the ONNX policy, build joint mappings. */
  void configRL();

  /** @brief Instantiate rlPolicy and initialize observation buffers. */
  void initializeRLPolicy();

   /**
   * @brief Apply RL torques directly, bypassing the QP (useQP=false mode).
   *
   * Computes τ = Kp*(q_rl - q) - Kd*q̇ and writes it to robot().mbc().jointTorque.
   * @return true if bypass was applied, false if QP should run instead.
   */
  bool byPassQPControl();
  bool useQP_ = true; ///< Route torques through CBF-QP (true) or apply directly (false)

  /** @brief Log warnings when joint position/velocity/torque limits are exceeded. */
  void computeLimits();
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

  // --- PD gains ---
  double pdGainsRatio_ = 1.0;
  Eigen::VectorXd kp_;      ///< Active proportional gains = pdGainsRatio_ * kpBase_
  Eigen::VectorXd kd_;      ///< Active derivative gains   = sqrt(pdGainsRatio_) * kdBase_
  Eigen::VectorXd kpBase_;  ///< Nominal Kp from config
  Eigen::VectorXd kdBase_;  ///< Nominal Kd from config

  // --- Policy ---
  std::vector<std::string> policyPaths_; ///< Paths to ONNX files (one per policy index)
  std::map<std::string, double> q0_map_; ///< Raw q0 map used to build mcRtcToRLFrameworkJointMap
};
