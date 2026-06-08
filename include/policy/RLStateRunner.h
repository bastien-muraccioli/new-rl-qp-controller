#pragma once
#include <Eigen/Core>
#include <mc_control/fsm/State.h>
#include <string>

/**
 * @brief FSM state lifecycle helpers and observation builder for RL controllers.
 *
 * utils provides three FSM hooks (start / run / teardown) and the observation
 * assembly function getCurrentObservation(). One utils instance lives inside
 * NewRLQPController and is called from each FSM RL state.
 *
 * ## Usage in an FSM state
 *
 * @code
 * // In MyState.cpp
 * bool MyState::run(mc_control::fsm::Controller & ctl)
 * {
 *   auto & c = static_cast<NewRLQPController&>(ctl);
 *   c.utilsClass.run_rl_state(ctl);
 *   // ... handle transitions ...
 * }
 * @endcode
 *
 * ## Adding a new policy
 *
 * 1. Add a new case to getCurrentObservation() that fills the observation vector
 *    to exactly match the training observation order and transformations.
 * 2. Uncomment and populate the relevant history buffers in NewRLQPController.h.
 * 3. Add the corresponding case index to run_rl_state() if needed.
 *
 * ## Observation construction rules
 *
 * - **History order**: oldest timestep first (index HISTORY_SIZE-1 → index 0).
 * - **Joint positions**: always relative to default pose: q_obs = q - q_zero.
 * - **Contact forces**: apply log1p compression before insertion:
 *     f_obs = sign(f) * log(1 + |f|)
 * - **Gravity vector**: rotate unit vector [0, 0, -1] from world to body frame:
 *     g_b = R_world_to_body * [0, 0, -1]
 * - **Velocities**: express in body frame using bodyVelB.
 * - **Last action**: use the raw policy output (before action_scale multiply).
 */
struct RLStateRunner
{  
  /**
   * @brief Called when an RL FSM state starts.
   *
   * Resets syncTime_ to trigger an immediate policy inference on the first run(),
   * adds policy info to the GUI, and calls initializeRLObservation() to populate
   * history buffers with the current robot state.
   */
  void start(mc_control::fsm::Controller & ctl_, const std::string & state_name);

  /**
   * @brief Called every controller timestep while an RL FSM state is active.
   *
   * Accumulates time and triggers policy inference every policyStepSize seconds.
   * On each inference:
   *   1. Build observation via getCurrentObservation().
   *   2. Run rlPolicy->predict(observation) → currentAction.
   *   3. Compute q_rl = currentAction * actionScale + q_zero.
   *
   * Between inference steps, q_rl is held constant. The PD torque is recomputed
   * at every controller timestep using fresh joint state (equivalent to a real
   * onboard PD loop running faster than the policy).
    */
  void run(mc_control::fsm::Controller & ctl_);

  /**
   * @brief Called when an RL FSM state ends.
   *
   * Removes the policy GUI panel added by start_rl_state().
    */
  void teardown(mc_control::fsm::Controller & ctl_);

  private:
   std::string stateName_;
};
