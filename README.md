mc_rtc new RL-QP controller template
==

This project is a template for a new RL-QP controller project wihtin [mc_rtc].

The goal of this template is to help users deploy reinforcement learning (RL) policies within a Quadratic Programming (QP) framework augmented with Control Barrier Functions (CBFs). This combination is enforcing physical and safety constraints, including:

* Joint position limits
* Joint velocity limits
* Torque limits
* Self-collision avoidance

Further details are available in:

[*Safe Execution of RL Policies via Acceleration-based CBF-QP Constraint Enforcement for Real-World Robotic Deployments*](https://hal.science/hal-05362571)


It comes with:
- a CMake project that can build a controller in [mc_rtc], the project can be put within [mc_rtc] source-tree for easier updates
- clang-format files
- automated GitHub Actions builds on three major platforms

Requirements
--
- [mc_rtc](https://github.com/bastien-muraccioli/mc_rtc)
- [tvm](https://github.com/bastien-muraccioli/tvm)
- [mc_external_forces_observer](https://github.com/isri-aist/mc_external_forces_observer)

Quick start
--

1. Renaming the controller from `NewRLQPController` to `MyController`. In a shell (Git Bash on Windows, replace sed with gsed on macOS):

```bash
sed -i -e's/NewRLQPController/MyController/g' `find . -not -path '*/.*' -type f`
git mv src/NewRLQPController.cpp src/MyController.cpp
git mv src/NewRLQPController.h src/MyController.h
git mv src/states/NewRLQPController_Initial.cpp src/states/MyController_Initial.cpp
git mv src/states/NewRLQPController_Initial.h src/states/MyController_Initial.h
git mv etc/NewRLQPController.in.yaml etc/MyController.in.yaml
```

2. You can customize the project name in vcpkg.json as well, note that this must follow [vcpkg manifest rules](https://github.com/microsoft/vcpkg/blob/master/docs/users/manifests.md)

2. Build and install the project

3. Run using your [mc_rtc] interface of choice, and setting `Enabled` to `MyController`

Installation
--

The recommended way is via the
[mc_rtc superbuild](https://github.com/mc-rtc/mc-rtc-superbuild).
For a standalone build, ensure `mc_rtc` is installed, then:

```bash
cmake -B build && cmake --build build && cmake --install build
```

Tutorial: Deploying your own RL policy
--

### Step 1 — Export your policy to ONNX

Place the `.onnx` file somewhere accessible and set `policy_path` in the YAML
config.

### Step 2 — Fill in the YAML config

Edit `etc/MyController.in.yaml`. Each entry under `policies` corresponds to
one ONNX file (indexed by `default_policy_index`):

```yaml
policy_path: ["my_policy.onnx"]
default_policy_index: 0

policies:
  - use_QP: true
    policy_step_size: 0.02      # Policy runs at 50 Hz
    physics_step_size: 0.0025   # Controller runs at 400 Hz
    pd_gains_ratio: 1.0

    # Per-joint action scale: typically effort_limit / Kp
    action_scale:
      joint_a: 0.88
      joint_b: 0.57
      # ...

    # PD gains matching those used during RL training
    kp:
      joint_a: 112.6
      joint_b: 417.2
      # ...
    kd:
      joint_a: 17.9
      joint_b: 66.4
      # ...

    # Default/reference pose used during training (radians)
    # A zero policy output commands this pose
    q0:
      joint_a: 0.0
      joint_b: 0.872   # 50°
      # ...

    # Order of joints in the policy's action vector
    ref_joint_order:
      - "joint_a"
      - "joint_b"
      # ...
```

### Step 3 — Implement the observation

Open `src/utils.cpp` and fill in `getCurrentObservation()` for your policy
index. The observation must exactly match the training environment.

Then implement `initializeRLObservation()` in `NewRLQPController.cpp` to
populate index 0 of each buffer from the current robot state.

Example for a policy with history_length=5 can be found in the commented parts of the code.

### Step 4 — Tune CBF parameters

The CBF gains control how aggressively the QP enforces safety constraints.
Start with the defaults and increase if limits are breached.
Enable limit monitoring from the GUI ("Toggle print joint limits") to see
which constraints are being approached.

### Step 5 — Test without QP first

Set `use_QP: false` in the config (or toggle from the GUI) to apply torques
directly without the QP safety layer. This is useful to:
- Verify the observation and action pipeline is correct
- Compare behavior with and without CBF corrections

Once the policy runs correctly without QP, enable it for safe deployment.

---

## Control flow summary

```
Every controller timestep (physics_step_size):
├── If syncTime >= policyStepSize:
│   ├── observation = getCurrentObservation()
│   ├── action      = rlPolicy->predict(observation)
│   ├── q_rl        = action * actionScale + q_zero
│   └── syncTime    = 0
│
├── τ = Kp*(q_rl - q) - Kd*q̇
│
├── useQP=true:  τ → TorqueJointTask → CBF-QP → robot
└── useQP=false: τ → robot (direct)
```

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/