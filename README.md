<p align="center">
  <a >
    <img src="ext/image/logo.png" alt="logo" width="300">
  </a>
</p>

---

# mc_rtc RL-QP controller template

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

Currently only ONNX format policies are supported.

This template has already been adapted to :
- [H1]()
- [HRP5P]()

If the target robot is already supported, please refer to the corresponding repository above. Otherwise, this template is intended as a starting point for integrating additional robots.

Quick start
--

1. Fork this repository and name it after your robot. Example : `robotname_rl_qp_controller`.

2. Renaming the controller from `NewRLQPController` to `RobotNameRLQPController`. In a shell (Git Bash on Windows, replace sed with gsed on macOS):

```bash
sed -i -e's/NewRLQPController/RobotNameRLQPController/g' `find . -not -path '*/.*' -type f`
git mv src/NewRLQPController.cpp src/RobotNameRLQPController.cpp
git mv src/NewRLQPController.h src/RobotNameRLQPController.h
git mv src/states/NewRLQPController_Initial.cpp src/states/RobotNameRLQPController_Initial.cpp
git mv src/states/NewRLQPController_Initial.h src/states/RobotNameRLQPController_Initial.h
git mv etc/NewRLQPController.in.yaml etc/RobotNameRLQPController.in.yaml
```

3. You can customize the project name in vcpkg.json as well, note that this must follow [vcpkg manifest rules](https://github.com/microsoft/vcpkg/blob/master/docs/users/manifests.md)

4. Build and install the project

5. Run using your [mc_rtc] interface of choice, and setting `Enabled` to `RobotNameRLQPController`

---

## Template layout

- `policies/minimalExample/`: smallest explicitly configured policy accepted by the current parser.
- `policies/fullExample/`: exhaustive reference for every currently accepted policy and observation fields.
- `policies/conventions.yaml`: placeholder joint groups, training orders, aliases and observation defaults.
- `etc/NewRLQPController.in.yaml`: controller, observer and constraint examples.
- `src/states/NewRLQPController_Initial.cpp`: policy-execution state.
- `src/observation/`: reusable observation implementations.
- `src/policy/`: policy loading, validation, ONNX inference and runtime state.

## Required robot adaptation

The source marks adaptation points with `TODO(robot)`. Search them before attempting to run the controller:

```bash
grep -rn "TODO(robot)"
```

## Deploy your own policy

Policies are stored in `policies`. Each subdirectory describes everything the controller needs to know to load. run and use the policy.
You here have access to 2 example directories. `minimalExample` with a basic, minimal configuration, and `fullExample` to have a better idea of the full potential of the default version of this controller.

1. Create a new subdirectory in `policies`
1. Add the exported model in ONNX format inside
2. set the action and controlled-joint groups, provide training-consistent `kp` and `kd`, and add any required action scaling, default pose, phase period or command parameters in `policy.yaml`
5. reproduce the exact observation order and history in `observations.yaml`.

Set `default_policy` in `etc/NewRLQPController.in.yaml`.

## Build

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build . -j
sudo cmake --install .
```

Enable your controller in mc_rtc after completing all required adaptation points.

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