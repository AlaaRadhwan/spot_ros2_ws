# Spot Quadruped: Learned Locomotion with ROS 2, SLAM and Nav2

A Spot-like quadruped that walks with a reinforcement-learning policy and navigates a warehouse on its own, in simulation. A PPO locomotion policy trained in MuJoCo/MjLab is deployed into ROS 2 Jazzy and Gazebo through `ros2_control`, and serves as the motion layer for LiDAR SLAM, AMCL localization and Nav2.

This repository is the deployment and autonomy side. Robot modeling, actuator investigation and training are in the companion repository [`spot_locomotion_v2`](https://github.com/AlaaRadhwan/spot_locomotion_v2).

> Everything runs in simulation; nothing has been run on physical hardware. The robot is a custom rigid-body Spot model, not Boston Dynamics' controller.

## Demo

**Warehouse autonomous navigation — Gazebo + RViz**

https://github.com/user-attachments/assets/4b25e34d-f044-4a09-b003-45ac95dbffc8

Robust PPO locomotion policy (`model_600`) integrated with ROS 2, AMCL and Nav2. The robot successfully navigates to a goal at map coordinates (6.0, 3.0) m.

**Learned locomotion in Gazebo**

https://github.com/user-attachments/assets/244a5445-84db-4460-a973-04d8abaa3dce

Scripted velocity commands: backward walking, arcs and lateral strafing. A continuous 56 s excerpt trimmed from a longer recording; playback speed unchanged.

<sub>Videos play at normal speed. Gazebo ran at about 0.55–0.6× real time on the recording machine, so the robot moves slower than it would in real time.</sub>

## What I built

- **Locomotion training** ([training repo](https://github.com/AlaaRadhwan/spot_locomotion_v2)): actuator characterization, MuJoCo model, and a PPO velocity-tracking policy (robust `model_600`).
- **Simulator integration**: Spot model in Gazebo with a `ros2_control` effort PID loop that reproduces the training actuator (Kp 300, Kd 40, torque-limited).
- **ROS 2 deployment**: `spot_policy` node builds the 48-D observation, runs the TorchScript policy at 50 Hz and publishes joint references.
- **Debugging**: found why the controller was unstable at 500 Hz (fixed at 2000 Hz) and traced intermittent falls to a Gazebo odometry bug. See [findings](#engineering-findings).
- **Autonomous navigation**: 2D LiDAR, SLAM Toolbox mapping, AMCL and Nav2 (MPPI, omnidirectional) tuned around the policy's command range.

## Architecture

```text
 Nav2 (MPPI, Omni)  /  keyboard or PS5 teleop  /  spot_eval
                  │  geometry_msgs/Twist  on /cmd_vel
                  ▼
 spot_policy  (policy_controller, 50 Hz, TorchScript policy_jit.pt)
   48-D observation ─► MLP ─► 12 actions
   q_ref = q_default + action_scale * action        (1 s blend-in at start-up)
                  │  control_msgs/MultiDOFCommand  on /spot_joint_controller/reference
                  ▼
 ros2_control  PidController, effort interface, 2000 Hz
   tau = Kp (q_ref − q) − Kd q̇     Kp=300, Kd=40, Ki=0
   clamp: hips ±45 N·m, knees ±115 N·m
                  ▼
 Gazebo Harmonic (DART, 0.5 ms step = 2000 Hz)  ── /spot/odom, /joint_states, /scan, /tf ──┐
                  ▲                                                                       │
                  └──────── odometry (50 Hz) drives the policy; LiDAR/odom/TF feed SLAM, AMCL, Nav2 ◄┘
```

The policy node is clocked by odometry messages, so one policy step is produced per `/spot/odom` message. The deployed policy is `model_600` of MjLab task `Mjlab-Velocity-Flat-Spot-V3-Robust`; [`src/spot_policy/models/`](src/spot_policy/models) holds the same files as the training repo's export, described by [`policy_metadata.json`](src/spot_policy/config/policy_metadata.json).

| Package | Role |
|---|---|
| `spot_description` | URDF/Xacro, meshes, `ros2_control` joint interfaces, 2D LiDAR |
| `spot_gazebo` | Worlds (`flat`, `warehouse`), controller config, spawn launch with `ros_gz_bridge` |
| `spot_policy` | Observation builder, policy inference, joint-reference publisher, monitoring tools |
| `spot_eval` | Scripted command-space evaluation ([package README](src/spot_eval/README.md)) |
| `spot_teleop` | Keyboard and PS5 teleoperation |
| `spot_navigation` | SLAM Toolbox, AMCL, Nav2 (MPPI) configs, maps, launch files |

## Engineering findings

**Controller rate (500 → 2000 Hz).** With Gazebo physics and the controller at 500 Hz, the same Kp/Kd that is stable in MuJoCo oscillated and hit the torque limits while merely standing. MuJoCo's actuator is implicit while Gazebo applies an explicit effort command. Running physics and controller at 2000 Hz, with unchanged gains, fixed it. [Report](Findings/Spot_Locomotion_V3_Gazebo_Sim2Sim_Deployment_Update.md)

**Odometry quaternion flip.** Intermittent collapses were traced with a ROS bag to a Gazebo odometry bug: a quaternion sign change produced a spike in angular velocity that corrupted the policy's observation. Gazebo Sim 8.15.0, which includes the upstream fix, resolved it; a scenario that failed within seconds ran about three minutes cleanly (a single-scenario check). [Report](Findings/Spot_V3_Gazebo_Odometry_Quaternion_Spike_Root_Cause.md)

**Navigation stack.** Nav2's MPPI controller runs in omnidirectional mode with velocity limits matched to the policy's command range (vx ±0.65, vy ±0.40, wz ±0.60), and critics tuned to reduce excessive diagonal strafing. [Report](Findings/Spot_V3_ROS2_Gazebo_SLAM_Nav2_Warehouse_Update_2026-09-27.md)

## Results and status

| Component | State |
|---|---|
| Robust `model_600` deployed in Gazebo at 50 Hz | Working; evaluated qualitatively (see the demo) |
| SLAM Toolbox mapping, AMCL and Nav2 in the warehouse | Working; no success-rate statistics collected |
| MuJoCo ↔ Gazebo velocity tracking | Measured for the earlier baseline **`model_550`** only |
| Dynamic obstacles, multi-goal missions, `twist_mux` | Not implemented |

On `model_550`, linear-velocity tracking error was about 0.02–0.05 m/s in both simulators, and yaw tracking was weaker (about 0.2 rad/s error on a 0.5 rad/s command in both MuJoCo and Gazebo), so yaw is a limitation of the policy, not of the transfer. Method and full table: [`docs/sim2sim_benchmark_model_550.md`](docs/sim2sim_benchmark_model_550.md). There is no equivalent quantitative benchmark for `model_600` yet.

## Quick start

Requires Ubuntu 24.04, ROS 2 Jazzy, Gazebo Sim ≥ 8.15.0 and PyTorch. Setup, build and all launch commands are in [`docs/setup_and_running.md`](docs/setup_and_running.md). After building:

```bash
source /opt/ros/jazzy/setup.bash && source .venv/bin/activate && source install/setup.bash
ros2 launch spot_navigation warehouse_navigation.launch.py   # Gazebo + policy + AMCL + Nav2 + RViz
```

The commands come from the launch files and project reports. The warehouse world downloads models from Gazebo Fuel, so the first launch needs network access.

## Documentation

- Engineering reports: [`Findings/`](Findings)
- Training, modeling and actuator investigation: [`spot_locomotion_v2`](https://github.com/AlaaRadhwan/spot_locomotion_v2) and its [sim-to-sim benchmark findings](https://github.com/AlaaRadhwan/spot_locomotion_v2/blob/feat/spot-v3-rl-env/FINDS/Spot_Locomotion_V3_Sim2Sim_Benchmark_Findings.md)

## Built on

ROS 2 Jazzy, `ros_gz`, `gz_ros2_control`, Gazebo Sim, SLAM Toolbox, Nav2, MuJoCo, MjLab and RSL-RL. The Spot meshes are BSD-3-Clause (Clearpath, via MuJoCo Menagerie), and the Gazebo warehouse models are downloaded from Fuel at runtime. Sources and licences are listed in [`THIRD_PARTY.md`](THIRD_PARTY.md).

## License

Original code: Apache-2.0, Copyright 2026 Alaa Radhwan ([`LICENSE`](LICENSE)). Third-party assets keep their own licences.
