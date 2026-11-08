# Spot Learned Locomotion → ROS 2 / Gazebo → SLAM and Nav2 (simulation)

End-to-end **simulation** project: a quadruped locomotion policy trained with PPO in MuJoCo/MjLab is deployed into ROS 2 Jazzy + Gazebo Harmonic through `ros2_control`, then used as the motion layer for LiDAR SLAM, AMCL localization and Nav2 navigation in a warehouse world.

This repository is the deployment and autonomy side. Robot modeling, actuator investigation and PPO training live in
[`spot_locomotion_v2`](https://github.com/AlaaRadhwan/spot_locomotion_v2).

> **Scope.** Everything here runs in simulation. Nothing has been deployed to physical Spot hardware.
> The robot model is a custom rigid-body Spot model (31.6 kg), not Boston Dynamics' controller.

<!-- DEMO MEDIA PLACEHOLDERS (not recorded yet; add real links only once the files exist)
  - Gazebo: robust policy walking in the warehouse
  - RViz: SLAM map building
  - RViz: Nav2 goal navigation with AMCL
-->

## Status

| Component | State | Evidence |
|---|---|---|
| Policy deployment (robust `model_600`) at 50 Hz | Working | Documented in [`Findings/`](Findings); no quantitative benchmark yet for `model_600` |
| MuJoCo ↔ Gazebo velocity-tracking comparison | Measured for **`model_550`** only | [`evaluation_runs/`](evaluation_runs), MuJoCo side in the training repo |
| Gazebo odometry quaternion-spike fix | Root cause found, fix verified by one long run | [report](Findings/Spot_V3_Gazebo_Odometry_Quaternion_Spike_Root_Cause.md) |
| SLAM Toolbox mapping, AMCL, Nav2 (MPPI) in warehouse | Working (qualitative) | [report](Findings/Spot_V3_ROS2_Gazebo_SLAM_Nav2_Warehouse_Update_2026-09-27.md); no success-rate metrics collected |
| Dynamic obstacles, multi-goal missions, `twist_mux` | Not implemented | Listed as next steps in the report |

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

The policy node is clocked by odometry messages (`/spot/odom`), so one policy step is produced per odometry message.

### Packages (`src/`)

| Package | Role |
|---|---|
| `spot_description` | URDF/Xacro, meshes, `ros2_control` joint interfaces, 2D LiDAR |
| `spot_gazebo` | Worlds (`flat`, `warehouse`), controller config, `spawn_spot.launch.py` with `ros_gz_bridge` (clock, odometry, scan, pose→TF) |
| `spot_policy` | Observation builder, policy inference, joint-reference publisher, dry-run and observation-monitor tools |
| `spot_eval` | Scripted command-space evaluation; writes `samples.csv` and `summary.json` ([package README](src/spot_eval/README.md)) |
| `spot_teleop` | Keyboard and PS5 teleoperation to `/cmd_vel` |
| `spot_navigation` | SLAM Toolbox, AMCL/map server, Nav2 (MPPI) configs, maps, top-level launch files |

## Deployed policy

The deployed policy is the robust **`model_600`** from MjLab task `Mjlab-Velocity-Flat-Spot-V3-Robust`
(run `2026-09-10_15-11-04_spot_v3_robust_v1`). Training used Kp=300, Kd=40 and torque limits ±45 / ±115 N·m.

- Files: [`src/spot_policy/models/policy_jit.pt`](src/spot_policy/models/policy_jit.pt) (loaded at runtime via `torch.jit.load`) and `policy.onnx` (exported alongside, not used by the ROS node).
- Identity: both files are byte-identical to the exports in the training repo's `exported/spot_v3_robust_model_600/`, and [`config/policy_metadata.json`](src/spot_policy/config/policy_metadata.json) describes that policy.
- Loading the TorchScript model with a zero input returns a `(1, 12)` output, consistent with the metadata.

**Observation (48)**, in order: base linear velocity (3), base angular velocity (3), projected gravity (3), joint position (12), joint velocity (12), previous action (12), command `vx, vy, wz` (3).
**Action (12)**: position offsets, joint order `fl, fr, hl, hr` × `hx, hy, kn`; scales 0.3 / 0.4 / 0.5 (hx / hy / kn); default pose 0.0 / 1.04 / −1.8 rad.

| Rate | MuJoCo (training) | Gazebo (deployment) |
|---|---|---|
| Policy | 50 Hz | 50 Hz |
| Physics | 500 Hz | 2000 Hz |
| Joint controller | 500 Hz (MuJoCo built-in position actuator, implicit) | 2000 Hz (external effort PID) |

## Engineering investigations

Full write-ups are in [`Findings/`](Findings); only the conclusions are summarised here.

**1. Controller-rate instability (500 Hz → 2000 Hz).**
With Gazebo physics and `ros2_control` both at 500 Hz, the approved Kp=300/Kd=40 controller oscillated and saturated the torque limits even when holding a fixed nominal pose, so the policy was not the cause. Raising physics and controller to 2000 Hz, without changing gains or limits, gave stable standing. The same nominal gains behave differently because MuJoCo integrates its actuator implicitly while Gazebo applies an explicit effort command.
Report: [Gazebo sim-to-sim deployment update](Findings/Spot_Locomotion_V3_Gazebo_Sim2Sim_Deployment_Update.md).

**2. Odometry quaternion sign-flip.**
Intermittent collapses were traced, using a ROS bag, to a Gazebo odometry bug: a quaternion sign change was interpreted as a near-full rotation, producing a spike in published angular velocity and corrupting the policy's observation. Duplicate publishers, controller state, torque limits and the policy were ruled out. Upgrading to Gazebo Sim 8.15.0 (which contains the upstream fix) removed it: a reproducer that previously failed within seconds ran about three minutes without collapse. This is a single-scenario confirmation, not a statistical one.
Report: [odometry quaternion spike](Findings/Spot_V3_Gazebo_Odometry_Quaternion_Spike_Root_Cause.md).

**3. Autonomy stack.**
2D LiDAR (720 samples, 10 Hz), `/spot/odom` and a Gazebo-pose→TF bridge feed SLAM Toolbox for mapping and AMCL on the saved map. Nav2 uses the MPPI controller in omnidirectional mode (20 Hz, 40 steps × 0.05 s, 1000 samples) with velocity limits matched to the policy's command range (vx ±0.65, vy ±0.40, wz ±0.60) and path-angle / prefer-forward critics tuned to reduce excessive diagonal strafing. Footprint is 1.1 × 0.6 m.
Report: [SLAM/Nav2 warehouse update](Findings/Spot_V3_ROS2_Gazebo_SLAM_Nav2_Warehouse_Update_2026-09-27.md).

## Sim-to-sim results (baseline `model_550` — not the deployed `model_600`)

The only quantitative MuJoCo–Gazebo comparison was run on the earlier non-robust baseline `model_550` (task `Mjlab-Velocity-Flat-Spot-V3`, Kp=300, Kd=40), using the same scripted 59 s command sequence in both simulators (stand, ±0.5 m/s forward/backward, ±0.3 m/s lateral, ±0.5 rad/s yaw, one combined command).
Data: [`evaluation_runs/spot_v3_gazebo_eval_20260910_101049/`](evaluation_runs/spot_v3_gazebo_eval_20260910_101049) (a second Gazebo run, `…_101248`, agrees closely except yaw, where the yaw-left error was 0.300 rad/s) and, in the training repo, `benchmark_runs/mujoco/spot_v3_mujoco_eval_20260910_111210/`.

Metric: for each simulator separately, the mean of |measured base velocity − commanded velocity| over the samples of a phase after its first 1.0 s. It is a **tracking error against the command**, not a direct MuJoCo-vs-Gazebo difference, and it includes oscillation as well as steady bias. Each row reports the error on the commanded axis (Gazebo run `…_101049`; one run per simulator, no confidence intervals):

| Phase | Command | MuJoCo error | Gazebo error |
|---|---|---|---|
| Forward | vx = 0.5 m/s | 0.028 m/s | 0.017 m/s |
| Backward | vx = −0.5 m/s | 0.024 m/s | 0.016 m/s |
| Left | vy = 0.3 m/s | 0.052 m/s | 0.024 m/s |
| Yaw left | wz = 0.5 rad/s | 0.209 rad/s | 0.220 rad/s |
| Combined | 0.4 m/s, 0.2 m/s, 0.3 rad/s | 0.031 / 0.021 / 0.030 | 0.017 / 0.021 / 0.062 |

(Combined shows vx / vy / wz error. The summaries record no joint-torque saturation in these phases.)

**Reading the results.** Linear-velocity tracking is within about 0.02–0.05 m/s in both simulators. Yaw tracking is much worse: roughly 0.2 rad/s of error on a 0.5 rad/s command (about 40 % of the command) in both simulators, and 0.30 rad/s in the second Gazebo run. Yaw tracking is therefore a limitation of this policy, present in MuJoCo as well as Gazebo, and not a sim-to-sim transfer effect. The comparison supports "similar tracking behaviour in the two simulators", not equivalence of the dynamics.

**Not yet measured:** an equivalent benchmark for the deployed `model_600`. The `model_600` behaviour in Gazebo is documented qualitatively only.

## Setup

**Tested environment (per project reports):** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic with **Gazebo Sim ≥ 8.15.0**.

Required ROS packages (from the package manifests and launch files): `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`, `ros2_controllers` (`pid_controller`, `joint_state_broadcaster`), `xacro`, `robot_state_publisher`, `slam_toolbox`, `nav2_bringup`, `rviz2`; `teleop_twist_keyboard` (keyboard teleop) or `joy` (PS5 teleop).

Python: the policy node needs PyTorch. The project used a workspace virtual environment created with system site packages, so that `rclpy` and `torch` are both importable (the reports used Python 3.12 and CPU PyTorch 2.13):

```bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install torch --index-url https://download.pytorch.org/whl/cpu   # install step not re-verified
```

Build (the venv must be active so generated entry points use its interpreter):

```bash
cd spot_ros2_ws
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
colcon build --symlink-install
source install/setup.bash
```

## Running

In every terminal, source the environments in this order first (ROS, then the virtual environment, then the workspace overlay):

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
```

Commands below are taken from the launch files and the project reports; none was re-run end-to-end for this documentation.

```bash
# Gazebo + controllers only (flat world; use world:=warehouse for the warehouse)
ros2 launch spot_gazebo spawn_spot.launch.py

# Policy node (separate terminal); then drive with teleop or /cmd_vel
ros2 run spot_policy policy_controller

# Mapping in the warehouse (teleop: keyboard | ps5 | none)
ros2 launch spot_navigation warehouse_mapping.launch.py teleop:=keyboard

# Autonomous navigation on the saved map (Gazebo + policy + AMCL + Nav2 + RViz)
ros2 launch spot_navigation warehouse_navigation.launch.py

# Scripted evaluation (stop teleop first; Gazebo and policy running)
ros2 run spot_eval gazebo_eval --ros-args -p use_sim_time:=true
```

The warehouse world pulls models from Gazebo Fuel (OpenRobotics `Warehouse`, MovAi `shelf_big`), so first launch needs network access.

## What is original and what is third-party

| Original to this project | Third-party / existing |
|---|---|
| Spot V3 model integration in Gazebo, `ros2_control` PID setup and the 500→2000 Hz investigation | Spot meshes (BSD-3-Clause, Clearpath, via MuJoCo Menagerie; see `THIRD_PARTY.md`) |
| `spot_policy` (observation builder, inference node), `spot_eval`, `spot_teleop`, launch files and the `ros_gz_bridge` wiring | ROS 2 Jazzy, `ros_gz`, `gz_ros2_control`, `pid_controller`, Gazebo Sim |
| Odometry spike diagnosis (ROS bag analysis) | The fix itself: upstream Gazebo Sim 8.15.0 |
| Nav2/MPPI/AMCL/SLAM parameter tuning and integration with the learned policy | SLAM Toolbox, Nav2 (MPPI controller, AMCL, planners), `teleop_twist_keyboard`; Nav2's default parameter structure |
| Policy training and exports (training repo) | MuJoCo, MjLab, RSL-RL, PPO algorithm; Fuel warehouse and shelf models |

## Related repository and documents

- Training, modeling and actuator investigation: [`spot_locomotion_v2`](https://github.com/AlaaRadhwan/spot_locomotion_v2)
- Reports: [`Findings/`](Findings)

## Third-party material

See [`THIRD_PARTY.md`](THIRD_PARTY.md) for sources, licences and what remains unverified. Summary: Spot meshes are BSD-3-Clause (Clearpath, via MuJoCo Menagerie); Gazebo Fuel warehouse models are runtime downloads with unverified licences; ROS 2, Gazebo, Nav2, SLAM Toolbox, MjLab, MuJoCo and RSL-RL are dependencies.

## License

Original code: Apache-2.0, Copyright 2026 Alaa Radhwan (see [`LICENSE`](LICENSE)). Third-party assets and dependencies keep their own licences; see [`THIRD_PARTY.md`](THIRD_PARTY.md).
