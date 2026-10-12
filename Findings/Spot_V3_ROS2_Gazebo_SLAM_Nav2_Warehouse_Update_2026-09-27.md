# Spot V3 ROS 2 / Gazebo Autonomy Integration Update

**Date:** 2026-09-27  
**Project:** Spot V3 locomotion → ROS 2 / Gazebo → LiDAR → SLAM → localization → Nav2 → warehouse autonomy  
**Document scope:** Work completed after `Spot_V3_Gazebo_Odometry_Quaternion_Spike_Root_Cause.md`

---

## Executive summary

After resolving the Gazebo `OdometryPublisher` quaternion sign-flip bug and freezing the locomotion stack, the project advanced from a locomotion-only Gazebo deployment into a complete 2D autonomous-navigation stack.

The major additions since the previous report are:

- 2D LiDAR integration in the Spot model.
- ROS 2 bridging for LiDAR, odometry, simulation clock, and Gazebo pose / TF.
- Validation of the TF chain required by SLAM and Nav2.
- PS5 teleoperation integration and later keyboard teleoperation.
- SLAM Toolbox mapping.
- Saved-map localization with AMCL.
- Nav2 integration using the MPPI controller in omnidirectional mode.
- Full-speed command-envelope tuning for the learned locomotion policy.
- Import and integration of a larger Clearpath-style warehouse environment.
- Top-level mapping and navigation launch files.
- Selectable teleoperation mode for mapping.
- Diagnosis of duplicate ROS / Gazebo nodes caused by failed launch attempts.
- Successful warehouse mapping.
- Successful autonomous navigation in the warehouse.
- MPPI heading-preference tuning to reduce excessive diagonal strafing and produce more natural forward-biased quadruped motion.

The current system can now:

```text
Launch the warehouse
        ↓
Spawn Spot
        ↓
Run ros2_control
        ↓
Run the learned locomotion policy
        ↓
Publish / receive LiDAR, odometry and TF
        ↓
Map the environment with SLAM Toolbox
        ↓
Save a warehouse map
        ↓
Localize in the saved map with AMCL
        ↓
Plan and execute Nav2 goals
        ↓
Convert Nav2 velocity commands into learned Spot locomotion
```

The locomotion baseline remains frozen. No evidence discovered during the autonomy integration required reopening the validated RL policy, low-level controller, torque limits, physics rate, or robot rigid-body model.

---

# 1. Starting point from the previous report

The previous report ended after identifying and fixing the Gazebo quaternion sign-discontinuity bug that occasionally generated impossible angular velocity values.

The validated locomotion deployment baseline entering the present phase was:

```text
Policy checkpoint:       robust model_600
Policy rate:             ~50 Hz

Gazebo physics:          2000 Hz
ros2_control:            2000 Hz

Kp:                      300
Kd:                      40

Hip torque limits:       ±45 N·m
Knee torque limits:      ±115 N·m
```

The command interface remained:

```text
/cmd_vel
   ↓
Spot RL policy
   ↓
12 joint-position references
   ↓
spot_joint_controller
   ↓
PD / PID effort control
   ↓
Gazebo
```

The policy was already validated for approximately:

```text
vx:  ±0.65 m/s
vy:  ±0.40 m/s
wz:  ±0.60 rad/s
```

These values later became the basis of the Nav2 velocity envelope.

The key engineering decision was:

> The locomotion subsystem is a validated lower-level controller and should be treated as an actuator-level service for the navigation stack rather than repeatedly retuned while autonomy is being developed.

---

# 2. LiDAR integration

## 2.1 Purpose

The first major autonomy requirement was a range sensor that could support:

- SLAM,
- AMCL localization,
- local obstacle detection,
- global / local costmaps,
- collision monitoring.

A 2D planar LiDAR was added to the Spot URDF / Xacro.

## 2.2 LiDAR frame

A dedicated frame was created:

```text
lidar_link
```

The final mount was approximately:

```text
xyz = 0.10 0 0.10
```

relative to the body mounting frame.

The resulting robot-side TF hierarchy is:

```text
base_link
   ↓
body
   ↓
lidar_link
```

## 2.3 Sensor configuration

The Gazebo sensor is a GPU LiDAR configured approximately as:

```text
horizontal field of view:  360°
horizontal samples:         720
update rate:                10 Hz

minimum range:              0.10 m
maximum range:              12.0 m
range resolution:           0.01 m
```

ROS output:

```text
/scan
sensor_msgs/msg/LaserScan
```

The scan was verified at approximately:

```text
~9.985 Hz
```

with:

```text
angle_min ≈ -π
angle_max ≈ +π
angle_increment ≈ 0.00873878 rad
```

The scan frame was validated as:

```text
lidar_link
```

## 2.4 Gazebo sensor plugin

The world includes the Gazebo sensors system with the rendering engine configured for:

```text
ogre2
```

This was required for the GPU LiDAR.

## 2.5 ROS-Gazebo bridge

The LiDAR is bridged through `ros_gz_bridge`:

```text
/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

The final mapping and navigation stack consumes `/scan` directly.

## 2.6 Validation

The sensor was considered valid after confirming:

```text
/scan publishes continuously
        +
frame_id = lidar_link
        +
TF exists from robot base to lidar_link
        +
RViz displays the environment correctly
```

This completed the sensor side of the autonomy pipeline.

---

# 3. Gazebo pose / TF integration

## 3.1 Why an additional TF source was needed

`robot_state_publisher` provides the internal robot kinematic tree:

```text
base_link
 → body
 → hips
 → upper legs
 → lower legs
 → lidar_link
```

However, SLAM and navigation also need the robot's motion relative to an odometry frame:

```text
odom → base_link
```

Gazebo already publishes model pose information, so the existing simulation state was reused rather than introducing a second estimator at this stage.

## 3.2 Pose bridge

Gazebo model pose is bridged from:

```text
/model/spot/pose
```

to ROS:

```text
/tf
```

using:

```text
/model/spot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
```

with the topic remapped to:

```text
/tf
```

The bridge node is:

```text
spot_odom_tf_bridge
```

## 3.3 Odometry bridge

Gazebo odometry is bridged from:

```text
/model/spot/odometry
```

to:

```text
/spot/odom
```

The bridge node is:

```text
spot_odometry_bridge
```

The policy also consumes this odometry.

## 3.4 Final TF chain

The required transform chain was successfully validated as:

```text
odom
  ↓
base_link
  ↓
body
  ↓
lidar_link
```

Once SLAM or localization is active, the complete navigation tree becomes:

```text
map
  ↓
odom
  ↓
base_link
  ↓
body
  ↓
lidar_link
```

where:

```text
map → odom
```

is generated by either:

- SLAM Toolbox during mapping, or
- AMCL during saved-map localization.

---

# 4. PS5 teleoperation

## 4.1 Existing custom teleop node

A custom package:

```text
spot_teleop
```

contains:

```text
ps5_teleop.py
```

which converts joystick input into:

```text
/cmd_vel
```

The useful policy envelope was mapped to approximately:

```text
vx_max = 0.65 m/s
vy_max = 0.40 m/s
wz_max = 0.60 rad/s
```

## 4.2 Initial problem

The teleop node initially appeared to run correctly but Spot did not move.

The issue was not the locomotion policy.

The real problem was that no node was publishing:

```text
/joy
```

The custom teleop node expected joystick messages, but `joy_node` had not been started.

## 4.3 Fix

A PS5 launch file was introduced to start both:

```text
joy/joy_node
spot_teleop/ps5_teleop
```

This allowed the controller to work as a complete subsystem rather than requiring the user to remember a separate `joy_node` command.

Once this was done, PS5 teleoperation was validated.

---

# 5. First SLAM Toolbox mapping

## 5.1 Mapping configuration

SLAM Toolbox was configured for:

```text
mode: mapping

map_frame:  map
odom_frame: odom
base_frame: base_link
scan_topic: /scan
```

The configuration was based on the standard asynchronous SLAM Toolbox setup and adapted to Spot.

Important simulation settings included:

```text
use_sim_time: true
resolution:   0.05 m
maximum laser range: approximately 12 m
```

## 5.2 First mapping test

Before moving to the warehouse world, mapping was tested in the simpler original Gazebo environment.

The robot was manually driven while SLAM Toolbox consumed:

```text
/scan
TF: odom → base_link → lidar_link
```

and produced:

```text
/map
map → odom
```

The resulting map rendered correctly in RViz.

This validated the complete chain:

```text
Gazebo robot motion
        ↓
odometry + TF
        ↓
LiDAR
        ↓
SLAM Toolbox
        ↓
occupancy grid
```

---

# 6. Saved-map localization with AMCL

After SLAM was validated, the next step was to separate:

```text
mapping
```

from:

```text
localization
```

## 6.1 Localization architecture

Saved-map localization uses:

```text
map_server
        +
AMCL
```

AMCL consumes:

```text
saved occupancy grid
/scan
odom → base_link
```

and estimates:

```text
map → odom
```

## 6.2 Spot-specific AMCL assumptions

The intended Spot localization parameters are:

```text
global_frame_id: map
odom_frame_id:   odom
base_frame_id:   base_link
scan_topic:      /scan

robot_model_type:
nav2_amcl::OmniMotionModel
```

The omnidirectional motion model is appropriate because the learned Spot controller can command nonzero lateral velocity.

A separate configuration file:

```text
spot_localization.yaml
```

is used for localization.

This separation is important because the generic Nav2 parameter file also contains inherited AMCL template sections that should not accidentally override the dedicated Spot localization configuration.

## 6.3 RViz map QoS issue

One apparent localization problem was actually an RViz QoS mismatch.

`map_server` published `/map` with:

```text
Reliability: RELIABLE
Durability:  TRANSIENT_LOCAL
```

RViz was initially using:

```text
VOLATILE
```

for the Map display.

Result:

```text
No map received
```

even though `map_server` itself was healthy.

Changing the RViz Map display to:

```text
Reliability: Reliable
Durability:  Transient Local
```

fixed the display immediately.

## 6.4 Localization validation

AMCL was considered validated after manually moving Spot and observing that:

- the laser scan remained aligned with the saved occupancy map,
- the estimated robot pose remained consistent,
- `map → odom` updated correctly,
- `odom → base_link` remained the smooth local-motion transform.

This demonstrated that the complete saved-map localization stack was functioning correctly.

---

# 7. Nav2 integration

## 7.1 Navigation architecture

Nav2 was then added above the learned locomotion system.

The resulting hierarchy became:

```text
Nav2
  ↓
velocity command
  ↓
Spot RL locomotion policy
  ↓
joint-position references
  ↓
ros2_control
  ↓
Gazebo
```

The navigation stack does not command joints directly.

It only produces body-level velocity commands.

This cleanly separates:

```text
high-level autonomy
```

from:

```text
learned locomotion
```

## 7.2 Core navigation components

The integrated stack includes:

```text
map_server
AMCL

planner_server
controller_server
bt_navigator

local_costmap
global_costmap

velocity_smoother
collision_monitor

RViz
```

The controller plugin is:

```text
nav2_mppi_controller::MPPIController
```

with the motion model:

```text
Omni
```

This retains Spot's lateral mobility.

---

# 8. Robot footprint used by Nav2

A rectangular footprint was used rather than a simple circular radius:

```yaml
footprint: "[[0.55, 0.30],
             [0.55, -0.30],
             [-0.55, -0.30],
             [-0.55, 0.30]]"
```

This corresponds to an effective navigation footprint of approximately:

```text
length: 1.10 m
width:  0.60 m
```

This is used by both the local and global costmaps.

The important design decision was to represent the robot's actual occupied planform rather than using the small default TurtleBot-style radius.

---

# 9. Nav2 velocity envelope

The navigation stack was deliberately configured to use the range already demonstrated by the locomotion policy.

Final nominal limits:

```text
vx:
  min = -0.65 m/s
  max = +0.65 m/s

vy:
  min = -0.40 m/s
  max = +0.40 m/s

wz:
  min = -0.60 rad/s
  max = +0.60 rad/s
```

The velocity smoother uses the same output limits:

```yaml
max_velocity: [0.65, 0.40, 0.60]
min_velocity: [-0.65, -0.40, -0.60]
```

This avoids artificially constraining Nav2 far below the already-tested locomotion capability.

---

# 10. MPPI controller configuration

The main MPPI timing parameters evolved to approximately:

```yaml
time_steps:      40
model_dt:        0.05
batch_size:      1000
iteration_count: 1
```

This gives a prediction horizon of:

```text
40 × 0.05 = 2.0 s
```

The current useful velocity and acceleration region is approximately:

```yaml
vx_max:  0.65
vx_min: -0.65
vy_max:  0.40
wz_max:  0.60

ax_max:  10.0
ax_min: -10.0
ay_max:  10.0
ay_min: -10.0
az_max:  10.0
```

The acceleration values should be understood as MPPI command-space constraints, not a physical statement that the robot should actually accelerate at 10 m/s².

Their main purpose here is to avoid making the optimizer unnecessarily slow because of an overly restrictive first-step velocity envelope.

---

# 11. Initial Nav2 speed problem

## 11.1 Symptom

The first Nav2 configuration technically worked, but the robot moved extremely slowly.

Observed commands were around:

```text
~0.03 m/s
```

despite much larger allowed velocity limits.

## 11.2 Diagnosis

The command was traced through the stack:

```text
/cmd_vel_nav
/cmd_vel_smoothed
/cmd_vel
```

All contained similarly small commands.

Therefore the bottleneck was not:

- the velocity smoother,
- the collision monitor,
- the RL policy.

The low velocity originated in MPPI itself.

## 11.3 Cause

With:

```text
model_dt = 0.05 s
```

and an acceleration bound of roughly:

```text
0.8 m/s²
```

the first feasible velocity increment is only:

```text
0.8 × 0.05 = 0.04 m/s
```

This matched the observed behavior.

## 11.4 Resolution

The MPPI acceleration bounds were substantially increased.

During intermediate testing:

```text
3.0 m/s²
```

produced approximately:

```text
3.0 × 0.05 = 0.15 m/s
```

first-step behavior, again matching observations.

The final navigation configuration moved to:

```yaml
ax_max:  10.0
ax_min: -10.0
ay_max:  10.0
ay_min: -10.0
az_max:  10.0
```

The velocity smoother was also configured with:

```yaml
max_accel: [8.0, 8.0, 8.0]
max_decel: [-8.0, -8.0, -8.0]
```

After this change the robot used the locomotion envelope much more effectively.

---

# 12. Navigation command pipeline

The working navigation command chain is approximately:

```text
MPPI controller
     ↓
cmd_vel_nav
     ↓
velocity_smoother
     ↓
cmd_vel_smoothed
     ↓
collision_monitor
     ↓
/cmd_vel
     ↓
Spot RL policy
```

The policy remains the sole consumer responsible for converting body commands into joint references.

This is an important architectural boundary.

Nav2 is not allowed to know or care about:

```text
individual joints
PD gains
joint torque limits
action scales
policy observations
```

Likewise the locomotion policy does not know about:

```text
maps
global paths
costmaps
goals
behavior trees
```

---

# 13. Warehouse world integration

## 13.1 Motivation

The simple original test world was sufficient for validating:

- TF,
- LiDAR,
- SLAM,
- AMCL,
- initial Nav2 bringup.

It was not sufficiently interesting for the intended industrial-inspection project.

A more warehouse-like world was therefore introduced.

## 13.2 Source environment

The selected environment was based on the Clearpath simulator warehouse assets for Gazebo Harmonic / ROS 2 Jazzy.

The warehouse includes items such as:

```text
warehouse shell
large shelving
smaller shelving
barriers
chairs / tables
other industrial-style obstacles
```

## 13.3 Important integration decision

The entire Clearpath world physics configuration was **not** adopted.

Instead, the project preserved the already validated Spot simulation baseline and imported the warehouse assets into the existing world structure.

This preserves:

```text
Spot Gazebo physics dt = 0.0005 s
physics frequency      = 2000 Hz
existing ros2_control behavior
existing plugins
existing sensor configuration
```

while changing only the environment.

This minimizes regression risk.

## 13.4 Selectable Gazebo world

`spawn_spot.launch.py` was extended with:

```text
world:=flat
world:=warehouse
```

and spawn arguments:

```text
spawn_x
spawn_y
spawn_z
spawn_yaw
```

The launch resolves:

```text
flat       → flat_world.sdf
warehouse  → warehouse_world.sdf
```

The Gazebo internal world name remains:

```text
spot_world
```

because the spawn node targets:

```text
/world/spot_world/create
```

## 13.5 Gazebo unpaused startup

For normal mapping and navigation, Gazebo is launched with:

```text
-r
```

so the simulation begins running immediately.

The relevant launch argument is:

```python
"gz_args": ["-r -v 3 ", world_file]
```

There is no need for the normal workflow to start paused.

---

# 14. `spawn_spot.launch.py` role

The current Gazebo bringup launch is responsible for:

```text
selecting the world
launching Gazebo
publishing robot_description
spawning Spot
starting joint_state_broadcaster
starting spot_joint_controller
bridging /clock
bridging /spot/odom
bridging /scan
bridging Gazebo model pose to /tf
```

The resulting ROS bridge nodes are:

```text
clock_bridge
spot_odometry_bridge
spot_scan_bridge
spot_odom_tf_bridge
```

This launch file remains the simulation-side foundation for both mapping and navigation.

---

# 15. Top-level mapping bringup

To avoid manually launching a large number of terminals, a higher-level launch file was created:

```text
spot_navigation/launch/warehouse_mapping.launch.py
```

It starts:

```text
Gazebo + Spot
policy controller
selected teleoperation method
SLAM Toolbox
RViz
```

The mapping launch accepts:

```text
teleop:=keyboard
teleop:=ps5
teleop:=none
```

The default was moved toward:

```text
keyboard
```

because a PS5 controller is not always available.

---

# 16. Keyboard teleoperation

## 16.1 Reason for adding it

The original mapping workflow depended on the PS5 controller.

For portability, keyboard teleoperation was added so that mapping can still be performed with no external controller.

## 16.2 Selectable teleop launch

A wrapper launch was introduced under:

```text
spot_teleop/launch/teleop.launch.py
```

with a mode argument:

```text
mode:=keyboard
mode:=ps5
mode:=none
```

PS5 mode includes the existing PS5 launch.

Keyboard mode runs:

```text
teleop_twist_keyboard
```

and publishes velocity commands to the same lower-level interface.

## 16.3 Why keyboard teleop opens separately

`teleop_twist_keyboard` requires an interactive terminal.

Therefore it cannot be treated exactly like a normal background ROS node if direct keyboard input is expected.

The wrapper launch opens a terminal process for keyboard control.

## 16.4 Holonomic keyboard control

The keyboard teleop package supports:

```text
forward / backward
yaw left / right
lateral strafing
combined diagonal translation
```

which is suitable for the omnidirectional Spot locomotion policy.

---

# 17. Packaging issue with `spot_teleop`

The first attempt at including:

```text
teleop.launch.py
```

failed with:

```text
No such file or directory:
install/spot_teleop/share/spot_teleop/launch/teleop.launch.py
```

The source launch file existed, but `setup.py` had not been configured to install the launch directory.

The fix was to install launch files through `data_files`, for example:

```python
(
    os.path.join('share', package_name, 'launch'),
    glob('launch/*.launch.py'),
),
```

This allows both:

```text
ps5_teleop.launch.py
teleop.launch.py
```

to appear in the package install space.

This is an important ROS 2 Python-package lesson:

> A file existing under `src/<package>/launch/` does not automatically make it available through `ros2 launch`. It must also be installed by `setup.py`.

---

# 18. Launch-file typo failures

Two small launch configuration errors were encountered during bringup.

## 18.1 SLAM argument typo

Incorrect:

```python
"slam_aram_file"
```

Correct:

```python
"slam_params_file"
```

Without the correct argument name, SLAM Toolbox may launch without receiving the intended Spot-specific configuration.

## 18.2 TimerAction typo

Incorrect:

```python
TimerAction(
    perious=5.0,
    ...
)
```

Correct:

```python
TimerAction(
    period=5.0,
    ...
)
```

The incorrect spelling produced:

```text
TimerAction.__init__() missing 1 required keyword-only argument: 'period'
```

These were simple code errors rather than architecture problems.

---

# 19. Duplicate ROS nodes / multiple `/clock` publishers

## 19.1 Symptom

During one mapping attempt SLAM Toolbox repeatedly reported:

```text
Failed to compute odom pose
```

and RViz / TF reported:

```text
Detected jump back in time
Clearing TF buffer
```

The policy also showed a collapsed body height in one affected run.

## 19.2 Discovery

Inspection of:

```bash
ros2 topic info /clock -v
```

showed:

```text
Publisher count: 3
```

All were:

```text
clock_bridge
```

There were also multiple copies of:

```text
spot_policy_controller
robot_state_publisher
odometry bridges
TF bridges
```

## 19.3 Root cause

This was not a Gazebo clock implementation bug.

Earlier launch attempts had failed after some child processes had already started.

The failure was originally caused by the missing installed teleop launch file.

The visible Gazebo process was manually closed, but some ROS child processes remained alive.

When the parent launch process disappeared, Linux re-parented the surviving child processes to:

```text
systemd --user
```

Therefore closing all visible terminals did not necessarily stop those already-orphaned processes.

Subsequent launches added additional copies.

The resulting condition was:

```text
old launch attempt #1
   └─ clock_bridge + policy + bridges

old launch attempt #2
   └─ clock_bridge + policy + bridges

current launch
   └─ clock_bridge + policy + bridges
```

This produced:

```text
multiple simulation clocks
        ↓
interleaved timestamps
        ↓
ROS time jumps backward
        ↓
TF buffers clear
        ↓
SLAM cannot recover odom pose
```

Multiple policy controllers could also simultaneously publish joint references, creating an additional control hazard.

## 19.4 Important diagnostic

The correct sanity check is:

```bash
ros2 topic info /clock -v
```

For one running Gazebo simulation:

```text
Publisher count: 1
```

Before launching any simulation:

```text
Publisher count: 0
```

## 19.5 Lesson

When a top-level ROS launch fails halfway through startup, it is useful to verify that child nodes did not survive before immediately relaunching.

Particularly important nodes to check are:

```text
clock_bridge
spot_policy_controller
robot_state_publisher
spot_odometry_bridge
spot_odom_tf_bridge
spot_scan_bridge
```

A normal clean `Ctrl+C` shutdown should remove the entire launched stack.

---

# 20. Warehouse mapping

Once the warehouse world and launch stack were stable, SLAM Toolbox was used to map the new environment.

The mapping workflow became:

```bash
ros2 launch spot_navigation warehouse_mapping.launch.py   teleop:=keyboard
```

or, when available:

```bash
ros2 launch spot_navigation warehouse_mapping.launch.py   teleop:=ps5
```

The robot was manually driven through warehouse aisles while SLAM Toolbox constructed the occupancy grid.

Good mapping practice used during this phase included:

```text
drive through several aisles
revisit previously seen areas
close loops
avoid only driving one long open perimeter
```

The larger environment provided a much more useful test of:

- scan matching,
- loop closure,
- odometry consistency,
- map scale,
- warehouse aisle geometry.

---

# 21. Saving the warehouse map

The resulting map was saved using:

```bash
ros2 run nav2_map_server map_saver_cli   -f ~/Projects/spot_ros2_ws/maps/warehouse_map
```

This creates:

```text
warehouse_map.yaml
warehouse_map.pgm
```

The map files were then copied into:

```text
src/spot_navigation/maps/
```

and the package rebuilt so the installed navigation package contains the map.

The navigation bringup can therefore reference:

```text
spot_navigation/maps/warehouse_map.yaml
```

rather than an external ad-hoc path.

---

# 22. Warehouse navigation bringup

A separate top-level launch file is used for saved-map autonomous navigation:

```text
spot_navigation/launch/warehouse_navigation.launch.py
```

Its conceptual contents are:

```text
Gazebo warehouse
        ↓
Spot + ros2_control
        ↓
RL locomotion policy
        ↓
map_server
        ↓
AMCL
        ↓
Nav2
        ↓
RViz
```

The key distinction from mapping is:

```text
mapping launch:
    SLAM Toolbox creates map → odom

navigation launch:
    AMCL creates map → odom
```

The warehouse navigation launch was tested successfully.

The robot:

- localized correctly,
- accepted Nav2 goals from RViz,
- planned paths,
- followed those paths,
- generated velocity commands compatible with the learned locomotion policy,
- moved through the warehouse environment autonomously.

This is the first complete autonomous-navigation milestone for the Spot project.

---

# 23. Manual teleop and Nav2 command ownership

Manual teleoperation is currently straightforward during mapping because teleop is the only intended `/cmd_vel` source.

Autonomous navigation is different.

Without an arbitration layer, running Nav2 and keyboard / PS5 teleop simultaneously would produce competing publishers:

```text
Nav2 ───────────────┐
                    ├── /cmd_vel
manual teleop ──────┘
```

This should be avoided.

The intended future architecture is:

```text
Nav2
  ↓
/cmd_vel_nav
        \
         \
          → twist_mux → /cmd_vel → policy
         /
        /
manual teleop
  ↓
/cmd_vel_teleop
```

with manual control given higher priority.

`twist_mux` has **not yet been integrated** in the current validated stack.

Until it is added, navigation and direct manual command sources should not intentionally compete for `/cmd_vel`.

---

# 24. First autonomous navigation behavior

Initial autonomous navigation worked, but the resulting quadruped motion looked unnatural.

Because MPPI was configured with:

```yaml
motion_model: "Omni"
```

and allowed:

```yaml
vy_max: 0.40
```

the optimizer often treated lateral translation as equally acceptable to turning the body.

Typical behavior was:

```text
goal lies diagonally ahead
        ↓
controller commands vx + vy
        ↓
Spot moves diagonally
        ↓
body heading changes little
```

This is mathematically valid for a holonomic robot.

However, it is not the preferred locomotion style for this Spot-like platform.

The desired behavior is closer to:

```text
goal lies diagonally ahead
        ↓
rotate while advancing
        ↓
body aligns toward path
        ↓
mostly forward walking
```

while still retaining lateral capability for cases where it is genuinely useful.

---

# 25. Path-heading tuning

## 25.1 PathAngleCritic

The main MPPI tool used to influence this behavior was:

```text
PathAngleCritic
```

The first tuning step increased its weight substantially.

This improved body alignment, confirming that heading preference was the correct optimization target.

However, excessive weight also made trajectories more conservative and slower.

The tuning was then softened.

A useful current configuration is approximately:

```yaml
PathAngleCritic:
  enabled: true
  cost_power: 1
  cost_weight: 6.0
  offset_from_furthest: 4
  threshold_to_consider: 0.5
  max_angle_to_furthest: 0.5
  mode: 0
```

Important interpretation:

```text
mode: 0
```

biases the controller toward forward-facing path travel rather than treating all orientations as equally desirable.

The heading threshold:

```text
0.5 rad ≈ 28.6°
```

makes heading misalignment relevant earlier than the previous ~1 rad configuration.

## 25.2 MPPI noise distribution

The sampling distribution was also intentionally biased away from lateral motion.

The latest working values were approximately:

```yaml
vx_std: 0.45
vy_std: 0.08
wz_std: 0.45
```

The rationale is:

```text
sample forward motion strongly
sample yaw strongly
sample lateral motion less aggressively
```

while still keeping:

```yaml
vy_max: 0.40
```

available.

This is preferable to simply removing Spot's lateral capability.

## 25.3 Other relevant critics

The currently observed MPPI configuration also contains approximately:

```yaml
PreferForwardCritic:
  enabled: true
  cost_weight: 5.0

PathAlignCritic:
  enabled: true
  cost_weight: 14.0

PathFollowCritic:
  enabled: true
  cost_weight: 5.0

GoalCritic:
  enabled: true
  cost_weight: 5.0

GoalAngleCritic:
  enabled: true
  cost_weight: 3.0
```

These values should be treated as the current navigation tuning state rather than immutable physical parameters.

Unlike the locomotion controller, the navigation optimizer is still an active tuning area.

---

# 26. Accidental MPPI parameter type error

During tuning, one line was accidentally entered as:

```yaml
wz_std: 0.45s
```

instead of:

```yaml
wz_std: 0.45
```

The trailing `s` converted the YAML value from a numeric scalar into a string.

This caused Nav2 startup to fail sufficiently that RViz reported:

```text
navigate_to_pose action server is not available.
Is the initial pose set?
```

The RViz wording was misleading.

The actual issue was that the Nav2 stack did not fully activate because the MPPI controller configuration contained an invalid parameter type.

The useful diagnostic commands are:

```bash
ros2 action list | grep navigate
```

Expected:

```text
/navigate_to_pose
/navigate_through_poses
```

and:

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

Expected:

```text
active [3]
```

This incident reinforces an important Nav2 debugging principle:

> If RViz reports that `navigate_to_pose` is unavailable, first verify Nav2 lifecycle state and controller startup logs before assuming that localization or the initial pose is the problem.

---

# 27. Current qualitative navigation result

After the latest heading and MPPI tuning:

```text
Spot follows the planned path substantially better.
```

Compared with the original omnidirectional tuning:

- body heading follows the path more closely,
- excessive diagonal strafing is reduced,
- forward motion is preferred more strongly,
- lateral capability remains available,
- navigation continues to work correctly.

The system is now much closer to a natural quadruped navigation style rather than behaving like an unconstrained planar holonomic platform.

Further tuning is still possible, but the current behavior is already useful and operational.

---

# 28. Current package-level architecture

The ROS 2 workspace is now conceptually organized as:

```text
spot_ros2_ws/
│
├── spot_description/
│   └── URDF / Xacro
│       robot geometry
│       LiDAR link
│       ros2_control interfaces
│
├── spot_gazebo/
│   ├── launch/
│   │   └── spawn_spot.launch.py
│   ├── worlds/
│   │   ├── flat_world.sdf
│   │   └── warehouse_world.sdf
│   └── config/
│       └── spot_controllers.yaml
│
├── spot_policy/
│   └── learned locomotion deployment
│       /cmd_vel → policy → q_ref
│
├── spot_teleop/
│   ├── ps5_teleop.py
│   └── launch/
│       ├── ps5_teleop.launch.py
│       └── teleop.launch.py
│
└── spot_navigation/
    ├── config/
    │   ├── spot_slam.yaml
    │   ├── spot_localization.yaml
    │   └── spot_nav2.yaml
    │
    ├── maps/
    │   └── warehouse_map.*
    │
    └── launch/
        ├── warehouse_mapping.launch.py
        └── warehouse_navigation.launch.py
```

This is a much cleaner separation than the original collection of manual terminal commands.

---

# 29. Current standard workflows

## 29.1 Warehouse mapping with keyboard

```bash
cd ~/Projects/spot_ros2_ws

source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash

ros2 launch spot_navigation warehouse_mapping.launch.py   teleop:=keyboard
```

## 29.2 Warehouse mapping with PS5

```bash
ros2 launch spot_navigation warehouse_mapping.launch.py   teleop:=ps5
```

## 29.3 Mapping without manual teleop

```bash
ros2 launch spot_navigation warehouse_mapping.launch.py   teleop:=none
```

## 29.4 Saved-map autonomous navigation

```bash
ros2 launch spot_navigation warehouse_navigation.launch.py
```

At the current stage, manual teleop should not be added to the autonomous navigation launch unless command arbitration is also introduced.

---

# 30. Current TF and topic contract

The most important navigation interfaces are now:

```text
/clock
    simulation clock

/spot/odom
    Gazebo odometry

/scan
    2D LiDAR

/tf
    dynamic transforms

/tf_static
    fixed robot transforms

/map
    SLAM or map_server occupancy grid

/amcl_pose
    AMCL pose estimate

/cmd_vel
    final velocity command consumed by Spot policy
```

Core frame chain:

```text
map
 ↓
odom
 ↓
base_link
 ↓
body
 ↓
lidar_link
```

This frame contract should be preserved.

---

# 31. Current Nav2 costmap state

The current navigation configuration uses:

```text
local costmap:
    global frame: odom
    robot frame:  base_link
    rolling window: true
    width:  3 m
    height: 3 m
    resolution: 0.05 m

global costmap:
    global frame: map
    robot frame:  base_link
    resolution: 0.05 m
```

The footprint is:

```text
±0.55 m longitudinally
±0.30 m laterally
```

Inflation is approximately:

```text
inflation radius: 0.70 m
cost scaling:     3.0
```

LiDAR obstacle handling uses `/scan`.

The local costmap uses the scan for marking and clearing, with practical obstacle / raytracing distances around:

```text
obstacle_max_range: ~2.5 m
raytrace_max_range: ~3.0 m
```

These values are navigation parameters and can be tuned later if obstacle behavior requires it.

---

# 32. Collision monitor

The current Nav2 stack includes:

```text
collision_monitor
```

with an approach-style polygon based on the local published footprint.

Its role is downstream of the velocity smoother:

```text
cmd_vel_smoothed
        ↓
collision_monitor
        ↓
cmd_vel
```

This creates an additional safety layer between the planner / controller and the locomotion policy.

It should remain conceptually separate from the learned policy.

---

# 33. Frozen components

The following components remain frozen unless new evidence requires reopening them.

## 33.1 Robot rigid-body baseline

No autonomy issue has required changing:

```text
mass distribution
inertias
joint geometry
joint ordering
home pose
collision geometry
```

## 33.2 Learned locomotion policy

The robust deployment policy remains valid.

No SLAM or Nav2 issue has demonstrated a policy fault.

## 33.3 Low-level servo

Remain:

```text
Kp = 300
Kd = 40
```

## 33.4 Torque limits

Remain:

```text
hips  ±45 N·m
knees ±115 N·m
```

## 33.5 Simulation / control rates

Remain:

```text
Gazebo physics:       2000 Hz
controller manager:   2000 Hz
policy:                 ~50 Hz
```

These should not be changed merely to tune autonomous navigation.

---

# 34. Components that are still active tuning areas

The following are **not frozen**:

```text
MPPI critic weights
MPPI noise standard deviations
heading preference
costmap inflation
local obstacle ranges
goal tolerances
planner behavior
recovery behaviors
collision-monitor geometry
mission-level autonomy
```

These sit above the locomotion layer and can be tuned without invalidating the locomotion baseline.

---

# 35. Important debugging checklist

For future failures, the following checks quickly separate several classes of problem.

## 35.1 Multiple simulation clocks

```bash
ros2 topic info /clock -v
```

Expected:

```text
one publisher while one simulation is running
```

## 35.2 Duplicate nodes

```bash
ros2 node list
```

There should not be duplicate names such as:

```text
/clock_bridge
/spot_policy_controller
/robot_state_publisher
```

## 35.3 Odometry

```bash
ros2 topic hz /spot/odom
```

Expected approximately:

```text
~50 Hz
```

## 35.4 LiDAR

```bash
ros2 topic hz /scan
```

Expected approximately:

```text
~10 Hz
```

## 35.5 Odometry TF

```bash
ros2 run tf2_ros tf2_echo odom base_link
```

Must continuously succeed.

## 35.6 LiDAR TF

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

Must succeed.

## 35.7 Nav2 action server

```bash
ros2 action list | grep navigate
```

Expected:

```text
/navigate_to_pose
/navigate_through_poses
```

## 35.8 Nav2 lifecycle

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

Expected:

```text
active [3]
```

---

# 36. Engineering lessons from this phase

Several important architectural lessons were established.

## 36.1 Keep navigation and locomotion separate

The RL policy should be treated as the robot's body-velocity tracking layer.

Nav2 should command:

```text
vx
vy
wz
```

not joints.

## 36.2 Preserve validated lower layers

When an autonomy problem appears, inspect:

```text
SLAM
TF
localization
costmaps
planner
controller
velocity pipeline
```

before reopening validated robot dynamics or low-level gains.

## 36.3 Holonomic does not mean "strafe all the time"

Spot can move laterally, so the correct Nav2 model is omnidirectional.

However, the objective function should still encode preferred locomotion style.

The final intent is:

```text
capability:
    omnidirectional

preference:
    mostly forward-facing travel
```

These are different concepts.

## 36.4 Launch files can fail after partial startup

A missing launch file or malformed parameter can occur after several child processes have already started.

Before relaunching after a startup exception, verify that old:

```text
Gazebo
bridges
policy nodes
robot_state_publisher
```

did not survive.

## 36.5 RViz error messages may be secondary symptoms

For example:

```text
navigate_to_pose action server is not available.
Is the initial pose set?
```

does not prove that the initial pose is the fault.

The real cause may be:

```text
controller server configuration failure
invalid parameter type
lifecycle activation failure
```

Always inspect server lifecycle state and launch logs.

---

# 37. Current project milestone

The project has moved through the following major stages:

```text
1. Spot rigid-body model
        PASS

2. Learned locomotion in MuJoCo
        PASS

3. MuJoCo → Gazebo sim-to-sim locomotion
        PASS

4. Gazebo quaternion / odometry failure
        ROOT CAUSE FOUND + FIXED

5. LiDAR integration
        PASS

6. TF integration
        PASS

7. Manual teleoperation
        PASS

8. SLAM Toolbox mapping
        PASS

9. Saved-map AMCL localization
        PASS

10. Nav2 autonomous navigation
        PASS

11. Warehouse environment
        PASS

12. Warehouse mapping
        PASS

13. Warehouse autonomous navigation
        PASS

14. Forward-biased omnidirectional path following
        WORKING / STILL TUNABLE
```

This is the first point where the project functions as a complete mobile robotic system rather than only as a locomotion demonstration.

---

# 38. Immediate remaining work

The next useful autonomy milestones are:

```text
1. Freeze the current working warehouse navigation baseline.

2. Add twist_mux:
       Nav2 command
       manual command
       priority arbitration
       emergency manual override

3. Add dynamic obstacles:
       obstacle enters aisle
       local costmap detects it
       controller slows / stops
       planner replans if necessary

4. Add semantic locations:
       "loading station"
       "inspection point A"
       "electrical cabinet"
       "charging area"

5. Add multi-goal mission logic:
       navigate to point
       inspect
       continue
       recover on failure

6. Add inspection sensing:
       RGB or RGB-D camera
       target / gauge / object detection

7. Generate mission reports.

8. Later:
       terrain-aware planning
       rough-terrain locomotion integration
```

The next phase should stay focused on **autonomy**, not on reopening the already validated locomotion controller.

---

# 39. Longer-term project direction

The current project naturally supports the larger objective:

# Autonomous Legged Robot for Industrial Inspection and Navigation

The intended system can evolve toward:

```text
robust learned locomotion
        ↓
warehouse localization
        ↓
global route planning
        ↓
dynamic obstacle avoidance
        ↓
semantic mission locations
        ↓
autonomous inspection
        ↓
mission report / anomaly logging
```

A later extension can introduce:

```text
rough terrain
stairs / ramps
terrain-aware path costs
locomotion-aware planning
```

without discarding the present 2D autonomy stack.

---

# 40. Current baseline summary

## Locomotion

```text
Policy:              robust model_600
Policy rate:         ~50 Hz

Command envelope:
vx                  ±0.65 m/s
vy                  ±0.40 m/s
wz                  ±0.60 rad/s
```

## Gazebo / ros2_control

```text
Gazebo physics:      2000 Hz
Controller manager:  2000 Hz

Kp:                  300
Kd:                  40

Hip clamp:           ±45 N·m
Knee clamp:          ±115 N·m

Gazebo runtime:      fixed 8.15.0-class runtime
                     containing quaternion sign fix
```

## LiDAR

```text
Frame:               lidar_link
Topic:               /scan
Rate:                ~10 Hz
FOV:                 360°
Samples:              720
Range:               0.1–12 m
```

## Navigation frames

```text
map
 ↓
odom
 ↓
base_link
 ↓
body
 ↓
lidar_link
```

## Nav2

```text
Controller:          MPPI
Motion model:        Omni

Prediction:
time_steps           40
model_dt             0.05 s
horizon              2.0 s
batch_size           1000
iteration_count      1

Velocity limits:
vx                   ±0.65 m/s
vy                   ±0.40 m/s
wz                   ±0.60 rad/s

Acceleration bounds:
ax                   ±10
ay                   ±10
az                   +10

Velocity smoother:
max accel            [8, 8, 8]
max decel            [-8, -8, -8]

Current MPPI sampling:
vx_std               ~0.45
vy_std               ~0.08
wz_std               ~0.45

Current heading bias:
PathAngleCritic
  weight             ~6.0
  max angle          ~0.5 rad
  mode               0

PreferForwardCritic
  weight             ~5.0
```

## Footprint

```text
[[ 0.55,  0.30],
 [ 0.55, -0.30],
 [-0.55, -0.30],
 [-0.55,  0.30]]
```

## Warehouse

```text
warehouse world:     working
warehouse map:       saved
SLAM:                working
AMCL:                working
Nav2 goals:          working
path following:      working and substantially improved
```

---

# 41. Final status

The project now has a validated end-to-end autonomy chain:

```text
Gazebo warehouse
        ↓
Spot rigid body
        ↓
ros2_control
        ↓
learned locomotion policy
        ↓
LiDAR + odometry + TF
        ↓
SLAM / saved map
        ↓
AMCL
        ↓
Nav2
        ↓
MPPI
        ↓
velocity smoothing
        ↓
collision monitoring
        ↓
/cmd_vel
        ↓
learned Spot locomotion
```

The major milestone is:

```text
AUTONOMOUS WAREHOUSE NAVIGATION: WORKING
```

The major autonomy behavior issue discovered after first integration was excessive omnidirectional strafing.

That behavior was not a locomotion fault. It was a consequence of Nav2 correctly exploiting the available holonomic command space.

MPPI heading and sampling preferences were then adjusted so that Spot now follows the path much more naturally while retaining lateral mobility.

The recommended next step is to **freeze this working warehouse navigation baseline**, add command arbitration with `twist_mux`, and then move into dynamic obstacles, semantic waypoints, multi-goal missions, and inspection behaviors.
