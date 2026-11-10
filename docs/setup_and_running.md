# Setup and running

Back to the [project README](../README.md).

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
