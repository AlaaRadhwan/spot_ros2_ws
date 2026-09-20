# spot_eval

Automated command-space evaluation for the Spot V3 ROS 2 / Gazebo deployment.

## Inputs

- `/spot/odom`
- `/joint_states`
- V3 deployment metadata from `spot_policy`

## Output

The evaluator publishes `/cmd_vel` at 50 Hz and writes `samples.csv` and `summary.json` to:

```text
~/Projects/spot_ros2_ws/evaluation_runs/spot_v3_gazebo_eval_<timestamp>/
```

## Important

Stop `spot_teleop` and any manual `/cmd_vel` publishers first. Start Gazebo and the live `spot_policy` controller, and let Spot stand normally.

Run using simulation time:

```bash
ros2 run spot_eval gazebo_eval --ros-args -p use_sim_time:=true
```

Sequence:

```text
5 s   stand
5 s   vx = +0.5
2 s   zero
5 s   vx = -0.5
2 s   zero
5 s   vy = +0.3
2 s   zero
5 s   vy = -0.3
2 s   zero
5 s   wz = +0.5
2 s   zero
5 s   wz = -0.5
2 s   zero
7 s   vx = +0.4, vy = +0.2, wz = +0.3
5 s   stand
```

The summary reports per-phase velocity tracking, body height, roll/pitch, maximum joint velocity, maximum effort, and effort saturation percentage.
