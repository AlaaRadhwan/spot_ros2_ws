#!/usr/bin/env python3
"""Automated Gazebo command-space evaluation for the Spot V3 PPO deployment."""

from __future__ import annotations

import csv
import json
import math
from datetime import datetime
from pathlib import Path
from statistics import fmean
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


PHASES = (
    ("stand_initial", 5.0, 0.0, 0.0, 0.0),
    ("forward",       5.0, 0.5, 0.0, 0.0),
    ("pause_1",       2.0, 0.0, 0.0, 0.0),
    ("backward",      5.0, -0.5, 0.0, 0.0),
    ("pause_2",       2.0, 0.0, 0.0, 0.0),
    ("left",          5.0, 0.0, 0.3, 0.0),
    ("pause_3",       2.0, 0.0, 0.0, 0.0),
    ("right",         5.0, 0.0, -0.3, 0.0),
    ("pause_4",       2.0, 0.0, 0.0, 0.0),
    ("yaw_left",      5.0, 0.0, 0.0, 0.5),
    ("pause_5",       2.0, 0.0, 0.0, 0.0),
    ("yaw_right",     5.0, 0.0, 0.0, -0.5),
    ("pause_6",       2.0, 0.0, 0.0, 0.0),
    ("combined",      7.0, 0.4, 0.2, 0.3),
    ("stand_final",   5.0, 0.0, 0.0, 0.0),
)

CSV_BASE_COLUMNS = (
    "t_s", "phase", "phase_t_s",
    "cmd_vx", "cmd_vy", "cmd_wz",
    "actual_vx", "actual_vy", "actual_vz",
    "actual_wx", "actual_wy", "actual_wz",
    "base_x", "base_y", "base_z",
    "roll", "pitch", "yaw",
)


def quaternion_to_rpy(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return math.nan, math.nan, math.nan
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def finite(values: list[float]) -> list[float]:
    return [v for v in values if math.isfinite(v)]


def mean(values: list[float]) -> float:
    vals = finite(values)
    return fmean(vals) if vals else math.nan


def rms(values: list[float]) -> float:
    vals = finite(values)
    return math.sqrt(fmean(v * v for v in vals)) if vals else math.nan


def rmse(values: list[float], target: float) -> float:
    vals = finite(values)
    return math.sqrt(fmean((v - target) ** 2 for v in vals)) if vals else math.nan


def mae(values: list[float], target: float) -> float:
    vals = finite(values)
    return fmean(abs(v - target) for v in vals) if vals else math.nan


def max_abs(values: list[float]) -> float:
    vals = [abs(v) for v in finite(values)]
    return max(vals) if vals else math.nan


class SpotGazeboEvaluator(Node):
    def __init__(self) -> None:
        super().__init__("spot_gazebo_evaluator")

        self.declare_parameter(
            "output_dir",
            str(Path.home() / "Projects" / "spot_ros2_ws" / "evaluation_runs"),
        )
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("steady_state_skip_s", 1.0)
        self.declare_parameter("saturation_fraction", 0.98)
        self.declare_parameter("allow_multiple_cmd_publishers", False)

        self.output_root = Path(str(self.get_parameter("output_dir").value)).expanduser()
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.steady_state_skip_s = float(self.get_parameter("steady_state_skip_s").value)
        self.saturation_fraction = float(self.get_parameter("saturation_fraction").value)
        self.allow_multiple_cmd_publishers = bool(
            self.get_parameter("allow_multiple_cmd_publishers").value
        )

        metadata_path = Path(get_package_share_directory("spot_policy")) / "config" / "policy_metadata.json"
        with metadata_path.open("r", encoding="utf-8") as f:
            self.metadata = json.load(f)

        self.joint_order = list(self.metadata["joint_order"])
        notes = self.metadata.get("deployment_notes", {})
        hip_limit = float(notes.get("hip_effort_limit_nm", 45.0))
        knee_limit = float(notes.get("knee_effort_limit_nm", 115.0))
        self.effort_limits = {
            name: knee_limit if name.endswith("_kn") else hip_limit
            for name in self.joint_order
        }

        self.latest_odom: Odometry | None = None
        self.latest_joint_state: JointState | None = None
        self.joint_index: dict[str, int] = {}
        self.rows: list[dict[str, Any]] = []

        self.started = False
        self.finished = False
        self.aborted = False
        self.start_time_ns = 0
        self.last_phase_index = -1

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/spot/odom", self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(JointState, "/joint_states", self.joint_callback, qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self.control_tick)

        total_s = sum(p[1] for p in PHASES)
        self.get_logger().info(f"Loaded V3 metadata from {metadata_path}")
        self.get_logger().info(f"Evaluation duration: {total_s:.1f} s of ROS/simulation time")
        self.get_logger().info("Waiting for /spot/odom and complete /joint_states before starting.")
        self.get_logger().warn(
            "Stop spot_teleop and any manual /cmd_vel publishers before running this evaluation."
        )

        try:
            use_sim_time = bool(self.get_parameter("use_sim_time").value)
        except Exception:
            use_sim_time = False
        if not use_sim_time:
            self.get_logger().warn(
                "use_sim_time is false. Run with --ros-args -p use_sim_time:=true."
            )

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def joint_callback(self, msg: JointState) -> None:
        self.latest_joint_state = msg
        self.joint_index = {name: i for i, name in enumerate(msg.name)}

    def complete_joint_state_available(self) -> bool:
        msg = self.latest_joint_state
        if msg is None or not all(name in self.joint_index for name in self.joint_order):
            return False
        max_index = max(self.joint_index[name] for name in self.joint_order)
        return len(msg.position) > max_index and len(msg.velocity) > max_index

    def publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())

    def check_cmd_publishers(self) -> bool:
        pubs = self.get_publishers_info_by_topic("/cmd_vel")
        if len(pubs) <= 1 or self.allow_multiple_cmd_publishers:
            return True
        names = [f"{p.node_namespace}/{p.node_name}" for p in pubs]
        self.get_logger().error(
            "Multiple /cmd_vel publishers detected; aborting to avoid conflicts: " + str(names)
        )
        return False

    def phase_for_elapsed(self, elapsed_s: float):
        cursor = 0.0
        for index, phase in enumerate(PHASES):
            if elapsed_s < cursor + phase[1]:
                return index, phase, elapsed_s - cursor
            cursor += phase[1]
        return None

    def control_tick(self) -> None:
        if self.finished:
            return

        if self.latest_odom is None or not self.complete_joint_state_available():
            self.publish_zero()
            return

        now_ns = self.get_clock().now().nanoseconds

        if not self.started:
            if not self.check_cmd_publishers():
                self.aborted = True
                self.finished = True
                self.publish_zero()
                return
            self.started = True
            self.start_time_ns = now_ns
            self.get_logger().info("Evaluation started.")
            return

        elapsed_s = (now_ns - self.start_time_ns) * 1e-9
        phase_info = self.phase_for_elapsed(elapsed_s)

        if phase_info is None:
            self.publish_zero()
            self.save_results()
            self.finished = True
            self.get_logger().info("Evaluation complete. Zero command published.")
            return

        phase_index, phase, phase_t_s = phase_info
        phase_name, _duration, vx, vy, wz = phase

        if phase_index != self.last_phase_index:
            self.last_phase_index = phase_index
            self.get_logger().info(
                f"Phase: {phase_name:14s} cmd=[{vx:+.2f}, {vy:+.2f}, {wz:+.2f}]"
            )

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.cmd_pub.publish(cmd)

        self.record_sample(elapsed_s, phase_name, phase_t_s, vx, vy, wz)

    def record_sample(self, elapsed_s, phase_name, phase_t_s, cmd_vx, cmd_vy, cmd_wz) -> None:
        odom = self.latest_odom
        joint = self.latest_joint_state
        if odom is None or joint is None:
            return

        pose = odom.pose.pose
        twist = odom.twist.twist
        roll, pitch, yaw = quaternion_to_rpy(
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
        )

        row: dict[str, Any] = {
            "t_s": elapsed_s,
            "phase": phase_name,
            "phase_t_s": phase_t_s,
            "cmd_vx": cmd_vx,
            "cmd_vy": cmd_vy,
            "cmd_wz": cmd_wz,
            "actual_vx": float(twist.linear.x),
            "actual_vy": float(twist.linear.y),
            "actual_vz": float(twist.linear.z),
            "actual_wx": float(twist.angular.x),
            "actual_wy": float(twist.angular.y),
            "actual_wz": float(twist.angular.z),
            "base_x": float(pose.position.x),
            "base_y": float(pose.position.y),
            "base_z": float(pose.position.z),
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        }

        effort_available = len(joint.effort) == len(joint.name)
        for name in self.joint_order:
            i = self.joint_index[name]
            row[f"{name}_q"] = float(joint.position[i])
            row[f"{name}_dq"] = float(joint.velocity[i])
            row[f"{name}_effort"] = float(joint.effort[i]) if effort_available else math.nan

        self.rows.append(row)

    def summarize_phase(self, phase_name: str, cmd: tuple[float, float, float]) -> dict[str, Any]:
        phase_rows = [r for r in self.rows if r["phase"] == phase_name]
        steady = [r for r in phase_rows if float(r["phase_t_s"]) >= self.steady_state_skip_s]
        if not steady:
            steady = phase_rows

        vx = [float(r["actual_vx"]) for r in steady]
        vy = [float(r["actual_vy"]) for r in steady]
        wz = [float(r["actual_wz"]) for r in steady]
        height = [float(r["base_z"]) for r in steady]
        roll = [float(r["roll"]) for r in steady]
        pitch = [float(r["pitch"]) for r in steady]

        joint_velocities: list[float] = []
        joint_efforts: list[float] = []
        effort_count = 0
        saturated_count = 0
        max_effort_ratio = 0.0

        for row in steady:
            for name in self.joint_order:
                dq = float(row[f"{name}_dq"])
                effort = float(row[f"{name}_effort"])
                if math.isfinite(dq):
                    joint_velocities.append(dq)
                if math.isfinite(effort):
                    joint_efforts.append(effort)
                    ratio = abs(effort) / self.effort_limits[name]
                    max_effort_ratio = max(max_effort_ratio, ratio)
                    effort_count += 1
                    if ratio >= self.saturation_fraction:
                        saturated_count += 1

        cmd_vx, cmd_vy, cmd_wz = cmd
        return {
            "phase": phase_name,
            "samples_total": len(phase_rows),
            "samples_steady": len(steady),
            "steady_state_skip_s": self.steady_state_skip_s,
            "command": {"vx_mps": cmd_vx, "vy_mps": cmd_vy, "wz_radps": cmd_wz},
            "tracking": {
                "mean_actual_vx_mps": mean(vx),
                "mean_actual_vy_mps": mean(vy),
                "mean_actual_wz_radps": mean(wz),
                "vx_mae_mps": mae(vx, cmd_vx),
                "vy_mae_mps": mae(vy, cmd_vy),
                "wz_mae_radps": mae(wz, cmd_wz),
                "vx_rmse_mps": rmse(vx, cmd_vx),
                "vy_rmse_mps": rmse(vy, cmd_vy),
                "wz_rmse_radps": rmse(wz, cmd_wz),
            },
            "body": {
                "height_mean_m": mean(height),
                "height_min_m": min(height) if height else math.nan,
                "height_max_m": max(height) if height else math.nan,
                "roll_rms_rad": rms(roll),
                "roll_max_abs_rad": max_abs(roll),
                "pitch_rms_rad": rms(pitch),
                "pitch_max_abs_rad": max_abs(pitch),
            },
            "joints": {
                "max_abs_velocity_radps": max_abs(joint_velocities),
                "max_abs_effort_nm": max_abs(joint_efforts),
                "max_effort_limit_ratio": max_effort_ratio if effort_count else math.nan,
                "saturation_threshold_fraction": self.saturation_fraction,
                "saturated_joint_samples": saturated_count,
                "effort_joint_samples": effort_count,
                "saturation_percent": 100.0 * saturated_count / effort_count if effort_count else math.nan,
            },
        }

    def save_results(self) -> None:
        if not self.rows:
            self.get_logger().error("No samples were recorded; no files written.")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = self.output_root / f"spot_v3_gazebo_eval_{timestamp}"
        run_dir.mkdir(parents=True, exist_ok=False)

        columns = list(CSV_BASE_COLUMNS)
        for name in self.joint_order:
            columns.extend((f"{name}_q", f"{name}_dq", f"{name}_effort"))

        csv_path = run_dir / "samples.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=columns)
            writer.writeheader()
            writer.writerows(self.rows)

        phase_summaries = [
            self.summarize_phase(name, (vx, vy, wz))
            for name, _duration, vx, vy, wz in PHASES
        ]

        notes = self.metadata.get("deployment_notes", {})
        summary = {
            "schema_version": 1,
            "task_id": self.metadata.get("task_id"),
            "checkpoint": self.metadata.get("checkpoint"),
            "policy_frequency_hz": self.metadata.get("policy_frequency_hz"),
            "gazebo_physics_hz": notes.get("gazebo_physics_hz"),
            "ros2_control_hz": notes.get("ros2_control_hz"),
            "low_level_kp": notes.get("low_level_kp"),
            "low_level_kd": notes.get("low_level_kd"),
            "hip_effort_limit_nm": notes.get("hip_effort_limit_nm"),
            "knee_effort_limit_nm": notes.get("knee_effort_limit_nm"),
            "requested_publish_rate_hz": self.publish_rate_hz,
            "samples_recorded": len(self.rows),
            "recorded_duration_s": float(self.rows[-1]["t_s"] - self.rows[0]["t_s"]),
            "phases": phase_summaries,
        }

        summary_path = run_dir / "summary.json"
        with summary_path.open("w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, allow_nan=True)

        self.print_terminal_summary(summary)
        self.get_logger().info(f"CSV:     {csv_path}")
        self.get_logger().info(f"Summary: {summary_path}")

    def print_terminal_summary(self, summary: dict[str, Any]) -> None:
        self.get_logger().info("\n===== Spot V3 Gazebo evaluation summary =====")
        for phase in summary["phases"]:
            cmd = phase["command"]
            tracking = phase["tracking"]
            body = phase["body"]
            joints = phase["joints"]
            self.get_logger().info(
                "\n"
                f"{phase['phase']}\n"
                f"  cmd:              vx={cmd['vx_mps']:+.2f}  vy={cmd['vy_mps']:+.2f}  wz={cmd['wz_radps']:+.2f}\n"
                f"  mean actual:      vx={tracking['mean_actual_vx_mps']:+.3f}  vy={tracking['mean_actual_vy_mps']:+.3f}  wz={tracking['mean_actual_wz_radps']:+.3f}\n"
                f"  RMSE:             vx={tracking['vx_rmse_mps']:.3f}  vy={tracking['vy_rmse_mps']:.3f}  wz={tracking['wz_rmse_radps']:.3f}\n"
                f"  body height:      mean={body['height_mean_m']:.3f} m  min={body['height_min_m']:.3f} m\n"
                f"  attitude max abs: roll={math.degrees(body['roll_max_abs_rad']):.2f} deg  pitch={math.degrees(body['pitch_max_abs_rad']):.2f} deg\n"
                f"  joints:           max|dq|={joints['max_abs_velocity_radps']:.2f} rad/s  max|effort|={joints['max_abs_effort_nm']:.1f} Nm  sat={joints['saturation_percent']:.3f}%"
            )

    def stop_safely(self) -> None:
        for _ in range(5):
            self.publish_zero()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SpotGazeboEvaluator()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn("Evaluation interrupted by user.")
    finally:
        node.stop_safely()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
