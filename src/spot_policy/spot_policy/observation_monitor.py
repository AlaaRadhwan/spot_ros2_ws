#!/usr/bin/env python3
"""Build and validate the Spot RL policy observation vector."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


def projected_gravity_from_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> np.ndarray:
    """Rotate world gravity direction [0, 0, -1] into the body frame."""

    norm = np.sqrt(x * x + y * y + z * z + w * w)

    if norm < 1e-8:
        raise ValueError("Received an invalid zero-length quaternion.")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    return np.array(
        [
            2.0 * (y * w - x * z),
            -2.0 * (y * z + x * w),
            2.0 * (x * x + y * y) - 1.0,
        ],
        dtype=np.float32,
    )


class SpotObservationMonitor(Node):
    """Reconstruct the actor observation from ROS topics."""

    def __init__(
        self,
        node_name: str = "spot_observation_monitor",
    ) -> None:
        super().__init__(node_name)

        package_share = Path(
            get_package_share_directory("spot_policy")
        )

        metadata_path = (
            package_share
            / "config"
            / "policy_metadata.json"
        )

        if not metadata_path.is_file():
            raise FileNotFoundError(
                f"Policy metadata was not found: {metadata_path}"
            )

        with metadata_path.open(
            "r",
            encoding="utf-8",
        ) as metadata_file:
            self.metadata = json.load(metadata_file)

        self.joint_order = list(
            self.metadata["joint_order"]
        )

        self.default_joint_positions = np.asarray(
            self.metadata["default_joint_positions"],
            dtype=np.float32,
        )

        self.action_size = int(
            self.metadata["action_size"]
        )

        self.observation_size = int(
            self.metadata["observation_size"]
        )

        self.observation_layout = list(
            self.metadata["observation_order"]
        )

        if len(self.joint_order) != self.action_size:
            raise ValueError(
                "Joint count and action size do not match: "
                f"{len(self.joint_order)} versus "
                f"{self.action_size}"
            )

        if (
            self.default_joint_positions.size
            != self.action_size
        ):
            raise ValueError(
                "Default joint-position count does not "
                "match action size."
            )

        self.latest_joint_positions: dict[str, float] = {}
        self.latest_joint_velocities: dict[str, float] = {}

        self.command = np.zeros(3, dtype=np.float32)
        self.previous_action = np.zeros(
            self.action_size,
            dtype=np.float32,
        )

        self.received_joint_state = False
        self.observation_count = 0

        self.last_log_time_ns = 0

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Odometry,
            "/spot/odom",
            self.odometry_callback,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Twist,
            "/cmd_vel",
            self.command_callback,
            10,
        )

        self.get_logger().info(
            "Loaded policy metadata:"
            f" observations={self.observation_size},"
            f" actions={self.action_size}"
        )

        self.get_logger().info(
            f"Joint order: {self.joint_order}"
        )

        for term in self.observation_layout:
            self.get_logger().info(
                "Observation term "
                f"{term['name']}: "
                f"[{term['start']}:{term['end']}] "
                f"shape={term['shape']}"
            )

    def joint_state_callback(
        self,
        message: JointState,
    ) -> None:
        velocity_available = (
            len(message.velocity) == len(message.name)
        )

        for index, joint_name in enumerate(message.name):
            if joint_name not in self.joint_order:
                continue

            if index < len(message.position):
                self.latest_joint_positions[joint_name] = (
                    float(message.position[index])
                )

            if velocity_available:
                self.latest_joint_velocities[joint_name] = (
                    float(message.velocity[index])
                )

        self.received_joint_state = all(
            joint_name in self.latest_joint_positions
            and joint_name in self.latest_joint_velocities
            for joint_name in self.joint_order
        )

    def command_callback(
        self,
        message: Twist,
    ) -> None:
        self.command[:] = [
            message.linear.x,
            message.linear.y,
            message.angular.z,
        ]

    def odometry_callback(
        self,
        message: Odometry,
    ) -> None:
        if not self.received_joint_state:
            self.get_logger().warn(
                "Waiting for all 12 joint positions "
                "and velocities...",
                throttle_duration_sec=2.0,
            )
            return

        try:
            observation, terms = self.build_observation(
                message
            )
        except (KeyError, ValueError) as error:
            self.get_logger().error(str(error))
            return

        self.observation_count += 1

        current_time_ns = self.get_clock().now().nanoseconds

        if (
            current_time_ns - self.last_log_time_ns
            >= 1_000_000_000
        ):
            self.last_log_time_ns = current_time_ns
            self.print_diagnostics(
                observation,
                terms,
                message,
            )

    def build_observation(
        self,
        odometry: Odometry,
    ) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        twist = odometry.twist.twist
        orientation = odometry.pose.pose.orientation

        base_linear_velocity = np.array(
            [
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
            ],
            dtype=np.float32,
        )

        base_angular_velocity = np.array(
            [
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ],
            dtype=np.float32,
        )

        projected_gravity = (
            projected_gravity_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
        )

        joint_positions = np.array(
            [
                self.latest_joint_positions[name]
                for name in self.joint_order
            ],
            dtype=np.float32,
        )

        joint_velocities = np.array(
            [
                self.latest_joint_velocities[name]
                for name in self.joint_order
            ],
            dtype=np.float32,
        )

        relative_joint_positions = (
            joint_positions
            - self.default_joint_positions
        )

        terms: dict[str, np.ndarray] = {
            "base_lin_vel": base_linear_velocity,
            "base_ang_vel": base_angular_velocity,
            "projected_gravity": projected_gravity,
            "joint_pos": relative_joint_positions,
            "joint_vel": joint_velocities,
            "actions": self.previous_action.copy(),
            "command": self.command.copy(),
        }

        observation = np.zeros(
            self.observation_size,
            dtype=np.float32,
        )

        for layout in self.observation_layout:
            term_name = layout["name"]
            start = int(layout["start"])
            end = int(layout["end"])
            required_size = end - start

            if term_name == "height_scan":
                # Flat-ground approximation used during the
                # standalone rough-policy test.
                base_height = float(
                    odometry.pose.pose.position.z
                )

                value = np.full(
                    required_size,
                    np.clip(base_height, -1.0, 1.0),
                    dtype=np.float32,
                )

                terms[term_name] = value
            else:
                if term_name not in terms:
                    raise KeyError(
                        "Unsupported observation term in "
                        f"metadata: {term_name}"
                    )

                value = terms[term_name].reshape(-1)

            if value.size != required_size:
                raise ValueError(
                    f"Observation term '{term_name}' "
                    f"requires {required_size} values, "
                    f"but the node produced {value.size}."
                )

            observation[start:end] = value

        if observation.size != self.observation_size:
            raise ValueError(
                "Constructed observation has the wrong size."
            )

        if not np.all(np.isfinite(observation)):
            invalid_indices = np.flatnonzero(
                ~np.isfinite(observation)
            )

            raise ValueError(
                "Observation contains NaN or infinity at "
                f"indices {invalid_indices.tolist()}."
            )

        return observation, terms

    def print_diagnostics(
        self,
        observation: np.ndarray,
        terms: dict[str, np.ndarray],
        odometry: Odometry,
    ) -> None:
        gravity = terms["projected_gravity"]
        joint_position = terms["joint_pos"]
        joint_velocity = terms["joint_vel"]

        message = (
            "\n"
            "Spot policy observation\n"
            f"  count:          {self.observation_count}\n"
            f"  shape:          {observation.shape}\n"
            f"  finite:         "
            f"{bool(np.all(np.isfinite(observation)))}\n"
            f"  observation:    "
            f"min={observation.min():+.3f}, "
            f"max={observation.max():+.3f}\n"
            f"  base height:    "
            f"{odometry.pose.pose.position.z:+.3f} m\n"
            f"  linear vel:     "
            f"{np.array2string(terms['base_lin_vel'], precision=3)}\n"
            f"  angular vel:    "
            f"{np.array2string(terms['base_ang_vel'], precision=3)}\n"
            f"  gravity:        "
            f"{np.array2string(gravity, precision=3)} "
            f"norm={np.linalg.norm(gravity):.3f}\n"
            f"  joint pos rel:  "
            f"min={joint_position.min():+.3f}, "
            f"max={joint_position.max():+.3f}\n"
            f"  joint velocity: "
            f"min={joint_velocity.min():+.3f}, "
            f"max={joint_velocity.max():+.3f}\n"
            f"  command:        "
            f"{np.array2string(terms['command'], precision=3)}"
        )

        if "height_scan" in terms:
            height_scan = terms["height_scan"]

            message += (
                "\n"
                f"  height scan:    "
                f"size={height_scan.size}, "
                f"value={height_scan[0]:+.3f}"
            )

        self.get_logger().info(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = SpotObservationMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()