#!/usr/bin/env python3
"""Run the Spot policy and publish joint-position references."""

from __future__ import annotations

import numpy as np
import rclpy
import torch
from control_msgs.msg import MultiDOFCommand
from nav_msgs.msg import Odometry

from spot_policy.policy_dry_run import SpotPolicyDryRun


class SpotPolicyController(SpotPolicyDryRun):
    """Evaluate the policy and command the joint PID controller."""

    def __init__(self) -> None:
        super().__init__(
            node_name="spot_policy_controller",
            dry_run=False,
        )

        self.command_publisher = self.create_publisher(
            MultiDOFCommand,
            "/spot_joint_controller/reference",
            10,
        )

        # Odometry arrives at 50 Hz of simulation time.
        # Fifty steps gives a one-second transition.
        self.blend_steps = 50
        self.control_step = 0
        self.last_control_log_time_ns = 0

        self.get_logger().warn(
            "LIVE POLICY CONTROL ENABLED."
        )
        self.get_logger().info(
            "Blending from the nominal pose over "
            f"{self.blend_steps} policy steps."
        )

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

            observation_tensor = torch.from_numpy(
                observation
            ).unsqueeze(0)

            with torch.inference_mode():
                output = self.policy(
                    observation_tensor
                )

            raw_actions = self.extract_actions(output)

        except (
            KeyError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as error:
            self.get_logger().error(
                f"Policy control failed: {error}"
            )
            return

        self.control_step += 1

        blend = min(
            self.control_step / self.blend_steps,
            1.0,
        )

        # During startup, apply only part of the policy action.
        # This also makes the previous-action observation match
        # the action that was actually sent to the controller.
        applied_actions = (
            np.float32(blend) * raw_actions
        )

        target_positions = (
            self.default_joint_positions
            + self.action_scales * applied_actions
        )

        if not np.all(np.isfinite(target_positions)):
            self.get_logger().error(
                "Policy produced invalid joint targets."
            )
            return

        command = MultiDOFCommand()
        command.dof_names = self.joint_order
        command.values = target_positions.tolist()
        command.values_dot = [0.0] * self.action_size

        self.command_publisher.publish(command)

        self.previous_action[:] = applied_actions

        current_time_ns = (
            self.get_clock().now().nanoseconds
        )

        if (
            current_time_ns
            - self.last_control_log_time_ns
            >= 1_000_000_000
        ):
            self.last_control_log_time_ns = (
                current_time_ns
            )

            self.get_logger().info(
                "\n"
                "Spot live policy\n"
                f"  step:        {self.control_step}\n"
                f"  blend:       {blend:.2f}\n"
                f"  base height: "
                f"{message.pose.pose.position.z:.3f} m\n"
                f"  command:     "
                f"{np.array2string(terms['command'], precision=3)}\n"
                f"  actions:     "
                f"min={applied_actions.min():+.3f}, "
                f"max={applied_actions.max():+.3f}\n"
                f"  targets:     "
                f"min={target_positions.min():+.3f}, "
                f"max={target_positions.max():+.3f}"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = SpotPolicyController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()