#!/usr/bin/env python3
"""Run the exported Spot policy without publishing joint commands."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry

from spot_policy.observation_monitor import SpotObservationMonitor


class SpotPolicyDryRun(SpotObservationMonitor):
    """Evaluate the policy and report its proposed joint targets."""

    # def __init__(self) -> None:
    #     super().__init__()
    def __init__(
            self,
            node_name: str = "spot_policy_dry_run",
            dry_run: bool = True,
    ) -> None:
        super().__init__(node_name=node_name)

        package_share = Path(
            get_package_share_directory("spot_policy")
        )

        model_path = (
            package_share
            / "models"
            / "policy_jit.pt"
        )

        if not model_path.is_file():
            raise FileNotFoundError(
                f"TorchScript policy not found: {model_path}"
            )

        self.action_scales = np.asarray(
            self.metadata["action_scales"],
            dtype=np.float32,
        )

        if self.action_scales.size != self.action_size:
            raise ValueError(
                "Action-scale count does not match action size: "
                f"{self.action_scales.size} versus "
                f"{self.action_size}"
            )

        self.policy = torch.jit.load(
            str(model_path),
            map_location="cpu",
        )
        self.policy.eval()

        self.inference_count = 0
        self.last_inference_log_time_ns = 0

        # Verify the model contract immediately.
        dummy_observation = torch.zeros(
            (1, self.observation_size),
            dtype=torch.float32,
        )

        with torch.inference_mode():
            dummy_output = self.policy(dummy_observation)

        dummy_actions = self.extract_actions(dummy_output)

        self.get_logger().info(
            f"Loaded TorchScript policy: {model_path}"
        )
        self.get_logger().info(
            "Policy contract verified: "
            f"input=(1, {self.observation_size}), "
            f"output=(1, {dummy_actions.size})"
        )

        if dry_run:
            self.get_logger().warn(
                "DRY-RUN MODE: no joint commands will be published."
            )

    def extract_actions(
        self,
        output: torch.Tensor,
    ) -> np.ndarray:
        """Validate and convert the network output."""

        if not isinstance(output, torch.Tensor):
            raise TypeError(
                "Expected the policy to return a tensor, "
                f"but received {type(output).__name__}."
            )

        actions = (
            output
            .detach()
            .cpu()
            .numpy()
            .astype(np.float32, copy=False)
            .reshape(-1)
        )

        if actions.size != self.action_size:
            raise ValueError(
                "Policy output has the wrong size: "
                f"{actions.size}, expected {self.action_size}."
            )

        if not np.all(np.isfinite(actions)):
            invalid_indices = np.flatnonzero(
                ~np.isfinite(actions)
            )

            raise ValueError(
                "Policy output contains NaN or infinity at "
                f"indices {invalid_indices.tolist()}."
            )

        return actions

    def odometry_callback(
        self,
        message: Odometry,
    ) -> None:
        """Build one observation and run one policy evaluation."""

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

            actions = self.extract_actions(output)

        except (
            KeyError,
            RuntimeError,
            TypeError,
            ValueError,
        ) as error:
            self.get_logger().error(
                f"Policy dry run failed: {error}"
            )
            return

        target_positions = (
            self.default_joint_positions
            + self.action_scales * actions
        )

        if not np.all(np.isfinite(target_positions)):
            self.get_logger().error(
                "Calculated joint targets contain "
                "NaN or infinity."
            )
            return

        # This reproduces the policy's last-action observation.
        # The actions are stored but are not sent to the robot.
        self.previous_action[:] = actions

        self.inference_count += 1

        current_time_ns = (
            self.get_clock().now().nanoseconds
        )

        if (
            current_time_ns
            - self.last_inference_log_time_ns
            >= 1_000_000_000
        ):
            self.last_inference_log_time_ns = (
                current_time_ns
            )

            self.print_policy_diagnostics(
                observation=observation,
                actions=actions,
                targets=target_positions,
                terms=terms,
                base_height=(
                    message.pose.pose.position.z
                ),
            )

    def print_policy_diagnostics(
        self,
        observation: np.ndarray,
        actions: np.ndarray,
        targets: np.ndarray,
        terms: dict[str, np.ndarray],
        base_height: float,
    ) -> None:
        """Print the network output and proposed joint targets."""

        action_lines = "\n".join(
            (
                f"    {joint_name:5s}: "
                f"action={action:+.4f}  "
                f"scale={scale:.3f}  "
                f"target={target:+.4f}"
            )
            for joint_name, action, scale, target
            in zip(
                self.joint_order,
                actions,
                self.action_scales,
                targets,
                strict=True,
            )
        )

        message = (
            "\n"
            "Spot policy dry run\n"
            f"  inference count: {self.inference_count}\n"
            f"  observation:     shape={observation.shape}, "
            f"min={observation.min():+.3f}, "
            f"max={observation.max():+.3f}\n"
            f"  base height:     {base_height:+.3f} m\n"
            f"  command:         "
            f"{np.array2string(terms['command'], precision=3)}\n"
            f"  actions:         "
            f"min={actions.min():+.3f}, "
            f"max={actions.max():+.3f}, "
            f"mean_abs={np.mean(np.abs(actions)):.3f}\n"
            f"  targets:         "
            f"min={targets.min():+.3f}, "
            f"max={targets.max():+.3f}\n"
            f"{action_lines}"
        )

        self.get_logger().info(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)

    node = SpotPolicyDryRun()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()