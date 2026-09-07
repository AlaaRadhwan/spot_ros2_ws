#!/usr/bin/env python3
"""Interactive joint-reference GUI for the Spot PID controller."""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk

import rclpy
from control_msgs.msg import MultiDOFCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    "fl_hx",
    "fl_hy",
    "fl_kn",
    "fr_hx",
    "fr_hy",
    "fr_kn",
    "hl_hx",
    "hl_hy",
    "hl_kn",
    "hr_hx",
    "hr_hy",
    "hr_kn",
]

NOMINAL_POSITIONS = {
    "fl_hx": 0.0,
    "fl_hy": 1.04,
    "fl_kn": -1.80,
    "fr_hx": 0.0,
    "fr_hy": 1.04,
    "fr_kn": -1.80,
    "hl_hx": 0.0,
    "hl_hy": 1.04,
    "hl_kn": -1.80,
    "hr_hx": 0.0,
    "hr_hy": 1.04,
    "hr_kn": -1.80,
}

# Keep the GUI slightly inside the URDF limits.
JOINT_LIMITS = {
    "fl_hx": (-0.75, 0.75),
    "fl_hy": (-0.85, 2.20),
    "fl_kn": (-2.75, -0.30),
    "fr_hx": (-0.75, 0.75),
    "fr_hy": (-0.85, 2.20),
    "fr_kn": (-2.75, -0.30),
    "hl_hx": (-0.75, 0.75),
    "hl_hy": (-0.85, 2.20),
    "hl_kn": (-2.75, -0.30),
    "hr_hx": (-0.75, 0.75),
    "hr_hy": (-0.85, 2.20),
    "hr_kn": (-2.75, -0.30),
}


class SpotJointCommandNode(Node):
    """Publish joint references and receive measured joint states."""

    def __init__(self) -> None:
        super().__init__("spot_joint_command_gui")

        self.publisher = self.create_publisher(
            MultiDOFCommand,
            "/spot_joint_controller/reference",
            10,
        )

        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

        self.measured_positions: dict[str, float] = {}
        self.received_first_state = False

    def _joint_state_callback(self, message: JointState) -> None:
        for name, position in zip(
            message.name,
            message.position,
            strict=False,
        ):
            if name in JOINT_NAMES:
                self.measured_positions[name] = float(position)

        if all(
            name in self.measured_positions
            for name in JOINT_NAMES
        ):
            self.received_first_state = True

    def publish_targets(
        self,
        targets: dict[str, float],
    ) -> None:
        message = MultiDOFCommand()

        message.dof_names = JOINT_NAMES
        message.values = [
            float(targets[name])
            for name in JOINT_NAMES
        ]
        message.values_dot = [0.0] * len(JOINT_NAMES)

        self.publisher.publish(message)


class SpotJointGui:
    """Tkinter interface for manually commanding all Spot joints."""

    def __init__(
        self,
        root: tk.Tk,
        node: SpotJointCommandNode,
    ) -> None:
        self.root = root
        self.node = node

        self.root.title("Spot Joint Controller")
        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.close,
        )

        self.variables: dict[str, tk.DoubleVar] = {}
        self.measured_labels: dict[str, ttk.Label] = {}

        self.initialized_from_robot = False
        self.publishing_enabled = tk.BooleanVar(value=False)

        self.status_variable = tk.StringVar(
            value="Waiting for /joint_states..."
        )

        self._create_widgets()

        # ROS processing and publishing at approximately 50 Hz.
        self.root.after(20, self._update)

    def _create_widgets(self) -> None:
        main_frame = ttk.Frame(
            self.root,
            padding=12,
        )
        main_frame.grid(
            row=0,
            column=0,
            sticky="nsew",
        )

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        ttk.Label(
            main_frame,
            text="Joint",
        ).grid(row=0, column=0, padx=5)

        ttk.Label(
            main_frame,
            text="Target",
        ).grid(row=0, column=1, padx=5)

        ttk.Label(
            main_frame,
            text="Measured",
        ).grid(row=0, column=2, padx=5)

        for row, joint_name in enumerate(
            JOINT_NAMES,
            start=1,
        ):
            lower, upper = JOINT_LIMITS[joint_name]

            variable = tk.DoubleVar(
                value=NOMINAL_POSITIONS[joint_name]
            )
            self.variables[joint_name] = variable

            ttk.Label(
                main_frame,
                text=joint_name,
                width=8,
            ).grid(
                row=row,
                column=0,
                padx=5,
                pady=2,
                sticky="w",
            )

            slider_frame = ttk.Frame(main_frame)
            slider_frame.grid(
                row=row,
                column=1,
                padx=5,
                pady=2,
                sticky="ew",
            )

            slider_frame.columnconfigure(0, weight=1)

            slider = ttk.Scale(
                slider_frame,
                from_=lower,
                to=upper,
                variable=variable,
                orient="horizontal",
                length=340,
            )
            slider.grid(
                row=0,
                column=0,
                sticky="ew",
            )

            value_label = ttk.Label(
                slider_frame,
                width=7,
                textvariable=variable,
            )
            value_label.grid(
                row=0,
                column=1,
                padx=(8, 0),
            )

            measured_label = ttk.Label(
                main_frame,
                text="---",
                width=8,
            )
            measured_label.grid(
                row=row,
                column=2,
                padx=5,
            )
            self.measured_labels[joint_name] = measured_label

        button_row = len(JOINT_NAMES) + 1

        controls = ttk.Frame(main_frame)
        controls.grid(
            row=button_row,
            column=0,
            columnspan=3,
            pady=(12, 4),
        )

        ttk.Button(
            controls,
            text="Use Current Pose",
            command=self.use_current_pose,
        ).grid(row=0, column=0, padx=4)

        ttk.Button(
            controls,
            text="Nominal Pose",
            command=self.use_nominal_pose,
        ).grid(row=0, column=1, padx=4)

        ttk.Checkbutton(
            controls,
            text="Enable Commands",
            variable=self.publishing_enabled,
        ).grid(row=0, column=2, padx=12)

        ttk.Label(
            main_frame,
            textvariable=self.status_variable,
        ).grid(
            row=button_row + 1,
            column=0,
            columnspan=3,
            pady=(6, 0),
        )

        main_frame.columnconfigure(1, weight=1)

    def use_current_pose(self) -> None:
        if not self.node.received_first_state:
            self.status_variable.set(
                "No complete joint-state message received yet."
            )
            return

        for joint_name in JOINT_NAMES:
            self.variables[joint_name].set(
                self.node.measured_positions[joint_name]
            )

        self.status_variable.set(
            "Targets copied from measured joint positions."
        )

    def use_nominal_pose(self) -> None:
        for joint_name in JOINT_NAMES:
            self.variables[joint_name].set(
                NOMINAL_POSITIONS[joint_name]
            )

        self.status_variable.set(
            "Nominal pose loaded."
        )

    def _update(self) -> None:
        rclpy.spin_once(
            self.node,
            timeout_sec=0.0,
        )

        if self.node.received_first_state:
            for joint_name in JOINT_NAMES:
                measured = self.node.measured_positions[
                    joint_name
                ]

                self.measured_labels[joint_name].configure(
                    text=f"{measured:+.3f}"
                )

            if not self.initialized_from_robot:
                self.use_current_pose()
                self.initialized_from_robot = True
                self.status_variable.set(
                    "Ready. Enable Commands to begin."
                )

        if self.publishing_enabled.get():
            targets = {
                joint_name: variable.get()
                for joint_name, variable
                in self.variables.items()
            }

            self.node.publish_targets(targets)

        self.root.after(20, self._update)

    def close(self) -> None:
        self.publishing_enabled.set(False)
        self.root.destroy()


def main() -> None:
    rclpy.init()

    node = SpotJointCommandNode()
    root = tk.Tk()
    gui = SpotJointGui(root, node)

    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()