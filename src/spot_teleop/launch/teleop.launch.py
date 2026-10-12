from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():

    mode = LaunchConfiguration("mode")

    spot_teleop_share = Path(
        get_package_share_directory("spot_teleop")
    )

    # ---------------------------------------------
    # PS5
    # ---------------------------------------------

    ps5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                spot_teleop_share
                / "launch"
                / "ps5_teleop.launch.py"
            )
        ),
        condition=IfCondition(
            PythonExpression(
                ["'", mode, "' == 'ps5'"]
            )
        ),
    )

    # ---------------------------------------------
    # Keyboard
    # ---------------------------------------------

    keyboard = ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--",
            "bash",
            "-lc",
            (
                "ros2 run teleop_twist_keyboard "
                "teleop_twist_keyboard "
                "--ros-args "
                "-p speed:=0.5 "
                "-p turn:=0.6; "
                "exec bash"
            ),
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", mode, "' == 'keyboard'"]
            )
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="keyboard",
                choices=[
                    "keyboard",
                    "ps5",
                    "none",
                ],
            ),

            ps5,
            keyboard,
        ]
    )