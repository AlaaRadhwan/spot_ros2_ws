from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    spot_gazebo_share = Path(
        get_package_share_directory("spot_gazebo")
    )

    spot_navigation_share = Path(
        get_package_share_directory("spot_navigation")
    )

    slam_toolbox_share = Path(
        get_package_share_directory("slam_toolbox")
    )

    teleop_mode = LaunchConfiguration("teleop")

    # Gazebo + Spot

    gazebo_spot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                spot_gazebo_share
                / "launch"
                / "spawn_spot.launch.py"
            )
        ),
        launch_arguments={
            "world": "warehouse",
            "spawn_x": "0.0",
            "spawn_y": "0.0",
            "spawn_z": "0.46",
            "spawn_yaw": "0.0",
        }.items()
    )

    # RL locomotion policy

    policy = Node(
        package="spot_policy",
        executable="policy_controller",
        name="spot_policy_controller",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    )

    # PS5 teleoperation

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(
                    get_package_share_directory(
                        "spot_teleop"
                    )
                )
                / "launch"
                / "teleop.launch.py"
            )
        ),
        launch_arguments={
            "mode": teleop_mode,
        }.items(),
    )
    # SLAM Toolbox

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                slam_toolbox_share
                / "launch"
                / "online_async_launch.py"
            )
        ),
        launch_arguments={
            "slam_params_file": str(
                spot_navigation_share
                / "config"
                / "spot_slam.yaml"
            ),
            "use_sim_time": "true",
        }.items(),
    )

    # RViz

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "teleop",
                default_value="keyboard",
                choices=[
                    "keyboard",
                    "ps5",
                    "none",
                ],
            ),

            gazebo_spot,
            policy,

            TimerAction(
                period=5.0,
                actions=[
                    teleop,
                ],
            ),

            TimerAction(
                period=8.0,
                actions=[
                    slam,
                    rviz,
                ],
            ),
        ]
    )

