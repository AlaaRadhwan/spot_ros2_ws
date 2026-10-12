from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    spot_gazebo_share = Path(
        get_package_share_directory("spot_gazebo")
    )

    spot_navigation_share = Path(
        get_package_share_directory("spot_navigation")
    )

    nav2_share = Path(
        get_package_share_directory("nav2_bringup")
    )

    # --------------------------------------------------
    # Gazebo + Spot
    # --------------------------------------------------

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
        }.items(),
    )

    # --------------------------------------------------
    # RL locomotion policy
    # --------------------------------------------------

    policy = Node(
        package="spot_policy",
        executable="policy_controller",
        name="spot_policy_controller",
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    )

    # --------------------------------------------------
    # Saved-map localization: map_server + AMCL
    # --------------------------------------------------

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                nav2_share
                / "launch"
                / "localization_launch.py"
            )
        ),
        launch_arguments={
            "map": str(
                spot_navigation_share
                / "maps"
                / "warehouse_map.yaml"
            ),
            "params_file": str(
                spot_navigation_share
                / "config"
                / "spot_localization.yaml"
            ),
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    # --------------------------------------------------
    # Nav2
    # --------------------------------------------------

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                nav2_share
                / "launch"
                / "navigation_launch.py"
            )
        ),
        launch_arguments={
            "params_file": str(
                spot_navigation_share
                / "config"
                / "spot_nav2.yaml"
            ),
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
    )

    # --------------------------------------------------
    # Nav2 RViz
    # --------------------------------------------------

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            str(
                nav2_share
                / "rviz"
                / "nav2_default_view.rviz"
            ),
        ],
        output="screen",
        parameters=[
            {"use_sim_time": True}
        ],
    )

    return LaunchDescription(
        [
            gazebo_spot,
            policy,
            localization,
            navigation,
            rviz,
        ]
    )