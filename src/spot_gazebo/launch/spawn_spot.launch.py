"""Spawn Spot in Gazebo with ros2_control."""

from pathlib import Path

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
import xacro


def generate_launch_description() -> LaunchDescription:
    description_share = Path(
        get_package_share_directory("spot_description")
    )

    gazebo_share = Path(
        get_package_share_directory("spot_gazebo")
    )

    ros_gz_share = Path(
        get_package_share_directory("ros_gz_sim")
    )

    xacro_file = (
        description_share
        / "urdf"
        / "spot.urdf.xacro"
    )

    world = LaunchConfiguration("world")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    world_file = PathJoinSubstitution(
        [
            str(gazebo_share),
            "worlds",
            PythonExpression(
                ["'", world, "_world.sdf'"]
            ),
        ]
    )

    controllers_file = (
        gazebo_share
        / "config"
        / "spot_controllers.yaml"
    )

    robot_description = xacro.process_file(
        str(xacro_file),
        mappings={
            "use_gazebo": "true",
            "controllers_file": str(
                controllers_file
            ),
        },
    ).toxml()



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                ros_gz_share
                / "launch"
                / "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": ["-r -v 3 ", world_file],
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    spawn_spot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-world",
            "spot_world",
            "-topic",
            "/robot_description",
            "-name",
            "spot",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "60",
        ],
    )

    spot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "spot_joint_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
            "--switch-timeout",
            "60",
        ],
    )

    start_spot_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                spot_controller_spawner,
            ],
        )
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ]
    )

    odometry_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="spot_odometry_bridge",
        output="screen",
        arguments=[
            "/model/spot/odometry"
            "@nav_msgs/msg/Odometry"
            "[gz.msgs.Odometry",
        ],
        remappings=[
            (
                "/model/spot/odometry",
                "/spot/odom",
            ),
        ],
    )

    scan_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="spot_scan_bridge",
        output="screen",
        arguments=[
            "/scan"
            "@sensor_msgs/msg/LaserScan"
            "[gz.msgs.LaserScan",
        ],
    )

    odom_tf_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="spot_odom_tf_bridge",
        output="screen",
        arguments=[
            "/model/spot/pose"
            "@tf2_msgs/msg/TFMessage"
            "[gz.msgs.Pose_V",
        ],
        remappings=[
            (
                "/model/spot/pose",
                "/tf",
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="flat",
                choices=["flat", "warehouse"],
            ),
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="0.0",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.46",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="0.0",
            ),

            gazebo,
            robot_state_publisher,
            spawn_spot,
            joint_state_broadcaster_spawner,
            start_spot_controller,
            clock_bridge,
            odometry_bridge,
            scan_bridge,
            odom_tf_bridge,
        ]
    )

