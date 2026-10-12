from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[
                    {
                        "deadzone": 0.08,
                        "autorepeat_rate": 20.0,
                    }
                ],
            ),

            Node(
                package="spot_teleop",
                executable="ps5_teleop",
                name="spot_ps5_teleop",
                output="screen",
            ),
        ]
    )
