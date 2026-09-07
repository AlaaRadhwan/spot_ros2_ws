from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_share = Path(get_package_share_directory('spot_description'))

    xacro_file = package_share / 'urdf' / 'spot.urdf.xacro'

    robot_description = {"robot_description": xacro.process_file(str(xacro_file)).toxml()}

    initial_positions = {
        "zeros.fl_hx": 0.0,
        "zeros.fl_hy": 1.04,
        "zeros.fl_kn": -1.80,

        "zeros.fr_hx": 0.0,
        "zeros.fr_hy": 1.04,
        "zeros.fr_kn": -1.80,

        "zeros.hl_hx": 0.0,
        "zeros.hl_hy": 1.04,
        "zeros.hl_kn": -1.80,

        "zeros.hr_hx": 0.0,
        "zeros.hr_hy": 1.04,
        "zeros.hr_kn": -1.80,
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        parameters=[robot_description, initial_positions],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )