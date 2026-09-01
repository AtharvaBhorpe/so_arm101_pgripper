from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


PACKAGE = "so_arm101_pgripper_moveit_config"


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE))

    return LaunchDescription(
        [
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
            ),
            Node(
                package=PACKAGE,
                executable="servo_joy_input",
                name="servo_joy_input",
                parameters=[str(share / "config" / "ps4.yaml")],
                output="screen",
            ),
        ]
    )
