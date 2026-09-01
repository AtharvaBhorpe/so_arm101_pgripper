from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def validate_inputs(port: str, joint_config: str, limits: str, read_only: bool, confirmation: str) -> None:
    for label, value in (("serial port", port), ("joint config", joint_config), ("joint limits", limits)):
        if not value or not Path(value).exists():
            raise RuntimeError(f"{label} '{value}' does not exist")
    if not read_only and confirmation != "MOVE_REAL_ARM":
        raise RuntimeError("Motion requires confirm_hardware:=MOVE_REAL_ARM")


def _launch_setup(context):
    port = LaunchConfiguration("port").perform(context)
    joint_config = LaunchConfiguration("joint_config_file").perform(context)
    limits = LaunchConfiguration("joint_limits_file").perform(context)
    read_only = _as_bool(LaunchConfiguration("read_only").perform(context))
    confirmation = LaunchConfiguration("confirm_hardware").perform(context)
    validate_inputs(port, joint_config, limits, read_only, confirmation)

    share = Path(get_package_share_directory("so_arm101_pgripper_description"))
    description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ", str(share / "urdf" / "so_arm101_pgripper.urdf.xacro"),
                " use_ros2_control:=true",
                " ros2_control_plugin:=so_arm101_pgripper_hardware/SoArm101PgripperHardwareInterface",
                f" usb_port:={port}",
                f" joint_config_file:={joint_config}",
                f" joint_limits_file:={limits}",
                f" command_hardware:={str(not read_only).lower()}",
            ]
        )
    }
    nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[description, str(share / "config" / "ros2_controllers.yaml")],
            output="screen",
        ),
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "config" / "display.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]
    if not read_only:
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value=""),
            DeclareLaunchArgument("joint_config_file", default_value=""),
            DeclareLaunchArgument("joint_limits_file", default_value=""),
            DeclareLaunchArgument("read_only", default_value="true"),
            DeclareLaunchArgument("confirm_hardware", default_value=""),
            DeclareLaunchArgument("rviz", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
