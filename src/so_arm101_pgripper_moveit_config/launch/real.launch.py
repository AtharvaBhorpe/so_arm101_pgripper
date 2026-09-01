from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE = "so_arm101_pgripper_moveit_config"


def validate_inputs(port: str, joint_config: str, limits: str, confirmation: str) -> None:
    for label, value in (("serial port", port), ("joint configuration", joint_config), ("joint limits", limits)):
        if not value or not Path(value).exists():
            raise RuntimeError(f"{label} '{value}' does not exist")
    if confirmation != "MOVE_REAL_ARM":
        raise RuntimeError("Real MoveIt requires confirm_hardware:=MOVE_REAL_ARM")


def _launch_setup(context):
    port = LaunchConfiguration("port").perform(context)
    joint_config = LaunchConfiguration("joint_config_file").perform(context)
    limits = LaunchConfiguration("joint_limits_file").perform(context)
    confirmation = LaunchConfiguration("confirm_hardware").perform(context)
    validate_inputs(port, joint_config, limits, confirmation)

    share = Path(get_package_share_directory(PACKAGE))
    moveit_config = (
        MoveItConfigsBuilder("so_arm101_pgripper", package_name=PACKAGE)
        .robot_description(
            file_path="config/so_arm101_pgripper.urdf.xacro",
            mappings={
                "ros2_control_plugin": "so_arm101_pgripper_hardware/SoArm101PgripperHardwareInterface",
                "usb_port": port,
                "joint_config_file": joint_config,
                "joint_limits_file": limits,
                "command_hardware": "true",
            },
        )
        .robot_description_semantic(file_path="config/so_arm101_pgripper.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    controller_file = str(share / "config" / "ros2_controllers.yaml")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, controller_file],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arm_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[moveit_config.to_dict()],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "config" / "moveit.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
            ],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            output="log",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value=""),
            DeclareLaunchArgument("joint_config_file", default_value=""),
            DeclareLaunchArgument("joint_limits_file", default_value=""),
            DeclareLaunchArgument("confirm_hardware", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
