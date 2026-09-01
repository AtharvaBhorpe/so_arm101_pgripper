from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE = "so_arm101_pgripper_moveit_config"


def _is_device(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration("input_device"), "' == '", name, "'"]))


def _launch_setup(context):
    port = LaunchConfiguration("port").perform(context)
    joint_config = LaunchConfiguration("joint_config_file").perform(context)
    limits = LaunchConfiguration("joint_limits_file").perform(context)
    confirmation = LaunchConfiguration("confirm_hardware").perform(context)
    for label, value in (("serial port", port), ("joint configuration", joint_config), ("joint limits", limits)):
        if not value or not Path(value).exists():
            raise RuntimeError(f"{label} '{value}' does not exist")
    if confirmation != "MOVE_REAL_ARM":
        raise RuntimeError("Real Servo requires confirm_hardware:=MOVE_REAL_ARM")

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
        .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .to_moveit_configs()
    )
    servo = yaml.safe_load((share / "config" / "servo.yaml").read_text())

    include_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "real.launch.py")),
        launch_arguments={
            "port": port,
            "joint_config_file": joint_config,
            "joint_limits_file": limits,
            "confirm_hardware": confirmation,
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    servo_container = ComposableNodeContainer(
        name="servo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_servo",
                plugin="moveit_servo::ServoNode",
                name="servo_node",
                parameters=[
                    servo,
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.joint_limits,
                ],
            )
        ],
        output="screen",
    )
    inputs = [
        Node(
            package=PACKAGE,
            executable="servo_keyboard_input",
            name="servo_keyboard_input",
            parameters=[{"command_frame": "base_link"}],
            condition=_is_device("keyboard"),
            output="screen",
            emulate_tty=True,
        ),
        Node(package="joy", executable="joy_node", name="joy_node", condition=_is_device("ps4"), output="screen"),
        Node(
            package=PACKAGE,
            executable="servo_joy_input",
            name="servo_joy_input",
            parameters=[str(share / "config" / "ps4.yaml")],
            condition=_is_device("ps4"),
            output="screen",
        ),
    ]
    return [include_real, TimerAction(period=5.0, actions=[servo_container]), TimerAction(period=9.0, actions=inputs)]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value=""),
            DeclareLaunchArgument("joint_config_file", default_value=""),
            DeclareLaunchArgument("joint_limits_file", default_value=""),
            DeclareLaunchArgument("confirm_hardware", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("input_device", default_value="none"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
