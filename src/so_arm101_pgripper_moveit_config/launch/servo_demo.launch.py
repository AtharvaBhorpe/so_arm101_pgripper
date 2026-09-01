from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from moveit_configs_utils import MoveItConfigsBuilder


PACKAGE = "so_arm101_pgripper_moveit_config"


def _is_device(name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration("input_device"), "' == '", name, "'"]))


def generate_launch_description():
    share = Path(get_package_share_directory(PACKAGE))
    moveit_config = (
        MoveItConfigsBuilder("so_arm101_pgripper", package_name=PACKAGE)
        .robot_description(file_path="config/so_arm101_pgripper.urdf.xacro")
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
    servo = yaml.safe_load((share / "config" / "servo.yaml").read_text())

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("input_device", default_value="none"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch" / "demo.launch.py")),
                launch_arguments={"use_rviz": LaunchConfiguration("use_rviz")}.items(),
            ),
            ComposableNodeContainer(
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
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package=PACKAGE,
                        executable="servo_keyboard_input",
                        name="servo_keyboard_input",
                        parameters=[{"command_frame": "base_link"}],
                        condition=_is_device("keyboard"),
                        output="screen",
                        emulate_tty=True,
                    ),
                    Node(
                        package="joy",
                        executable="joy_node",
                        name="joy_node",
                        condition=_is_device("ps4"),
                        output="screen",
                    ),
                    Node(
                        package=PACKAGE,
                        executable="servo_joy_input",
                        name="servo_joy_input",
                        parameters=[str(share / "config" / "ps4.yaml")],
                        condition=_is_device("ps4"),
                        output="screen",
                    ),
                ],
            ),
        ]
    )
