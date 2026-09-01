import importlib.util
from pathlib import Path

from launch import LaunchDescription
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config"
ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def _yaml(name: str):
    path = CONFIG / name
    assert path.is_file(), f"required file is missing: {path}"
    return yaml.safe_load(path.read_text())


def test_servo_uses_the_five_joint_arm_and_existing_controller():
    servo = _yaml("servo.yaml")["moveit_servo"]

    assert servo["move_group_name"] == "arm"
    assert servo["command_out_type"] == "trajectory_msgs/JointTrajectory"
    assert servo["command_out_topic"] == "/arm_controller/joint_trajectory"
    assert servo["cartesian_command_in_topic"] == "/servo_node/delta_twist_cmds"
    assert servo["joint_command_in_topic"] == "/servo_node/delta_joint_cmds"
    assert servo["incoming_command_timeout"] <= 0.1
    assert "robot_link_command_frame" not in servo
    assert "planning_frame" not in servo
    assert servo["lower_singularity_threshold"] == 30.0
    assert servo["hard_stop_singularity_threshold"] == 60.0
    assert servo["self_collision_proximity_threshold"] == 0.0001


def test_servo_launch_is_mock_only_and_keeps_gripper_separate():
    launch_path = PACKAGE / "launch" / "servo_demo.launch.py"
    assert launch_path.is_file()
    source = launch_path.read_text()

    assert "demo.launch.py" in source
    assert "moveit_servo" in source
    assert "arm_controller" in _yaml("ros2_controllers.yaml")
    assert _yaml("ros2_controllers.yaml")["gripper_controller"]["ros__parameters"]["joints"] == ["gripper"]
    assert "feetech_ros2_driver" not in source
    assert "/dev/tty" not in source
    assert "ros2 service call" not in source
    servo_parameters = source.split("parameters=[", 1)[1].split("]", 1)[0]
    assert "robot_description_kinematics" in servo_parameters


def test_servo_launch_module_loads():
    launch_path = PACKAGE / "launch" / "servo_demo.launch.py"
    assert launch_path.is_file()
    spec = importlib.util.spec_from_file_location("servo_demo_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert callable(module.generate_launch_description)


def test_real_servo_launch_is_guarded_and_uses_real_hardware():
    launch_path = PACKAGE / "launch" / "servo_real.launch.py"
    assert launch_path.is_file()
    source = launch_path.read_text()

    assert "MOVE_REAL_ARM" in source
    assert "so_arm101_pgripper_hardware/SoArm101PgripperHardwareInterface" in source
    assert "real.launch.py" in source
    assert "moveit_servo::ServoNode" in source
    assert '"command_hardware": "true"' in source

    spec = importlib.util.spec_from_file_location("servo_real_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert isinstance(module.generate_launch_description(), LaunchDescription)


def test_ps4_input_launch_contains_only_input_nodes():
    launch_path = PACKAGE / "launch" / "servo_ps4_input.launch.py"
    source = launch_path.read_text()

    assert 'package="joy"' in source
    assert 'executable="joy_node"' in source
    assert 'executable="servo_joy_input"' in source
    assert "move_group" not in source
    assert "ServoNode" not in source


def test_joint_contract_remains_five_plus_one():
    controllers = _yaml("ros2_controllers.yaml")

    assert controllers["arm_controller"]["ros__parameters"]["joints"] == ARM_JOINTS
    assert controllers["arm_controller"]["ros__parameters"]["allow_nonzero_velocity_at_trajectory_end"] is True
    assert controllers["gripper_controller"]["ros__parameters"]["joints"] == ["gripper"]
