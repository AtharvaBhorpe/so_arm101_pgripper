import importlib.util
import os
import subprocess
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml
import pytest
from launch import LaunchDescription


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / "config"
ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
GRIPPER_JOINTS = ["gripper"]
MIMIC_JOINTS = {"pgripper_left_finger_joint", "pgripper_right_finger_joint"}


def test_moveit_package_files_exist():
    required = {
        "CMakeLists.txt",
        "package.xml",
    }
    missing = {path for path in required if not (PACKAGE / path).is_file()}
    assert missing == set()


def _require(path: Path) -> Path:
    assert path.is_file(), f"required file is missing: {path}"
    return path


def _yaml(name: str):
    return yaml.safe_load(_require(CONFIG / name).read_text())


def test_srdf_defines_a_five_joint_chain_and_one_joint_gripper():
    root = ET.parse(_require(CONFIG / "so_arm101_pgripper.srdf")).getroot()
    groups = {group.attrib["name"]: group for group in root.findall("group")}

    assert groups["arm"].find("chain").attrib == {
        "base_link": "base_link",
        "tip_link": "gripper_frame_link",
    }
    assert [joint.attrib["name"] for joint in groups["gripper"].findall("joint")] == GRIPPER_JOINTS
    assert {joint.attrib["name"] for joint in root.findall("passive_joint")} == MIMIC_JOINTS
    assert root.find("end_effector").attrib == {
        "name": "pgripper",
        "parent_link": "gripper_frame_link",
        "group": "gripper",
        "parent_group": "arm",
    }

    states = {
        state.attrib["name"]: float(state.find("joint").attrib["value"])
        for state in root.findall("group_state[@group='gripper']")
    }
    assert states == {"open": 0.0, "closed": 2.2028}


def test_srdf_zero_state_stays_inside_the_calibrated_shoulder_limit():
    root = ET.parse(_require(CONFIG / "so_arm101_pgripper.srdf")).getroot()
    zero = root.find("group_state[@name='zero'][@group='arm']")
    values = {joint.attrib["name"]: float(joint.attrib["value"]) for joint in zero.findall("joint")}

    assert values == {
        "shoulder_pan": -0.076699,
        "shoulder_lift": -1.70,
        "elbow_flex": 1.560058,
        "wrist_flex": 0.408039,
        "wrist_roll": -0.009204,
    }
    assert values["shoulder_lift"] > -1.74413615583


def test_srdf_allows_intentional_pgripper_assembly_contacts():
    root = ET.parse(_require(CONFIG / "so_arm101_pgripper.srdf")).getroot()
    allowed = {
        frozenset((entry.attrib["link1"], entry.attrib["link2"]))
        for entry in root.findall("disable_collisions")
    }

    assert {
        frozenset(("wrist_link", "pgripper_base_link")),
        frozenset(("pgripper_gear_link", "pgripper_left_jaw_link")),
        frozenset(("pgripper_gear_link", "pgripper_right_jaw_link")),
    } <= allowed


def test_pick_ik_uses_position_and_orientation_costs():
    arm = _yaml("kinematics.yaml")["arm"]

    assert arm["kinematics_solver"] == "pick_ik/PickIkPlugin"
    assert arm["position_scale"] > 0
    assert arm["rotation_scale"] > 0


def test_moveit_and_mock_controllers_use_the_same_5_plus_1_contract():
    moveit = _yaml("moveit_controllers.yaml")["moveit_simple_controller_manager"]
    control = _yaml("ros2_controllers.yaml")

    assert moveit["arm_controller"]["joints"] == ARM_JOINTS
    assert moveit["gripper_controller"]["joints"] == GRIPPER_JOINTS
    assert control["arm_controller"]["ros__parameters"]["joints"] == ARM_JOINTS
    assert control["gripper_controller"]["ros__parameters"]["joints"] == GRIPPER_JOINTS

    controlled = set(ARM_JOINTS + GRIPPER_JOINTS)
    assert controlled.isdisjoint(MIMIC_JOINTS)
    assert control["controller_manager"]["ros__parameters"]["enforce_command_limits"] is True


def test_ompl_adapters_use_ros_parameter_array_types():
    ompl = _yaml("ompl_planning.yaml")

    assert ompl["planning_plugins"] == ["ompl_interface/OMPLPlanner"]
    assert isinstance(ompl["request_adapters"], list)
    assert isinstance(ompl["response_adapters"], list)


def test_moveit_xacro_uses_mock_hardware_and_six_real_motors():
    xacro = _require(CONFIG / "so_arm101_pgripper.urdf.xacro")
    xml = subprocess.run(
        ["xacro", str(xacro)], check=True, capture_output=True, text=True
    ).stdout
    root = ET.fromstring(xml)
    control = root.find("ros2_control")

    assert root.find("link[@name='world']") is not None
    world_joint = root.find("joint[@name='world_joint']")
    assert world_joint.attrib["type"] == "fixed"
    assert world_joint.find("parent").attrib["link"] == "world"
    assert world_joint.find("child").attrib["link"] == "base_link"
    assert control.find("./hardware/plugin").text == "mock_components/GenericSystem"
    assert [joint.attrib["name"] for joint in control.findall("joint")] == ARM_JOINTS + GRIPPER_JOINTS


def test_demo_launch_is_mock_only_and_starts_both_controllers():
    launch_path = _require(PACKAGE / "launch" / "demo.launch.py")
    _require(CONFIG / "moveit.rviz")
    source = launch_path.read_text()

    assert "joint_state_broadcaster" in source
    assert "arm_controller" in source
    assert "gripper_controller" in source
    assert "feetech_ros2_driver" not in source
    assert "/dev/tty" not in source

    log_dir = PACKAGE / ".test-logs"
    log_dir.mkdir(exist_ok=True)
    os.environ["ROS_LOG_DIR"] = str(log_dir)
    spec = importlib.util.spec_from_file_location("moveit_demo_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert isinstance(module.generate_launch_description(), LaunchDescription)


def test_real_launch_is_guarded_and_uses_the_feetech_driver():
    launch_path = _require(PACKAGE / "launch" / "real.launch.py")
    source = launch_path.read_text()

    assert "MOVE_REAL_ARM" in source
    assert "feetech_ros2_driver/FeetechHardwareInterface" in source
    assert "joint_config_file" in source
    assert "joint_limits_file" in source
    assert "arm_controller" in source
    assert "gripper_controller" in source

    spec = importlib.util.spec_from_file_location("moveit_real_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert isinstance(module.generate_launch_description(), LaunchDescription)
    with pytest.raises(RuntimeError, match="MOVE_REAL_ARM"):
        module.validate_inputs(
            "/dev/null",
            str(CONFIG / "joint_limits.yaml"),
            str(CONFIG / "joint_limits.yaml"),
            "",
        )


def test_rviz_loads_motion_planning_as_a_display_only():
    rviz = _yaml("moveit.rviz")
    panel_classes = {panel["Class"] for panel in rviz["Panels"]}
    display_classes = {
        display["Class"] for display in rviz["Visualization Manager"]["Displays"]
    }

    assert "moveit_rviz_plugin/MotionPlanning" not in panel_classes
    assert "moveit_rviz_plugin/MotionPlanning" in display_classes


def test_rviz_enables_approximate_ik_for_moveit_2_12():
    rviz = _yaml("moveit.rviz")
    motion_planning = next(
        display
        for display in rviz["Visualization Manager"]["Displays"]
        if display["Class"] == "moveit_rviz_plugin/MotionPlanning"
    )

    assert motion_planning["MoveIt_Allow_Approximate_IK"] is True
