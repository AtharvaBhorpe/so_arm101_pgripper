import subprocess
import struct
from pathlib import Path
from xml.etree import ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
XACRO = PACKAGE / "urdf" / "so_arm101_pgripper.urdf.xacro"
ACTUATED = {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
}


def _mesh_x_bounds(path):
    data = path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    vertices = [
        struct.unpack_from("<fff", data, 84 + triangle * 50 + 12 + vertex * 12)
        for triangle in range(triangle_count)
        for vertex in range(3)
    ]
    return min(vertex[0] for vertex in vertices) * 0.001, max(vertex[0] for vertex in vertices) * 0.001


def test_description_tree_limits_mimics_and_meshes():
    xml = subprocess.run(
        ["xacro", str(XACRO)], check=True, capture_output=True, text=True
    ).stdout
    robot = ET.fromstring(xml)
    links = {link.attrib["name"] for link in robot.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    assert robot.attrib["name"] == "so_arm101_pgripper"
    assert ACTUATED <= joints.keys()
    assert len(links) == len(robot.findall("link"))
    assert len(joints) == len(robot.findall("joint"))

    children = [joint.find("child").attrib["link"] for joint in joints.values()]
    assert set(children) == links - {"base_link"}
    assert len(children) == len(set(children))

    for name in ACTUATED:
        joint = joints[name]
        limit = joint.find("limit")
        assert joint.attrib["type"] == "revolute"
        assert float(limit.attrib["lower"]) < float(limit.attrib["upper"])
        assert float(limit.attrib["effort"]) > 0
        assert float(limit.attrib["velocity"]) > 0

    left = joints["pgripper_left_finger_joint"].find("mimic")
    right = joints["pgripper_right_finger_joint"].find("mimic")
    assert left.attrib == {"joint": "gripper", "multiplier": "-0.0115", "offset": "0"}
    assert right.attrib == {"joint": "gripper", "multiplier": "0.0115", "offset": "0"}

    prefix = "package://so_arm101_pgripper_description/"
    for mesh in robot.findall(".//mesh"):
        uri = mesh.attrib["filename"]
        assert uri.startswith(prefix)
        assert (PACKAGE / uri.removeprefix(prefix)).is_file()


def test_pgripper_mount_matches_the_physical_wrist_orientation():
    xml = subprocess.run(
        ["xacro", str(XACRO)], check=True, capture_output=True, text=True
    ).stdout
    mount = ET.fromstring(xml).find("joint[@name='pgripper_mount_joint']/origin")

    assert mount.attrib["rpy"] == "-1.57079632679 0 -1.57079632679"


def test_positive_gripper_position_closes_the_rendered_jaws():
    xml = subprocess.run(
        ["xacro", str(XACRO)], check=True, capture_output=True, text=True
    ).stdout
    robot = ET.fromstring(xml)
    gripper_closed = float(robot.find("joint[@name='gripper']/limit").attrib["upper"])

    centers = {0.0: {}, gripper_closed: {}}
    for side, mesh_name in (("left", "Gripper_Jaw_02_v1_1.stl"), ("right", "Gripper_Jaw_01_v1_1.stl")):
        joint = robot.find(f"joint[@name='pgripper_{side}_finger_joint']")
        origin_x = float(joint.find("origin").attrib["xyz"].split()[0])
        axis_x = float(joint.find("axis").attrib["xyz"].split()[0])
        multiplier = float(joint.find("mimic").attrib["multiplier"])
        mesh = robot.find(f"link[@name='pgripper_{side}_jaw_link']/visual")
        mesh_origin_x = float(mesh.find("origin").attrib["xyz"].split()[0])
        mesh_min, mesh_max = _mesh_x_bounds(PACKAGE / "meshes" / "pgripper" / mesh_name)
        mesh_center_x = mesh_origin_x + (mesh_min + mesh_max) / 2
        for position in centers:
            centers[position][side] = origin_x + axis_x * multiplier * position + mesh_center_x

    open_distance = abs(centers[0.0]["right"] - centers[0.0]["left"])
    closed_distance = abs(centers[gripper_closed]["right"] - centers[gripper_closed]["left"])
    assert closed_distance < open_distance


def test_ros2_control_exposes_only_real_motors():
    xml = subprocess.run(
        ["xacro", str(XACRO), "use_ros2_control:=true"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    control = ET.fromstring(xml).find("ros2_control")
    assert control is not None
    assert {joint.attrib["name"] for joint in control.findall("joint")} == ACTUATED


def test_real_state_only_control_has_ids_mapping_and_no_commands(tmp_path):
    limits = tmp_path / "limits.yaml"
    limits.write_text(
        "joint_limits:\n"
        + "".join(f"  {name}:\n    lower: -1.0\n    upper: 1.0\n" for name in ACTUATED)
    )
    xml = subprocess.run(
        [
            "xacro",
            str(XACRO),
            "use_ros2_control:=true",
            "ros2_control_plugin:=so_arm101_pgripper_hardware/SoArm101PgripperHardwareInterface",
            "usb_port:=/dev/test-arm",
            "joint_config_file:=/tmp/state.yaml",
            f"joint_limits_file:={limits}",
            "command_hardware:=false",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    robot = ET.fromstring(xml)
    control = robot.find("ros2_control")
    params = {param.attrib["name"]: param.text for param in control.findall("./hardware/param")}
    assert params == {"usb_port": "/dev/test-arm", "joint_config_file": "/tmp/state.yaml"}
    assert [joint.find("param[@name='id']").text for joint in control.findall("joint")] == [
        "1", "2", "3", "4", "5", "6"
    ]
    assert not control.findall("./joint/command_interface")
    assert robot.find("joint[@name='shoulder_pan']/limit").attrib["lower"] == "-1.0"


def test_real_motion_control_exposes_position_commands():
    xml = subprocess.run(
        [
            "xacro",
            str(XACRO),
            "use_ros2_control:=true",
            "command_hardware:=true",
        ],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    control = ET.fromstring(xml).find("ros2_control")
    assert len(control.findall("./joint/command_interface[@name='position']")) == 6
