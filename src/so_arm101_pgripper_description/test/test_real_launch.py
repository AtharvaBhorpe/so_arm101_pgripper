import importlib.util
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch" / "real.launch.py"
SPEC = importlib.util.spec_from_file_location("real_launch", LAUNCH)
real_launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(real_launch)


def test_real_launch_requires_existing_inputs_and_motion_confirmation(tmp_path):
    port = tmp_path / "ttyARM"
    state = tmp_path / "robot.state.yaml"
    limits = tmp_path / "robot.joint_limits.yaml"
    for path in (port, state, limits):
        path.touch()

    real_launch.validate_inputs(str(port), str(state), str(limits), True, "")
    with pytest.raises(RuntimeError, match="MOVE_REAL_ARM"):
        real_launch.validate_inputs(str(port), str(state), str(limits), False, "")
    real_launch.validate_inputs(str(port), str(state), str(limits), False, "MOVE_REAL_ARM")
    with pytest.raises(RuntimeError, match="does not exist"):
        real_launch.validate_inputs(str(tmp_path / "missing"), str(state), str(limits), True, "")


@pytest.mark.parametrize("filename", ["display.launch.py", "real.launch.py"])
def test_robot_description_is_evaluated_as_xml_string(filename, tmp_path, monkeypatch):
    from launch import LaunchContext
    from launch_ros.actions import Node
    from launch_ros.utilities import evaluate_parameters, normalize_parameters

    spec = importlib.util.spec_from_file_location("description_launch", PACKAGE / "launch" / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    parameters = []

    def capture_node(**kwargs):
        parameters.extend(p for p in kwargs.get("parameters", []) if isinstance(p, dict))
        return Node(**kwargs)

    monkeypatch.setattr(module, "Node", capture_node)
    context = LaunchContext()
    context.launch_configurations.update(
        use_ros2_control="false", port=str(tmp_path), joint_config_file=str(tmp_path),
        joint_limits_file=str(PACKAGE / "config" / "canonical_joint_limits.yaml"),
        read_only="true", confirm_hardware="",
    )
    if filename == "display.launch.py":
        module.generate_launch_description()
    else:
        module._launch_setup(context)
    assert parameters
    for parameter in parameters:
        value = evaluate_parameters(context, normalize_parameters([parameter]))[0]["robot_description"]
        assert isinstance(value, str)
        assert '<link name="camera_optical_frame"' in value
