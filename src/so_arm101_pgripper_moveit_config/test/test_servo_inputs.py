import importlib.util
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
ARM_JOINTS = {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
}


def _module(name: str):
    path = SCRIPTS / f"{name}.py"
    assert path.is_file(), f"required file is missing: {path}"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_keyboard_cartesian_map_has_six_axes():
    keyboard = _module("servo_keyboard_input")

    assert keyboard.twist_for_key("w") == (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert keyboard.twist_for_key("l") == (0.0, 0.0, 0.0, 0.0, 0.0, -1.0)
    assert keyboard.twist_for_key("?") is None


def test_keyboard_joint_map_contains_only_five_arm_joints():
    keyboard = _module("servo_keyboard_input")

    mapped = {keyboard.joint_for_key(key)[0] for key in "12345qwert"}
    assert mapped == ARM_JOINTS
    assert keyboard.joint_for_key("6") is None


def test_gripper_toggle_uses_measured_position_in_both_inputs():
    keyboard = _module("servo_keyboard_input")
    gamepad = _module("servo_joy_input")

    for module in (keyboard, gamepad):
        assert module.next_gripper_position(0.0) == module.GRIPPER_CLOSED_POSITION
        assert module.next_gripper_position(module.GRIPPER_CLOSED_POSITION) == module.GRIPPER_OPEN_POSITION


def test_ps4_requires_deadman_and_selects_one_command_type():
    gamepad = _module("servo_joy_input")
    axes = [0.25, -0.5, 1.0, -0.75, 0.5, -1.0]
    buttons = [0] * 14

    assert gamepad.command_from_joy(axes, buttons) is None

    buttons[4] = 1
    command = gamepad.command_from_joy(axes, buttons)
    assert command.kind == "twist"
    assert len(command.values) == 6

    buttons[5] = 1
    command = gamepad.command_from_joy(axes, buttons)
    assert command.kind == "joint"
    assert command.names == tuple(sorted(ARM_JOINTS))


def test_ps4_joint_map_never_contains_the_gripper():
    gamepad = _module("servo_joy_input")
    buttons = [0] * 14
    buttons[4] = 1
    buttons[5] = 1
    command = gamepad.command_from_joy([0.0] * 6, buttons)

    assert "gripper" not in command.names
    assert all("finger" not in name for name in command.names)


def test_ps4_gripper_toggle_requires_deadman_and_a_button_edge():
    gamepad = _module("servo_joy_input")
    buttons = [0] * 14

    buttons[0] = 1
    assert not gamepad.gripper_toggle_pressed(buttons, previous_down=False)
    buttons[4] = 1
    assert gamepad.gripper_toggle_pressed(buttons, previous_down=False)
    assert not gamepad.gripper_toggle_pressed(buttons, previous_down=True)


def test_input_nodes_wait_for_a_successful_mode_switch_response():
    for name in ("servo_keyboard_input", "servo_joy_input"):
        source = (SCRIPTS / f"{name}.py").read_text()
        assert "self.pending_mode" in source
        assert "future.add_done_callback" in source
        assert "if response.success:" in source
