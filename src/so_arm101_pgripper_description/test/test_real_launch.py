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
