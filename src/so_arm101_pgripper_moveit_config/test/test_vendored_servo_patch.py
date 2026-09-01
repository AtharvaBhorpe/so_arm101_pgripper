from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[2]
COMMON = WORKSPACE / "moveit_servo" / "src" / "utils" / "common.cpp"
COMMAND = WORKSPACE / "moveit_servo" / "src" / "utils" / "command.cpp"
CMAKE = WORKSPACE / "moveit_servo" / "CMakeLists.txt"


def test_rectangular_jacobian_uses_svd_dimension():
    source = COMMON.read_text()

    assert "const Eigen::Index dims = current_svd.singularValues().size();" in source
    assert "const size_t dims = target_delta_x.size();" not in source


def test_patched_library_has_a_unique_runtime_name():
    assert "OUTPUT_NAME moveit_servo_lib_cpp_so101" in CMAKE.read_text()


def test_underactuated_groups_use_the_jacobian_path():
    assert "current_joint_positions.size() >= 6" in COMMAND.read_text()
