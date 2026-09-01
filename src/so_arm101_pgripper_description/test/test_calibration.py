import importlib.util
import math
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
MODULE = PACKAGE / "scripts" / "calibration_common.py"
SPEC = importlib.util.spec_from_file_location("calibration_common", MODULE)
calibration_common = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(calibration_common)


def calibration_with_gripper(range_min=2047, range_max=3400):
    return {
        "shoulder_pan": {"id": 1, "drive_mode": 0, "homing_offset": 100, "range_min": 1000, "range_max": 3000},
        "shoulder_lift": {"id": 2, "drive_mode": 0, "homing_offset": -50, "range_min": 900, "range_max": 3200},
        "elbow_flex": {"id": 3, "drive_mode": 0, "homing_offset": 20, "range_min": 1100, "range_max": 3100},
        "wrist_flex": {"id": 4, "drive_mode": 0, "homing_offset": 0, "range_min": 1200, "range_max": 2900},
        "wrist_roll": {"id": 5, "drive_mode": 0, "homing_offset": -20, "range_min": 0, "range_max": 4095},
        "gripper": {"id": 6, "drive_mode": 0, "homing_offset": 5, "range_min": range_min, "range_max": range_max},
    }


def test_postprocess_centers_body_and_maps_increasing_gripper_to_positive_closure():
    lerobot, driver, limits = calibration_common.postprocess_calibration(
        calibration_with_gripper()
    )

    assert lerobot["shoulder_pan"] == {
        "id": 1,
        "drive_mode": 0,
        "homing_offset": 99,
        "range_min": 1095,
        "range_max": 3001,
    }
    assert lerobot["wrist_roll"]["homing_offset"] == -21
    assert lerobot["wrist_roll"]["range_min"] == 260
    assert lerobot["wrist_roll"]["range_max"] == 3836
    assert lerobot["gripper"]["drive_mode"] == 0
    assert lerobot["gripper"]["range_min"] == 2048
    assert lerobot["gripper"]["range_max"] == 3401

    assert driver["gripper"]["center_tick"] == 3401
    assert driver["gripper"]["direction"] == -1
    gripper_span = (3401 - 2048) * 2 * math.pi / 4096
    encoder_tolerance = 3 * 2 * math.pi / 4096
    assert limits["gripper"]["lower"] <= -encoder_tolerance
    assert limits["gripper"]["upper"] >= gripper_span + encoder_tolerance
    assert limits["gripper"]["upper"] * 0.0115 <= 0.0255
    assert limits["shoulder_pan"]["lower"] == pytest.approx(
        -953 * 2 * math.pi / 4096
    )
    assert limits["shoulder_pan"]["upper"] == pytest.approx(
        953 * 2 * math.pi / 4096
    )


def test_postprocess_maps_decreasing_gripper_to_positive_closure():
    lerobot, driver, limits = calibration_common.postprocess_calibration(
        calibration_with_gripper(range_min=700, range_max=2047)
    )

    assert lerobot["gripper"]["drive_mode"] == 1
    assert lerobot["gripper"]["range_min"] == 700 + 1
    assert lerobot["gripper"]["range_max"] == 2048
    assert driver["gripper"]["center_tick"] == 701
    assert driver["gripper"]["direction"] == 1
    gripper_span = (2048 - 701) * 2 * math.pi / 4096
    encoder_tolerance = 3 * 2 * math.pi / 4096
    assert limits["gripper"]["lower"] <= -encoder_tolerance
    assert limits["gripper"]["upper"] >= gripper_span + encoder_tolerance


def test_wrist_roll_recalibration_uses_upright_zero_and_asymmetric_stops():
    aligned, _, _ = calibration_common.postprocess_calibration(calibration_with_gripper())

    updated = calibration_common.update_wrist_roll_calibration(
        aligned,
        homing_offset=-1650,
        negative_endpoint_tick=3990,
        positive_endpoint_tick=3420,
        margin_ticks=64,
    )
    driver, limits = calibration_common.build_ros_mapping(updated)

    canonical_lower_ticks = math.ceil(
        calibration_common.CANONICAL_LIMITS["wrist_roll"][0] / calibration_common.RAD_PER_TICK
    )
    assert updated["wrist_roll"]["homing_offset"] == -1651
    assert updated["wrist_roll"]["range_min"] == 2048 + canonical_lower_ticks
    assert updated["wrist_roll"]["range_max"] == 2048 + (3420 - 2047 - 64)
    assert driver["wrist_roll"]["center_tick"] == 2048
    assert limits["wrist_roll"]["lower"] == pytest.approx(canonical_lower_ticks * 2 * math.pi / 4096)
    assert limits["wrist_roll"]["upper"] == pytest.approx((3420 - 2047 - 64) * 2 * math.pi / 4096)


def test_postprocess_rejects_incomplete_or_bad_gripper_sweep():
    incomplete = calibration_with_gripper()
    del incomplete["wrist_roll"]
    with pytest.raises(ValueError, match="exactly these motors"):
        calibration_common.postprocess_calibration(incomplete)

    with pytest.raises(ValueError, match="closed endpoint"):
        calibration_common.postprocess_calibration(
            calibration_with_gripper(range_min=1500, range_max=2600)
        )


def test_write_ros_files_separates_read_only_mapping_from_eeprom_values(tmp_path):
    _, driver, limits = calibration_common.postprocess_calibration(
        calibration_with_gripper()
    )
    state_path, control_path, limits_path = calibration_common.write_ros_files(
        tmp_path, "lab_follower", driver, limits
    )

    state_text = state_path.read_text()
    control_text = control_path.read_text()
    limits_text = limits_path.read_text()
    assert "center_tick: 2048" in state_text
    assert "direction: 1" in state_text
    assert "homing_offset" not in state_text
    assert "range_min" not in state_text
    assert "homing_offset: 99" in control_text
    assert "protection_current: 250" in control_text
    assert "joint_limits:" in limits_text
    assert "shoulder_pan:" in limits_text
    assert "gripper:" in limits_text


def test_archive_existing_calibration_prevents_double_processing(tmp_path):
    calibration = tmp_path / "lab.json"
    calibration.write_text('{"already": "aligned"}\n')
    backup = calibration_common.archive_existing_calibration(calibration, "20260831T120000Z")

    assert not calibration.exists()
    assert backup.name == "lab.20260831T120000Z.previous.json"
    assert backup.read_text() == '{"already": "aligned"}\n'
    assert calibration_common.archive_existing_calibration(calibration, "later") is None
