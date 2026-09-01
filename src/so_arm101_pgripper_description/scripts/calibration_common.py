#!/usr/bin/env python3
"""Pure calibration transformations shared by the hardware calibration CLI."""

from __future__ import annotations

import copy
import math
import os
import tempfile
from pathlib import Path


MOTOR_IDS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6,
}
CENTER_TICK = 2048
LEROBOT_CENTER_TICK = 2047
RAD_PER_TICK = 2 * math.pi / 4096
GRIPPER_LIMIT_TOLERANCE_TICKS = 8
CANONICAL_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (0.0, 2.2028),
}


def _validate(calibration: dict) -> None:
    if set(calibration) != set(MOTOR_IDS):
        raise ValueError(f"calibration must contain exactly these motors: {', '.join(MOTOR_IDS)}")
    for name, expected_id in MOTOR_IDS.items():
        item = calibration[name]
        if item.get("id") != expected_id:
            raise ValueError(f"{name} must use servo id {expected_id}")
        if not 0 <= item["range_min"] < item["range_max"] <= 4095:
            raise ValueError(f"{name} has an invalid tick range")


def build_ros_mapping(calibration: dict) -> tuple[dict, dict]:
    """Create ROS driver data and joint limits from aligned calibration."""
    _validate(calibration)
    driver = {}
    limits = {}

    for name, item in calibration.items():
        if not item["range_min"] <= CENTER_TICK <= item["range_max"]:
            raise ValueError(f"{name} range does not contain tick 2048")

        if name == "gripper":
            if item["drive_mode"] == 0:
                center_tick = item["range_max"]
                direction = -1
            elif item["drive_mode"] == 1:
                center_tick = item["range_min"]
                direction = 1
            else:
                raise ValueError("gripper drive mode must be 0 or 1")
            tolerance = GRIPPER_LIMIT_TOLERANCE_TICKS * RAD_PER_TICK
            limits[name] = {
                "lower": -tolerance,
                "upper": abs(center_tick - CENTER_TICK) * RAD_PER_TICK + tolerance,
            }
        else:
            center_tick = CENTER_TICK
            direction = 1
            limits[name] = {
                "lower": (item["range_min"] - CENTER_TICK) * RAD_PER_TICK,
                "upper": (item["range_max"] - CENTER_TICK) * RAD_PER_TICK,
            }

        driver[name] = {
            "id": item["id"],
            "homing_offset": item["homing_offset"],
            "range_min": item["range_min"],
            "range_max": item["range_max"],
            "center_tick": center_tick,
            "direction": direction,
        }

    return driver, limits


def postprocess_calibration(calibration: dict) -> tuple[dict, dict, dict]:
    """Align LeRobot 0.6.1 calibration with the ROS joint conventions."""
    _validate(calibration)
    result = copy.deepcopy(calibration)

    for name, item in result.items():
        item["homing_offset"] -= CENTER_TICK - LEROBOT_CENTER_TICK
        shifted_min = item["range_min"] + 1
        shifted_max = min(4095, item["range_max"] + 1)

        if name == "gripper":
            near_min = abs(shifted_min - CENTER_TICK)
            near_max = abs(shifted_max - CENTER_TICK)
            if min(near_min, near_max) > 32:
                raise ValueError("gripper sweep has no closed endpoint near tick 2048")
            open_direction = 1 if near_min <= near_max else -1
            open_tick = shifted_max if open_direction == 1 else shifted_min
            max_ticks = int(CANONICAL_LIMITS[name][1] / RAD_PER_TICK)
            open_tick = CENTER_TICK + open_direction * min(abs(open_tick - CENTER_TICK), max_ticks)
            item["range_min"], item["range_max"] = sorted((CENTER_TICK, open_tick))
            item["drive_mode"] = 0 if open_direction == 1 else 1
        else:
            measured_radius = min(CENTER_TICK - shifted_min, shifted_max - CENTER_TICK)
            canonical_radius = int(min(abs(CANONICAL_LIMITS[name][0]), CANONICAL_LIMITS[name][1]) / RAD_PER_TICK)
            radius = min(measured_radius, canonical_radius)
            if radius <= 0:
                raise ValueError(f"{name} range does not contain tick 2048")
            item["range_min"] = CENTER_TICK - radius
            item["range_max"] = CENTER_TICK + radius
            item["drive_mode"] = 0

    driver, limits = build_ros_mapping(result)
    return result, driver, limits


def update_wrist_roll_calibration(
    calibration: dict,
    *,
    homing_offset: int,
    negative_endpoint_tick: int,
    positive_endpoint_tick: int,
    margin_ticks: int = 64,
) -> dict:
    """Set upright wrist zero and safe asymmetric limits from two stops."""
    _validate(calibration)
    if not 0 <= negative_endpoint_tick <= 4095 or not 0 <= positive_endpoint_tick <= 4095:
        raise ValueError("wrist endpoint ticks must be in the range 0..4095")
    if margin_ticks < 0:
        raise ValueError("wrist margin must not be negative")

    negative_delta = negative_endpoint_tick - LEROBOT_CENTER_TICK
    if negative_delta >= 0:
        negative_delta -= 4096
    positive_delta = positive_endpoint_tick - LEROBOT_CENTER_TICK
    if positive_delta <= 0:
        positive_delta += 4096

    canonical_lower = math.ceil(CANONICAL_LIMITS["wrist_roll"][0] / RAD_PER_TICK)
    canonical_upper = math.floor(CANONICAL_LIMITS["wrist_roll"][1] / RAD_PER_TICK)
    lower_ticks = max(negative_delta + margin_ticks, canonical_lower)
    upper_ticks = min(positive_delta - margin_ticks, canonical_upper)
    if lower_ticks >= 0 or upper_ticks <= 0:
        raise ValueError("wrist endpoints do not contain the upright zero with the required margin")

    result = copy.deepcopy(calibration)
    result["wrist_roll"].update(
        {
            "drive_mode": 0,
            "homing_offset": homing_offset - (CENTER_TICK - LEROBOT_CENTER_TICK),
            "range_min": CENTER_TICK + lower_ticks,
            "range_max": CENTER_TICK + upper_ticks,
        }
    )
    _validate(result)
    return result


def _atomic_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as tmp:
        tmp.write(text)
        tmp_path = Path(tmp.name)
    os.replace(tmp_path, path)


def archive_existing_calibration(path: Path, stamp: str) -> Path | None:
    path = Path(path)
    if not path.exists():
        return None
    backup = path.with_name(f"{path.stem}.{stamp}.previous{path.suffix}")
    if backup.exists():
        raise FileExistsError(backup)
    path.replace(backup)
    return backup


def _joint_yaml(joints: dict, *, include_eeprom: bool) -> str:
    lines = ["joints:"]
    for name in MOTOR_IDS:
        item = joints[name]
        lines.extend(
            [
                f"  {name}:",
                f"    id: {item['id']}",
                f"    center_tick: {item['center_tick']}",
                f"    direction: {item['direction']}",
            ]
        )
        if include_eeprom:
            lines.extend(
                [
                    f"    homing_offset: {item['homing_offset']}",
                    f"    range_min: {item['range_min']}",
                    f"    range_max: {item['range_max']}",
                    "    p_coefficient: 16",
                    "    i_coefficient: 0",
                    "    d_coefficient: 32",
                ]
            )
            if name == "gripper":
                lines.extend(
                    [
                        "    max_torque_limit: 500",
                        "    protection_current: 250",
                        "    overload_torque: 25",
                    ]
                )
    return "\n".join(lines) + "\n"


def write_ros_files(directory: Path, robot_id: str, driver: dict, limits: dict) -> tuple[Path, Path, Path]:
    directory = Path(directory)
    state_path = directory / f"{robot_id}.state.yaml"
    control_path = directory / f"{robot_id}.control.yaml"
    limits_path = directory / f"{robot_id}.joint_limits.yaml"

    _atomic_write(state_path, _joint_yaml(driver, include_eeprom=False))
    _atomic_write(control_path, _joint_yaml(driver, include_eeprom=True))
    limit_lines = ["joint_limits:"]
    for name in MOTOR_IDS:
        limit_lines.extend(
            [
                f"  {name}:",
                f"    lower: {limits[name]['lower']:.12g}",
                f"    upper: {limits[name]['upper']:.12g}",
            ]
        )
    _atomic_write(limits_path, "\n".join(limit_lines) + "\n")
    return state_path, control_path, limits_path
