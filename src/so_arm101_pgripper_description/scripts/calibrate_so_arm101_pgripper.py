#!/usr/bin/env python3
"""Run official LeRobot calibration, then align its result with ROS/URDF."""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path

from calibration_common import (
    _atomic_write,
    archive_existing_calibration,
    build_ros_mapping,
    postprocess_calibration,
    update_wrist_roll_calibration,
    write_ros_files,
)


def _actual_tick(present_tick: int, homing_offset: int) -> int:
    return (present_tick + homing_offset) % 4096


def recalibrate_wrist_roll(args: argparse.Namespace) -> int:
    """Re-zero only wrist roll at the upright jaw pose and record both stops."""
    from lerobot.motors import MotorCalibration
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    directory = args.calibration_dir.resolve()
    json_path = directory / f"{args.robot_id}.json"
    if not json_path.is_file():
        raise FileNotFoundError(f"Existing calibration is required: {json_path}")
    aligned = json.loads(json_path.read_text())
    build_ros_mapping(aligned)

    robot = SO101Follower(
        SO101FollowerConfig(port=args.port, id=args.robot_id, calibration_dir=directory)
    )
    robot.bus.connect()
    try:
        robot.bus.disable_torque()
        previous = robot.bus.read_calibration()["wrist_roll"]
        expected = MotorCalibration(**aligned["wrist_roll"])
        if asdict(previous) != asdict(expected):
            raise RuntimeError(
                "Motor 5 EEPROM does not match the saved calibration. "
                f"EEPROM={asdict(previous)}, JSON={asdict(expected)}"
            )

        input("Place the jaws visually upright, then press ENTER: ")
        upright_present = int(
            robot.bus.sync_read("Present_Position", ["wrist_roll"], normalize=False, num_retry=5)[
                "wrist_roll"
            ]
        )
        upright_actual = _actual_tick(upright_present, previous.homing_offset)
        raw_homing_offset = upright_actual - 2047

        input("Move wrist roll gently counterclockwise to its physical stop, then press ENTER: ")
        negative_present = int(
            robot.bus.sync_read("Present_Position", ["wrist_roll"], normalize=False, num_retry=5)[
                "wrist_roll"
            ]
        )
        negative_actual = _actual_tick(negative_present, previous.homing_offset)
        negative_tick = (negative_actual - raw_homing_offset) % 4096

        input("Move wrist roll gently clockwise to its physical stop, then press ENTER: ")
        positive_present = int(
            robot.bus.sync_read("Present_Position", ["wrist_roll"], normalize=False, num_retry=5)[
                "wrist_roll"
            ]
        )
        positive_actual = _actual_tick(positive_present, previous.homing_offset)
        positive_tick = (positive_actual - raw_homing_offset) % 4096

        updated = update_wrist_roll_calibration(
            aligned,
            homing_offset=raw_homing_offset,
            negative_endpoint_tick=negative_tick,
            positive_endpoint_tick=positive_tick,
        )
        driver, limits = build_ros_mapping(updated)
        wrist = updated["wrist_roll"]
        wrist_limits = limits["wrist_roll"]
        print("\nProposed wrist-roll calibration:")
        print(f"  upright actual tick: {upright_actual}")
        print(f"  homing offset:       {wrist['homing_offset']}")
        print(f"  safe tick range:     {wrist['range_min']} .. {wrist['range_max']}")
        print(f"  safe ROS range:      {wrist_limits['lower']:.6f} .. {wrist_limits['upper']:.6f} rad")
        if input("Type WRITE_WRIST_EEPROM to write motor 5 only: ").strip() != "WRITE_WRIST_EEPROM":
            print("Cancelled. EEPROM and files were not changed.")
            return 2

        replacement = MotorCalibration(**wrist)
        try:
            robot.bus.write_calibration({"wrist_roll": replacement}, cache=False)
            readback = robot.bus.read_calibration()["wrist_roll"]
            if asdict(readback) != asdict(replacement):
                raise RuntimeError(f"Motor 5 EEPROM readback differs: {asdict(readback)}")
        except Exception:
            robot.bus.write_calibration({"wrist_roll": previous}, cache=False)
            raise
    finally:
        robot.bus.disconnect(disable_torque=True)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    archive_existing_calibration(json_path, stamp)
    _atomic_write(json_path, json.dumps(updated, indent=4) + "\n")
    state_path, control_path, limits_path = write_ros_files(directory, args.robot_id, driver, limits)
    print(f"Updated LeRobot calibration: {json_path}")
    print(f"ROS state-only mapping:      {state_path}")
    print(f"ROS motion configuration:   {control_path}")
    print(f"Calibrated URDF limits:      {limits_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True, help="Validated serial port, for example /dev/ttyACM0")
    parser.add_argument("--robot-id", required=True, help="Stable LeRobot calibration ID")
    parser.add_argument("--calibration-dir", type=Path, default=Path("config/hardware"))
    parser.add_argument(
        "--wrist-roll-only",
        action="store_true",
        help="Re-zero wrist roll at the upright jaw pose and record its asymmetric stops",
    )
    args = parser.parse_args()

    if args.wrist_roll_only:
        print("WARNING: this disables torque and can rewrite EEPROM on motor 5 only.")
        print("Support the arm. Keep every joint clear of obstacles.")
        return recalibrate_wrist_roll(args)

    print("WARNING: calibration disables torque and rewrites servo EEPROM on IDs 1..6.")
    print("Support the arm. Put all arm joints at physical middle and close the pgripper fully.")
    if input("Type WRITE_EEPROM to continue: ").strip() != "WRITE_EEPROM":
        print("Cancelled; no command was sent.")
        return 2

    directory = args.calibration_dir.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    json_path = directory / f"{args.robot_id}.json"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    archive_existing_calibration(json_path, stamp)
    subprocess.run(
        [
            "lerobot-calibrate",
            "--robot.type=so101_follower",
            f"--robot.port={args.port}",
            f"--robot.id={args.robot_id}",
            f"--robot.calibration_dir={directory}",
        ],
        check=True,
    )

    raw = json.loads(json_path.read_text())
    aligned, driver, limits = postprocess_calibration(raw)
    shutil.copy2(json_path, directory / f"{args.robot_id}.{stamp}.raw-lerobot.json")

    from lerobot.motors import MotorCalibration
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

    robot = SO101Follower(
        SO101FollowerConfig(
            port=args.port,
            id=args.robot_id,
            calibration_dir=directory,
        )
    )
    converted = {name: MotorCalibration(**values) for name, values in aligned.items()}
    robot.bus.connect()
    try:
        robot.bus.disable_torque()
        robot.bus.write_calibration(converted)
    finally:
        robot.bus.disconnect(disable_torque=True)

    _atomic_write(json_path, json.dumps(aligned, indent=4) + "\n")
    state_path, control_path, limits_path = write_ros_files(directory, args.robot_id, driver, limits)
    print(f"Aligned LeRobot calibration: {json_path}")
    print(f"ROS state-only mapping:    {state_path}")
    print(f"ROS motion configuration: {control_path}")
    print(f"Calibrated URDF limits:    {limits_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
