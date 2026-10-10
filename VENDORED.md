# External source pins

The workspace pins external source packages as Git submodules.
Workspace-specific build configuration stays outside those directories.

| Package | Repository | Version | Commit |
| --- | --- | --- | --- |
| `pick_ik` | <https://github.com/PickNikRobotics/pick_ik> | `1.1.1` | `f47bd8fbfa3c78d4236131a0577d9ec7346f77a9` |
| `feetech_ros2_driver` | <https://github.com/ros-physical-ai/feetech_ros2_driver> | `0.2.2` | `18aed7fb26d3e2b4c0b47762f39d8698b7032422` |
| `moveit_servo` | <https://github.com/AtharvaBhorpe/moveit_servo> | `2.12.4` | `dcee64e41aacadb329733a05a76777c913e9db3f` |

After cloning the workspace, populate the recorded sources:

```bash
git submodule update --init --recursive
```

## Build integration

`colcon.meta` disables the upstream Pick IK test build.
This prevents its configure-time Catch2 download without changes to Pick IK.

`so_arm101_pgripper_hardware` reuses the Feetech protocol library and YAML helpers.
It implements the SO-ARM101 coordinate mapping and raw-current state interface.

The upstream Feetech package does not install its core headers.
The local plugin reads those headers from the sibling submodule.

## MoveIt Servo patch

The pinned fork corrects the thin-SVD indexing for the robot's 6-by-5 Jacobian.
It routes the five-joint Servo group through the Jacobian pseudoinverse.
MoveGroup retains Pick IK for pose planning.

The [MoveIt guide](src/so_arm101_pgripper_moveit_config/README.md)
describes the operating limits.

## Source notices

Upstream source packages retain their licenses and notices.
The [model attribution](src/so_arm101_pgripper_description/ATTRIBUTION.md)
records the robot-description sources separately.
