# Vendored sources

Vendored build and runtime source files are kept identical to their recorded
upstream revisions. Workspace-specific build settings live outside those
directories.

| Package | Repository | Version | Commit |
| --- | --- | --- | --- |
| `pick_ik` | <https://github.com/PickNikRobotics/pick_ik> | `1.1.1` | `f47bd8fbfa3c78d4236131a0577d9ec7346f77a9` |
| `feetech_ros2_driver` | <https://github.com/ros-physical-ai/feetech_ros2_driver> | `0.2.2` | `18aed7fb26d3e2b4c0b47762f39d8698b7032422` |
| `moveit_servo` | <https://github.com/AtharvaBhorpe/moveit2>, branch `so-arm101-servo-2.12.4`, path `moveit_ros/moveit_servo` | `2.12.4` | `6d5bf009e2d9819c9db269090126ffa6900e308a` |

`colcon.meta` disables Pick IK's upstream test build during normal workspace
builds. This avoids its configure-time Catch2 download without patching Pick IK.

`so_arm101_pgripper_hardware` is the workspace-local `ros2_control` plugin. It
reuses Feetech's protocol library and YAML helpers, and owns the SO-ARM101
coordinate mapping and raw-current state interface. The upstream Feetech core
headers are not installed by its package, so this workspace plugin intentionally
includes them from the sibling vendored source directory.
