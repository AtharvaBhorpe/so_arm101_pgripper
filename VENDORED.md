# Vendored sources

Vendored package directories are kept identical to their recorded upstream
revisions. Workspace-specific build settings live outside those directories.

| Package | Repository | Version | Commit |
| --- | --- | --- | --- |
| `pick_ik` | <https://github.com/PickNikRobotics/pick_ik> | `1.1.1` | `f47bd8fbfa3c78d4236131a0577d9ec7346f77a9` |

`colcon.meta` disables Pick IK's upstream test build during normal workspace
builds. This avoids its configure-time Catch2 download without patching Pick IK.
