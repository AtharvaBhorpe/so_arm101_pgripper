# Attribution and licenses

This package adapts robot descriptions from TheRobotStudio and NormaCore.

## SO-ARM101 body

| Field | Value |
| --- | --- |
| Source | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101) |
| Revision | `7629d2ad9853d10fb903093a33ef6114099d97e5` |
| Source file | `so101_new_calib.urdf` |
| License | Apache License 2.0 |
| License text | `LICENSES/Apache-2.0.txt` |

The imported material includes:

- SO-101 follower meshes
- Link inertias
- Joint frames
- Joint axes
- Joint limits.

Changes include:

- Conversion to Xacro
- ROS package URIs for mesh paths
- Removal of the stock gripper with a moving jaw
- A wrist-roll output link for the pgripper attachment.

## NormaCore pgripper

| Field | Value |
| --- | --- |
| Source | [norma-core/norma-core](https://github.com/norma-core/norma-core/tree/main/hardware/pgripper) |
| Kinematic source | [`hardware/elrobot/simulation/elrobot_follower.urdf`](https://github.com/norma-core/norma-core/blob/main/hardware/elrobot/simulation/elrobot_follower.urdf) |
| Revision | `b935fc4b887e576538921b15630a30db22759ec8` |
| License for imported simulation material | MIT |
| License text | `LICENSES/MIT.txt` |

The imported material includes:

- Pgripper base, ST3215, gear, and jaw meshes
- Link inertias
- Internal joint frames
- Joint axes and limits
- Mimic ratios.

Changes include:

- ROS-safe names
- Package mesh URIs
- A fixed transform between the SO-101 and NormaCore CAD frames
- `gripper_frame_link` and `tool0`
- A positive gear inertia instead of the upstream zero tensor.

The package's original integration code and documentation use Apache-2.0.
Upstream assets retain their respective licenses and notices.

## Feetech ROS 2 driver

| Field | Value |
| --- | --- |
| Source | [ros-physical-ai/feetech_ros2_driver](https://github.com/ros-physical-ai/feetech_ros2_driver) |
| Revision | `18aed7fb26d3e2b4c0b47762f39d8698b7032422` |
| Workspace path | `src/feetech_ros2_driver` |
| License | BSD, as recorded in the upstream license file |
| License text | `src/feetech_ros2_driver/LICENSE` |

The workspace includes the upstream hardware plugin and serial protocol package.
The vendored source has no local changes.

The separate `so_arm101_pgripper_hardware` plugin applies per-joint `center_tick` and `direction`.
Its pgripper mapping uses 0 rad for open jaws and positive angles for closure.

## Gripper camera mount

| Field | Value |
| --- | --- |
| Source | [NormaCore camera mount](https://github.com/norma-core/norma-core/blob/3aa88d4ade5a1ed287f2a8bdc0677db208572b86/hardware/pgripper/STL/camera_mounts/CameraMount_square_27mm.stl) |
| Reference revision | `3aa88d4ade5a1ed287f2a8bdc0677db208572b86` |
| Local path | `meshes/gripper_camera_mount/CameraMount_square_27mm.stl` |
| Directory license | Apache-2.0 |
| Upstream license | [hardware/pgripper/LICENSE](https://github.com/norma-core/norma-core/blob/3aa88d4ade5a1ed287f2a8bdc0677db208572b86/hardware/pgripper/LICENSE) |
| Local license text | `LICENSES/Apache-2.0.txt` |

The reference revision identifies the upstream snapshot.
