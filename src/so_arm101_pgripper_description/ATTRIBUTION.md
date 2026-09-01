# Attribution and licenses

This package combines and adapts two authoritative upstream robot descriptions.

## SO-ARM101 body

- Source: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101)
- Source revision: `7629d2ad9853d10fb903093a33ef6114099d97e5`
- Imported material: SO-101 follower mesh assets, link inertias, joint frames, axes, and joint limits from `so101_new_calib.urdf`.
- License: Apache License 2.0; see `LICENSES/Apache-2.0.txt`.
- Changes: converted to Xacro, changed mesh paths to ROS package URIs, removed the stock moving-jaw gripper, and retained a stable wrist-roll output link for the pgripper attachment.

## NormaCore pgripper

- Source: [norma-core/norma-core](https://github.com/norma-core/norma-core/tree/main/hardware/pgripper)
- Kinematic source: [`hardware/elrobot/simulation/elrobot_follower.urdf`](https://github.com/norma-core/norma-core/blob/main/hardware/elrobot/simulation/elrobot_follower.urdf)
- Source revision: `b935fc4b887e576538921b15630a30db22759ec8`
- Imported material: pgripper base, ST3215, gear and jaw meshes; link inertias; internal fixed/revolute/prismatic frames; axes; limits; and mimic ratios.
- License for the imported ElRobot simulation material: MIT; see `LICENSES/MIT.txt`.
- Changes: ROS-safe names, package mesh URIs, a fixed SO-101-to-NormaCore CAD frame conversion, `gripper_frame_link`/`tool0`, and a small positive gear inertia in place of the upstream all-zero inertia tensor so URDF dynamics consumers receive a valid tensor.

The package's original glue code and documentation are offered under Apache-2.0. Upstream assets retain their respective licenses and notices.

## Feetech ROS 2 driver

- Source: [ros-physical-ai/feetech_ros2_driver](https://github.com/ros-physical-ai/feetech_ros2_driver)
- Source revision: `18aed7fb26d3e2b4c0b47762f39d8698b7032422`
- Imported material: the complete ROS 2 hardware plugin and Feetech serial protocol package under `src/feetech_ros2_driver`.
- License: BSD; retained verbatim in `src/feetech_ros2_driver/LICENSE`.
- Changes: added tested per-joint `center_tick` and `direction` parameters. The pgripper uses 0 rad open and positive radians closed.
