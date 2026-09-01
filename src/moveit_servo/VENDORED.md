# Vendored MoveIt Servo

This directory contains `moveit_ros/moveit_servo` from the MoveIt 2 repository.

- Upstream: https://github.com/moveit/moveit2
- Release tag: `2.12.4`
- Source path: `moveit_ros/moveit_servo`
- License: BSD-3-Clause. See `LICENSE.txt`.

## Local patch

`src/utils/common.cpp` uses the SVD singular-value count for the least singular direction.

The upstream code uses the six-element Cartesian command size. A 6-by-5 Jacobian has only five thin-SVD columns.

This patch keeps the upstream algorithm and makes the index valid for the five-joint arm.

`src/utils/command.cpp` sends groups with fewer than six joints through the Jacobian pseudoinverse. The loaded IK plugin still supplies the planning and tool frames. This avoids unchanged approximate pose solutions from a full-pose IK request on a five-joint chain.

`CMakeLists.txt` gives the internal patched library a unique `so101` filename.

This name prevents the Pixi binary library from replacing the patched library at runtime.
