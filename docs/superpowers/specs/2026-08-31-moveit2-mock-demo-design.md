# SO-ARM101 Pgripper MoveIt 2 Mock Demo Design

## Goal

Add a ROS 2 Jazzy MoveIt 2 configuration and mock-hardware demo for the existing
SO-ARM101 follower with NormaCore pgripper. The demo must support planning and
execution in RViz without connecting to physical motors.

MoveIt Servo and torque-enabled real-hardware launch are outside this change.

## Kinematic model

The robot is **5+1 DoF**, not a six-axis manipulator:

- The `arm` planning group contains exactly `shoulder_pan`, `shoulder_lift`,
  `elbow_flex`, `wrist_flex`, and `wrist_roll`.
- The `gripper` planning group contains only the actuated `gripper` joint.
- `pgripper_left_finger_joint` and `pgripper_right_finger_joint` remain passive
  URDF mimic joints and are not controller or planning variables.
- The arm IK chain ends at `gripper_frame_link`, with `tool0` retained as the
  coincident tool frame. The SRDF end-effector declaration attaches the
  one-joint gripper group at its actual parent branch, `pgripper_servo_link`.

A five-joint chain cannot independently satisfy arbitrary six-dimensional
Cartesian targets. The RViz pose marker will still accept position and
orientation targets, but the solver can reject unreachable targets or return an
approximate best-achievable orientation.

## MoveIt configuration

Create a separate package named `so_arm101_pgripper_moveit_config`. It will
contain:

- SRDF groups, end-effector declaration, named home/open/closed states, passive
  mimic-joint declarations, and a conservative self-collision matrix.
- `pick_ik/PickIkPlugin` for the five-joint arm. Position has higher priority
  than rotation, while rotation remains enabled. Approximate IK solutions are
  allowed in the RViz MotionPlanning display.
- OMPL as the only planning pipeline for this first demo.
- MoveIt joint-limit overrides derived from the existing canonical URDF limits.
- `moveit_simple_controller_manager` mappings for separate arm and gripper
  trajectory controllers.

The Jazzy release of `pick_ik` is used through the workspace's RoboStack Jazzy
environment. No custom IK plugin is added.

## Mock ros2_control

The existing description Xacro already supports
`mock_components/GenericSystem`. The demo expands it with
`use_ros2_control:=true` and starts:

- `joint_state_broadcaster`
- `arm_controller`, controlling the five arm joints
- `gripper_controller`, controlling only `gripper`

Both command controllers use `JointTrajectoryController`, giving MoveIt two
standard `FollowJointTrajectory` action interfaces. This mock-only controller
configuration does not modify the current real-driver configuration.

## Demo launch

`demo.launch.py` starts one coherent system:

1. static `world` to `base_link` transform
2. `robot_state_publisher`
3. mock `ros2_control_node`
4. three controller spawners
5. `move_group`
6. RViz with the MoveIt MotionPlanning display

The robot description, semantic description, kinematics parameters, planning
pipeline, and joint limits are passed consistently to `move_group` and RViz.
The launch must never load `feetech_ros2_driver` or open a serial port.

## User workflow

The documented workflow is:

```bash
pixi run build
pixi run moveit-demo
```

In RViz, the user selects `arm`, chooses joint-space or pose goals, plans, and
executes against mock hardware. The separate `gripper` group is used for
open/closed plans. The README explains that a red/unreachable pose or imperfect
orientation is expected when a requested six-dimensional target exceeds the
five-joint chain's reachable task space.

## Validation

Automated checks must prove:

- the SRDF arm group has exactly five joints and excludes `gripper`
- the gripper group contains exactly one active joint
- mimic jaw joints are passive and absent from controllers
- MoveIt and ros2_control controller joint lists agree
- the generated URDF uses `mock_components/GenericSystem`
- MoveIt configuration files parse and the launch description loads
- the complete workspace builds and tests successfully
- a bounded smoke launch brings up `move_group`, both controllers, and RViz
  dependencies without accessing hardware

Physical execution is not part of validation. Real-hardware MoveIt integration
will be a later gated change after mock planning is accepted.

## Authoritative references

- MoveIt configuration packages and `MoveItConfigsBuilder`:
  https://moveit.picknik.ai/main/doc/how_to_guides/moveit_configuration/moveit_configuration_tutorial.html
- MoveIt launch structure:
  https://moveit.picknik.ai/main/doc/how_to_guides/moveit_launch_files/moveit_launch_files_tutorial.html
- MoveIt controller interfaces:
  https://moveit.picknik.ai/main/doc/examples/controller_configuration/controller_configuration_tutorial.html
- `pick_ik` weighted position/orientation configuration:
  https://github.com/moveit/moveit2_tutorials/blob/main/doc/how_to_guides/pick_ik/config/kinematics_pick_ik.yaml
