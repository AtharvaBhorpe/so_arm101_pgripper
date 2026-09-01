# MoveIt Servo Teleoperation Design

## Goal

Add mock-hardware MoveIt Servo control for the SO-ARM101 pgripper robot.

The operator can send Cartesian commands from a keyboard or a PS4 controller. The operator can also send named joint commands.

## Robot control boundary

The `arm` Servo group contains these five joints:

1. `shoulder_pan`
2. `shoulder_lift`
3. `elbow_flex`
4. `wrist_flex`
5. `wrist_roll`

The `gripper` joint is the sixth actuator. It remains on `gripper_controller` and does not enter the Servo group.

## Data flow

```text
keyboard or joy_node
        |
        +-- geometry_msgs/TwistStamped --> /servo_node/delta_twist_cmds
        |
        +-- control_msgs/JointJog -------> /servo_node/delta_joint_cmds
                                               |
                                               v
                                         MoveIt Servo
                                               |
                                               v
                         /arm_controller/joint_trajectory
                                               |
                                               v
                                     mock ros2_control
```

MoveIt Servo publishes `trajectory_msgs/JointTrajectory`. The existing arm controller consumes that message type.

## Cartesian control

The input message contains six velocity components. The robot has only five arm joints.

Servo computes the best feasible joint velocity. Some Cartesian position and orientation combinations are not feasible.

The operator selects `base_link` or `gripper_frame_link` as the command frame.

## Joint control

Joint commands contain only the five arm joint names. The adapter rejects the gripper and all mimic joint names.

## Operator safety

The keyboard publisher sends commands only while it receives key input. The PS4 publisher requires a deadman button.

MoveIt Servo rejects stale commands after `0.1 s`. Initial speed scales remain conservative.

This stage uses mock hardware only. A later stage will connect the same Servo output contract to the real-arm launch.

## Files

- `config/servo.yaml` contains MoveIt Servo parameters.
- `config/ps4.yaml` contains the PS4 axis and button map.
- `launch/servo_demo.launch.py` starts mock MoveIt, Servo, and an optional input node.
- `scripts/servo_keyboard_input.py` publishes keyboard commands.
- `scripts/servo_joy_input.py` converts `sensor_msgs/Joy` messages.
- `test/test_servo_config.py` protects the 5+1 boundary and topic contracts.

