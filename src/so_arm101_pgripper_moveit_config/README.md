# SO-ARM101 pgripper MoveIt 2 configuration

This package starts MoveIt 2, RViz, and mock `ros2_control` controllers for the SO-ARM101 pgripper robot.

Five body joints form the `arm` planning group. The `gripper` motor forms a separate one-joint planning group.

The two jaw joints mimic the `gripper` motor. MoveIt does not send commands to the jaw joints.

## Start the mock demo

Install the Pixi environment.

```bash
pixi install
```

Build the workspace.

```bash
pixi run build
```

Start MoveIt 2 and RViz.

```bash
pixi run moveit-demo
```

CAUTION: Do not start the real driver during this demo. Two drivers can send conflicting motor commands.

The demo uses `mock_components/GenericSystem`. It does not open a serial port or enable motor torque.

## Plan in RViz

1. Select `arm` in the MotionPlanning panel.
2. Set the start state to `Current`.
3. Move the interactive tool marker to a reachable pose.
4. Keep `Use Approximate IK` active.
5. Select `Plan & Execute`.

Select `gripper` to plan the open or closed motor position.

## Orientation behavior

The arm has five body degrees of freedom. It cannot satisfy every position and orientation combination in six-dimensional task space.

The vendored `pick_ik` solver minimizes position and orientation errors together. The configuration gives position more weight than orientation.

For an unreachable orientation, MoveIt finds the best pose inside the configured error thresholds or reports no solution.

## Configuration files

- `so_arm101_pgripper.srdf` defines the planning groups and the tool.
- `kinematics.yaml` selects `pick_ik` for the arm.
- `ompl_planning.yaml` configures OMPL planning.
- `moveit_controllers.yaml` maps MoveIt trajectories to two controllers.
- `ros2_controllers.yaml` configures the mock controllers.
- `joint_limits.yaml` gives MoveIt the description limits.

The vendored sources are in `src/pick_ik` and `src/moveit_servo`.

Each `VENDORED.md` file records the source revision, local patch, and license.

## Test pick_ik on the real robot

CAUTION: Support the arm and keep the power connector accessible. This task enables torque on all six motors.

```bash
pixi run moveit-real
```

The launch uses the calibrated Feetech configuration and position limits. It starts the real controllers, MoveGroup, `pick_ik`, and RViz.

Before execution, set velocity and acceleration scaling to `0.05` in RViz. Use `Plan` first. Use `Execute` only for a small reachable change.

Stop the launch immediately if the arm jumps, vibrates, buzzes, or moves in the wrong direction.

## Start MoveIt Servo

Servo controls the five arm joints. The gripper remains a separate sixth actuator.

Start Servo without an input device.

```bash
pixi run moveit-servo-demo
```

Use two terminals for separate Servo and keyboard processes.

Start Servo and RViz in terminal 1.

```bash
pixi run moveit-servo-demo
```

Wait for `Servo initialized successfully`.

Start the keyboard input in terminal 2.

```bash
pixi run servo-keyboard-input
```

Focus terminal 2 before you press a control key. Do not press Enter after a control key.

The combined `pixi run moveit-servo-keyboard` task remains available.

Use the same terminal 1 command for separate PS4 input.

Start the PS4 input in terminal 2.

```bash
pixi run servo-ps4-input
```

The combined `pixi run moveit-servo-ps4` task remains available.

These launches use mock hardware. They do not open a serial port or enable motor torque.

## Start MoveIt Servo on the real robot

CAUTION: Support the arm and keep the power connector accessible. This task enables torque on the five arm motors and the gripper motor.

Start the real Servo launch with no input device:

```bash
pixi run moveit-servo-real
```

The task uses the calibrated control YAML and joint-limit YAML for `so_arm101_pgripper_follower`. It uses `/dev/ttyACM0` by default. Change the port in `pixi.toml` when Linux assigns a different device path.

Start keyboard input in another terminal:

```bash
pixi run servo-keyboard-input
```

Start PS4 input in another terminal:

```bash
pixi run servo-ps4-input
```

Start with low scale values and small single-joint motions. Stop the launch if the arm jumps, vibrates, buzzes, or moves in the wrong direction.

### Keyboard map

The keyboard starts in Cartesian mode.

- Use `w/s`, `a/d`, and `r/f` for linear X, Y, and Z motion.
- Use `u/o`, `i/k`, and `j/l` for roll, pitch, and yaw motion.
- Press `m` to change between Cartesian mode and joint mode.
- In joint mode, use `1/q` through `5/t` for positive and negative motion.
- Press `g` to toggle the gripper between open and closed.
- Press `c` to change between `base_link` and `gripper_frame_link`.
- Press Space to send a zero command.
- Press `x` to exit.

### PS4 map

Hold `L1` as the deadman control.

- The left stick controls linear X and Y motion.
- The triggers control linear Z motion.
- The right stick controls angular X and Y motion.
- The horizontal direction pad controls angular Z motion.
- Hold `R1` with `L1` to send joint commands.
- In joint mode, the sticks control four flex joints and the triggers control wrist roll.
- Press Triangle to change the command frame.
- Hold `L1` and press Cross to toggle the gripper between open and closed.

The Linux `joy` axis order can differ between controller drivers. Read `/joy` before real-arm use.

## Servo limits

The Cartesian command contains six velocity components. The five-joint arm cannot satisfy all six components at every pose.

Servo finds a feasible joint velocity or stops near a singularity. Position and orientation behavior depends on the current arm pose.

The Servo configuration uses measured Jacobian condition numbers for this robot. It starts singularity scaling at `30` and stops at `60`.

The local MoveIt Servo patch uses five thin-SVD columns for the 6-by-5 Jacobian. Upstream 2.12.4 incorrectly indexes a sixth column. Servo also routes the five-joint group through the Jacobian pseudoinverse. MoveGroup keeps the vendored `pick_ik` solver for pose planning.

CAUTION: Do not connect this mock Servo launch to the real driver. The real-arm Servo path needs separate hardware safety gates.
