# SO-ARM101 MoveIt and Servo

This package provides MoveIt 2 and Servo launches for mock and real hardware.

The `arm` planning group contains five arm joints.
The `gripper` motor forms a separate one-joint group.
The two jaw joints mimic the gripper motor.
MoveIt does not command the jaw joints directly.

## Start mock MoveIt

From the workspace root, install the dependencies:

```bash
pixi install
```

Build the workspace:

```bash
pixi run build
```

Start the mock demo:

```bash
pixi run moveit-demo
```

The demo uses `mock_components/GenericSystem`.
It does not open a serial port or enable motor torque.

Do not start the real driver during this demo.
The mock launch is not a real-arm startup procedure.

## Plan in RViz

1. Select the `arm` group in the MotionPlanning panel.
2. Set the start state to the current state.
3. Keep approximate IK active.
4. Move the tool marker to a reachable pose.
5. Select **Plan**.
6. Examine the path.
7. Select **Execute**.

For gripper motion, select the `gripper` group.
Choose the `open` or `close` named state.

## IK limits

The arm has five degrees of freedom.
It cannot satisfy every six-dimensional position and orientation target.

The `pick_ik` solver minimizes position and orientation errors together.
The configuration gives position more weight than orientation.
The solver searches for a pose within the configured error thresholds.
It can report no solution.

## Real MoveIt

> [!WARNING]
> Complete the read-only robot checks before this task.
> Support the arm. Keep the power connector accessible.
> This task enables torque on all six motors.

Run:

```bash
pixi run moveit-real
```

The launch starts the real controllers, MoveGroup, Pick IK, and RViz.
It uses the calibrated motor configuration and joint limits.

1. Set velocity and acceleration scales to `0.05`.
2. Select **Plan**.
3. Examine the path.
4. Execute only a small reachable motion.

If the arm jumps, vibrates, buzzes, or moves in the wrong direction, stop the launch immediately.

## Mock Servo

Servo controls the five arm joints.
The gripper remains a separate actuator.

In terminal 1, start Servo and RViz:

```bash
pixi run moveit-servo-demo
```

Wait for `Servo initialized successfully`.

Choose one input process in terminal 2.

### Keyboard input

```bash
pixi run servo-keyboard-input
```

Focus the input terminal before pressing control keys.
Do not press Enter after a control key.

The combined `pixi run moveit-servo-keyboard` task is also available.

### PS4 input

```bash
pixi run servo-ps4-input
```

The combined `pixi run moveit-servo-ps4` task is also available.

These demo tasks use mock hardware.
They do not open a serial port or enable motor torque.

## Real Servo

> [!WARNING]
> Complete the read-only robot checks before this task.
> Support the arm. Keep the power connector accessible.
> This task enables torque on the arm and gripper motors.

In terminal 1, run:

```bash
pixi run moveit-servo-real
```

The task uses the calibration for `so_arm101_pgripper_follower`.
The default port is `/dev/ttyACM0`.

If Linux assigns another port, update the task in `pixi.toml`.

When Servo reports that it is ready, choose one input process in another terminal.

For keyboard input:

```bash
pixi run servo-keyboard-input
```

For PS4 input:

```bash
pixi run servo-ps4-input
```

Start with low speed scales and small single-joint motions.

If the arm jumps, vibrates, buzzes, or moves in the wrong direction, stop the launch immediately.

Do not connect the mock Servo launch to the real driver.

## Keyboard controls

The keyboard starts in Cartesian mode.

| Keys | Action |
| --- | --- |
| `w/s`, `a/d`, `r/f` | Linear X, Y, and Z motion |
| `u/o`, `i/k`, `j/l` | Roll, pitch, and yaw motion |
| `m` | Switch between Cartesian and joint modes |
| `1/q` through `5/t` | Positive and negative joint motion in joint mode |
| `g` | Toggle the gripper |
| `c` | Switch between `base_link` and `gripper_frame_link` |
| Space | Send a zero command |
| `x` | Exit keyboard input |

## PS4 controls

Hold `L1` as the deadman control.

| Input | Action |
| --- | --- |
| Left stick | Linear X and Y motion |
| Triggers | Linear Z motion |
| Right stick | Angular X and Y motion |
| Horizontal direction pad | Angular Z motion |
| `R1` with `L1` | Joint-command mode |
| Sticks in joint mode | Four flex joints |
| Triggers in joint mode | Wrist roll |
| Triangle | Change the command frame |
| Cross with `L1` | Toggle the gripper |

The Linux `joy` axis order can differ between controller drivers.
Before real-arm use, examine `/joy` and the axis configuration.

## Servo limits and local patch

Cartesian commands contain six velocity components.
The five-joint arm cannot satisfy all six components at every pose.

Servo computes a feasible joint velocity or stops near a singularity.
The result depends on the current arm pose.

The configuration starts singularity scaling at `30` and stops at `60`.

The local Servo patch uses five thin-SVD columns for the 6-by-5 Jacobian.
Upstream 2.12.4 incorrectly indexes a sixth column.
The patch also routes the five-joint group through the Jacobian pseudoinverse.
MoveGroup retains Pick IK for pose planning.

## Configuration files

| File | Purpose |
| --- | --- |
| `so_arm101_pgripper.srdf` | Planning groups, named states, and collision exclusions |
| `kinematics.yaml` | Pick IK configuration |
| `ompl_planning.yaml` | OMPL planning configuration |
| `moveit_controllers.yaml` | MoveIt trajectory-controller mappings |
| `ros2_controllers.yaml` | Mock controller configuration |
| `joint_limits.yaml` | MoveIt joint limits |
| `servo.yaml` | Servo configuration |
| `ps4.yaml` | PS4 axis and button mapping |

The workspace [source record](../../VENDORED.md)
lists the pinned repositories and commits.
