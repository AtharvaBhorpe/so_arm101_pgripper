# SO-ARM101 pgripper

Use this ROS 2 Jazzy workspace to control an SO-ARM101 follower arm with a
NormaCore pgripper. You can use RViz, MoveIt 2, Pick IK, and MoveIt Servo.

> [!WARNING]
> CAUTION: Start with a mock task. Real tasks can enable motor torque and move
> the arm.

## Contents

~~~text
URDF and calibration ──> ros2_control hardware plugin ──> controllers
           │                         │                        │
           └──────────────── RViz, MoveIt, and Servo ─────────┘
~~~

- Five arm joints and one gripper motor.
- Calibrated joint mappings and limits in `config/hardware/`.
- Mock and real MoveIt 2 tasks with Pick IK.
- Mock and real MoveIt Servo tasks with keyboard or PS4 input.
- Pinned external source repositories as Git submodules.

## Start the mock robot

Install Pixi on Linux x86_64. Clone the workspace with its recorded
submodules.

~~~bash
git clone --recurse-submodules https://github.com/AtharvaBhorpe/so_arm101_pgripper.git
cd so_arm101_pgripper
pixi install
pixi run build
pixi run test
~~~

Start the robot model with joint sliders:

~~~bash
pixi run display
~~~

Start mock MoveIt:

~~~bash
pixi run moveit-demo
~~~

Mock tasks use simulated controllers. They do not open a serial port or enable
motor torque.

In RViz, select the `arm` group. Keep **Approx IK Solutions** active. Move the
end-effector marker to a small reachable pose. Select **Plan** or **Plan & Execute**.

Select the `gripper` group to use its open and closed named states.

## Use the real robot

CAUTION: Support the arm. Keep the power connector accessible. Stop the task
after unexpected motion, vibration, or buzzing.

### 1. Read-only RViz

Run this task first:

~~~bash
pixi run real-rviz
~~~

The driver reads physical joint states. It does not enable motor torque.

With the jaws open, make sure that the gripper shows `0 rad`. Move each joint
by hand. Make sure that RViz follows the same direction.

### 2. MoveIt planning and execution

If read-only RViz matches the real arm, run:

~~~bash
pixi run moveit-real
~~~

This task enables motor torque. In RViz, use **Plan** before execution. Start
with a small reachable motion and low velocity and acceleration values.

### 3. MoveIt Servo

Start the real Servo task in terminal 1:

~~~bash
pixi run moveit-servo-real
~~~

When Servo reports that it is ready, start keyboard input in terminal 2:

~~~bash
pixi run servo-keyboard-input
~~~

Start with small single-joint motions. Press `g` to toggle the gripper. Press
Space to send a zero command. Press `x` to exit.

The real tasks use `/dev/ttyACM0` by default. Make sure that this path belongs
to the robot. If Linux assigns another path, change the task in
[`pixi.toml`](pixi.toml).

## Calibrate the robot

CAUTION: Support the arm during calibration. Calibration disables motor torque
and writes EEPROM only after an explicit confirmation.

Run the full calibration:

~~~bash
pixi run calibrate --port /dev/ttyACM0 \
  --robot-id so_arm101_pgripper_follower \
  --calibration-dir config/hardware
~~~

Run the wrist-roll procedure after full calibration:

~~~bash
pixi run calibrate --port /dev/ttyACM0 \
  --robot-id so_arm101_pgripper_follower \
  --calibration-dir config/hardware \
  --wrist-roll-only
~~~

The wrist-roll procedure measures its own asymmetric physical stops. Read the
[description guide](src/so_arm101_pgripper_description/README.md) before you
start either calibration task.

## Commands

| Command | Purpose | Motor torque |
| --- | --- | --- |
| `pixi run build` | Build all packages | Off |
| `pixi run test` | Run workspace tests | Off |
| `pixi run check` | Examine the generated URDF | Off |
| `pixi run display` | Start the model with joint sliders | Off |
| `pixi run moveit-demo` | Start mock MoveIt | Off |
| `pixi run moveit-servo-demo` | Start mock MoveIt Servo | Off |
| `pixi run real-rviz` | Read physical joint states in RViz | Off |
| `pixi run moveit-real` | Plan and execute with MoveIt | On |
| `pixi run moveit-servo-real` | Control the arm with Servo | On |
| `pixi run calibrate ...` | Calibrate the motors | Off during motion |

## Workspace layout

| Path | Purpose |
| --- | --- |
| `src/so_arm101_pgripper_description` | URDF, Xacro, calibration, and RViz |
| `src/so_arm101_pgripper_hardware` | Local SO-ARM101 hardware plugin |
| `src/so_arm101_pgripper_moveit_config` | MoveIt, Servo, and input programs |
| `config/hardware` | Machine-specific calibration and limits |
| `src/pick_ik` | Pinned upstream Pick IK |
| `src/feetech_ros2_driver` | Pinned upstream Feetech driver |
| `src/moveit_servo` | Pinned Servo fork with the 5-DoF fix |

Read [VENDORED.md](VENDORED.md) for exact source revisions and the Servo patch.

## More detail

- [MoveIt and Servo guide](src/so_arm101_pgripper_moveit_config/README.md)
- [Calibration and read-only RViz guide](src/so_arm101_pgripper_description/README.md)
- [Pinned external sources](VENDORED.md)
