# SO-ARM101 pgripper

ROS 2 Jazzy workspace for an SO-ARM101 follower arm fitted with a NormaCore
pgripper. It provides a calibrated robot description, read-only RViz
mirroring, MoveIt planning with Pick IK, and MoveIt Servo teleoperation.

> [!WARNING]
> This repository can command physical motors. Start with the mock tasks,
> keep the arm supported during real-hardware work, and stop immediately for
> unexpected motion, vibration, or buzzing.

## What is included

```text
robot description ──> ros2_control hardware plugin ──> controllers
       │                        │                         │
       └──────────────────── RViz / MoveIt / Servo ───────┘
```

- Five-joint arm (`shoulder_pan` through `wrist_roll`) plus one gripper motor.
- Calibrated joint mapping and limits under `config/hardware/`.
- Mock and real MoveIt 2 launches using Pick IK for pose planning.
- Mock and real MoveIt Servo launches with keyboard or PS4 input.
- A local hardware plugin; external dependencies are pinned Git submodules.

## Quick start: mock robot

Prerequisites: Linux x86_64 and [Pixi](https://pixi.sh). Clone the workspace
with its exact external source revisions:

```bash
git clone --recurse-submodules https://github.com/AtharvaBhorpe/so_arm101_pgripper.git
cd so_arm101_pgripper
pixi install
pixi run build
pixi run test
```

Start the basic robot model:

```bash
pixi run display
```

Start the mock MoveIt system. It uses simulated controllers and **does not**
open a serial port or enable motor torque:

```bash
pixi run moveit-demo
```

In RViz, select the `arm` planning group, keep **Approx IK Solutions** enabled,
move the end-effector marker to a small reachable pose, then use **Plan** or
**Plan & Execute**. Select `gripper` to test its open and closed named states.

## Real robot workflow

Do not skip directly to torque-enabled commands. The first gate validates that
the calibrated software matches the physical robot; the second enables motion.

1. **Read-only RViz mirroring** — torque stays off:

   ```bash
   pixi run real-rviz
   ```

   Move each joint by hand. RViz must follow the same direction and the
   gripper must open at `0 rad`.

2. **MoveIt planning and execution** — torque-enabled:

   ```bash
   pixi run moveit-real
   ```

   Support the arm. In RViz, use **Plan** first and begin with a small,
   reachable motion at low velocity and acceleration scaling.

3. **MoveIt Servo** — continuous teleoperation:

   ```bash
   # Terminal 1
   pixi run moveit-servo-real

   # Terminal 2, once Servo is ready
   pixi run servo-keyboard-real-input
   ```

   Start with small single-joint motions. The `g` key toggles the gripper;
   Space sends a stop command; `x` exits. See the MoveIt package README for
   the complete keyboard and PS4 mappings.

Every real task uses `/dev/ttyACM0` by default. Confirm that it is the robot
before launch; change the port in [`pixi.toml`](pixi.toml) if Linux assigned a
different device path.

## Calibration

Calibration rewrites motor EEPROM and requires the arm to be supported. Use:

```bash
pixi run calibrate --port /dev/ttyACM0 \
  --robot-id so_arm101_pgripper_follower \
  --calibration-dir config/hardware
```

Wrist roll is intentionally handled in its own safe procedure because its
mechanical stops are asymmetric:

```bash
pixi run calibrate --port /dev/ttyACM0 \
  --robot-id so_arm101_pgripper_follower \
  --calibration-dir config/hardware \
  --wrist-roll-only
```

See the [description package README](src/so_arm101_pgripper_description/README.md)
for the complete procedure and generated calibration files.

## Commands

| Command | Purpose | Hardware effect |
| --- | --- | --- |
| `pixi run build` | Build all packages | None |
| `pixi run test` | Run the workspace tests | None |
| `pixi run check` | Validate the generated URDF | None |
| `pixi run display` | Robot model with joint sliders | None |
| `pixi run moveit-demo` | Mock MoveIt planning | None |
| `pixi run moveit-servo-demo` | Mock MoveIt Servo | None |
| `pixi run real-rviz` | Read-only physical-state RViz mirroring | Reads serial state only |
| `pixi run moveit-real` | Real MoveIt planning and execution | Enables torque |
| `pixi run moveit-servo-real` | Real MoveIt Servo | Enables torque |
| `pixi run calibrate ...` | Calibrate motors | Disables torque; writes EEPROM after confirmation |

## Workspace layout

| Path | Responsibility |
| --- | --- |
| `src/so_arm101_pgripper_description` | URDF/Xacro, calibration wrapper, RViz launch |
| `src/so_arm101_pgripper_hardware` | Local SO-ARM101 `ros2_control` hardware plugin |
| `src/so_arm101_pgripper_moveit_config` | MoveIt, Servo, controllers, keyboard and PS4 input |
| `config/hardware` | Machine-specific calibration and joint-limit files |
| `src/pick_ik` | Pinned upstream Pick IK submodule |
| `src/feetech_ros2_driver` | Pinned upstream Feetech driver submodule |
| `src/moveit_servo` | Pinned Servo fork containing the 5-DoF compatibility fix |

The upstream sources are deliberately kept separate from workspace code. Exact
versions and the one Servo-specific patch are recorded in [VENDORED.md](VENDORED.md).

## Further reading

- [MoveIt and Servo usage](src/so_arm101_pgripper_moveit_config/README.md)
- [Calibration, real-state mirroring, and hardware provenance](src/so_arm101_pgripper_description/README.md)
- [Pinned external dependencies](VENDORED.md)
