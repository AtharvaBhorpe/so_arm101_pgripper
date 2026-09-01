# SO-ARM101 + NormaCore pgripper

ROS 2 Jazzy/Pixi workspace containing the combined robot description, a vendored Feetech `ros2_control` driver, exact real-state RViz mirroring, and a LeRobot 0.6.1 calibration workflow. The six physical motors keep the standard names and IDs:

| ID | Joint | Calibration meaning |
|---:|---|---|
| 1–4 | `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex` | tick 2048 = 0 rad; measured range is safely symmetrized around zero |
| 5 | `wrist_roll` | tick 2048 = 0 rad at the visually upright jaw pose; separately measured physical stops give asymmetric safe limits |
| 6 | `gripper` | EEPROM closed = tick 2048; ROS open = 0 rad; closing increases the ROS angle |

The two jaw joints are prismatic mimics of the single `gripper` motor. `gripper_frame_link`/`tool0` remain the tool frames for IK.

## Install and validate

```bash
pixi install
pixi run build
pixi run test
pixi run check
pixi run display
```

`pixi run display` is simulation-only and opens the joint sliders. No serial device is touched.

## MoveIt 2 mock demo

Build and start the mock MoveIt 2 system.

```bash
pixi run moveit-demo
```

CAUTION: Do not start the real driver during this demo. Two drivers can send conflicting motor commands.

The demo starts RViz, MoveIt 2, OMPL, vendored `pick_ik`, and mock controllers. It does not open the serial port.

The arm group has five body joints. The gripper group has one motor joint and two passive mimic jaw joints.

The solver includes end-effector orientation in its cost. Five body joints cannot satisfy every six-dimensional pose exactly.

Use approximate IK in RViz for the best reachable position and orientation. Read the MoveIt package [README](../so_arm101_pgripper_moveit_config/README.md) for details.

## Calibrate the physical follower

Calibration disables torque and rewrites EEPROM on motor IDs 1–6. Support the arm throughout. First identify the current port and verify read-only communication; never assume an old `/dev/ttyACM0` assignment is still this robot:

```bash
pixi run lerobot-find-port
pixi run ./install/feetech_ros2_driver/bin/demo ping /dev/ttyACM0
# Enter servo ID 1 when prompted; continue only after "Ping success".
```

Then run the package wrapper, which invokes the official `lerobot-calibrate` command and post-processes its output:

```bash
pixi run calibrate \
  --port /dev/ttyACM0 \
  --robot-id lab_follower \
  --calibration-dir config/hardware
```

At the first pose prompt:

1. Put shoulder pan/lift, elbow, wrist flex, and wrist roll at their physical middle positions.
2. Close the pgripper completely without squeezing it under load.
3. Press Enter.
4. During the sweep, move joints 1–4 slowly through their safe travel and move the pgripper from fully closed to fully open. LeRobot intentionally excludes wrist roll from this sweep.

LeRobot 0.6.1 initially centers at 2047. The wrapper shifts EEPROM offsets and ranges by one tick.

The wrapper symmetrizes the body ranges around 2048. It maps the measured gripper open tick to 0 rad.

The stock LeRobot sweep excludes wrist roll. Its mechanical limiter is not necessarily centered on the upright jaw pose. After the full calibration, correct motor 5 alone:

```bash
pixi run calibrate \
  --port /dev/ttyACM0 \
  --robot-id lab_follower \
  --calibration-dir config/hardware \
  --wrist-roll-only
```

Keep the arm supported. First place the jaws visually upright. Then move wrist roll gently counterclockwise to one stop and clockwise to the other stop when prompted. The script keeps torque disabled, adds a 64-tick margin at both measured stops, displays the proposed asymmetric ROS range, and writes only motor 5 after the exact `WRITE_WRIST_EEPROM` confirmation. It refuses to start if motor 5 EEPROM does not match the saved JSON calibration.

The wrapper writes the aligned values back to EEPROM and saves:

```text
config/hardware/lab_follower.json                 # LeRobot calibration
config/hardware/lab_follower.state.yaml           # ROS read-only mapping; no EEPROM fields
config/hardware/lab_follower.control.yaml         # ROS motion + EEPROM/safety fields
config/hardware/lab_follower.joint_limits.yaml    # calibrated Xacro/controller limits
config/hardware/lab_follower.*.raw-lerobot.json   # untouched backup
config/hardware/lab_follower.*.previous.json      # prior aligned run, if present
```

## Gate 1: torque-off RViz mirroring

Do not run LeRobot at the same time; both programs need exclusive ownership of the serial bus.

For the calibrated follower on `/dev/ttyACM0`, use the read-only Pixi task:

```bash
pixi run real-rviz
```

The equivalent full command is:

```bash
source install/setup.bash
ros2 launch so_arm101_pgripper_description real.launch.py \
  port:=/dev/ttyACM0 \
  joint_config_file:=$PWD/config/hardware/lab_follower.state.yaml \
  joint_limits_file:=$PWD/config/hardware/lab_follower.joint_limits.yaml \
  read_only:=true
```

Move each limp body joint by hand. RViz must follow the same direction. The centered body pose must read 0 rad.

Open the limp gripper. RViz must show 0 rad. Closing the gripper must increase the ROS `gripper` value.

Stop on wraparound, a read timeout, a wrong joint, or a direction mismatch.

## Gate 2: torque-enabled motion

Only after Gate 1 passes:

```bash
source install/setup.bash
ros2 launch so_arm101_pgripper_description real.launch.py \
  port:=/dev/ttyACM0 \
  joint_config_file:=$PWD/config/hardware/lab_follower.control.yaml \
  joint_limits_file:=$PWD/config/hardware/lab_follower.joint_limits.yaml \
  read_only:=false \
  confirm_hardware:=MOVE_REAL_ARM
```

This activates `/arm_controller/follow_joint_trajectory`, the standard interface for MoveIt 2 and ROS agents. On activation the driver seeds commands from measured positions, then enables torque. Begin with supported, low-speed, single-joint goals well inside the calibrated limits.

## LeRobot, datasets, and a future leader

Use the same ID and directory in LeRobot so policies see the aligned JSON:

```bash
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/ttyACM0 \
  --robot.id=lab_follower \
  --robot.calibration_dir=$PWD/config/hardware
```

The follower remains a normal `so101_follower`: body observations/actions are degrees, while `gripper.pos` is 0–100. For the future WH148 leader, map potentiometer midpoints to the five body-joint zero positions and map the click button to gripper 0/100; dataset recording and ACT/VLA code can then retain LeRobot's standard feature names. Add that leader driver when the potentiometer ADC, electrical range, and button behavior are known.

## Optional gripper contact stop

The Feetech driver exposes each servo's raw `Present_Current` value as the `current` state interface. It also includes an opt-in gripper contact stop. When enabled, the driver filters gripper current, detects repeated high-current samples while the jaw position stalls, and holds the last measured position. Opening the gripper releases the latch.

The feature is disabled in `so_arm101_pgripper_follower.control.yaml`. The example threshold and torque cap are placeholders, not a safe grasp setting. Do not tune them with fingers. First use a compliant test object and observe the current stream:

```bash
pixi run real-rviz
# In another terminal, from this workspace:
pixi run bash -c 'source install/setup.bash && ros2 topic echo /dynamic_joint_states'
```

Record the empty-jaw baseline and the current at first contact. Set `contact_current_threshold` above the empty-jaw peak, set `contact_torque_limit` conservatively, then set `contact_stop_enabled: true` and rebuild. The stop limits force, but it is not a certified pinch-force controller. Keep the physical disconnect accessible during every test.

## Model and provenance

The body geometry and frames come from TheRobotStudio's SO-ARM101 description. The pgripper geometry and mimic kinematics come from NormaCore's ElRobot URDF. The measured pgripper mount is `xyz="0 0 0"`, `rpy="-pi/2 0 -pi/2"`. Full source revisions, modifications, and licenses are in [ATTRIBUTION.md](ATTRIBUTION.md).

Software validation does not prove the physical mount transform, wiring, bus integrity, or collision safety. Those require the two gates above on your actual arm.
