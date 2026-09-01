# MoveIt Servo Teleoperation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add mock MoveIt Servo control through keyboard and PS4 inputs for the five-joint arm.

**Architecture:** A standalone Servo node consumes standard `TwistStamped` and `JointJog` topics. It sends `JointTrajectory` commands to the existing arm controller.

**Tech Stack:** ROS 2 Jazzy, MoveIt Servo 2.12.4, `rclpy`, `joy`, `ros2_control`, and Pixi.

**Spec:** `docs/superpowers/specs/2026-08-31-moveit-servo-teleop-design.md`

## Global Constraints

- The Servo planning group contains five arm joints.
- The gripper remains a separate sixth actuator.
- The mock launch does not open a serial port.
- The input topics remain `/servo_node/delta_twist_cmds` and `/servo_node/delta_joint_cmds`.
- The Servo output uses `/arm_controller/joint_trajectory`.

---

### Task 1: Servo configuration and launch

**Files:**
- Create: `src/so_arm101_pgripper_moveit_config/config/servo.yaml`
- Create: `src/so_arm101_pgripper_moveit_config/launch/servo_demo.launch.py`
- Modify: `src/so_arm101_pgripper_moveit_config/package.xml`
- Modify: `pixi.toml`
- Test: `src/so_arm101_pgripper_moveit_config/test/test_servo_config.py`

**Interfaces:**
- Consumes: the `arm` group, `robot_description`, and `/joint_states`
- Produces: `/arm_controller/joint_trajectory`

- [ ] **Step 1: Write configuration tests**

Assert the five-joint group, standard input topics, trajectory output topic, and mock-only launch.

- [ ] **Step 2: Run the test and observe a missing-file failure**

Run: `pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_servo_config.py -q`

Expected: FAIL because `config/servo.yaml` does not exist.

- [ ] **Step 3: Add the minimum Servo configuration and launch**

Use `moveit_servo::ServoNode`. Pass the existing MoveIt configuration and `servo.yaml` parameters.

- [ ] **Step 4: Run the configuration tests**

Run: `pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_servo_config.py -q`

Expected: PASS.

### Task 2: Keyboard and PS4 input

**Files:**
- Create: `src/so_arm101_pgripper_moveit_config/scripts/servo_keyboard_input.py`
- Create: `src/so_arm101_pgripper_moveit_config/scripts/servo_joy_input.py`
- Create: `src/so_arm101_pgripper_moveit_config/config/ps4.yaml`
- Modify: `src/so_arm101_pgripper_moveit_config/CMakeLists.txt`
- Modify: `src/so_arm101_pgripper_moveit_config/package.xml`
- Test: `src/so_arm101_pgripper_moveit_config/test/test_servo_inputs.py`

**Interfaces:**
- Consumes: terminal keys or `sensor_msgs/Joy`
- Produces: stamped commands on the two standard Servo input topics

- [ ] **Step 1: Write input mapping tests**

Assert that Cartesian mappings set the correct frame and components. Assert that joint mappings use only five arm joints.

- [ ] **Step 2: Run the test and observe an import failure**

Run: `pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_servo_inputs.py -q`

Expected: FAIL because the input scripts do not exist.

- [ ] **Step 3: Add stateless mapping functions and ROS publishers**

Keep key and PS4 mapping logic in the executable files. Publish only while the deadman input is active.

- [ ] **Step 4: Run the input tests**

Run: `pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_servo_inputs.py -q`

Expected: PASS.

### Task 3: Instructions and full verification

**Files:**
- Modify: `src/so_arm101_pgripper_moveit_config/README.md`
- Modify: `pixi.toml`

**Interfaces:**
- Consumes: the completed mock Servo launch and input executables
- Produces: Pixi tasks for Servo, keyboard, and PS4 operation

- [ ] **Step 1: Add mock start and control instructions**

Document the 5+1 boundary, input maps, command frames, and mock-only limit.

- [ ] **Step 2: Build and run all tests**

Run: `pixi run build && pixi run test`

Expected: all packages build and all tests pass.

- [ ] **Step 3: Start the headless Servo launch**

Run: `pixi run moveit-servo-demo -- use_rviz:=false input_device:=none`

Expected: Servo starts, controllers become active, and no serial device opens.

