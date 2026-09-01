# SO-ARM101 Pgripper MoveIt 2 Mock Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` or `superpowers:executing-plans`. Complete each task in sequence.

**Goal:** Add a MoveIt 2 mock demo for the five-joint arm and the independent gripper.

**Architecture:** A new MoveIt configuration package wraps the existing robot Xacro. Separate trajectory controllers operate the arm and gripper.

**Tech Stack:** ROS 2 Jazzy, MoveIt 2, OMPL, `pick_ik`, `ros2_control`, Xacro, RViz, Pixi, and pytest.

**Spec:** `docs/superpowers/specs/2026-08-31-moveit2-mock-demo-design.md`

## Global Constraints

- Keep the arm planning group at exactly five joints.
- Keep `gripper` outside the arm planning group.
- Keep both prismatic jaw joints as mimic joints.
- Use `pick_ik` with position and rotation costs.
- Use mock hardware only in `demo.launch.py`.
- Do not load `feetech_ros2_driver` from the mock launch.
- Do not add MoveIt Servo configuration.
- Use `agent-ste` for all new documentation and messages.
- Do not run physical calibration or write motor EEPROM.

## File Map

- `pixi.toml` adds the MoveIt 2 and `pick_ik` dependencies and the demo task.
- `src/so_arm101_pgripper_moveit_config/CMakeLists.txt` installs the configuration package.
- `src/so_arm101_pgripper_moveit_config/package.xml` declares the runtime dependencies.
- `src/so_arm101_pgripper_moveit_config/config/so_arm101_pgripper.urdf.xacro` wraps the existing description.
- `src/so_arm101_pgripper_moveit_config/config/so_arm101_pgripper.srdf` defines the two planning groups.
- `src/so_arm101_pgripper_moveit_config/config/kinematics.yaml` configures weighted `pick_ik`.
- `src/so_arm101_pgripper_moveit_config/config/joint_limits.yaml` defines MoveIt velocity and acceleration limits.
- `src/so_arm101_pgripper_moveit_config/config/ompl_planning.yaml` defines the OMPL planner set.
- `src/so_arm101_pgripper_moveit_config/config/moveit_controllers.yaml` maps MoveIt to two action servers.
- `src/so_arm101_pgripper_moveit_config/config/ros2_controllers.yaml` defines two mock trajectory controllers.
- `src/so_arm101_pgripper_moveit_config/config/moveit.rviz` configures the MotionPlanning panel.
- `src/so_arm101_pgripper_moveit_config/launch/demo.launch.py` starts the complete mock demo.
- `src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py` tests the configuration contracts.
- `src/so_arm101_pgripper_moveit_config/README.md` gives the demo procedure and explains the 5-DoF limit.

---

### Task 1: Add the MoveIt package and dependency contract

**Files:**

- Modify: `pixi.toml`
- Create: `src/so_arm101_pgripper_moveit_config/CMakeLists.txt`
- Create: `src/so_arm101_pgripper_moveit_config/package.xml`
- Create: `src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py`

**Interfaces:**

- Consumes: The existing `so_arm101_pgripper_description` package.
- Produces: An installable `so_arm101_pgripper_moveit_config` package.

- [ ] **Step 1: Write the failing package test**

Create a test that requires all planned files and both package names.

```python
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]


def test_required_moveit_files_exist():
    required = {
        "config/so_arm101_pgripper.urdf.xacro",
        "config/so_arm101_pgripper.srdf",
        "config/kinematics.yaml",
        "config/joint_limits.yaml",
        "config/ompl_planning.yaml",
        "config/moveit_controllers.yaml",
        "config/ros2_controllers.yaml",
        "config/moveit.rviz",
        "launch/demo.launch.py",
    }
    assert {path for path in required if not (PACKAGE / path).is_file()} == set()
```

- [ ] **Step 2: Run the test and make sure that it fails**

Run:

```bash
pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py -q
```

Expected result: The test reports the missing configuration files.

- [ ] **Step 3: Add the dependencies and package files**

Add these Pixi dependencies:

```toml
ros-jazzy-moveit = "*"
ros-jazzy-moveit-configs-utils = "*"
ros-jazzy-moveit-planners-ompl = "*"
ros-jazzy-pick-ik = "*"
```

Add this Pixi task:

```toml
moveit-demo = { cmd = "bash -c 'source install/setup.bash && ros2 launch so_arm101_pgripper_moveit_config demo.launch.py'", depends-on = ["build"] }
```

Install `config`, `launch`, and `README.md` from `CMakeLists.txt`. Register the pytest file with `ament_add_pytest_test`.

- [ ] **Step 4: Resolve the lock file and build the package**

Run:

```bash
pixi lock
pixi run colcon build --symlink-install --packages-select so_arm101_pgripper_description so_arm101_pgripper_moveit_config
```

Expected result: Pixi resolves Jazzy `pick_ik`. Colcon builds both packages.

---

### Task 2: Define the 5+1 planning and controller configuration

**Files:**

- Create: `src/so_arm101_pgripper_moveit_config/config/so_arm101_pgripper.urdf.xacro`
- Create: `src/so_arm101_pgripper_moveit_config/config/so_arm101_pgripper.srdf`
- Create: `src/so_arm101_pgripper_moveit_config/config/kinematics.yaml`
- Create: `src/so_arm101_pgripper_moveit_config/config/joint_limits.yaml`
- Create: `src/so_arm101_pgripper_moveit_config/config/ompl_planning.yaml`
- Create: `src/so_arm101_pgripper_moveit_config/config/moveit_controllers.yaml`
- Create: `src/so_arm101_pgripper_moveit_config/config/ros2_controllers.yaml`
- Modify: `src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py`

**Interfaces:**

- Consumes: The six command joints from the description Xacro.
- Produces: A five-joint arm group and a one-joint gripper group.

- [ ] **Step 1: Add failing semantic and controller tests**

Parse the XML and YAML files. Test these exact joint lists:

```python
ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
GRIPPER_JOINTS = ["gripper"]
MIMIC_JOINTS = {"pgripper_left_finger_joint", "pgripper_right_finger_joint"}
```

The tests must make sure that the following rules are true:

- The `arm` group resolves to `ARM_JOINTS`.
- The `gripper` group equals `GRIPPER_JOINTS`.
- The two controller files use the same joint lists.
- No controller contains a mimic joint.
- The arm solver equals `pick_ik/PickIkPlugin`.
- `position_scale` and `rotation_scale` are greater than zero.

- [ ] **Step 2: Run the tests and make sure that they fail**

Run:

```bash
pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py -q
```

Expected result: The tests fail because the semantic files do not exist.

- [ ] **Step 3: Add the robot wrapper and SRDF**

The wrapper must include the existing Xacro with these mappings:

```xml
<xacro:arg name="use_ros2_control" default="true"/>
<xacro:arg name="ros2_control_plugin" default="mock_components/GenericSystem"/>
<xacro:include filename="$(find so_arm101_pgripper_description)/urdf/so_arm101_pgripper.urdf.xacro"/>
```

Define `arm` as a chain from `base_link` to `gripper_frame_link`. Define `gripper` with only the `gripper` joint.

Add named states for `home`, `open`, and `closed`. Mark both jaw joints as passive mimic joints.

- [ ] **Step 4: Add weighted IK and OMPL**

Use this solver configuration:

```yaml
arm:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.1
  mode: local
  position_scale: 1.0
  rotation_scale: 0.4
  position_threshold: 0.005
  orientation_threshold: 0.08
  cost_threshold: 0.01
  minimal_displacement_weight: 0.001
```

Use OMPL as the only planning pipeline. Use `RRTConnectkConfigDefault` as the default planner.

- [ ] **Step 5: Add separate mock controllers**

Configure `arm_controller` with `ARM_JOINTS`. Configure `gripper_controller` with `GRIPPER_JOINTS`.

Use `FollowJointTrajectory` for both MoveIt controller mappings. Set both `ros2_control` controllers to `JointTrajectoryController`.

- [ ] **Step 6: Run the focused tests**

Run:

```bash
pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py -q
```

Expected result: All semantic, IK, mimic-joint, and controller tests pass.

---

### Task 3: Add the mock demo launch and RViz configuration

**Files:**

- Create: `src/so_arm101_pgripper_moveit_config/launch/demo.launch.py`
- Create: `src/so_arm101_pgripper_moveit_config/config/moveit.rviz`
- Modify: `src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py`

**Interfaces:**

- Consumes: The configuration from Task 2.
- Produces: `ros2 launch so_arm101_pgripper_moveit_config demo.launch.py`.

- [ ] **Step 1: Add a failing launch test**

Load `demo.launch.py` as a Python module. Make sure that `generate_launch_description()` returns a launch description.

The test must also search the launch source for these controller names:

```python
{"joint_state_broadcaster", "arm_controller", "gripper_controller"}
```

The test must reject `feetech_ros2_driver` and serial device paths.

- [ ] **Step 2: Run the launch test and make sure that it fails**

Run:

```bash
pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py -q
```

Expected result: The launch test fails because `demo.launch.py` does not exist.

- [ ] **Step 3: Add the demo launch**

Use `MoveItConfigsBuilder` to load the wrapper Xacro, SRDF, kinematics, OMPL, limits, and controller configuration.

Start these nodes:

1. `static_transform_publisher`
2. `robot_state_publisher`
3. `ros2_control_node`
4. `joint_state_broadcaster` spawner
5. `arm_controller` spawner
6. `gripper_controller` spawner
7. `move_group`
8. `rviz2`

Pass the same MoveIt parameter dictionaries to `move_group` and RViz.

- [ ] **Step 4: Add the RViz configuration**

Set `base_link` as the fixed frame. Add the RobotModel and MotionPlanning displays.

Set `arm` as the default planning group. Enable approximate IK solutions in the MotionPlanning display.

- [ ] **Step 5: Run the focused tests**

Run:

```bash
pixi run pytest src/so_arm101_pgripper_moveit_config/test/test_moveit_config.py -q
```

Expected result: The complete configuration test passes.

---

### Task 4: Document and test the complete demo

**Files:**

- Create: `src/so_arm101_pgripper_moveit_config/README.md`
- Modify: `src/so_arm101_pgripper_description/README.md`
- Modify: `outputs/so_arm101_pgripper_description.tar.gz`

**Interfaces:**

- Consumes: The working demo launch.
- Produces: User instructions and the updated archive.

- [ ] **Step 1: Write the README files with `agent-ste`**

Document these commands:

```bash
pixi run build
pixi run moveit-demo
```

Explain the five-joint orientation limit. Explain that the mock launch does not connect to motors.

Add this warning before all real-hardware instructions:

```text
CAUTION: Do not start the real driver during this demo. Two drivers can send conflicting motor commands.
```

- [ ] **Step 2: Build and run all tests**

Run:

```bash
pixi run build
pixi run test
pixi run check
```

Expected result: All packages build. All tests pass. The Xacro parser reports a valid robot tree.

- [ ] **Step 3: Run a bounded launch test without RViz**

Add a `use_rviz:=false` launch argument. Start the launch for 20 seconds.

Make sure that these components become active:

- `move_group`
- `joint_state_broadcaster`
- `arm_controller`
- `gripper_controller`

Stop the launch after the component status appears. Make sure that no serial port opens.

- [ ] **Step 4: Rebuild and inspect the delivery archive**

Exclude `.pixi`, `build`, `install`, and `log` from the archive. Make sure that the archive contains the new package.

Record the SHA-256 value for the archive in the completion report.

## Commit Policy

This workspace contains a read-only `.git` placeholder. It is not a Git repository.

Do not run commit commands. If the user initializes a repository, commit each completed task separately.
