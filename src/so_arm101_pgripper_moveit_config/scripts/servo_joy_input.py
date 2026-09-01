#!/usr/bin/env python3
from typing import NamedTuple

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = (
    "elbow_flex",
    "shoulder_lift",
    "shoulder_pan",
    "wrist_flex",
    "wrist_roll",
)
GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSED_POSITION = 2.2028


class Command(NamedTuple):
    kind: str
    names: tuple
    values: tuple


def _value(values, index):
    return float(values[index]) if index < len(values) else 0.0


def _deadzone(value, limit=0.10):
    return 0.0 if abs(value) < limit else value


def next_gripper_position(current_position, open_position=GRIPPER_OPEN_POSITION, closed_position=GRIPPER_CLOSED_POSITION):
    midpoint = (open_position + closed_position) / 2.0
    return open_position if current_position > midpoint else closed_position


def gripper_toggle_pressed(buttons, previous_down, deadman_button=4, gripper_toggle_button=0):
    return (
        _value(buttons, deadman_button) >= 0.5
        and _value(buttons, gripper_toggle_button) >= 0.5
        and not previous_down
    )


def command_from_joy(axes, buttons, deadman_button=4, joint_mode_button=5, deadzone=0.10):
    if _value(buttons, deadman_button) < 0.5:
        return None

    trigger = (_value(axes, 5) - _value(axes, 2)) / 2.0
    if _value(buttons, joint_mode_button) >= 0.5:
        by_name = {
            "shoulder_pan": _value(axes, 0),
            "shoulder_lift": _value(axes, 1),
            "elbow_flex": _value(axes, 3),
            "wrist_flex": _value(axes, 4),
            "wrist_roll": trigger,
        }
        values = tuple(_deadzone(by_name[name], deadzone) for name in JOINT_NAMES)
        return Command("joint", JOINT_NAMES, values)

    values = (
        _deadzone(_value(axes, 1), deadzone),
        _deadzone(_value(axes, 0), deadzone),
        _deadzone(trigger, deadzone),
        _deadzone(_value(axes, 4), deadzone),
        _deadzone(_value(axes, 3), deadzone),
        _deadzone(_value(axes, 6), deadzone),
    )
    return Command("twist", (), values)


class JoyInput(Node):
    def __init__(self):
        super().__init__("servo_joy_input")
        self.declare_parameter("command_frame", "base_link")
        self.declare_parameter("deadman_button", 4)
        self.declare_parameter("joint_mode_button", 5)
        self.declare_parameter("frame_button", 3)
        self.declare_parameter("gripper_toggle_button", 0)
        self.declare_parameter("deadzone", 0.10)
        self.frame = self.get_parameter("command_frame").value
        self.deadman_button = self.get_parameter("deadman_button").value
        self.joint_mode_button = self.get_parameter("joint_mode_button").value
        self.frame_button = self.get_parameter("frame_button").value
        self.gripper_toggle_button = self.get_parameter("gripper_toggle_button").value
        self.deadzone = self.get_parameter("deadzone").value
        self.frame_button_was_down = False
        self.gripper_button_was_down = False
        self.gripper_position = None
        self.selected_mode = None
        self.pending_mode = None
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.mode_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_timer(0.2, lambda: self.select_mode("twist"))

    def select_mode(self, mode):
        if self.pending_mode is not None:
            return False
        if mode == self.selected_mode:
            return True
        if not self.mode_client.service_is_ready():
            return False
        request = ServoCommandType.Request()
        request.command_type = request.TWIST if mode == "twist" else request.JOINT_JOG
        self.pending_mode = mode
        future = self.mode_client.call_async(request)
        future.add_done_callback(lambda result, requested_mode=mode: self.on_mode_response(requested_mode, result))
        return False

    def on_mode_response(self, requested_mode, future):
        self.pending_mode = None
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().warning(f"Servo mode switch failed: {error}")
            return
        if response.success:
            self.selected_mode = requested_mode
            return
        self.get_logger().warning(f"Servo rejected {requested_mode} input mode.")

    def on_joy(self, message):
        frame_button_down = _value(message.buttons, self.frame_button) >= 0.5
        if frame_button_down and not self.frame_button_was_down:
            self.frame = "gripper_frame_link" if self.frame == "base_link" else "base_link"
            self.get_logger().info(f"Command frame: {self.frame}")
        self.frame_button_was_down = frame_button_down

        gripper_button_down = _value(message.buttons, self.gripper_toggle_button) >= 0.5
        if gripper_toggle_pressed(
            message.buttons,
            self.gripper_button_was_down,
            self.deadman_button,
            self.gripper_toggle_button,
        ):
            self.toggle_gripper()
            self.gripper_button_was_down = gripper_button_down
            return
        self.gripper_button_was_down = gripper_button_down

        command = command_from_joy(
            message.axes,
            message.buttons,
            self.deadman_button,
            self.joint_mode_button,
            self.deadzone,
        )
        if command is None or not self.select_mode(command.kind):
            return

        stamp = self.get_clock().now().to_msg()
        if command.kind == "twist":
            output = TwistStamped()
            output.header.stamp = stamp
            output.header.frame_id = self.frame
            (
                output.twist.linear.x,
                output.twist.linear.y,
                output.twist.linear.z,
                output.twist.angular.x,
                output.twist.angular.y,
                output.twist.angular.z,
            ) = command.values
            self.twist_pub.publish(output)
        else:
            output = JointJog()
            output.header.stamp = stamp
            output.joint_names = list(command.names)
            output.velocities = list(command.values)
            self.joint_pub.publish(output)

    def on_joint_state(self, message):
        if "gripper" in message.name:
            self.gripper_position = message.position[message.name.index("gripper")]

    def toggle_gripper(self):
        if self.gripper_position is None:
            self.get_logger().warning("Waiting for gripper joint state.")
            return
        target = next_gripper_position(self.gripper_position)
        message = JointTrajectory()
        message.joint_names = ["gripper"]
        point = JointTrajectoryPoint()
        point.positions = [target]
        point.time_from_start.sec = 1
        message.points = [point]
        self.gripper_pub.publish(message)
        self.gripper_position = target
        self.get_logger().info("Gripper opening" if target == GRIPPER_OPEN_POSITION else "Gripper closing")


def main():
    rclpy.init()
    node = JoyInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
