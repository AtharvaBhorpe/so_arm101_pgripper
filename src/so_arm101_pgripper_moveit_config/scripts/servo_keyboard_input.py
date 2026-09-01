#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


GRIPPER_OPEN_POSITION = 0.0
GRIPPER_CLOSED_POSITION = 2.2028


TWIST_KEYS = {
    "w": (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
    "d": (0.0, -1.0, 0.0, 0.0, 0.0, 0.0),
    "r": (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
    "f": (0.0, 0.0, -1.0, 0.0, 0.0, 0.0),
    "u": (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
    "o": (0.0, 0.0, 0.0, -1.0, 0.0, 0.0),
    "i": (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
    "k": (0.0, 0.0, 0.0, 0.0, -1.0, 0.0),
    "j": (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    "l": (0.0, 0.0, 0.0, 0.0, 0.0, -1.0),
}
JOINT_KEYS = {
    "1": ("shoulder_pan", 1.0),
    "q": ("shoulder_pan", -1.0),
    "2": ("shoulder_lift", 1.0),
    "w": ("shoulder_lift", -1.0),
    "3": ("elbow_flex", 1.0),
    "e": ("elbow_flex", -1.0),
    "4": ("wrist_flex", 1.0),
    "r": ("wrist_flex", -1.0),
    "5": ("wrist_roll", 1.0),
    "t": ("wrist_roll", -1.0),
}


def twist_for_key(key):
    return TWIST_KEYS.get(key)


def joint_for_key(key):
    return JOINT_KEYS.get(key)


def next_gripper_position(current_position, open_position=GRIPPER_OPEN_POSITION, closed_position=GRIPPER_CLOSED_POSITION):
    midpoint = (open_position + closed_position) / 2.0
    return open_position if current_position > midpoint else closed_position


class KeyboardInput(Node):
    def __init__(self):
        super().__init__("servo_keyboard_input")
        self.declare_parameter("command_frame", "base_link")
        self.frame = self.get_parameter("command_frame").value
        self.mode = "twist"
        self.selected_mode = None
        self.pending_mode = None
        self.gripper_position = None
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.joint_pub = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.mode_client = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_timer(0.2, lambda: self.select_mode(self.mode))

    def select_mode(self, mode):
        if self.pending_mode is not None:
            return False
        if mode == self.selected_mode:
            return True
        if not self.mode_client.service_is_ready():
            self.get_logger().warning("The Servo mode service is not ready.")
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

    def publish_twist(self, values):
        if not self.select_mode("twist"):
            return
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame
        (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
            message.twist.angular.x,
            message.twist.angular.y,
            message.twist.angular.z,
        ) = values
        self.twist_pub.publish(message)

    def publish_joint(self, name, velocity):
        if not self.select_mode("joint"):
            return
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = [name]
        message.velocities = [velocity]
        self.joint_pub.publish(message)

    def toggle_frame(self):
        self.frame = "gripper_frame_link" if self.frame == "base_link" else "base_link"
        self.get_logger().info(f"Command frame: {self.frame}")

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


HELP = """
Cartesian mode: w/s x, a/d y, r/f z, u/o roll, i/k pitch, j/l yaw
Joint mode: 1/q pan, 2/w lift, 3/e elbow, 4/r wrist flex, 5/t wrist roll
g toggles the gripper. c changes the command frame. m changes the mode. Space stops. x exits.
"""


def main():
    rclpy.init()
    node = KeyboardInput()
    stream = sys.stdin
    owned_stream = None
    if not stream.isatty():
        owned_stream = open("/dev/tty", "r")
        stream = owned_stream
    old_settings = termios.tcgetattr(stream)
    print(HELP)
    try:
        tty.setcbreak(stream.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if not select.select([stream], [], [], 0.05)[0]:
                continue
            key = stream.read(1)
            if key == "x":
                break
            if key == "m":
                node.mode = "joint" if node.mode == "twist" else "twist"
                node.get_logger().info(f"Input mode: {node.mode}")
            elif key == "g":
                node.toggle_gripper()
            elif key == "c":
                node.toggle_frame()
            elif key == " ":
                if node.mode == "twist":
                    node.publish_twist((0.0,) * 6)
                else:
                    node.publish_joint("shoulder_pan", 0.0)
            elif node.mode == "twist" and (values := twist_for_key(key)) is not None:
                node.publish_twist(values)
            elif node.mode == "joint" and (joint := joint_for_key(key)) is not None:
                node.publish_joint(*joint)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(stream, termios.TCSADRAIN, old_settings)
        if owned_stream is not None:
            owned_stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
