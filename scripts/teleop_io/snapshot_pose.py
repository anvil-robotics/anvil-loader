#!/usr/bin/env python3
"""Snapshot the arms' current /joint_states as a single-frame MCAP 'motion'.

Usage:
    python3 snapshot_pose.py /data/initial_stand.mcap
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    from mcap_ros2.writer import Writer
except ImportError:
    sys.exit(
        "Missing dependency: mcap-ros2-support.\n"
        "Install with: pip3 install --break-system-packages mcap-ros2-support"
    )

LEFT_TOPIC = "/follower_l_forward_position_controller/commands"
RIGHT_TOPIC = "/follower_r_forward_position_controller/commands"

LEFT_JOINTS = [
    "follower_l_joint1", "follower_l_joint2", "follower_l_joint3", "follower_l_joint4",
    "follower_l_joint5", "follower_l_joint6", "follower_l_joint7", "follower_l_finger_joint1",
]
RIGHT_JOINTS = [
    "follower_r_joint1", "follower_r_joint2", "follower_r_joint3", "follower_r_joint4",
    "follower_r_joint5", "follower_r_joint6", "follower_r_joint7", "follower_r_finger_joint1",
]

FLOAT64_MULTIARRAY_MSGDEF = """
MultiArrayLayout layout
float64[] data
================================================================================
MSG: std_msgs/MultiArrayLayout
MultiArrayDimension[] dim
uint32 data_offset
================================================================================
MSG: std_msgs/MultiArrayDimension
string label
uint32 size
uint32 stride
""".strip()


def read_current_joint_positions(node, joint_names, timeout_sec=5.0):
    result = {}

    def _cb(msg: JointState):
        for name, position in zip(msg.name, msg.position):
            result[name] = position

    sub = node.create_subscription(JointState, "/joint_states", _cb, 10)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and not all(j in result for j in joint_names):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)

    missing = [j for j in joint_names if j not in result]
    if missing:
        sys.exit(f"Timed out waiting for /joint_states — missing joints: {missing}")
    return [result[j] for j in joint_names]


def main():
    if len(sys.argv) != 2:
        sys.exit("Usage: python3 snapshot_pose.py <out_path>.mcap")
    out_path = sys.argv[1]

    rclpy.init()
    node = Node("snapshot_pose")

    left = read_current_joint_positions(node, LEFT_JOINTS)
    right = read_current_joint_positions(node, RIGHT_JOINTS)

    f = open(out_path, "wb")
    writer = Writer(f)
    schema = writer.register_msgdef(
        datatype="std_msgs/Float64MultiArray",
        msgdef_text=FLOAT64_MULTIARRAY_MSGDEF,
    )
    now = time.time_ns()
    writer.write_message(topic=LEFT_TOPIC, schema=schema, message=Float64MultiArray(data=left),
                          log_time=now, publish_time=now)
    writer.write_message(topic=RIGHT_TOPIC, schema=schema, message=Float64MultiArray(data=right),
                          log_time=now, publish_time=now)
    writer.finish()
    f.close()

    node.destroy_node()
    rclpy.shutdown()

    print(f"Saved: {out_path}")
    print(f"left:  {left}")
    print(f"right: {right}")


if __name__ == "__main__":
    main()
