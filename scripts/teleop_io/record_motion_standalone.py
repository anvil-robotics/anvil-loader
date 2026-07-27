#!/usr/bin/env python3
"""Standalone motion recorder — no webapp, no session, just a file path.

Subscribes directly to the arm command topics and writes them to an MCAP
file you can replay later with replay_motion.py. Run it, teleop the motion,
Ctrl+C to stop and finalize the file.

Usage:
    python3 record_motion_standalone.py out.mcap
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
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

# Concatenated ROS2 msg IDL (top-level type + '==='-separated nested types),
# the format mcap_ros2's serialize_dynamic() expects.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out_path")
    args = parser.parse_args()

    rclpy.init()
    node = Node("motion_recorder")

    f = open(args.out_path, "wb")
    writer = Writer(f)
    schema = writer.register_msgdef(
        datatype="std_msgs/Float64MultiArray",
        msgdef_text=FLOAT64_MULTIARRAY_MSGDEF,
    )

    def make_cb(topic):
        def _cb(msg):
            writer.write_message(
                topic=topic, schema=schema, message=msg,
                log_time=time.time_ns(), publish_time=time.time_ns(),
            )
            print(f"{topic}: {list(msg.data)}", end="\r")
        return _cb

    node.create_subscription(Float64MultiArray, LEFT_TOPIC, make_cb(LEFT_TOPIC), 10)
    node.create_subscription(Float64MultiArray, RIGHT_TOPIC, make_cb(RIGHT_TOPIC), 10)

    print(f"Recording {LEFT_TOPIC} and {RIGHT_TOPIC} to {args.out_path}")
    print("Teleop now. Press Ctrl+C to stop.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        writer.finish()
        f.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"\nSaved: {args.out_path}")


if __name__ == "__main__":
    main()
