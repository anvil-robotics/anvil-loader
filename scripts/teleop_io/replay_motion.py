#!/usr/bin/env python3
"""Replay a recorded teleop episode straight to the arm position controllers.

Reads /follower_l_forward_position_controller/commands and
/follower_r_forward_position_controller/commands (std_msgs/Float64MultiArray)
out of a recorded episode's MCAP file and republishes them at their original
relative timing. This mirrors exactly what pico4_teleop_controller.py sends
during live teleop, so no VR headset or pico4-teleop container is needed —
only the `ros2` container (controllers + CAN hardware interface) must be up.

Before playing the recorded stream, the arms are ramped from their current
/joint_states position to the episode's first recorded command over
--lead-in seconds, so replay never snaps the arms straight to the start pose.
After the recorded stream finishes, the arms are ramped back to the
--stand-pose file (default /data/initial_stand.mcap, see snapshot_pose.py)
over --lead-out seconds, so every replay ends in the same known pose.

Requires `mcap-ros2-support` (not installed in the ros2 image by default):
    pip3 install --break-system-packages mcap-ros2-support

Usage:
    python3 replay_motion.py /data/recordings/<dataset>/<episode>/<episode>_0.mcap
    python3 replay_motion.py <path>.mcap --rate 0.5       # half speed
    python3 replay_motion.py <path>.mcap --lead-in 4.0    # slower ramp-in
    python3 replay_motion.py <path>.mcap --lead-out 0     # skip return-to-stand
    python3 replay_motion.py <path>.mcap --dry-run        # print instead of publish
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    from mcap_ros2.reader import read_ros2_messages
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


def load_episode(mcap_path: str):
    """Return sorted (timestamp_ns, topic, data_list) tuples for both arms."""
    events = []
    for msg in read_ros2_messages(mcap_path, topics=[LEFT_TOPIC, RIGHT_TOPIC]):
        events.append((msg.log_time_ns, msg.channel.topic, list(msg.ros_msg.data)))
    events.sort(key=lambda e: e[0])
    return events


def read_current_joint_positions(node: Node, joint_names: list, timeout_sec: float = 5.0):
    """Block until /joint_states has a fresh sample, then return positions in joint_names order."""
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


def ramp(publishers, left_start, right_start, left_target, right_target, duration, hz=50):
    """Publish an interpolated ramp between two joint poses over `duration` seconds."""
    steps = max(1, int(duration * hz))
    for i in range(1, steps + 1):
        alpha = i / steps
        left = [a + (b - a) * alpha for a, b in zip(left_start, left_target)]
        right = [a + (b - a) * alpha for a, b in zip(right_start, right_target)]
        publishers[LEFT_TOPIC].publish(Float64MultiArray(data=left))
        publishers[RIGHT_TOPIC].publish(Float64MultiArray(data=right))
        time.sleep(1.0 / hz)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap_path")
    parser.add_argument("--rate", type=float, default=1.0, help="playback speed multiplier (1.0 = real time)")
    parser.add_argument("--lead-in", type=float, default=2.0,
                         help="seconds to ramp from current position to the episode's first pose (0 to disable)")
    parser.add_argument("--lead-out", type=float, default=2.0,
                         help="seconds to ramp from the episode's last pose back to --stand-pose (0 to disable)")
    parser.add_argument("--stand-pose", default="/data/initial_stand.mcap",
                         help="MCAP file (see snapshot_pose.py) to return to after replay")
    parser.add_argument("--dry-run", action="store_true", help="print commands instead of publishing")
    args = parser.parse_args()

    events = load_episode(args.mcap_path)
    if not events:
        sys.exit(f"No {LEFT_TOPIC} or {RIGHT_TOPIC} messages found in {args.mcap_path}")

    left_target = next((data for _, topic, data in events if topic == LEFT_TOPIC), None)
    right_target = next((data for _, topic, data in events if topic == RIGHT_TOPIC), None)

    print(f"Loaded {len(events)} commands spanning "
          f"{(events[-1][0] - events[0][0]) / 1e9:.2f}s from {args.mcap_path}")

    if args.dry_run:
        t0 = events[0][0]
        for t_ns, topic, data in events:
            print(f"t={(t_ns - t0) / 1e9:8.3f}s  {topic}  {data}")
        return

    rclpy.init()
    node = Node("motion_replay")
    left_pub = node.create_publisher(Float64MultiArray, LEFT_TOPIC, 10)
    right_pub = node.create_publisher(Float64MultiArray, RIGHT_TOPIC, 10)
    publishers = {LEFT_TOPIC: left_pub, RIGHT_TOPIC: right_pub}

    print("Replay starting in 2s — hands clear of the workspace.")
    time.sleep(2.0)

    if args.lead_in > 0:
        print("Reading current joint positions...")
        left_start = read_current_joint_positions(node, LEFT_JOINTS)
        right_start = read_current_joint_positions(node, RIGHT_JOINTS)
        print(f"Ramping to episode start pose over {args.lead_in}s...")
        ramp(publishers, left_start, right_start,
             left_target or left_start, right_target or right_start, args.lead_in)

    last_left, last_right = left_target, right_target
    t0_ns = events[0][0]
    wall_start = time.monotonic()
    try:
        for t_ns, topic, data in events:
            target_elapsed = (t_ns - t0_ns) / 1e9 / args.rate
            actual_elapsed = time.monotonic() - wall_start
            sleep_for = target_elapsed - actual_elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)
            publishers[topic].publish(Float64MultiArray(data=data))
            if topic == LEFT_TOPIC:
                last_left = data
            else:
                last_right = data
    except KeyboardInterrupt:
        print("\nReplay interrupted.")
    finally:
        if args.lead_out > 0:
            stand_events = load_episode(args.stand_pose)
            stand_left = next((d for _, t, d in stand_events if t == LEFT_TOPIC), None)
            stand_right = next((d for _, t, d in stand_events if t == RIGHT_TOPIC), None)
            if stand_left and stand_right:
                print(f"Ramping back to stand pose over {args.lead_out}s...")
                ramp(publishers, last_left or stand_left, last_right or stand_right,
                     stand_left, stand_right, args.lead_out)
            else:
                print(f"WARNING: no commands found in --stand-pose {args.stand_pose}, skipping lead-out.")
        node.destroy_node()
        rclpy.shutdown()
        print("Replay finished.")


if __name__ == "__main__":
    main()
