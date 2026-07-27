#!/usr/bin/env python3
"""Native Pico4 teleop controller using the Quest IK stack.

This module replaces the old Quest TCP bridge with a ROS2-native subscriber.
Instead of parsing 112-byte TCP packets from a Quest headset, it subscribes to
``/xr_pose`` (``xr_msgs/Custom`` published by the picoxr talker running on the
Pico4).  All downstream logic — PyBullet nullspace IK via
``AbsoluteControlModality``, velocity limiting, clutch, recording shortcuts, and
rehome — is identical to ``quest_teleop_controller.py``.

Key design decisions:
    - No TCP socket, no ADB health checks, no fake-adb.sh, no bridge adapter.
    - The controls ownership requester string is hardcoded to ``"quest_teleop"``
      so the existing ownership manager recognises this node without any config
      changes.
    - ``ROBOT_DESCRIPTION_URDF`` must point to a path inside
      ``/workspace/ros2/src/quest_teleop/config/`` so PyBullet's parent-directory
      mesh search can reach ``openarm_description/meshes/`` two levels up.

Requirements:
    xr_msgs must be built for Python 3.12 / ROS2 Jazzy — see
    ``Dockerfile.pico4teleop``.
"""

import os
import threading
import time
from typing import Self

import numpy as np
from amplitude import Amplitude
import pybullet as p
import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from anvil_msgs.action import Reset
from anvil_msgs.msg import CommandedEEPose, ControlsOwner, RecordingStatus
from anvil_msgs.srv import SwitchControlsOwner
from control.workcell_config import ArmsControlConfig, load_arms_control_config
from anvil_metrics import InfluxDbWriter
from quest_teleop.absolute_control_modality import AbsoluteControlModality
from quest_teleop.quest_metrics import QuestMetrics
from quest_teleop.utils import RobotConfig, WebAppClient, publish_pose
from xr_msgs.msg import Custom
import json as _json
import urllib.request as _urllib_request


_RECORDING_BUTTON_COOLDOWN = 1.5  # seconds — prevents button bounce from firing multiple requests
# Tracking-loss debounce: only report a controller as "not tracked" after this
# many consecutive frames of status==0, so single-frame glitches don't lock the
# arm and force a reclutch.
_TRACKING_LOSS_FRAMES = 3


def _get_default_session_id(base_url: str) -> int | None:
    """Return the current default session ID from the webapp, or None if unset."""
    try:
        url = f"{base_url.rstrip('/')}/api/default-session"
        with _urllib_request.urlopen(url, timeout=2) as resp:
            data = _json.loads(resp.read())
            return data["id"] if data and "id" in data else None
    except Exception:
        return None


def create_robot_config(arms_config: ArmsControlConfig) -> RobotConfig:
    """Build a ``RobotConfig`` for a known arm type.

    Encodes per-robot constants (rest positions, gripper travel, and the two
    fixed-orientation transforms) that ``AbsoluteControlModality`` requires for
    nullspace IK.

    Args:
        arms_config: Loaded workcell configuration that carries the arm type
            string and per-arm joint/controller mappings.

    Returns:
        A fully populated ``RobotConfig`` instance for the given arm type.

    Raises:
        ValueError: If ``arms_config.arm_type`` is not ``"openarm"`` or
            ``"openyam"``.
    """
    if arms_config.arm_type == "openarm":
        return RobotConfig(
            arms_control_config=arms_config,
            rest_position_left=[0.0, -0.174, 0.0, 1.5708, 0.0, 0.0, 0.0, 0.05],
            rest_position_right=[0.0, 0.174, 0.0, 1.5708, 0.0, 0.0, 0.0, 0.05],
            gripper_max=0.05,
            controller_to_gripper_matrix=np.array([
                [0, -1, 0, 0],
                [-1, 0, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]),
            # Pivot 5 cm back along the controller's Z axis so IK targets the
            # physical grip point rather than the tracked sensor origin.
            controller_T_pivot=np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, -0.05],
                [0, 0, 0, 1],
            ]),
        )
    elif arms_config.arm_type == "openyam":
        return RobotConfig(
            arms_control_config=arms_config,
            rest_position_left=[0.0, 0.873, 0.873, -0.698, 0.0, 0.0, 0.044],
            rest_position_right=[0.0, 0.873, 0.873, -0.698, 0.0, 0.0, 0.044],
            gripper_max=0.044,
            controller_to_gripper_matrix=np.array([
                [1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 0],
                [0, 0, 0, 1],
            ]),
            # Identity pivot — openyam sensor origin already coincides with grip.
            controller_T_pivot=np.array([
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]),
        )
    else:
        raise ValueError(f"Unknown arm_type: {arms_config.arm_type}")


def load_robot_config(config_path: str) -> RobotConfig:
    """Resolve, load, and convert an arms control config file to a ``RobotConfig``.

    Relative paths are anchored to ``/config/`` so the function works both
    inside Docker (where configs are mounted there) and in local dev when an
    absolute path is supplied directly.

    Args:
        config_path: Absolute or relative path to the YAML arms control config.
            May be set via the ``arms_control_config_file`` ROS parameter or the
            ``ARMS_CONTROL_CONFIG_FILE`` environment variable.

    Returns:
        A ``RobotConfig`` built from the parsed arms control configuration.

    Raises:
        ValueError: If ``config_path`` is an empty string.
    """
    if not config_path:
        raise ValueError("arms_control_config_file parameter is empty")
    if not os.path.isabs(config_path):
        config_path = os.path.join("/config", config_path)
    return create_robot_config(load_arms_control_config(config_path))


class Pico4TeleopController(Node):
    """ROS2 node that drives dual-arm teleoperation from a Pico4 headset.

    Subscribes to ``/xr_pose`` (``xr_msgs/Custom``) published by the picoxr
    talker on the headset and feeds controller pose + button data into the
    shared Quest IK stack (``AbsoluteControlModality``).  Joint commands are
    published at up to 1 kHz via a timer; the actual rate is throttled by the
    arrival rate of ``/xr_pose`` messages because stale data is never reused
    (consume-once pattern).

    The node claims ``"quest_teleop"`` ownership through the controls ownership
    manager on startup and re-claims it whenever ownership is released, ensuring
    the IK pipeline has priority over other controllers.

    Attributes:
        robot_config: Parsed robot geometry and arm configuration.
        position_scale: Scalar applied to all controller position deltas before
            they reach the IK solver.  Allows tuning workspace sensitivity
            without changing the IK configuration.
        control_modality: ``AbsoluteControlModality`` instance that owns the
            PyBullet IK state and translates controller poses to joint targets.
        quest_metrics: Telemetry helper for recording per-arm controller events.
    """

    def __init__(self, writer: InfluxDbWriter):
        """Initialise the node, load config, create publishers/subscribers, and start the timer.

        Args:
            writer: InfluxDB writer used by ``QuestMetrics`` to record telemetry.
        """
        super().__init__("pico4_teleop_controller")

        self._device_id = os.environ.get("DEVICE_ID", "unknown")
        api_key = os.environ.get("AMPLITUDE_API_KEY", "")
        self._amplitude: Amplitude | None = Amplitude(api_key) if api_key else None

        self.declare_parameter(
            "arms_control_config_file",
            os.environ.get("ARMS_CONTROL_CONFIG_FILE", ""),
        )
        arms_control_config_file = (
            self.get_parameter("arms_control_config_file")
            .get_parameter_value()
            .string_value
        )
        self.robot_config = load_robot_config(arms_control_config_file)
        self.get_logger().info(f"Loaded robot config for arm_type: {self.robot_config.arm_type}")

        arms_cfg = self.robot_config.arms_control_config.arms
        self.commanded_ee = self.robot_config.arms_control_config.commanded_ee

        self.declare_parameter("teleop_position_scale", float(os.environ.get("TELEOP_POSITION_SCALE", "1.0")))
        self.position_scale = (
            self.get_parameter("teleop_position_scale")
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(f"Using teleop position scale: {self.position_scale}")

        self.declare_parameter("webapp_url", os.environ.get("WEBAPP_URL", "http://127.0.0.1:3000"))
        webapp_url = (
            self.get_parameter("webapp_url")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(f"Using webapp URL: {webapp_url}")

        # PyBullet robot model identifiers, populated by populate_joint_info().
        self.robot_id = None
        self.num_joints = None
        self.left_ee_index = -1
        self.left_joint_indices = []
        self.right_ee_index = -1
        self.right_joint_indices = []
        self.lower_limits = []
        self.upper_limits = []
        self.joint_ranges = []
        self.rest_poses = []
        self.home_positions = []
        self.left_velocity_limits = []
        self.right_velocity_limits = []

        # Tracking last-published positions enables per-joint velocity clamping.
        self.last_published_left = None
        self.last_published_right = None
        self.last_time = time.time()

        self.left_publisher = self.create_publisher(
            Float64MultiArray, "/follower_l_forward_position_controller/commands", 10
        )
        self.right_publisher = self.create_publisher(
            Float64MultiArray, "/follower_r_forward_position_controller/commands", 10
        )
        self.left_target_pose_publisher = self.create_publisher(
            PoseStamped, "/quest_teleop/left_target_pose", 10
        )
        self.right_target_pose_publisher = self.create_publisher(
            PoseStamped, "/quest_teleop/right_target_pose", 10
        )
        self.left_controller_pose_publisher = self.create_publisher(
            PoseStamped, "/quest_teleop/left_controller_pose", 10
        )
        self.right_controller_pose_publisher = self.create_publisher(
            PoseStamped, "/quest_teleop/right_controller_pose", 10
        )
        self.left_ee_pose_publisher = self.create_publisher(
            CommandedEEPose, "ee_pose_left", 10
        )
        self.right_ee_pose_publisher = self.create_publisher(
            CommandedEEPose, "ee_pose_right", 10
        )

        self.latest_left_commanded_ee: CommandedEEPose | None = None
        self.latest_right_commanded_ee: CommandedEEPose | None = None
        if self.commanded_ee:
            if "follower_l" in arms_cfg:
                self.create_subscription(
                    CommandedEEPose, "commanded_ee_left", self._on_left_commanded_ee, 10
                )
            if "follower_r" in arms_cfg:
                self.create_subscription(
                    CommandedEEPose, "commanded_ee_right", self._on_right_commanded_ee, 10
                )

        self.current_left_gripper: float = 0.0
        self.current_right_gripper: float = 0.0
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)

        # Config-driven mapping: a workcell YAML can swap which physical Pico4
        # controller (left/right hand) drives which robot arm.  Defaults keep
        # the natural left→left, right→right assignment.
        self._left_arm_controller = (
            arms_cfg["follower_l"].vr_controller if "follower_l" in arms_cfg else "left"
        ) or "left"
        self._right_arm_controller = (
            arms_cfg["follower_r"].vr_controller if "follower_r" in arms_cfg else "right"
        ) or "right"

        # Previous button states for rising-edge detection (None means "not yet seen").
        self.prev_right_a_button: bool | None = None
        self.prev_right_b_button: bool | None = None
        self.prev_left_y_button: bool | None = None

        # Per-controller counters for the _TRACKING_LOSS_FRAMES debounce (module-level constant).
        self._right_not_tracked_count = 0
        self._left_not_tracked_count  = 0

        # Recording state — prevents A from firing when already recording and
        # prevents button bounce from sending multiple start/stop requests.
        self._is_recording = False
        self._last_recording_button_time = 0.0

        # Single-slot buffer written by _on_xr_pose and consumed once by _teleop_tick.
        self._latest_raw_data: dict | None = None

        _be = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Custom, "xr_pose", self._on_xr_pose, _be)
        self.get_logger().info("Subscribed to /xr_pose — waiting for Pico4 controller data")

        self._reset_action_client = ActionClient(self, Reset, "/arms_resetter/reset")

        self._current_controls_owner = ""
        self._switch_owner_client = self.create_client(
            SwitchControlsOwner, "/controls_owner_manager/switch_owner"
        )
        # TRANSIENT_LOCAL so we receive the last-latched owner immediately on subscribe.
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            ControlsOwner, "/controls_owner", self._on_controls_owner, latched_qos
        )

        self.webapp = WebAppClient(base_url=webapp_url, logger=self.get_logger())

        # Ground-truth recording state from the recorder node — overrides optimistic tracking.
        self.create_subscription(RecordingStatus, "/recording_status", self._on_recording_status, 10)

        self.populate_joint_info()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.control_modality = AbsoluteControlModality(
            num_joints=self.num_joints,
            left_ee_index=self.left_ee_index,
            right_ee_index=self.right_ee_index,
            left_joint_indices=self.left_joint_indices,
            right_joint_indices=self.right_joint_indices,
            lower_limits=self.lower_limits,
            upper_limits=self.upper_limits,
            joint_ranges=self.joint_ranges,
            rest_poses=self.rest_poses,
            robot_id=self.robot_id,
            left_target_pose_publisher=self.left_target_pose_publisher,
            right_target_pose_publisher=self.right_target_pose_publisher,
            left_ee_pose_publisher=self.left_ee_pose_publisher,
            right_ee_pose_publisher=self.right_ee_pose_publisher,
            tf_buffer=self.tf_buffer,
            robot_config=self.robot_config,
            logger=self.get_logger(),
        )

        self.quest_metrics = QuestMetrics(self, writer)

        # Teleop is event-driven: _teleop_tick is invoked directly from
        # _on_xr_pose on each /xr_pose message (~72-90 Hz with a headset, 0 Hz
        # idle). The previous design polled at 1 kHz via a timer, which span the
        # single-threaded rclpy executor at ~0.8 core continuously even with no
        # controller data arriving. Kept as None so __exit__ cleanup is a no-op.
        self._teleop_timer = None
        self._claim_controls_ownership()

    def __enter__(self) -> Self:
        """Support usage as a context manager.

        Returns:
            This node instance.
        """
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        """Destroy the node on context manager exit.

        Args:
            exc_type: Exception type, if any.
            exc_val: Exception value, if any.
            exc_tb: Exception traceback, if any.
        """
        self.destroy_node()

    def destroy_node(self):
        """Run cleanup before tearing down the ROS2 node.

        Cancels the teleop timer, destroys the action client, disconnects
        PyBullet, and shuts down the Amplitude client if one was created.
        """
        self.cleanup()
        if hasattr(self, "_amplitude") and self._amplitude:
            self._amplitude.shutdown()
        super().destroy_node()

    def _on_xr_pose(self, msg: Custom) -> None:
        """Handle an incoming ``/xr_pose`` message from the picoxr talker.

        Converts ``xr_msgs/Controller`` fields to a flat ``raw_data`` dict whose
        keys match the format expected by ``quest_teleop_controller`` (and by
        ``AbsoluteControlModality``).  This is the sole translation layer between
        the Pico4 message schema and the existing Quest IK stack.

        Position values are scaled by ``self.position_scale`` here so the IK
        solver always works in calibrated robot-workspace units.

        Button state is converted from level to edge: each ``*_pressed`` key is
        ``True`` only on the single tick where the button transitions from
        released to pressed, mirroring the edge-detect used by the original TCP
        packet parser.

        The populated dict is stored in ``_latest_raw_data`` and will be
        consumed exactly once by the next ``_teleop_tick`` call.

        Args:
            msg: Incoming ``xr_msgs/Custom`` message containing pose and button
                state for both controllers.
        """
        r = msg.right_controller
        l = msg.left_controller

        def _edge(attr: str, val: bool) -> bool:
            """Detect a low-to-high transition on a boolean button value.

            Reads the previous state from ``self.<attr>``, writes the new
            state back, and returns ``True`` only on a rising edge.

            Args:
                attr: Name of the instance attribute holding the previous state.
                val: Current button value from the incoming message.

            Returns:
                ``True`` if the button just became pressed this tick, otherwise
                ``False``.
            """
            prev = getattr(self, attr)
            pressed = prev is not None and val and not prev
            setattr(self, attr, val)
            return pressed

        # Pico4 XRoboToolkit coordinate frame is rotated 180° around Z relative to
        # Quest OpenXR. Negate X/Y of position and X/Y of quaternion to convert
        # without runtime matrix math (equivalent to q_z180 ⊗ q ⊗ q_z180*).
        # Debounce tracking loss: increment counter on each untracked frame,
        # reset on any tracked frame.  Only report "not tracked" once the
        # counter reaches the threshold so brief 1-2 frame glitches are invisible.
        if r.status != 0:
            self._right_not_tracked_count = 0
        else:
            self._right_not_tracked_count += 1
        if l.status != 0:
            self._left_not_tracked_count = 0
        else:
            self._left_not_tracked_count += 1

        r_data = dict(
            pos=(-float(r.pose[0]), -float(r.pose[1]), float(r.pose[2])),
            orn=(-float(r.pose[3]), -float(r.pose[4]), float(r.pose[5]), float(r.pose[6])),
            trigger1=float(r.trigger),
            trigger2=float(r.gripper),
            tracked=(self._right_not_tracked_count < _TRACKING_LOSS_FRAMES),
        )
        l_data = dict(
            pos=(-float(l.pose[0]), -float(l.pose[1]), float(l.pose[2])),
            orn=(-float(l.pose[3]), -float(l.pose[4]), float(l.pose[5]), float(l.pose[6])),
            trigger1=float(l.trigger),
            trigger2=float(l.gripper),
            tracked=(self._left_not_tracked_count < _TRACKING_LOSS_FRAMES),
        )

        # Apply the config-driven controller-to-arm mapping.  When the workcell
        # YAML swaps controllers, the physical right-hand controller populates
        # the left-arm slot and vice versa.
        la = l_data if self._left_arm_controller == "left" else r_data
        ra = r_data if self._right_arm_controller == "right" else l_data

        # Scale positions before storing so downstream code never sees raw headset units.
        la_pos = tuple(self.position_scale * x for x in la["pos"])
        ra_pos = tuple(self.position_scale * x for x in ra["pos"])

        right_a = r.primary_button
        right_b = r.secondary_button
        left_y  = l.secondary_button

        self._latest_raw_data = dict(
            left_controller_pos=la_pos,
            left_controller_orn=la["orn"],
            left_trigger1=la["trigger1"],
            left_trigger2=la["trigger2"],
            left_controller_tracked=la["tracked"],
            right_controller_pos=ra_pos,
            right_controller_orn=ra["orn"],
            right_trigger1=ra["trigger1"],
            right_trigger2=ra["trigger2"],
            right_controller_tracked=ra["tracked"],
            right_a_pressed=_edge("prev_right_a_button", right_a),
            right_b_pressed=_edge("prev_right_b_button", right_b),
            left_y_pressed=_edge("prev_left_y_button", left_y),
        )

        # Drive control directly from the message instead of a polling timer.
        # _teleop_tick consumes _latest_raw_data once (sets it back to None), so
        # behaviour is identical to the old 1 kHz timer but with lower latency
        # and no idle CPU spin.
        self._teleop_tick()

    def _teleop_tick(self) -> None:
        """Run one iteration of the teleop control loop at up to 1 kHz.

        Implements a consume-once pattern: the slot ``_latest_raw_data`` is
        atomically read and cleared at the start of each tick.  If no new
        ``/xr_pose`` message has arrived since the last tick the slot is
        ``None`` and the tick returns immediately, preventing the IK solver from
        acting on stale data.  This mirrors the original TCP ``recv`` behaviour
        where a blocking read naturally serialised control updates.

        On each tick with fresh data the method:

        1. Publishes raw controller poses for visualisation / logging.
        2. Skips IK if another owner (e.g. ``"homer"``) currently holds controls,
           and forces the modality to disengage so the IK reference frame resets
           cleanly when ownership is returned.
        3. Dispatches button-press side effects (recording start/stop, rehome).
        4. Calls ``AbsoluteControlModality.get_target_joint_positions`` to obtain
           IK-solved joint targets.
        5. Applies per-joint velocity limits using the elapsed wall-clock dt and
           publishes the clamped commands to the forward position controllers.
        """
        # Consume the latest data atomically; return early if nothing is new.
        data = self._latest_raw_data
        if data is None:
            return
        self._latest_raw_data = None  # consume once — next tick waits for new xr_pose

        left_pos = data.get("left_controller_pos")
        left_orn = data.get("left_controller_orn")
        if left_pos is not None and left_orn is not None:
            publish_pose(
                self.left_controller_pose_publisher,
                left_pos,
                left_orn,
                "quest_world",
                stamp=self.get_clock().now().to_msg(),
            )
            self.quest_metrics.record_left()

        right_pos = data.get("right_controller_pos")
        right_orn = data.get("right_controller_orn")
        if right_pos is not None and right_orn is not None:
            publish_pose(
                self.right_controller_pose_publisher,
                right_pos,
                right_orn,
                "quest_world",
                stamp=self.get_clock().now().to_msg(),
            )
            self.quest_metrics.record_right()

        # Yield to whichever subsystem currently owns controls and reset the IK
        # reference frame so re-engagement doesn't cause a position jump.
        if self._current_controls_owner and self._current_controls_owner != "quest_teleop":
            self.control_modality.force_disengage()
            self.last_published_left = None
            self.last_published_right = None
            return

        if data.get("right_a_pressed"):
            now = time.time()
            if now - self._last_recording_button_time < _RECORDING_BUTTON_COOLDOWN:
                pass  # debounce — button is bouncing, ignore
            elif self._is_recording:
                self.get_logger().info("A button: recording already in progress — ignoring")
            else:
                # Mark immediately to block double-press; reset in thread if session lookup fails.
                self._is_recording = True
                self._last_recording_button_time = now
                def _start():
                    session_id = _get_default_session_id(self.webapp._base_url)
                    if session_id is None:
                        self.get_logger().warn(
                            "A button pressed but no session is selected — "
                            "open a session in the web UI first."
                        )
                        self._is_recording = False
                    else:
                        self.get_logger().info(
                            f"Sending request to start recording (session {session_id})"
                        )
                        self.webapp.post("recording.start", {"sessionId": session_id})
                threading.Thread(target=_start, daemon=True).start()

        if data.get("right_b_pressed"):
            now = time.time()
            if now - self._last_recording_button_time < _RECORDING_BUTTON_COOLDOWN:
                pass  # debounce
            elif not self._is_recording:
                self.get_logger().info("B button: not recording — ignoring")
            else:
                self.get_logger().info("Sending request to stop recording")
                self.webapp.post("recording.stop")
                self._is_recording = False
                self._last_recording_button_time = now

        if data.get("left_y_pressed") and self._current_controls_owner != "homer":
            self.get_logger().info("Y button pressed — sending rehome request")
            self._send_rehome_request()

        active_poses_left, active_poses_right = (
            self.control_modality.get_target_joint_positions(
                data,
                left_commanded_ee=self.latest_left_commanded_ee,
                right_commanded_ee=self.latest_right_commanded_ee,
                current_left_gripper=self.current_left_gripper,
                current_right_gripper=self.current_right_gripper,
            )
        )

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        if active_poses_left is not None:
            if self.last_published_left is None:
                # First command ever: seed so the limiter has a reference.
                self.last_published_left = active_poses_left
            limited = []
            for i in range(len(self.left_joint_indices)):
                target = active_poses_left[i]
                prev   = self.last_published_left[i]
                limit  = self.left_velocity_limits[i]
                limited.append(float(np.clip(target, prev - limit * dt, prev + limit * dt)))
            # Gripper value is appended after the velocity-limited joints; it is
            # passed through unclipped because the gripper has its own rate control.
            limited.append(active_poses_left[-1])
            self.last_published_left = limited
            self.left_publisher.publish(Float64MultiArray(data=limited))

        if active_poses_right is not None:
            if self.last_published_right is None:
                # First command ever: seed so the limiter has a reference.
                self.last_published_right = active_poses_right
            limited = []
            for i in range(len(self.right_joint_indices)):
                target = active_poses_right[i]
                prev   = self.last_published_right[i]
                limit  = self.right_velocity_limits[i]
                limited.append(float(np.clip(target, prev - limit * dt, prev + limit * dt)))
            limited.append(active_poses_right[-1])
            self.last_published_right = limited
            self.right_publisher.publish(Float64MultiArray(data=limited))

    # --- unchanged from quest_teleop_controller.py below ---

    def cleanup(self):
        """Cancel the teleop timer and release PyBullet resources.

        Safe to call more than once; the PyBullet disconnect is wrapped in a
        try/except because it raises if the physics server was never connected
        or has already been disconnected.
        """
        self.get_logger().info("Running cleanup...")
        if self._teleop_timer:
            self._teleop_timer.cancel()
        self._reset_action_client.destroy()
        try:
            p.disconnect()
        except Exception:
            pass
        self.get_logger().info("Cleanup complete.")

    def _send_rehome_request(self):
        """Send a rehome goal to the ``/arms_resetter/reset`` action server.

        Uses ``final_pose_only=True`` so the arms move only to the final rest
        pose without executing a full calibration sequence.  The 7.5-second
        duration override gives the arms enough time to reach the rest pose
        smoothly regardless of their current position.

        Does nothing if the action server is not yet available, logging a
        warning instead of blocking the teleop loop.
        """
        if not self._reset_action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn("Arms resetter action server not available")
            return
        goal = Reset.Goal()
        goal.final_pose_only = True
        goal.homing_duration_override = 7.5
        future = self._reset_action_client.send_goal_async(goal)
        future.add_done_callback(self._rehome_goal_response_callback)

    def _rehome_goal_response_callback(self, future):
        """Handle the goal-acceptance response from the arms resetter.

        Chains a result callback if the goal was accepted; logs a warning if
        the server rejected it.

        Args:
            future: Completed future whose result is the action goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Rehome goal rejected")
            return
        self.get_logger().info("Rehome goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._rehome_result_callback)

    def _rehome_result_callback(self, future):
        """Handle the final result from the arms resetter action.

        Args:
            future: Completed future whose result carries the action status.
        """
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Rehome completed successfully")
        else:
            self.get_logger().warn(f"Rehome finished with status: {result.status}")

    def _on_left_commanded_ee(self, msg: CommandedEEPose):
        """Cache the latest commanded end-effector pose for the left arm.

        Used by ``AbsoluteControlModality`` when ``commanded_ee`` mode is
        enabled to seed the IK target from the current commanded pose rather
        than the measured joint state.

        Args:
            msg: Incoming commanded end-effector pose for the left arm.
        """
        self.latest_left_commanded_ee = msg

    def _on_right_commanded_ee(self, msg: CommandedEEPose):
        """Cache the latest commanded end-effector pose for the right arm.

        Args:
            msg: Incoming commanded end-effector pose for the right arm.
        """
        self.latest_right_commanded_ee = msg

    def _on_joint_states(self, msg: JointState):
        """Track the current gripper positions from the joint state topic.

        Only the finger joints are extracted here; arm joint positions are read
        directly from PyBullet inside ``AbsoluteControlModality``.

        Args:
            msg: Incoming ``sensor_msgs/JointState`` message from the robot.
        """
        for name, position in zip(msg.name, msg.position):
            if name == "follower_l_finger_joint1":
                self.current_left_gripper = float(position)
            elif name == "follower_r_finger_joint1":
                self.current_right_gripper = float(position)

    def _on_controls_owner(self, msg: ControlsOwner):
        """React to changes in the controls ownership topic.

        Re-claims ownership whenever it is released (empty owner string) so
        teleop resumes automatically after a rehome or other temporary handover.

        Args:
            msg: Latched ``ControlsOwner`` message carrying the current owner name.
        """
        self._current_controls_owner = msg.owner
        if not msg.owner:
            self._claim_controls_ownership()

    def _claim_controls_ownership(self):
        """Request controls ownership from the ownership manager service.

        The requester name is hardcoded to ``"quest_teleop"`` to remain
        compatible with the existing ownership manager configuration, which
        expects this identifier regardless of whether the teleop source is a
        Quest or a Pico4 headset.

        Does nothing if the service is not yet available, logging a warning
        instead of blocking.
        """
        if not self._switch_owner_client.service_is_ready():
            self.get_logger().warn("Controls owner manager not available — skipping claim")
            return
        req = SwitchControlsOwner.Request()
        req.requester = "quest_teleop"
        req.release = False
        future = self._switch_owner_client.call_async(req)
        future.add_done_callback(self._claim_ownership_callback)

    def _claim_ownership_callback(self, future):
        """Handle the response to a controls ownership claim request.

        Args:
            future: Completed future whose result is a ``SwitchControlsOwner``
                response carrying a ``success`` flag and a human-readable
                ``message``.
        """
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"Claimed controls ownership: {result.message}")
            else:
                self.get_logger().warn(f"Failed to claim controls ownership: {result.message}")
        except Exception as e:
            self.get_logger().warn(f"Controls ownership claim error: {e}")

    def _on_recording_status(self, msg: RecordingStatus) -> None:
        """Sync _is_recording with the recorder node's ground truth."""
        self._is_recording = msg.is_recording

    def populate_joint_info(self):
        """Load the robot URDF into PyBullet and extract joint metadata.

        Connects to a headless PyBullet physics server (``p.DIRECT``), loads the
        URDF specified by ``ROBOT_DESCRIPTION_URDF``, and populates all joint
        index lists, limit arrays, velocity limit arrays, and end-effector
        indices used by the IK solver.

        The URDF path must reside inside
        ``/workspace/ros2/src/quest_teleop/config/`` so that PyBullet's
        parent-directory mesh search can locate ``openarm_description/meshes/``
        two levels up from the config directory.

        Raises:
            ValueError: If the left end-effector link is not found in the URDF.
                A missing right end-effector triggers a warning instead (single-
                arm mode is valid).
        """
        p.connect(p.DIRECT)
        urdf_path = os.environ.get(
            "ROBOT_DESCRIPTION_URDF",
            "/workspace/ros2/src/quest_teleop/config/robot_description.urdf",
        )
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)
        self.num_joints = p.getNumJoints(self.robot_id)

        arms_cfg = self.robot_config.arms_control_config
        left_expected  = arms_cfg.expected_joint_names("follower_l")
        right_expected = arms_cfg.expected_joint_names("follower_r")
        tcp_left  = self.robot_config.tcp_link("follower_l")
        tcp_right = self.robot_config.tcp_link("follower_r")

        for i in range(self.num_joints):
            info       = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode("utf-8")

            if info[2] != p.JOINT_FIXED:
                lower_limit    = info[8]
                upper_limit    = info[9]
                velocity_limit = info[11]
                self.lower_limits.append(lower_limit)
                self.upper_limits.append(upper_limit)
                self.joint_ranges.append(upper_limit - lower_limit)

                # Rest pose is taken from the arm-specific list in RobotConfig;
                # joints that belong to neither arm fall back to 0 (e.g. torso).
                if joint_name in left_expected:
                    rest = self.robot_config.rest_position_left[left_expected.index(joint_name)]
                elif joint_name in right_expected:
                    rest = self.robot_config.rest_position_right[right_expected.index(joint_name)]
                else:
                    rest = 0
                self.rest_poses.append(rest)
                self.get_logger().info(
                    f"Joint {i}: {joint_name}, lower={lower_limit}, upper={upper_limit}, vel={velocity_limit}"
                )

            # End-effector indices are stored by joint index, not link index,
            # because PyBullet IK targets are specified per joint.
            if tcp_left in joint_name:
                self.left_ee_index = i
            elif tcp_right in joint_name:
                self.right_ee_index = i

            if "follower_l_joint" in joint_name and info[2] != p.JOINT_FIXED:
                self.left_joint_indices.append(i)
                self.left_velocity_limits.append(info[11])
            if "follower_r_joint" in joint_name and info[2] != p.JOINT_FIXED:
                self.right_joint_indices.append(i)
                self.right_velocity_limits.append(info[11])

        self.get_logger().info(f"Left joint indices:  {self.left_joint_indices}")
        self.get_logger().info(f"Right joint indices: {self.right_joint_indices}")

        if self.left_ee_index == -1:
            raise ValueError("Left end-effector index not found in URDF.")
        if self.right_ee_index == -1:
            self.get_logger().warn("Right end-effector not found — single-arm mode.")


def main(args=None):
    """Entry point: initialise ROS2, spin the controller, and shut down cleanly.

    Args:
        args: Optional argument list passed to ``rclpy.init``.  Defaults to
            ``None``, which causes rclpy to read from ``sys.argv``.
    """
    rclpy.init(args=args)
    try:
        with InfluxDbWriter() as writer, Pico4TeleopController(writer) as controller:
            rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
