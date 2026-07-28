#!/bin/bash
set -e

# Source ROS2 Jazzy base + anvil workspace (quest_teleop, anvil_msgs, pybullet venv) + xr_ws (xr_msgs, picoxr)
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
source /xr_ws/install/setup.bash

# Match the ros2 container's RMW/CycloneDDS config (see its /entrypoint.sh) instead of
# hardcoding something different here. This host has 6+ network interfaces (WiFi,
# Tailscale, several docker bridges); without an explicit single NetworkInterface,
# FastDDS/CycloneDDS announce locators on all of them, and ROS2 service calls (e.g.
# /arms_resetter/reset) silently hang across containers even though plain pub/sub
# still gets through — pinning both containers to the same interface (CYCLONEDDS_IFACE,
# normally 'lo' since ros2 and pico4-teleop always run on this same host) fixes it.
if [ "${ENABLE_CYCLONEDDS}" = "true" ]; then
    _iface=""; [ -n "${CYCLONEDDS_IFACE}" ] && \
        _iface="<NetworkInterface name=\"${CYCLONEDDS_IFACE}\" />"
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI="<CycloneDDS><Domain id='any'><General><Interfaces>${_iface}</Interfaces></General><Discovery><Peers><Peer address='${CYCLONEDDS_PEER_IP}'/></Peers></Discovery></Domain></CycloneDDS>"
fi

# Mirror the env var setup from the ros2 container entrypoint.sh so InfluxDbWriter initialises correctly
export DEVICE_ID=$(tr -d '[:space:]' < /etc/machine-id 2>/dev/null || echo "unknown")
export DEVICE_NAME=${DEVICE_NAME:-unknown}
export VERSION=${VERSION:-unknown}

if [ -f /repo-git/HEAD ]; then
    LOADER_REPO_VERSION=$(cat /repo-git/HEAD | tr -d '[:space:]')
    if echo "$LOADER_REPO_VERSION" | grep -q "^ref:"; then
        ref=$(echo "$LOADER_REPO_VERSION" | sed 's/^ref: *//')
        if [ -f "/repo-git/$ref" ]; then
            LOADER_REPO_VERSION=$(cat "/repo-git/$ref" | tr -d '[:space:]')
        elif [ -f /repo-git/packed-refs ]; then
            LOADER_REPO_VERSION=$(grep " $ref$" /repo-git/packed-refs | cut -d ' ' -f 1)
        fi
    fi
fi
export LOADER_REPO_VERSION=${LOADER_REPO_VERSION:-unknown}
export ANVIL_OS_VERSION=$(tr -d '[:space:]' < /etc/anvil-os-version 2>/dev/null || echo "unknown")
export ANVIL_OS_BUILDER_COMMIT=$(tr -d '[:space:]' < /etc/anvil-os-builder-commit 2>/dev/null || echo "unknown")

# PyBullet resolves package:// URIs by searching parent dirs of the URDF.
# Loading from /workspace/ros2/src/quest_teleop/config/ means two levels up
# is /workspace/ros2/src/ where openarm_description/meshes/ lives.
# Filename is robot_description_pico4.urdf (not robot_description.urdf) because
# the ros2 image's anvil_robot_manager unconditionally deletes /config/robot_description.urdf
# and /config/generated_controllers.yaml on every startup ("backward compat: older
# versions generated these files into /config at launch") — since /config is a
# read-write bind mount shared with the ros2 container, that wipes the real file
# on the host every time ros2 restarts. Renaming sidesteps the vendor cleanup entirely.
mkdir -p /workspace/ros2/src/quest_teleop/config
ln -sf /config/robot_description_pico4.urdf /workspace/ros2/src/quest_teleop/config/robot_description.urdf 2>/dev/null || true
export ROBOT_DESCRIPTION_URDF=/workspace/ros2/src/quest_teleop/config/robot_description.urdf

# picoxr talker: publishes /xr_pose from Pico4 XRoboToolkit stream
restart_talker() {
    while true; do
        ros2 run picoxr talker || true
        echo "[entrypoint] picoxr talker exited, restarting in 2s..."
        sleep 2
    done
}
restart_talker &

# pico4_teleop_controller: subscribes to /xr_pose, runs PyBullet IK, publishes joint commands
# Runs in quest_teleop_venv so pybullet + amplitude etc. are available.
# PYTHONPATH from sourced overlays is inherited by the venv Python.
while true; do
    /quest_teleop_venv/bin/python /app/pico4_teleop_controller.py || true
    echo "[entrypoint] pico4_teleop_controller.py exited, restarting in 2s..."
    sleep 2
done
