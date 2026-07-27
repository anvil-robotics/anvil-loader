#!/bin/bash
set -e

# Source ROS2 Jazzy base + anvil workspace (quest_teleop, anvil_msgs, pybullet venv) + xr_ws (xr_msgs, picoxr)
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
source /xr_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

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
mkdir -p /workspace/ros2/src/quest_teleop/config
ln -sf /config/robot_description.urdf /workspace/ros2/src/quest_teleop/config/robot_description.urdf 2>/dev/null || true
export ROBOT_DESCRIPTION_URDF=/workspace/ros2/src/quest_teleop/config/robot_description.urdf
# host-network peer discovery — both ros2 and pico4-teleop containers are on the host network
export CYCLONEDDS_URI="<CycloneDDS><Domain id='any'><Discovery><Peers><Peer address='127.0.0.1'/></Peers></Discovery></Domain></CycloneDDS>"

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
