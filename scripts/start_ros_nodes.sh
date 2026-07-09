#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="/home/user/project/detect_box_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"
CAMERA_START_DELAY=5
export ROS_DOMAIN_ID=19
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI=/home/user/dds/cyclonedds.xml

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WS_SETUP}" ]]; then
  echo "Workspace setup file not found: ${WS_SETUP}" >&2
  exit 1
fi

# ROS setup scripts may reference unset variables internally, so do not use
# nounset while sourcing them.
set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

cd "${WORKSPACE_DIR}"

cleanup() {
  trap - SIGINT SIGTERM EXIT
  if [[ -n "${DETECT_PID:-}" ]]; then
    kill "${DETECT_PID}" 2>/dev/null || true
  fi
  if [[ -n "${CAMERA_PID:-}" ]]; then
    kill "${CAMERA_PID}" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
}

trap cleanup SIGINT SIGTERM EXIT

ros2 launch d435_publisher d435i_camera.launch.py &
CAMERA_PID=$!

sleep "${CAMERA_START_DELAY}"

ros2 run detect_pkg detect_server_node &
DETECT_PID=$!

wait -n "${CAMERA_PID}" "${DETECT_PID}"
