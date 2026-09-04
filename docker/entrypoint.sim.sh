#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /opt/detect_box_ws/install/setup.bash
set -u

mkdir -p "${ROS_LOG_DIR}" "${MPLCONFIGDIR}" "${YOLO_CONFIG_DIR}"
cd /opt/detect_box_ws

exec "$@"
