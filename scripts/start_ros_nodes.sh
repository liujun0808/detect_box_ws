#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="/home/user/detect_box_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"
export ROS_DOMAIN_ID=22
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

# 检测节点收到服务请求后才通过 librealsense2 打开相机，节点空闲时不占用设备。
# 使用 exec 让 systemd 直接监督 ROS 进程，并将 SIGINT 直接传给节点。
exec ros2 run detect_pkg detect_server_node
