#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="/home/user/liujun/detect_box_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"
# ROS launch does not require an interactive `conda activate`; use the
# interpreter in the configured Conda environment directly. Override this
# absolute path when Conda is installed elsewhere.
export DETECT_BOX_PYTHON="${DETECT_BOX_PYTHON:-/home/user/miniforge3/envs/detect_box/bin/python}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
# Use the project-specific CycloneDDS configuration only when it is present.
# A stale/missing file URI makes rmw_cyclonedds_cpp fail before rclpy can
# create a node ("rcl node's rmw handle is invalid").
CYCLONEDDS_CONFIG="/home/user/dds/cyclonedds.xml"
if [[ -f "${CYCLONEDDS_CONFIG}" ]]; then
  export CYCLONEDDS_URI="file://${CYCLONEDDS_CONFIG}"
else
  unset CYCLONEDDS_URI
fi

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
# 使用 launch 入口确保自动加载 box_position_estimation.yaml；
# 使用 exec 让 systemd 直接监督 ROS 进程，并将 SIGINT 直接传给 launch。
exec ros2 launch detect_pkg box_position_estimation.launch.py
