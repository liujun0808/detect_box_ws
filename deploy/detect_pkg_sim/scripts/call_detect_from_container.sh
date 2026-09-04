#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# This is a deployment smoke test. Production callers should use their own
# ROS 2 client and the same upper_limb_interface service definition.
exec "${SCRIPT_DIR}/compose.sh" exec -T detector bash -lc \
  'export ROS2CLI_DISABLE_DAEMON=1; ros2 service call /detect upper_limb_interface/srv/DetectAprilTag "{capture_once: true}"'
