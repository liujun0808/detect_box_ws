#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="detect_box_ws.service"
WORKSPACE_DIR="/home/user/project/detect_box_ws"
SERVICE_SOURCE="${WORKSPACE_DIR}/scripts/${SERVICE_NAME}"
SERVICE_TARGET="/etc/systemd/system/${SERVICE_NAME}"

if [[ ! -f "${SERVICE_SOURCE}" ]]; then
  echo "Service file not found: ${SERVICE_SOURCE}" >&2
  exit 1
fi

sudo install -m 0644 "${SERVICE_SOURCE}" "${SERVICE_TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable "${SERVICE_NAME}"
sudo systemctl restart "${SERVICE_NAME}"

echo "Installed and started ${SERVICE_NAME}"
echo "View logs with: journalctl -u ${SERVICE_NAME} -f"
