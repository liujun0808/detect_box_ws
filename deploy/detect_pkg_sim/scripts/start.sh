#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

mkdir -p "${DEPLOY_DIR}/debug_box_position_sim"
# Recreate so edits to the mounted YAML take effect on node startup.
exec "${SCRIPT_DIR}/compose.sh" up -d --no-build --force-recreate detector
