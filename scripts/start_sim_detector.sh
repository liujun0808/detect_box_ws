#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

mkdir -p "${WORKSPACE_DIR}/debug_box_position_sim"
exec "${SCRIPT_DIR}/sim_compose.sh" up -d --no-build detector
