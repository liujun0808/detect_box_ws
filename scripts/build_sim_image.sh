#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

test -f "${WORKSPACE_DIR}/models/yolov8s-worldv2.pt"
test -f "${WORKSPACE_DIR}/weights/clip/ViT-B-32.pt"

exec "${SCRIPT_DIR}/sim_compose.sh" \
  -f "${WORKSPACE_DIR}/docker/compose.sim.build.yaml" \
  build detector
