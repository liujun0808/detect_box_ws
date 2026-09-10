#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

test -f "${WORKSPACE_DIR}/models/yolov8s-worldv2.pt"
test -f "${WORKSPACE_DIR}/weights/clip/ViT-B-32.pt"

# Keep the local release image free of BuildKit's default provenance
# attestation so it can be pushed to Alibaba Cloud Container Registry.
export BUILDX_NO_DEFAULT_ATTESTATIONS=1

exec "${SCRIPT_DIR}/sim_compose.sh" \
  -f "${WORKSPACE_DIR}/docker/compose.sim.build.yaml" \
  build detector
