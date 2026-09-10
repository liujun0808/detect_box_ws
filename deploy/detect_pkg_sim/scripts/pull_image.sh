#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${DEPLOY_DIR}/.env"

if [[ ! -f "${ENV_FILE}" ]]; then
  printf 'Missing %s. Obtain the prepared deployment .env from the publisher.\n' "${ENV_FILE}" >&2
  exit 1
fi

set -a
# shellcheck disable=SC1090
source "${ENV_FILE}"
set +a

if [[ "${DETECT_BOX_SIM_IMAGE:-}" == "registry.cn-hangzhou.aliyuncs.com/"* ]]; then
  if [[ -z "${ACR_REGISTRY:-}" || -z "${ACR_USERNAME:-}" || -z "${ACR_PASSWORD:-}" ]]; then
    printf 'Missing ACR_REGISTRY, ACR_USERNAME, or ACR_PASSWORD in %s.\n' "${ENV_FILE}" >&2
    exit 1
  fi

  # Use stdin so the password is absent from process arguments and command logs.
  printf '%s\n' "${ACR_PASSWORD}" \
    | docker login --username="${ACR_USERNAME}" --password-stdin "${ACR_REGISTRY}"
fi

exec "${SCRIPT_DIR}/compose.sh" pull detector
