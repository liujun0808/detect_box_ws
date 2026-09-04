#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="${DEPLOY_DIR}/.env"

# Make bind-mounted output files editable by the invoking host user.
export LOCAL_UID="$(id -u)"
export LOCAL_GID="$(id -g)"

compose_args=(
  --project-directory "${DEPLOY_DIR}"
  -f "${DEPLOY_DIR}/compose.yaml"
)
if [[ -f "${ENV_FILE}" ]]; then
  # The deployment file is authoritative over inherited terminal variables.
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
  compose_args=(--env-file "${ENV_FILE}" "${compose_args[@]}")
fi

exec docker compose "${compose_args[@]}" "$@"
