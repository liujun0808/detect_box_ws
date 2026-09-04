#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
COMPOSE_FILE="${WORKSPACE_DIR}/docker/compose.sim.yaml"
ENV_FILE="${WORKSPACE_DIR}/docker/.env"

# Bind-mounted files such as debug snapshots should remain editable by the
# user who started the container, rather than becoming root-owned.
export LOCAL_UID="$(id -u)"
export LOCAL_GID="$(id -g)"

compose_args=(
  --project-directory "${WORKSPACE_DIR}"
  -f "${COMPOSE_FILE}"
)
if [[ -f "${ENV_FILE}" ]]; then
  # Compose gives the caller's exported variables precedence over --env-file.
  # Load the project configuration explicitly so its values are deterministic.
  set -a
  # shellcheck disable=SC1090
  source "${ENV_FILE}"
  set +a
  compose_args=(--env-file "${ENV_FILE}" "${compose_args[@]}")
fi

exec docker compose "${compose_args[@]}" "$@"
