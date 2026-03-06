#!/usr/bin/env bash
set -euo pipefail

# Root privileges required for Tailscale setup
if [ "$EUID" -ne 0 ]; then 
  printf '%s\n' "This script requires root privileges. Re-running with sudo..."
  exec sudo "$0" "$@"
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROVISION_ENV_FILE="${1:-${SCRIPT_DIR}/.env.provision}"

# Load required environment file
if [ ! -f "${PROVISION_ENV_FILE}" ]; then
  printf '%s\n' "Error: Required environment file not found at ${PROVISION_ENV_FILE}"
  exit 1
fi

# shellcheck disable=SC1090
source "${PROVISION_ENV_FILE}"

TARGET_USER="${SUDO_USER:-${USER:-anvil}}"

if [[ -z "${GCP_KEY_JSON_B64:-}" ]]; then
  printf '%s\n' "Error: GCP_KEY_JSON_B64 is not set in ${PROVISION_ENV_FILE}"
  exit 1
fi

# Attempt to log into Tailscale if the auth key is present
if [[ -n "${TAILSCALE_AUTH_KEY:-}" ]]; then
  printf '%s\n' "Logging into Tailscale..."
  if ! sudo tailscale up --ssh --accept-routes=false --accept-dns=false --hostname "${DEVICE_NAME}" --authkey "$TAILSCALE_AUTH_KEY"; then
    printf '%s\n' "Warning: Tailscale login failed. Please check your auth key."
  else
    printf '%s\n' "Tailscale login successful."
  fi
else
  printf '%s\n' "No Tailscale auth key found. Skipping Tailscale login."
fi

printf '%s\n' "Logging into Google Artifact Registry Docker repository..."
echo "${GCP_KEY_JSON_B64}" | \
  sudo -u "${TARGET_USER}" HOME="/home/${TARGET_USER}" \
  docker login -u _json_key_base64 --password-stdin \
  "https://$GAR_REGION-docker.pkg.dev"

ENV_TARGET="${SCRIPT_DIR}/.env"

printf '%s\n' "Writing ${ENV_TARGET}..."
cat > "${ENV_TARGET}" <<EOF
# Automatically generated — do not edit manually

# Fluent Bit - Grafana Cloud Loki (Logs)
GRAFANA_LOKI_HOST=${GRAFANA_LOKI_HOST:-}
GRAFANA_LOKI_USER=${GRAFANA_LOKI_USER:-}
GRAFANA_LOKI_API_KEY=${GRAFANA_LOKI_API_KEY:-}

# Telegraf - InfluxDB Cloud (Metrics)
INFLUXDB_CLOUD_URL=${INFLUXDB_CLOUD_URL:-}
INFLUXDB_CLOUD_ORG=${INFLUXDB_CLOUD_ORG:-}
INFLUXDB_CLOUD_BUCKET=${INFLUXDB_CLOUD_BUCKET:-}
INFLUXDB_CLOUD_TOKEN=${INFLUXDB_CLOUD_TOKEN:-}
EOF
printf '%s\n' "Written to ${ENV_TARGET}"

DESKTOP_FILE="/home/${TARGET_USER}/Desktop/webappLink.desktop"
if [[ -f "${DESKTOP_FILE}" ]]; then
  printf '%s\n' "Launch the Anvil Web App by double-clicking the 'Anvil Web App' icon on the desktop after running docker compose up."
else
  printf '%s\n' "Launch the Anvil Web App by opening http://localhost:3000 in a web browser after running docker compose up."
fi

printf '%s\n' "Done."
