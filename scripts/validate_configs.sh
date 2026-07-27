#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CONFIG_DIR="${SCRIPT_DIR}/../config"

FILTER="${1:-}"   # optional: "pico" or "quest" to filter by headset

REQUIRED_FIELDS=(arm_type control_mode arms)
ERRORS=0

# Pre-flight check: python3 required
command -v python3 >/dev/null 2>&1 || { printf 'Error: python3 is required but not found\n' >&2; exit 1; }

# Check if config directory exists
[[ -d "${CONFIG_DIR}" ]] || { printf 'Error: config directory not found: %s\n' "${CONFIG_DIR}" >&2; exit 1; }

shopt -s nullglob
for yaml_file in "${CONFIG_DIR}"/*.yaml; do
  # Skip if filter set and filename doesn't match
  if [[ -n "${FILTER}" && "${yaml_file}" != *"${FILTER}"* ]]; then
    continue
  fi

  printf 'Checking %s ... ' "$(basename "${yaml_file}")"

  # 1. Valid YAML
  if ! YAML_FILE="${yaml_file}" python3 -c "import yaml, os; yaml.safe_load(open(os.environ['YAML_FILE']))" 2>/dev/null; then
    printf 'FAIL (invalid YAML)\n'
    ERRORS=$((ERRORS + 1))
    continue
  fi

  # 2. Required top-level fields
  FIELD_ERROR=0
  for field in "${REQUIRED_FIELDS[@]}"; do
    if ! YAML_FILE="${yaml_file}" FIELD="${field}" python3 -c "
import yaml, os, sys
doc = yaml.safe_load(open(os.environ['YAML_FILE']))
sys.exit(0 if os.environ['FIELD'] in doc else 1)
" 2>/dev/null; then
      printf 'FAIL (missing field: %s)\n' "${field}"
      ERRORS=$((ERRORS + 1))
      FIELD_ERROR=1
      break
    fi
  done
  [[ ${FIELD_ERROR} -eq 1 ]] && continue

  # 3. VR teleop configs must have vr_controller on at least one arm
  BASE=$(basename "${yaml_file}")
  if [[ "${BASE}" == *quest* || "${BASE}" == *pico* ]]; then
    if ! YAML_FILE="${yaml_file}" python3 -c "
import yaml, os, sys
doc = yaml.safe_load(open(os.environ['YAML_FILE']))
arms = doc.get('arms', {})
has_vr = any(
    isinstance(cfg, dict) and 'vr_controller' in cfg
    for cfg in arms.values()
)
sys.exit(0 if has_vr else 1)
" 2>/dev/null; then
      printf 'FAIL (vr teleop config missing vr_controller on arms)\n'
      ERRORS=$((ERRORS + 1))
      continue
    fi
  fi

  printf 'OK\n'
done
shopt -u nullglob

if [[ ${ERRORS} -gt 0 ]]; then
  printf '\n%d error(s) found.\n' "${ERRORS}"
  exit 1
fi

printf '\nAll configs valid.\n'
