#!/usr/bin/env bash
set -euo pipefail

ROS_DOMAIN_ID_VALUE="${1:-0}"
TARGET_DIR="${HOME}/.ros"
TARGET_FILE="${TARGET_DIR}/fastdds.xml"
SOURCE_FILE="/home/antonin/workspace/isaac_sim/fastdds/fastdds.xml"

mkdir -p "${TARGET_DIR}"
cp "${SOURCE_FILE}" "${TARGET_FILE}"

if ! grep -q "FASTRTPS_DEFAULT_PROFILES_FILE" "${HOME}/.bashrc" 2>/dev/null; then
  {
    echo ""
    echo "# Isaac Sim ROS 2 bridge"
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=${TARGET_FILE}"
    echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
  } >> "${HOME}/.bashrc"
fi

cat <<EOF
WSL Fast DDS config installed:
  ${TARGET_FILE}

Reload your shell or run:
  export FASTRTPS_DEFAULT_PROFILES_FILE=${TARGET_FILE}
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}
EOF
