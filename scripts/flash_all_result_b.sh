#!/usr/bin/env bash
# Clean-rebuild the Release firmware with UWB_RESULT_B (0x241) enabled and
# flash it to every ST-LINK connected to this machine.
#
# Usage: scripts/flash_all_result_b.sh
#
# Builds into build/Release-ResultB so the default build/Release (0x241 off)
# stays untouched, then hands the image to flash_all.sh.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_PRESET="Release"
BUILD_DIR="${REPO_ROOT}/build/Release-ResultB"
FIRMWARE="${BUILD_DIR}/botbox-uwb-hub.elf"
# Tools installed by the STM32Cube VS Code extension.
BUNDLES="${HOME}/Library/Application Support/stm32cube/bundles"

# Put a tool on PATH from the bundles unless it is already available.
require_tool() {
  local bundle="$1" tool="$2" path
  if command -v "${tool}" >/dev/null 2>&1; then
    return
  fi
  path="$(find "${BUNDLES}/${bundle}" -type f -perm +111 -name "${tool}" 2>/dev/null | sort -V | tail -n 1)"
  if [[ -z "${path}" ]]; then
    echo "error: ${tool} not found" >&2
    exit 1
  fi
  PATH="$(dirname "${path}"):${PATH}"
}

require_tool cmake cmake
require_tool ninja ninja
require_tool gnu-tools-for-stm32 arm-none-eabi-gcc

echo "--- clean rebuild (${BUILD_PRESET}, UWB_RESULT_B 0x241 ON)"
(
  cd "${REPO_ROOT}"
  cmake --preset "${BUILD_PRESET}" -B "${BUILD_DIR}" \
    -DUWB_HUB_RESULT_B_ENABLE=ON | grep "UWB_RESULT_B"
  cmake --build "${BUILD_DIR}" --clean-first
)

exec "${REPO_ROOT}/scripts/flash_all.sh" "${FIRMWARE}"
