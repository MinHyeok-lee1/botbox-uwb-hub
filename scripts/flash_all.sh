#!/usr/bin/env bash
# Clean-rebuild the Release firmware and flash it to every ST-LINK connected
# to this machine.
#
# Usage: scripts/flash_all.sh [path/to/firmware.elf]
#
# With a firmware path, that file is flashed as it is and nothing is built.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_PRESET="Release"
DEFAULT_FIRMWARE="${REPO_ROOT}/build/${BUILD_PRESET}/botbox-uwb-hub.elf"
# Tools installed by the STM32Cube VS Code extension.
BUNDLES="${HOME}/Library/Application Support/stm32cube/bundles"

# Print the directory holding the newest installed copy of a bundled tool.
newest_tool_dir() {
  local bundle="$1" tool="$2" path
  path="$(find "${BUNDLES}/${bundle}" -type f -perm +111 -name "${tool}" 2>/dev/null | sort -V | tail -n 1)"
  if [[ -n "${path}" ]]; then
    dirname "${path}"
  fi
}

# Put a tool on PATH from the bundles unless it is already available.
require_tool() {
  local bundle="$1" tool="$2" dir
  if command -v "${tool}" >/dev/null 2>&1; then
    return
  fi
  dir="$(newest_tool_dir "${bundle}" "${tool}")"
  if [[ -z "${dir}" ]]; then
    echo "error: ${tool} not found" >&2
    exit 1
  fi
  PATH="${dir}:${PATH}"
}

clean_rebuild() {
  require_tool cmake cmake
  require_tool ninja ninja
  require_tool gnu-tools-for-stm32 arm-none-eabi-gcc

  echo "--- clean rebuild (${BUILD_PRESET})"
  (
    cd "${REPO_ROOT}"
    cmake --preset "${BUILD_PRESET}" >/dev/null
    cmake --build --preset "${BUILD_PRESET}" --clean-first
  )
}

if [[ $# -ge 1 ]]; then
  FIRMWARE="$1"
else
  FIRMWARE="${DEFAULT_FIRMWARE}"
  clean_rebuild
fi

if [[ ! -f "${FIRMWARE}" ]]; then
  echo "error: firmware not found: ${FIRMWARE}" >&2
  exit 1
fi

require_tool programmer STM32_Programmer_CLI

SERIALS=()
while IFS= read -r serial; do
  SERIALS+=("${serial}")
done < <(STM32_Programmer_CLI -l stlink | sed -n 's/.*ST-LINK SN *: *\([0-9A-Fa-f]*\).*/\1/p')

if [[ ${#SERIALS[@]} -eq 0 ]]; then
  echo "error: no ST-LINK probe connected" >&2
  exit 1
fi

echo "firmware: ${FIRMWARE} (built $(date -r "${FIRMWARE}" '+%Y-%m-%d %H:%M:%S'))"
echo "probes:   ${#SERIALS[@]}"

FAILED=0
for serial in "${SERIALS[@]}"; do
  echo "--- flashing ${serial}"
  if STM32_Programmer_CLI -c port=SWD sn="${serial}" -w "${FIRMWARE}" -v -rst >/dev/null; then
    echo "    ok"
  else
    echo "    FAILED" >&2
    FAILED=1
  fi
done

exit "${FAILED}"
