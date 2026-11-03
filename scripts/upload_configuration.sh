#!/bin/bash
# Generate the configuration image from JSON and program it with ST-LINK.
# Only the configuration area at the end of Flash is erased; the firmware stays.
#
# Usage: scripts/upload_configuration.sh [config.json]
#   DRY_RUN=1    generate the image but do not touch the board
#   STLINK_SN=.. pick one probe when several are connected
#   NO_RESET=1   leave the target halted state as it is after programming

set -e

cd "$(dirname "$0")/.."

ADDRESS="0x0807F000"
CONFIG_JSON="${1:-configuration/default.json}"
OUTPUT_FILE="build/configuration/firmware-configuration-v1.bin"
PYTHON_BIN="${PYTHON_BIN:-}"
STLINK_SN="${STLINK_SN:-}"

source scripts/stm32_programmer.sh

find_python3() {
  if [ -n "$PYTHON_BIN" ]; then
    echo "$PYTHON_BIN"
    return 0
  fi

  if [ -x /usr/bin/python3 ]; then
    echo /usr/bin/python3
    return 0
  fi

  if command -v python3 >/dev/null 2>&1; then
    command -v python3
    return 0
  fi

  return 1
}

PYTHON_BIN="$(find_python3 || true)"

if [ -z "$PYTHON_BIN" ] || [ ! -x "$PYTHON_BIN" ]; then
  echo "Error: python3 not found. Set PYTHON_BIN=/path/to/python3 if needed."
  exit 1
fi

if [ ! -f "$CONFIG_JSON" ]; then
  echo "Error: configuration JSON not found: $CONFIG_JSON"
  exit 1
fi

"$PYTHON_BIN" tools/generate_configuration.py "$CONFIG_JSON" "$OUTPUT_FILE"

echo "Searching STM32_Programmer_CLI..."
CUBEPROG_CLI="$(require_programmer_cli)"

echo "Programmer:    $CUBEPROG_CLI"
echo "Configuration: $CONFIG_JSON"
echo "Binary:        $OUTPUT_FILE"
echo "Target:        ST-LINK SWD @ $ADDRESS"

if [ "${DRY_RUN:-0}" = "1" ]; then
  echo "DRY_RUN=1: skipping configuration upload."
  exit 0
fi

preflight_stm32g474 "$CUBEPROG_CLI" "$STLINK_SN"

# The preflight above is read-only and uses HOTPLUG. Programming uses NORMAL so
# CubeProgrammer resets and halts the core before erasing/writing Flash.
CONNECT_ARGS=(-c port=SWD mode=NORMAL)
if [ -n "$STLINK_SN" ]; then CONNECT_ARGS+=("sn=$STLINK_SN"); fi
RESET_ARGS=()
if [ "${NO_RESET:-0}" != "1" ]; then RESET_ARGS=(-rst); fi

"$CUBEPROG_CLI" \
  "${CONNECT_ARGS[@]}" \
  -d "$OUTPUT_FILE" "$ADDRESS" \
  -v \
  "${RESET_ARGS[@]}"

echo "Configuration upload done."
