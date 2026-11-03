#!/bin/bash

find_programmer_cli() {
  shopt -s nullglob

  local appdata="${APPDATA:-}"
  local localappdata="${LOCALAPPDATA:-}"
  local programfiles="${ProgramFiles:-}"
  local programfiles_x86
  programfiles_x86="$(printenv 'ProgramFiles(x86)' 2>/dev/null || true)"

  if command -v cygpath >/dev/null 2>&1; then
    [ -z "$appdata" ] || appdata="$(cygpath -u "$appdata" 2>/dev/null || echo "$appdata")"
    [ -z "$localappdata" ] || localappdata="$(cygpath -u "$localappdata" 2>/dev/null || echo "$localappdata")"
    [ -z "$programfiles" ] || programfiles="$(cygpath -u "$programfiles" 2>/dev/null || echo "$programfiles")"
    [ -z "$programfiles_x86" ] || programfiles_x86="$(cygpath -u "$programfiles_x86" 2>/dev/null || echo "$programfiles_x86")"
  fi

  local candidates=(
    "$HOME"/Library/Application\ Support/stm32cube/bundles/programmer/*/bin/STM32_Programmer_CLI
    "${XDG_DATA_HOME:-$HOME/.local/share}"/stm32cube/bundles/programmer/*/bin/STM32_Programmer_CLI
    "$appdata"/stm32cube/bundles/programmer/*/bin/STM32_Programmer_CLI.exe
    "$localappdata"/stm32cube/bundles/programmer/*/bin/STM32_Programmer_CLI.exe
    /Applications/STM32CubeProgrammer*/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI
    /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer*/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI
    /Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer*/bin/STM32_Programmer_CLI
    /opt/ST/STM32CubeCLT_*/STM32CubeProgrammer/bin/STM32_Programmer_CLI
    /opt/ST/STM32CubeCLT/STM32CubeProgrammer/bin/STM32_Programmer_CLI
    /opt/ST/STM32CubeProgrammer*/bin/STM32_Programmer_CLI
    /usr/local/STMicroelectronics/STM32Cube/STM32CubeProgrammer*/bin/STM32_Programmer_CLI
    "$programfiles"/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe
    "$programfiles_x86"/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe
  )

  local candidate
  for candidate in "${candidates[@]}"; do
    if [ -f "$candidate" ]; then
      echo "$candidate"
      return 0
    fi
  done

  if command -v STM32_Programmer_CLI >/dev/null 2>&1; then
    command -v STM32_Programmer_CLI
    return 0
  fi

  return 1
}

require_programmer_cli() {
  local programmer_cli
  programmer_cli="$(find_programmer_cli || true)"

  if [ -z "$programmer_cli" ] || [ ! -f "$programmer_cli" ]; then
    echo "Error: STM32_Programmer_CLI not found."
    echo "Install STM32CubeProgrammer or add STM32_Programmer_CLI to PATH."
    return 1
  fi

  echo "$programmer_cli"
}

# Read-only check that the connected target is the hub's MCU before any Flash
# write. The configuration is written by address, so the bank mode (DBANK) does
# not matter here.
preflight_stm32g474() {
  local programmer_cli="$1"
  local serial_number="${2:-}"
  local connect_args=(-c port=SWD mode=HOTPLUG)
  local output

  if [ -n "$serial_number" ]; then
    connect_args+=("sn=$serial_number")
  fi

  if ! output="$("$programmer_cli" "${connect_args[@]}" 2>&1)"; then
    echo "$output" >&2
    echo "Error: target preflight failed." >&2
    return 1
  fi

  if ! echo "$output" | grep -Eiq 'Device ID[^:]*:[[:space:]]*(0x)?0*469([^0-9A-Fa-f]|$)'; then
    echo "Error: target is not a supported STM32G47x/G48x device (Device ID 0x469)." >&2
    return 1
  fi
  if ! echo "$output" | grep -Eiq 'Flash size[^:]*:[[:space:]]*512[[:space:]]*(KBytes?|KB|KiB)'; then
    echo "Error: target Flash size is not confirmed as 512 KiB." >&2
    return 1
  fi

  local voltage
  voltage="$(echo "$output" | awk -F: 'tolower($1) ~ /voltage/ { gsub(/,/, ".", $2); gsub(/[^0-9.]/, "", $2); print $2; exit }')"
  if [ -z "$voltage" ] || ! awk -v value="$voltage" 'BEGIN { exit(value >= 1.7 ? 0 : 1) }'; then
    echo "Error: target voltage is below 1.7 V or cannot be confirmed." >&2
    return 1
  fi

  echo "Target preflight: STM32G47x/G48x (Device ID 0x469), 512 KiB."
}
