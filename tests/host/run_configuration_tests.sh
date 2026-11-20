#!/bin/sh
# Build and run the flash configuration loader host tests (schema 1 migration).
set -eu
root=$(cd "$(dirname "$0")/../.." && pwd)
out=${TMPDIR:-/tmp}/botbox_configuration_tests
mkdir -p "$out/include"
# Stand-in for the CMake generated header: values must match the manifest.
cat > "$out/include/schema_compatibility.hpp" <<'EOF'
#pragma once
#include <cstdint>
namespace botbox { namespace compatibility {
static constexpr uint16_t FIRMWARE_METADATA_SCHEMA_VERSION = 1U;
static constexpr uint16_t MINIMUM_SUPPORTED_CONFIGURATION_BINARY_FORMAT_VERSION = 1U;
static constexpr uint16_t CURRENT_CONFIGURATION_BINARY_FORMAT_VERSION = 1U;
static constexpr uint16_t MINIMUM_SUPPORTED_CONFIGURATION_SCHEMA_VERSION = 1U;
static constexpr uint16_t CURRENT_CONFIGURATION_SCHEMA_VERSION = 2U;
} }
EOF
c++ -std=c++17 -Wall -Wextra -Werror -I"$out/include" \
  -I"$root/App/configuration" \
  "$root/tests/host/test_configuration.cpp" \
  "$root/App/configuration/firmware_configuration.cpp" \
  -o "$out/test_configuration"
"$out/test_configuration"
