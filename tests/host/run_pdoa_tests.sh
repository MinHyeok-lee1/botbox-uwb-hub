#!/bin/sh
# Build and run the PDoA (0x244) host tests with the host C++ compiler.
set -eu
root=$(cd "$(dirname "$0")/../.." && pwd)
out=${TMPDIR:-/tmp}/botbox_pdoa_tests
c++ -std=c++17 -Wall -Wextra -Werror -I"$root/App/pdoa" \
  "$root/tests/host/test_pdoa.cpp" \
  "$root/App/pdoa/pdoa_result.cpp" \
  "$root/App/pdoa/pdoa_publisher.cpp" \
  -o "$out"
"$out"
