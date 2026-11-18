#!/bin/sh
# Build and run the IF-5 host tests with the host C++ compiler.
set -eu
root=$(cd "$(dirname "$0")/../.." && pwd)
out=${TMPDIR:-/tmp}/botbox_if5_tests
c++ -std=c++17 -Wall -Wextra -Werror -I"$root/App/protocol" \
  "$root/tests/host/test_if5.cpp" \
  "$root/App/protocol/if5_codec.cpp" \
  "$root/App/protocol/if5_node.cpp" \
  -o "$out"
"$out"
