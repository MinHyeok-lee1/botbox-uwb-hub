#!/bin/sh
# Build and run the PDoA host tests: the 0x244 packer/publisher, and the IF-0
# decoder + E2 Core pipeline (the vendored Core compiled unchanged, V3 only).
set -eu
root=$(cd "$(dirname "$0")/../.." && pwd)
tmp=${TMPDIR:-/tmp}
core="$root/App/third_party/uwb_core"

c++ -std=c++17 -Wall -Wextra -Werror -I"$root/App/pdoa" \
  "$root/tests/host/test_pdoa.cpp" \
  "$root/App/pdoa/pdoa_result.cpp" \
  "$root/App/pdoa/pdoa_publisher.cpp" \
  -o "$tmp/botbox_pdoa_tests"
"$tmp/botbox_pdoa_tests"

objs=""
for src in uwb_core uwb_math uwb_engines; do
  cc -std=c11 -O2 -DUWB_V3_ONLY=1 -ffp-contract=off -fno-fast-math -I"$core" \
    -c "$core/$src.c" -o "$tmp/botbox_pdoa_$src.o"
  objs="$objs $tmp/botbox_pdoa_$src.o"
done
# shellcheck disable=SC2086
c++ -std=c++17 -Wall -Wextra -Werror -I"$root/App/pdoa" -I"$core" \
  "$root/tests/host/test_pdoa_pipeline.cpp" \
  "$root/App/pdoa/pdoa_if0.cpp" \
  "$root/App/pdoa/pdoa_pipeline.cpp" \
  "$root/App/pdoa/pdoa_result.cpp" \
  "$root/App/pdoa/pdoa_publisher.cpp" \
  $objs -lm -o "$tmp/botbox_pdoa_pipeline_tests"
"$tmp/botbox_pdoa_pipeline_tests"
