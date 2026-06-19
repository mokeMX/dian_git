#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/../.." && pwd)"
build_dir="$repo_root/work/test-build"
mkdir -p "$build_dir"

gcc \
  -std=c11 \
  -Wall \
  -Wextra \
  -Werror \
  -I"$repo_root/components/sensors/a02yyuw" \
  -I"$repo_root/components/sensors/bu_uwb" \
  -I"$repo_root/components/sensors/fsr_adc" \
  "$repo_root/tests/protocol/test_sensor_parsers.c" \
  "$repo_root/components/sensors/a02yyuw/a02yyuw.c" \
  "$repo_root/components/sensors/bu_uwb/bu_uwb.c" \
  "$repo_root/components/sensors/fsr_adc/fsr_adc.c" \
  -lm \
  -o "$build_dir/test_sensor_parsers"

"$build_dir/test_sensor_parsers"
