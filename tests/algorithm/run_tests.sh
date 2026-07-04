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
  -I"$repo_root/components/control/follow_avoid" \
  -I"$repo_root/components/control/chassis" \
  "$repo_root/tests/algorithm/test_follow_avoid.c" \
  "$repo_root/components/control/follow_avoid/follow_avoid.c" \
  "$repo_root/components/control/chassis/chassis.c" \
  -lm \
  -o "$build_dir/test_follow_avoid"

"$build_dir/test_follow_avoid"
