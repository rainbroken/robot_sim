#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$BAOBAO_WS"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-4}"
export MAKEFLAGS="-j${CMAKE_BUILD_PARALLEL_LEVEL}"
colcon build --symlink-install --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF "$@"
