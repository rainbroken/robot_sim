#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec ros2 launch robot_web web.launch.py map_root:="$BAOBAO_WS/maps" "$@"
