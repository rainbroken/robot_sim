#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
if [ "$#" -eq 0 ]; then
  echo '用法: bash scripts/start_navigation.sh map_dir:=/绝对路径/地图目录（先启动仿真，关闭建图进程）' >&2
  exit 2
fi
exec ros2 launch robot_slam navigation.launch.py "$@"
