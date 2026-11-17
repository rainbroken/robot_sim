#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec ros2 run robot_slam save_map "$@"
