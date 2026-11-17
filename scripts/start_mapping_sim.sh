#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
exec ros2 launch robot_slam simulation.launch.py mode:=mapping "$@"
