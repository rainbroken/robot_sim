#!/usr/bin/env bash
# Source this workspace alone; never source the reference robot's overlay.
BAOBAO_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for baobao_prefix in ${AMENT_PREFIX_PATH//:/ }; do
  if [ "$baobao_prefix" != "${baobao_prefix#"$BAOBAO_WS/install"}" ]; then continue; fi
  if [ -f "$baobao_prefix/share/ament_index/resource_index/packages/robot_description" ] ||
     [ -f "$baobao_prefix/share/ament_index/resource_index/packages/robot_navigation" ]; then
    echo "检测到其他机器人工作空间 overlay: $baobao_prefix；请在干净终端加载 baobao，避免同名包混用。" >&2
    return 1 2>/dev/null || exit 1
  fi
done
source /opt/ros/humble/setup.bash
if [ -f "$BAOBAO_WS/install/local_setup.bash" ]; then
  source "$BAOBAO_WS/install/local_setup.bash"
fi
