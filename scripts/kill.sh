#!/usr/bin/env bash
# 清理当前工作空间的 ROS/仿真进程；不依赖 ROS 环境加载成功。
set -eo pipefail
workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
exec python3 - "$workspace" "$@" <<'PY'
import argparse
import os
from pathlib import Path
import signal
import sys
import time

parser = argparse.ArgumentParser(description="清理当前工作空间的 ROS/仿真进程（仅当前用户）。")
parser.add_argument("--dry-run", action="store_true", help="只列出目标，不发送信号")
workspace = Path(sys.argv.pop(1))
args = parser.parse_args()


def inside(value, directory):
    return value == str(directory) or value.startswith(str(directory) + "/")


def inspect(pid):
    try:
        proc = Path("/proc") / str(pid)
        if proc.stat().st_uid != os.getuid():
            return None
        # comm 字段可能含空格或括号，从最后一个右括号之后解析。
        stat = (proc / "stat").read_text().rsplit(")", 1)[1].split()
        if stat[0] == "Z":
            return None
        command = [os.fsdecode(x) for x in (proc / "cmdline").read_bytes().split(b"\0") if x]
        if not command:
            return None
        return {"pid": pid, "ppid": int(stat[1]), "start": stat[19], "cmd": command}
    except (OSError, ValueError, IndexError):
        return None


def belongs(process):
    command = process["cmd"]
    # 前两个参数覆盖原生可执行文件和 Python 节点，避免匹配查看配置的编辑器。
    if any(inside(arg, workspace / "install") for arg in command[:2]):
        return True
    names = {Path(arg).name for arg in command[:2]}
    is_ros = any(inside(arg, Path("/opt/ros")) for arg in command[:2])
    is_sim = bool(names & {"gzserver", "gzclient", "gazebo", "rviz2"})
    is_launcher = (
        "ros2" in names and "launch" in command and
        any(package in command for package in (
            "robot_bringup", "robot_sim", "robot_slam", "robot_navigation", "robot_description"
        ))
    )
    if not (is_ros or is_sim or is_launcher):
        return False
    if any(inside(arg, workspace / "install") for arg in command):
        return True
    try:
        proc = Path("/proc") / str(process["pid"])
        environment = dict(
            entry.split(b"=", 1) for entry in (proc / "environ").read_bytes().split(b"\0")
            if b"=" in entry
        )
        # 环境可识别父 launch 已退出、且参数只有 /tmp 路径的系统 ROS 节点。
        overlay = any(
            inside(prefix, workspace / "install")
            for variable in (b"AMENT_PREFIX_PATH", b"COLCON_PREFIX_PATH", b"CMAKE_PREFIX_PATH")
            for prefix in os.fsdecode(environment.get(variable, b"")).split(":") if prefix
        )
        return overlay or inside(str((proc / "cwd").resolve()), workspace)
    except OSError:
        return False


processes = {}
for entry in Path("/proc").iterdir():
    if entry.name.isdigit():
        process = inspect(int(entry.name))
        if process:
            processes[process["pid"]] = process

# 永远保留脚本本身及调用它的终端/父进程。
protected = {os.getpid()}
parent = os.getppid()
while parent > 1 and parent not in protected:
    protected.add(parent)
    parent = processes.get(parent, {}).get("ppid", 1)
targets = {pid for pid, process in processes.items() if pid not in protected and belongs(process)}
while True:
    children = {
        pid for pid, process in processes.items()
        if process["ppid"] in targets and pid not in protected
    }
    if children <= targets:
        break
    targets.update(children)

if not targets:
    print(f"未发现需要清理的工作空间进程：{workspace}")
    sys.exit(0)

print(f"{'预览' if args.dry_run else '准备清理'} {len(targets)} 个进程：", flush=True)
for pid in sorted(targets):
    print(f"  PID {pid}: {' '.join(processes[pid]['cmd'])}", flush=True)
if args.dry_run:
    sys.exit(0)


def still_running(pid):
    current = inspect(pid)
    # 检查启动时间，避免 PID 复用后向无关进程发信号。
    return current is not None and current["start"] == processes[pid]["start"]


for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
    targets = {pid for pid in targets if still_running(pid)}
    if not targets:
        break
    print(f"发送 {sig.name}：{', '.join(map(str, sorted(targets)))}", flush=True)
    for pid in sorted(targets):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass
        except PermissionError as error:
            print(f"无法清理 PID {pid}：{error}", file=sys.stderr)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline and any(still_running(pid) for pid in targets):
        time.sleep(0.1)

remaining = sorted(pid for pid in targets if still_running(pid))
if remaining:
    print(f"仍有进程未退出：{remaining}", file=sys.stderr)
    sys.exit(1)
print("工作空间进程清理完成。")
PY
