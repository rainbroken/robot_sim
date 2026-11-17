"""ROS-independent validation, occupancy storage and owned process lifecycle."""
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

import yaml


def finite(value, low, high):
    if isinstance(value, bool):
        raise ValueError('数值不能是布尔值')
    result = float(value)
    if not math.isfinite(result) or not low <= result <= high:
        raise ValueError(f'数值必须在 {low}..{high} 内且有限')
    return result


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


class MapStore:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        self.root.mkdir(parents=True, exist_ok=True)

    def directory(self, name):
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', name):
            raise ValueError('地图名请使用 1–64 位字母、数字、下划线或连字符，以字母或数字开头')
        directory = self.root / name
        if directory.is_symlink() or directory.resolve().parent != self.root:
            raise ValueError('地图目录不能是符号链接或超出 map_root')
        return directory

    def validate(self, name):
        directory = self.directory(name)
        for filename in ('localization.pcd', 'full_map.pcd', 'occupancy.yaml'):
            path = directory / filename
            if not path.is_file() or path.stat().st_size == 0 or path.resolve().parent != directory:
                raise ValueError(f'地图不完整或路径不合法：{filename}')
        data = yaml.safe_load((directory / 'occupancy.yaml').read_text())
        if not isinstance(data, dict):
            raise ValueError('occupancy.yaml 格式错误')
        image = directory / str(data['image'])
        if image.resolve().parent != directory or not image.is_file() or image.stat().st_size == 0:
            raise ValueError('栅格图像丢失或超出地图目录')
        finite(data['resolution'], 0.001, 10)
        if len(data['origin']) != 3:
            raise ValueError('地图 origin 必须包含 x/y/yaw')
        for value in data['origin']:
            finite(value, -1e6, 1e6)
        return directory

    def reserve(self, name):
        directory = self.directory(name)
        directory.mkdir(exist_ok=False)  # Never overwrite, including partial saves.
        return directory

    def list(self):
        result = []
        for directory in sorted(self.root.iterdir()):
            if not directory.is_dir() or directory.is_symlink():
                continue
            try:
                self.validate(directory.name)
                error = ''
            except (ValueError, KeyError, TypeError, OSError, yaml.YAMLError) as exc:
                error = str(exc)
            result.append({'name': directory.name, 'ready': not error, 'error': error})
        return result


class Occupancy:
    """Store the full map plus incremental updates; revision is monotonic."""
    def __init__(self):
        self.revision = 0
        self.meta = None
        self.data = []

    def reset(self):
        self.revision += 1
        self.meta, self.data = None, []

    def full(self, msg):
        width, height = msg.info.width, msg.info.height
        if width * height != len(msg.data) or width * height > 8_000_000:
            return
        p = msg.info.origin
        self.meta = {'width': width, 'height': height, 'resolution': msg.info.resolution,
                     'origin': [p.position.x, p.position.y, yaw(p.orientation)],
                     'frame': msg.header.frame_id}
        self.data = list(msg.data)
        self.revision += 1

    def update(self, msg):
        m = self.meta
        if (not m or msg.header.frame_id != m['frame'] or msg.x < 0 or msg.y < 0 or
                msg.x + msg.width > m['width'] or msg.y + msg.height > m['height'] or
                len(msg.data) != msg.width * msg.height):
            return
        for row in range(msg.height):
            start = (msg.y + row) * m['width'] + msg.x
            self.data[start:start + msg.width] = msg.data[row*msg.width:(row+1)*msg.width]
        self.revision += 1

    def contains_free(self, x, y):
        m = self.meta
        if not m or m['frame'] != 'map' or m['resolution'] <= 0:
            return False
        ox, oy, angle = m['origin']
        dx, dy = x - ox, y - oy
        col = math.floor((math.cos(angle)*dx + math.sin(angle)*dy) / m['resolution'])
        row = math.floor((-math.sin(angle)*dx + math.cos(angle)*dy) / m['resolution'])
        return (0 <= col < m['width'] and 0 <= row < m['height'] and
                0 <= self.data[row*m['width'] + col] < 50)


class VelocityLease:
    def __init__(self):
        self.owner, self.until, self.command = None, 0.0, (0.0, 0.0)

    def set(self, owner, linear, angular, now):
        if not isinstance(owner, str) or not 8 <= len(owner) <= 80:
            raise ValueError('遥控 client_id 无效')
        if self.owner not in (None, owner) and now < self.until:
            raise ValueError('另一浏览器正在遥控，请先松开控制')
        command = (finite(linear, -0.2, 0.3), finite(angular, -0.6, 0.6))
        self.owner, self.until = owner, now + 0.35
        self.command = command

    def clear(self):
        self.owner, self.until, self.command = None, 0.0, (0.0, 0.0)


class Backend:
    """Manage only our own ros2 launch process group, never pkill by name."""
    def __init__(self, logs):
        self.logs = Path(logs).expanduser().resolve()
        self.logs.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.process = None
        self.mode, self.map_name, self.log_path = 'idle', '', ''
        self.stream = None
        self.exit_code = None

    def snapshot(self):
        with self.lock:
            code = self.process.poll() if self.process else self.exit_code
            return {'mode': self.mode if code is None else 'error', 'map_name': self.map_name,
                    'pid': self.process.pid if self.process else None,
                    'exit_code': code, 'log_path': self.log_path}

    def start(self, mode, directory=None):
        with self.lock:
            if self.process is not None:
                raise RuntimeError('先停止已拥有的后端')
            if mode not in ('mapping', 'navigation'):
                raise ValueError('无效后端模式')
            launch = 'slam.launch.py' if mode == 'mapping' else 'navigation.launch.py'
            cmd = ['ros2', 'launch', 'robot_slam', launch]
            if mode == 'navigation':
                cmd.append(f'map_dir:={directory}')
            self.log_path = str(self.logs / f'{time.time_ns()}_{mode}.log')
            self.stream = open(self.log_path, 'ab', buffering=0)
            try:
                self.process = subprocess.Popen(cmd, stdout=self.stream, stderr=subprocess.STDOUT,
                                                start_new_session=True)
            except Exception:
                self.stream.close()
                self.stream = None
                raise
            self.mode, self.map_name = mode, directory.name if directory else ''
            self.exit_code = None

    def stop(self):
        # Do not hold the status lock while waiting: the ROS deadman must tick.
        with self.lock:
            process = self.process
        if process is not None:
            # Signal descendants even if the ros2 launch parent has already exited.
            for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    process.poll()  # Reap the parent so it cannot keep the group alive.
                    try:
                        os.killpg(process.pid, 0)
                    except ProcessLookupError:
                        break
                    time.sleep(0.1)
                else:
                    continue
                break
            process.wait(timeout=2)
        with self.lock:
            if self.stream:
                self.stream.close()
            self.process, self.stream = None, None
            self.mode, self.map_name, self.exit_code = 'idle', '', None

    def tail(self):
        if not self.log_path:
            return ''
        with open(self.log_path, 'rb') as stream:
            stream.seek(0, 2)
            stream.seek(max(0, stream.tell() - 8000))
            return stream.read().decode('utf-8', errors='replace')
