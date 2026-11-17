"""Serialize long jobs without blocking ROS feedback or the velocity deadman."""
from concurrent.futures import ThreadPoolExecutor, TimeoutError
from datetime import datetime
import threading
import time
import uuid


class Application:
    JOBS = {'start_mapping', 'start_navigation', 'stop_backend', 'save_map', 'goal', 'initial_pose'}
    CONFLICTS = {'fastlio_frontend', 'fastlio_mapping', 'loop_closure_mapper',
                 'open3d_localizer', 'bt_navigator', 'mapping_tf_bridge'}

    def __init__(self, bridge, backend, maps):
        self.bridge, self.backend, self.maps = bridge, backend, maps
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix='robot_web_job')
        self.lock = threading.Lock()
        self.job = None
        self.closing = False

    def snapshot(self):
        state = self.bridge.snapshot()
        state['backend'] = self.backend.snapshot()
        state['map_root'] = str(self.maps.root)
        with self.lock:
            state['job'] = dict(self.job) if self.job else None
        return state

    def command(self, action, data):
        if action in ('velocity', 'release', 'stop'):
            # Independent of the job worker: stopping during a slow map save still works.
            return self.bridge.call(action, data, timeout=3)
        if action not in self.JOBS:
            raise ValueError('未知操作')
        with self.lock:
            if self.closing:
                raise RuntimeError('服务正在退出')
            if self.job and self.job['state'] == 'running':
                raise RuntimeError('上一操作尚未结束，请等待其结果')
            self.job = {'id': uuid.uuid4().hex, 'action': action, 'state': 'running',
                        'message': '处理中', 'started_at': datetime.now().isoformat(timespec='seconds')}
            result = dict(self.job)
        self.worker.submit(self._run, result['id'], action, data)
        return result

    def _run(self, job_id, action, data):
        try:
            result = self._execute(action, data)
            state, message = 'succeeded', result
        except Exception as exc:
            state, message = 'failed', f'{type(exc).__name__}: {exc}'
        with self.lock:
            if self.job['id'] == job_id:
                self.job.update(state=state, message=message)

    def _watchdog(self, required):
        result = self.bridge.call('watchdog', {'required': required})
        if not result.results or not all(item.successful for item in result.results):
            raise RuntimeError('无法设置底盘定位门控，已中止模式切换')

    def _switch(self, mode, directory=None):
        probe = self.bridge.call('probe')
        if self.backend.snapshot()['pid'] is None:
            conflicts = self.CONFLICTS.intersection(probe['nodes'])
            if conflicts:
                raise RuntimeError('检测到外部后端，请先在原终端关闭：' + ', '.join(sorted(conflicts)))
        if mode != 'idle' and not probe['sim_ready']:
            raise RuntimeError('请先启动 Gazebo；当前 /clock 未推进或 /odom 不新鲜')
        self.bridge.call('prepare')
        try:
            self._watchdog(True)  # Latch the existing base gate before stopping the old backend.
            self.backend.stop()
            deadline = time.monotonic() + 6
            while True:
                if self.closing:
                    raise RuntimeError('控制台正在退出')
                conflicts = self.CONFLICTS.intersection(self.bridge.call('probe')['nodes'])
                if not conflicts:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError('检测到外部/残留后端，请先在原终端关闭：' + ', '.join(sorted(conflicts)))
                time.sleep(0.25)
            self.bridge.call('reset')
            if mode != 'idle':
                self.backend.start(mode, directory)
                time.sleep(0.4)
                if self.backend.snapshot()['mode'] == 'error':
                    raise RuntimeError('后端启动失败，请查看后端日志')
                if mode == 'mapping':
                    self._watchdog(False)
            return '后端已停止，底盘保持门控停车' if mode == 'idle' else (
                '建图后端已启动，请静止等待重力初始化' if mode == 'mapping' else
                '导航后端已启动，等待定位 TRACKING 和 Nav2 激活；位置不符时请设置初始位姿')
        except Exception:
            # A failed transition must not leave a newly spawned backend driving.
            self.backend.stop()
            raise
        finally:
            self.bridge.call('finish')

    def _execute(self, action, data):
        if action == 'start_mapping':
            return self._switch('mapping')
        if action == 'start_navigation':
            return self._switch('navigation', self.maps.validate(data.get('name')))
        if action == 'stop_backend':
            return self._switch('idle')
        if action == 'save_map':
            if not self.bridge.snapshot().get('mapping_ready'):
                raise RuntimeError('建图尚未就绪，请先采集数据')
            directory = self.maps.reserve(data.get('name'))
            self.bridge.call('prepare')
            try:
                response = self.bridge.call('save', {'directory': str(directory)}, timeout=180)
                if not response.success or response.occupancy_empty:
                    raise RuntimeError(f'地图保存失败或为空：{response.message}。保留目录供检查，请换名重试')
                self.maps.validate(directory.name)
                return f'已保存 {directory.name}：全局点 {response.full_point_count}，定位点 {response.localization_point_count}'
            finally:
                self.bridge.call('finish')
        if action == 'goal':
            try:
                return self.bridge.call(action, data, timeout=15)['message']
            except TimeoutError:
                # A late goal acknowledgement must never start unattended motion.
                self.bridge.call('stop', timeout=3)
                raise RuntimeError('导航目标应答超时，已请求取消；请检查导航状态')
        if action == 'initial_pose':
            response = self.bridge.call(action, data, timeout=10)
            if not response.success:
                raise RuntimeError(response.message)
            return response.message or '初始位姿已设置，等待定位收敛'
        raise ValueError('未知操作')

    def close(self):
        with self.lock:
            self.closing = True
        # The executor remains alive until this finishes; services can still respond.
        try:
            self.bridge.call('prepare', timeout=2)
            if self.backend.snapshot()['pid'] is not None:
                self._watchdog(True)
        except Exception:
            pass
        self.bridge.abort_wait.set()
        self.worker.shutdown(wait=True, cancel_futures=True)
        self.backend.stop()
