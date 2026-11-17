"""Single ROS executor owns subscriptions, actions, services and velocity lease."""
from concurrent.futures import Future, TimeoutError
import math
import queue
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import NavigateToPose
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robot_interfaces.msg import LocalizationStatus
from robot_interfaces.srv import SavePcdMap, SetLocalizationPose
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from .core import Occupancy, VelocityLease, finite, yaw


class Bridge(Node):
    def __init__(self, backend):
        super().__init__('robot_web')
        self.backend = backend
        self.data_lock = threading.Lock()
        self.cached = {}
        self.grid = Occupancy()
        self.lease = VelocityLease()
        self.queue = queue.Queue(maxsize=50)
        self.guard = self.create_guard_condition(self._dispatch)
        self.transitioning = False
        self.closed = False
        self.abort_wait = threading.Event()
        self.client_sequences = {}
        self.received = {}
        self.localized = False
        self.localization = {'state': 'UNINITIALIZED', 'message': '尚未收到定位状态'}
        self.pose = None
        self.scan = None
        self.path = []
        self.odom = {'linear': 0.0, 'angular': 0.0}
        self.navigation = {'state': 'idle'}
        self.goal_handle = None
        self.goal_pending = False
        self.epoch = 0
        self.nav_active = False
        self.lifecycle_future = None
        self.lifecycle_sent = 0.0
        self.last_probe = 0.0
        self.last_visual = 0.0
        self.was_driving = False
        self.zero_until = 0.0
        self.clock_value = None
        self.clock_advanced = 0.0
        self.tf = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf, self)
        self.velocity_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.save_client = self.create_client(SavePcdMap, '/mapping/save_pcd_map')
        self.pose_client = self.create_client(SetLocalizationPose, '/localization/set_initial_pose')
        self.params_client = self.create_client(SetParameters, '/command_watchdog/set_parameters')
        self.lifecycle = self.create_client(GetState, '/bt_navigator/get_state')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._map, latched)
        self.create_subscription(OccupancyGridUpdate, '/map_updates', self._map_update, 20)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._on_path, 10)
        self.create_subscription(PointCloud2, '/slam/leveled_cloud',
                                 lambda _: self._touch('mapping'), qos_profile_sensor_data)
        self.create_subscription(LocalizationStatus, '/localization/status', self._localization, latched)
        self.create_subscription(Bool, '/navigation/localization_valid', self._gate, 1)
        self.create_subscription(ClockMessage, '/clock', self._on_clock, qos_profile_sensor_data)
        # Deadman and command processing must keep running when Gazebo time pauses.
        self.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _touch(self, key):
        self.received[key] = time.monotonic()

    def _fresh(self, key, seconds=2.0):
        return time.monotonic() - self.received.get(key, 0.0) < seconds

    def _on_clock(self, msg):
        value = (msg.clock.sec, msg.clock.nanosec)
        if value != self.clock_value:
            self.clock_value, self.clock_advanced = value, time.monotonic()
        self._touch('clock')

    def sim_ready(self):
        return self._fresh('odom') and self._fresh('clock') and time.monotonic() - self.clock_advanced < 2

    def _map(self, msg):
        with self.data_lock:
            self.grid.full(msg)
        self._touch('map')

    def _map_update(self, msg):
        with self.data_lock:
            self.grid.update(msg)

    def _on_odom(self, msg):
        self._touch('odom')
        self.odom = {'linear': msg.twist.twist.linear.x, 'angular': msg.twist.twist.angular.z}

    def _on_scan(self, msg):
        self.scan = msg
        self._touch('scan')

    def _on_path(self, msg):
        if msg.header.frame_id == 'map':
            step = max(1, math.ceil(len(msg.poses) / 1000))
            self.path = [[p.pose.position.x, p.pose.position.y] for p in msg.poses[::step]]

    def _localization(self, msg):
        self._touch('localization')
        self.localized = (msg.state == LocalizationStatus.TRACKING and msg.pose_valid and
                          msg.map_loaded and msg.lidar_data_fresh and msg.lidar_timestamp_valid)
        self.localization = {'state': msg.state_label, 'message': msg.message,
                             'fitness': msg.inlier_ratio if math.isfinite(msg.inlier_ratio) else None,
                             'rmse': msg.registration_error if math.isfinite(msg.registration_error) else None}

    def _gate(self, msg):
        self.gate_valid = msg.data
        self._touch('gate')

    def nav_ready(self):
        return (self.sim_ready() and self.localized and self._fresh('localization', 0.8) and
                self._fresh('gate', 0.8) and getattr(self, 'gate_valid', False) and
                self.nav_active and self.nav.server_is_ready() and self.pose is not None)

    def manual_ready(self):
        mode = self.backend.snapshot()['mode']
        return (not self.transitioning and self.sim_ready() and self.pose is not None and
                ((mode == 'mapping' and self._fresh('mapping')) or
                 (mode == 'navigation' and self.nav_ready())) and
                not self.goal_pending and self.goal_handle is None)

    def snapshot(self):
        with self.data_lock:
            return dict(self.cached)

    def map_snapshot(self):
        with self.data_lock:
            return {'revision': self.grid.revision, 'info': self.grid.meta,
                    'data': self.grid.data.copy()}

    def call(self, action, data=None, timeout=5.0):
        future = Future()
        deadline = time.monotonic() + timeout
        self.queue.put_nowait((action, data or {}, deadline, future))
        self.guard.trigger()
        result = future.result(timeout=timeout + 0.5)
        if isinstance(result, Future):
            deadline = time.monotonic() + timeout
            while True:
                if self.abort_wait.is_set():
                    raise RuntimeError('服务正在退出，操作等待已中止')
                try:
                    return result.result(timeout=min(0.2, max(0, deadline-time.monotonic())))
                except TimeoutError:
                    if time.monotonic() >= deadline:
                        raise
        return result

    def _dispatch(self):
        for _ in range(50):
            try:
                action, data, deadline, future = self.queue.get_nowait()
            except queue.Empty:
                break
            if future.cancelled():
                continue
            try:
                if self.closed or time.monotonic() > deadline:
                    raise RuntimeError('ROS 请求已过期或服务正在关闭')
                future.set_result(self._command(action, data))
            except Exception as exc:
                future.set_exception(exc)

    @staticmethod
    def _service(client, request):
        if not client.service_is_ready():
            raise RuntimeError(f'ROS 服务尚未就绪：{client.srv_name}')
        answer = Future()

        def done(future):
            try:
                answer.set_result(future.result())
            except Exception as exc:
                answer.set_exception(exc)
        client.call_async(request).add_done_callback(done)
        return answer

    def _stop_velocity(self):
        self.lease.clear()
        self.velocity_pub.publish(Twist())
        self.was_driving = False

    def _cancel(self):
        self.epoch += 1
        self._stop_velocity()
        self.zero_until = time.monotonic() + 1.0
        if self.goal_handle is None:
            return {'message': '已停车；等待中的目标会在应答后取消'}
        handle = self.goal_handle
        self.navigation = {**self.navigation, 'state': 'canceling'}
        answer = Future()

        def done(future):
            try:
                if not future.result().goals_canceling:
                    raise RuntimeError('导航取消未获确认，请停止后端')
                answer.set_result({'message': '导航取消已确认'})
            except Exception as exc:
                answer.set_exception(exc)
        handle.cancel_goal_async().add_done_callback(done)
        return answer

    def _command(self, action, data):
        if action in ('velocity', 'release'):
            owner, sequence = data.get('client_id'), data.get('sequence')
            if not isinstance(owner, str) or not 8 <= len(owner) <= 80:
                raise ValueError('client_id 无效')
            if not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 0:
                raise ValueError('遥控 sequence 必须为非负整数')
            if sequence <= self.client_sequences.get(owner, -1):
                raise ValueError('已丢弃乱序或重复遥控请求')
            self.client_sequences[owner] = sequence
            if len(self.client_sequences) > 100:
                del self.client_sequences[next(iter(self.client_sequences))]
        if action == 'probe':
            return {'nodes': self.get_node_names(), 'sim_ready': self.sim_ready()}
        if action == 'prepare':
            self.transitioning = True
            self._cancel()
            return True
        if action == 'reset':
            with self.data_lock:
                self.grid.reset()
            self.received.pop('mapping', None)
            self.received.pop('localization', None)
            self.received.pop('gate', None)
            self.pose, self.path, self.scan = None, [], None
            self.localized, self.nav_active = False, False
            self.goal_handle, self.goal_pending = None, False
            self.navigation = {'state': 'idle'}
            self.localization = {'state': 'UNINITIALIZED', 'message': '等待定位后端'}
            self.zero_until = 0.0
            self.tf.clear()
            return True
        if action == 'finish':
            self.transitioning = False
            return True
        if action == 'watchdog':
            request = SetParameters.Request(parameters=[Parameter(
                'require_localization', value=bool(data['required'])).to_parameter_msg()])
            return self._service(self.params_client, request)
        if action == 'velocity':
            if not self.manual_ready():
                raise RuntimeError('遥控未就绪：检查仿真、建图初始化/定位状态，或先取消导航')
            self.lease.set(data.get('client_id'), data.get('linear'), data.get('angular'), time.monotonic())
            return {'message': '遥控已更新'}
        if action == 'release':
            if self.lease.owner in (None, data.get('client_id')):
                self._stop_velocity()
            return {'message': '遥控已释放'}
        if action == 'stop':
            return self._cancel()
        if action == 'save':
            if self.backend.snapshot()['mode'] != 'mapping' or not self._fresh('mapping'):
                raise RuntimeError('建图尚未就绪')
            self._stop_velocity()
            return self._service(self.save_client, SavePcdMap.Request(output_directory=data['directory']))
        if action == 'initial_pose':
            if self.backend.snapshot()['mode'] != 'navigation' or self.goal_pending or self.goal_handle:
                raise RuntimeError('请在导航模式且无活动目标时设置初值')
            if not self.sim_ready() or not self._fresh('mapping'):
                raise RuntimeError('FAST-LIO 重力初始化尚未完成，请保持静止并等待')
            self._stop_velocity()
            pose = self._make_pose(data)
            return self._service(self.pose_client, SetLocalizationPose.Request(pose=pose))
        if action == 'goal':
            if self.transitioning or self.backend.snapshot()['mode'] != 'navigation' or not self.nav_ready():
                raise RuntimeError('Nav2 或定位门控尚未就绪')
            if self.goal_handle or self.goal_pending:
                raise RuntimeError('已有导航目标，请先取消')
            pose = self._make_pose(data)
            with self.data_lock:
                if not self.grid.contains_free(pose.pose.pose.position.x, pose.pose.pose.position.y):
                    raise ValueError('目标必须落在已知自由栅格内；机器人包络是否可达由 Nav2 判断')
            self._stop_velocity()
            self.zero_until = 0.0
            goal = NavigateToPose.Goal()
            goal.pose.header, goal.pose.pose = pose.header, pose.pose.pose
            self.goal_pending = True
            epoch = self.epoch
            self.navigation = {'state': 'sending', 'goal': [data['x'], data['y'], data['yaw']]}
            answer = Future()

            def accepted(future):
                self.goal_pending = False
                try:
                    handle = future.result()
                    if not handle.accepted:
                        raise RuntimeError('Nav2 拒绝目标')
                    self.goal_handle = handle
                    handle.get_result_async().add_done_callback(lambda f: self._result(f, handle))
                    if epoch != self.epoch:
                        handle.cancel_goal_async()
                        self.navigation['state'] = 'canceling'
                        raise RuntimeError('目标发送期间已请求停车，正在取消目标')
                    self.navigation['state'] = 'navigating'
                    answer.set_result({'message': 'Nav2 已接受目标'})
                except Exception as exc:
                    self.navigation['state'] = 'error'
                    self.navigation['message'] = str(exc)
                    answer.set_exception(exc)
            self.nav.send_goal_async(goal, feedback_callback=self._feedback).add_done_callback(accepted)
            return answer
        raise ValueError(f'未知操作：{action}')

    def _make_pose(self, data):
        x, y = finite(data['x'], -10000, 10000), finite(data['y'], -10000, 10000)
        angle = finite(data['yaw'], -math.pi, math.pi)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp = 'map', self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
        msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[35] = 0.25, 0.25, 0.12
        return msg

    def _feedback(self, msg):
        f = msg.feedback
        self.navigation = {**self.navigation, 'distance_remaining': f.distance_remaining,
                           'recoveries': f.number_of_recoveries,
                           'navigation_seconds': f.navigation_time.sec}

    def _result(self, future, handle):
        if self.goal_handle is not handle:
            return
        try:
            status = future.result().status
            self.navigation = {**self.navigation, 'state': {
                4: 'succeeded', 5: 'canceled', 6: 'aborted'}.get(status, 'error'), 'status': status}
        except Exception as exc:
            self.navigation = {'state': 'error', 'message': str(exc)}
        self.goal_handle = None
        self.path = []
        self._stop_velocity()

    def _visual(self):
        points = []
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            age = (self.get_clock().now().nanoseconds / 1e9 -
                   (tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9))
            if not -0.2 <= age <= 1.5:
                raise ValueError('map TF stale')
            p, q = tf.transform.translation, tf.transform.rotation
            self.pose = [p.x, p.y, yaw(q)]
        except (TransformException, ValueError):
            self.pose = None
        scan = self.scan
        if scan and self._fresh('scan') and self.pose:
            try:
                tf = self.tf.lookup_transform('map', scan.header.frame_id,
                                              rclpy.time.Time.from_msg(scan.header.stamp))
                angle, t = yaw(tf.transform.rotation), tf.transform.translation
                step = max(1, math.ceil(len(scan.ranges)/500))
                for i in range(0, len(scan.ranges), step):
                    r = scan.ranges[i]
                    if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                        a = angle + scan.angle_min + i*scan.angle_increment
                        points.append([t.x + r*math.cos(a), t.y + r*math.sin(a)])
            except TransformException:
                pass
        return points

    def _tick(self):
        now = time.monotonic()
        if now - self.last_probe > 1:
            self.last_probe = now
            if self.lifecycle_future is not None and self.lifecycle_future.done():
                try:
                    self.nav_active = self.lifecycle_future.result().current_state.id == 3
                except Exception:
                    self.nav_active = False
                self.lifecycle_future = None
            if self.lifecycle_future is not None and now - self.lifecycle_sent > 3:
                self.lifecycle.remove_pending_request(self.lifecycle_future)
                self.lifecycle_future = None
                self.nav_active = False
            if self.lifecycle_future is None and self.lifecycle.service_is_ready():
                self.lifecycle_future = self.lifecycle.call_async(GetState.Request())
                self.lifecycle_sent = now
            if not self.lifecycle.service_is_ready():
                self.nav_active = False
        driving = now < self.lease.until and self.manual_ready()
        if driving:
            cmd = Twist()
            cmd.linear.x, cmd.angular.z = self.lease.command
            self.velocity_pub.publish(cmd)
        elif self.was_driving or now < self.zero_until:
            self.velocity_pub.publish(Twist())
        self.was_driving = driving
        if not driving:
            self.lease.clear()
        if now - self.last_visual < 0.2:
            return
        self.last_visual = now
        points = self._visual()
        state = {'sim_ready': self.sim_ready(), 'scan_fresh': self._fresh('scan'),
                 'mapping_ready': self._fresh('mapping') and self.sim_ready(),
                 'frontend_ready': self._fresh('mapping') and self.sim_ready(),
                 'navigation_ready': self.nav_ready(), 'nav2_active': self.nav_active,
                 'manual_ready': self.manual_ready(), 'pose': self.pose, 'odom': self.odom,
                 'localization': dict(self.localization), 'localization_fresh': self._fresh('localization', 0.8),
                 'navigation': dict(self.navigation), 'scan': points, 'path': self.path,
                 'transitioning': self.transitioning, 'sim_time': self.get_clock().now().nanoseconds/1e9}
        with self.data_lock:
            state['map_revision'] = self.grid.revision
            self.cached = state
