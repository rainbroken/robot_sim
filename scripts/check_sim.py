#!/usr/bin/env python3
"""Read-only live smoke check; source scripts/env.sh before running.

Reports rates in simulation time and checks real message contents, not just topics.
Ground truth is diagnostic only and is never fed into the mapping/navigation stack.
"""
import argparse
import json
import math
from pathlib import Path
import struct
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--duration', type=float, default=10.0)
    p.add_argument('--output')
    p.add_argument('--pedestrians', type=int, default=3)
    p.add_argument('--require-map', action='store_true',
                   help='Also require /map and map-to-lidar TF from an external mapping/localization provider')
    args = p.parse_args()
    rclpy.init()
    node = rclpy.create_node('baobao_smoke_check')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    latest, times, wall_times, states = {}, {}, {}, []
    refs = []

    def receive(topic, msg):
        latest[topic] = msg
        if hasattr(msg, 'header'):
            times.setdefault(topic, []).append(stamp(msg))
            wall_times.setdefault(topic, []).append(time.monotonic())
        if topic == '/gazebo/model_states': states.append(msg)

    for topic, cls in [('/livox/lidar', PointCloud2), ('/livox/imu', Imu),
                       ('/scan', LaserScan), ('/odom', Odometry), ('/gazebo/model_states', ModelStates)]:
        refs.append(node.create_subscription(cls, topic, lambda m, t=topic: receive(t, m), qos_profile_sensor_data))
    refs.append(node.create_subscription(OccupancyGrid, '/map', lambda m: receive('/map', m),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    report, failures = {'wall_duration_s': time.monotonic() - start}, []
    expected = ['/livox/lidar', '/livox/imu', '/scan', '/odom', '/gazebo/model_states']
    if args.require_map:
        expected.append('/map')
    for topic in expected:
        if topic not in latest: failures.append('No message: ' + topic)
    for topic, ts in times.items():
        if len(ts) > 1 and ts[-1] > ts[0]:
            report[topic + '_hz_sim'] = (len(ts) - 1) / (ts[-1] - ts[0])
            elapsed = wall_times[topic][-1] - wall_times[topic][0]
            if elapsed > 0:
                report[topic + '_hz_wall_received'] = (len(ts) - 1) / elapsed
                if topic == '/livox/lidar':
                    report['real_time_factor_estimate'] = (ts[-1] - ts[0]) / elapsed
    cloud = latest.get('/livox/lidar')
    if cloud:
        report['cloud_frame'] = cloud.header.frame_id
        report['cloud_points'] = cloud.width * cloud.height
        if cloud.header.frame_id != 'mid360_link' or cloud.width < 100: failures.append('Invalid cloud frame/count')
        offsets = {f.name: f.offset for f in cloud.fields}
        timestamp = struct.unpack_from('<d', cloud.data, offsets['timestamp'])[0]
        report['cloud_point_time_error_s'] = abs(timestamp - stamp(cloud))
        if report['cloud_point_time_error_s'] > 1e-6: failures.append('Incorrect snapshot point timestamp')
    scan = latest.get('/scan')
    if scan:
        report['scan_finite_bins'] = sum(math.isfinite(r) for r in scan.ranges)
        report['scan_unobserved_nan_bins'] = sum(math.isnan(r) for r in scan.ranges)
        if scan.header.frame_id != 'scan_link' or report['scan_finite_bins'] < 40: failures.append('Invalid projected scan')
        if any(math.isinf(r) for r in scan.ranges): failures.append('Unobserved rays incorrectly marked as free space')
    imu = latest.get('/livox/imu')
    if imu:
        values = [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
                  imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]
        report['imu_values'] = values
        if not all(math.isfinite(x) for x in values): failures.append('Nonfinite IMU')
    odom = latest.get('/odom')
    if odom:
        report['odom_xy'] = [odom.pose.pose.position.x, odom.pose.pose.position.y]
        if not all(math.isfinite(x) for x in report['odom_xy']): failures.append('Nonfinite odometry')
    grid = latest.get('/map')
    if grid:
        report['map_size'] = [grid.info.width, grid.info.height]
        report['map_known_cells'] = sum(x >= 0 for x in grid.data)
        if report['map_known_cells'] < 100: failures.append('Map contains insufficient observations')
    if states:
        m = states[-1]
        report['pedestrian_count'] = sum(x.startswith('dynamic_pedestrian_') for x in m.name)
        if report['pedestrian_count'] != args.pedestrians: failures.append('Unexpected pedestrian count')
        if 'robot' in m.name:
            pose = m.pose[m.name.index('robot')]
            report['ground_truth_xyz'] = [pose.position.x, pose.position.y, pose.position.z]
            if not all(math.isfinite(x) for x in report['ground_truth_xyz']) or abs(pose.position.z) > 0.12:
                failures.append('Robot is falling or physics is invalid')
        # Gazebo may publish an empty world before pedestrians are inserted.
        # Compare each actor against its first observed pose, not states[0].
        first_positions = {}
        for state in states:
            for i, name in enumerate(state.name):
                if name.startswith('dynamic_pedestrian_'):
                    first_positions.setdefault(name, (state.pose[i].position.x, state.pose[i].position.y))
        motion = {}
        for name, a in first_positions.items():
            if name.startswith('dynamic_pedestrian_') and name in m.name:
                b = m.pose[m.name.index(name)]
                motion[name] = math.hypot(a[0] - b.position.x, a[1] - b.position.y)
        report['pedestrian_displacements_m'] = motion
        if args.pedestrians and not any(x > 0.03 for x in motion.values()): failures.append('No pedestrian movement observed')
    try:
        parent_frame = 'map' if args.require_map else 'odom'
        t = buffer.lookup_transform(parent_frame, 'mid360_link', rclpy.time.Time())
        report['tf_' + parent_frame + '_to_lidar'] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    except Exception as exc:
        failures.append('Incomplete ' + parent_frame + '-to-lidar TF: ' + str(exc))
    report['failures'] = failures
    report['passed'] = not failures
    text = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)
    print(text)
    if args.output:
        path = Path(args.output); path.parent.mkdir(parents=True, exist_ok=True); path.write_text(text + '\n')
    node.destroy_node(); rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    raise SystemExit(main())
