#!/usr/bin/env python3
"""Read-only mapping/navigation evidence. Gazebo truth never controls the robot."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from robot_interfaces.msg import LocalizationStatus
from tf2_ros import Buffer, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=900)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('navigation_evidence', parameter_overrides=[Parameter('use_sim_time', value=True)])
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    latest, counts, commands, samples, transitions = {}, Counter(), {}, [], []
    start = time.monotonic()
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def receive(topic, msg):
        latest[topic] = msg
        counts[topic] += 1
        if isinstance(msg, Twist):
            values = [msg.linear.x, msg.linear.y, msg.angular.z]
            stat = commands.setdefault(topic, {'samples': 0, 'nonzero': 0, 'below_021': 0,
                                               'startup_021': 0, 'max_abs_y': 0.0,
                                               'min_nonzero_norm': None, 'max_abs_x': 0.0,
                                               'max_abs_yaw': 0.0})
            norm = math.sqrt(sum(v*v for v in values))
            stat['samples'] += 1
            stat['max_abs_y'] = max(stat['max_abs_y'], abs(values[1]))
            stat['max_abs_x'] = max(stat['max_abs_x'], abs(values[0]))
            stat['max_abs_yaw'] = max(stat['max_abs_yaw'], abs(values[2]))
            if norm > 1e-4:
                stat['nonzero'] += 1
                stat['below_021'] += int(norm < .21-1e-6)
                stat['startup_021'] += int(abs(norm-.21) < 1e-6)
                stat['min_nonzero_norm'] = min(stat['min_nonzero_norm'] or norm, norm)
        if isinstance(msg, LocalizationStatus):
            if not transitions or transitions[-1]['state'] != msg.state_label:
                transitions.append({'wall_seconds': time.monotonic()-start,
                                    'state': msg.state_label, 'message': msg.message})

    subscriptions = []
    for topic, kind in [('/gazebo/model_states', ModelStates), ('/cmd_vel', Twist),
                        ('/cmd_vel_safe', Twist), ('/localization/status', LocalizationStatus)]:
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, t=topic: receive(t, msg), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        OccupancyGrid, '/map', lambda msg: receive('/map', msg),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    next_sample = 0.0
    try:
        while not stop and time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.05)
            if time.monotonic() < next_sample:
                continue
            next_sample = time.monotonic()+.2
            row = {'wall_seconds': time.monotonic()-start,
                   'sim_seconds': node.get_clock().now().nanoseconds * 1e-9}
            gt = latest.get('/gazebo/model_states')
            if gt and 'robot' in gt.name:
                pose = gt.pose[gt.name.index('robot')]
                row['truth'] = [pose.position.x, pose.position.y, yaw(pose.orientation)]
                row['pedestrians'] = {name: [p.position.x, p.position.y]
                                      for name, p in zip(gt.name, gt.pose)
                                      if name.startswith('dynamic_pedestrian_')}
            try:
                tf = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                row['map'] = [tf.transform.translation.x, tf.transform.translation.y,
                              yaw(tf.transform.rotation)]
                row['tf_sim_seconds'] = tf.header.stamp.sec + tf.header.stamp.nanosec*1e-9
                if 'truth' in row:
                    row['xy_error_m'] = math.dist(row['truth'][:2], row['map'][:2])
                    delta = row['truth'][2]-row['map'][2]
                    row['yaw_error_rad'] = abs(math.atan2(math.sin(delta), math.cos(delta)))
            except Exception:
                pass
            status = latest.get('/localization/status')
            if status:
                row['localization'] = {'state': status.state_label, 'valid': status.pose_valid,
                                       'fitness': status.inlier_ratio, 'rmse': status.registration_error}
            samples.append(row)
    finally:
        report = {'wall_seconds': time.monotonic()-start, 'counts': dict(counts),
                  'commands': commands, 'transitions': transitions, 'samples': samples,
                  'measurement_note': 'Latest TF versus latest Gazebo model_states; no strict time alignment or trajectory alignment. Diagnostic only.'}
        for key in ['xy_error_m', 'yaw_error_rad']:
            values = [r[key] for r in samples if key in r]
            if values:
                report[key] = {'mean': sum(values)/len(values), 'max': max(values), 'last': values[-1]}
        grid = latest.get('/map')
        if grid:
            report['occupancy'] = {'width': grid.info.width, 'height': grid.info.height,
                                   'resolution': grid.info.resolution,
                                   'known': sum(v >= 0 for v in grid.data),
                                   'occupied': sum(v >= 65 for v in grid.data)}
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k != 'samples'}, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
