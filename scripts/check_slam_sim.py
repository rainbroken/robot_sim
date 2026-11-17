#!/usr/bin/env python3
"""Read-only Gazebo diagnostics; ground truth is never fed to SLAM/navigation."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from robot_interfaces.msg import LocalizationStatus
from tf2_ros import Buffer, TransformListener


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=30)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('slam_sim_diagnostic', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    latest, counts, samples, states = {}, {}, [], []
    def remember(name):
        def callback(msg):
            latest[name] = msg
            counts[name] = counts.get(name, 0) + 1
        return callback
    subscriptions = []
    for topic, kind in [('/gazebo/model_states', ModelStates), ('/odom', Odometry),
                        ('/fastlio/base_odom', Odometry), ('/mapping/optimized_odom', Odometry),
                        ('/livox/lidar', PointCloud2), ('/slam/body_cloud', PointCloud2)]:
        subscriptions.append(node.create_subscription(kind, topic, remember(topic), qos_profile_sensor_data))
    transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions.append(node.create_subscription(OccupancyGrid, '/map', remember('/map'), transient))
    def status(msg):
        remember('/localization/status')(msg)
        if not states or states[-1]['state'] != msg.state_label:
            states.append({'wall_elapsed': time.monotonic()-start, 'state': msg.state_label, 'message': msg.message})
    subscriptions.append(node.create_subscription(LocalizationStatus, '/localization/status', status, transient))
    start, next_sample = time.monotonic(), 0.0
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() < next_sample:
                continue
            next_sample = time.monotonic()+0.2
            row = {'wall_elapsed': time.monotonic()-start, 'sim_time': node.get_clock().now().nanoseconds*1e-9}
            gt = latest.get('/gazebo/model_states')
            if gt and 'robot' in gt.name:
                pose = gt.pose[gt.name.index('robot')]
                row['ground_truth'] = [pose.position.x, pose.position.y]
            for topic in ['/odom','/fastlio/base_odom','/mapping/optimized_odom']:
                msg = latest.get(topic)
                if msg:
                    p = msg.pose.pose.position
                    row[topic] = [p.x,p.y,p.z]
            try:
                tr = buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
                row['map_pose'] = [tr.transform.translation.x,tr.transform.translation.y]
                if 'ground_truth' in row:
                    row['map_xy_error'] = math.dist(row['map_pose'],row['ground_truth'])
            except Exception:
                pass
            msg = latest.get('/localization/status')
            if msg:
                row['localization'] = {'state':msg.state_label,'fitness':msg.inlier_ratio,
                                       'rmse':msg.registration_error,'pose_valid':msg.pose_valid}
            samples.append(row)
    finally:
        errors = [x['map_xy_error'] for x in samples if 'map_xy_error' in x]
        grid = latest.get('/map')
        report = {'counts':counts,'state_transitions':states,'samples':samples,
                  'map_xy_error_max':max(errors) if errors else None,
                  'map_xy_error_mean':sum(errors)/len(errors) if errors else None,
                  'map_xy_error_last':errors[-1] if errors else None}
        if grid:
            report['occupancy'] = {'width':grid.info.width,'height':grid.info.height,
                'resolution':grid.info.resolution,'known_cells':sum(v>=0 for v in grid.data),
                'occupied_cells':sum(v>=65 for v in grid.data)}
        Path(args.output).write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k!='samples'},indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
