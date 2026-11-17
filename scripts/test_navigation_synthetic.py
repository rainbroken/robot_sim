#!/usr/bin/env python3
"""Isolated Nav2 integration test with ideal differential motion and ray scans.

Runs real planner/controller/costmaps/BT in a private ROS domain. No Gazebo,
SLAM, hardware transport, person prediction or real sensor noise is exercised.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from lifecycle_msgs.srv import GetState
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--domain', type=int, default=217)
    parser.add_argument('--person', choices=['standing', 'crossing', 'none'], default='standing')
    parser.add_argument('--wall', type=float, default=.95, help='corridor half-width in metres')
    parser.add_argument('--goal-yaw', type=float, default=0.)
    parser.add_argument('--timeout', type=float, default=65.)
    parser.add_argument('--params-file', default=str(
        Path(get_package_share_directory('robot_navigation')) / 'config/nav2.yaml'))
    args = parser.parse_args()
    if not 200 <= args.domain <= 232:
        parser.error('Use a private test domain in 200..232, never a robot domain')
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    params = output.with_suffix('.params.yaml')
    shutil.copyfile(Path(args.params_file).resolve(), params)
    costmap_config = yaml.safe_load(params.read_text())['local_costmap']['local_costmap']['ros__parameters']
    footprint = yaml.safe_load(costmap_config['footprint'])
    half_x = max(abs(p[0]) for p in footprint)
    half_y = max(abs(p[1]) for p in footprint)
    padding = costmap_config['footprint_padding']
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    node = Node('synthetic_navigation_test')
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    odom_pub = node.create_publisher(Odometry, '/odom', 10)
    scan_pub = node.create_publisher(LaserScan, '/scan', 10)
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        reliability=ReliabilityPolicy.RELIABLE)
    map_pub = node.create_publisher(OccupancyGrid, '/map', latched)
    tf_pub = TransformBroadcaster(node)
    static_pub = StaticTransformBroadcaster(node)
    command = [0., 0.]
    last_command = 0.
    sim_time = 0.
    pose = [-1.8, 0., 0.]
    velocity = [0., 0.]
    samples = []
    latest_costmap = None

    def on_costmap(message):
        nonlocal latest_costmap
        latest_costmap = message

    costmap_subscription = node.create_subscription(
        Costmap, '/local_costmap/costmap_raw', on_costmap, latched)
    feedback = {'max_recoveries': 0}
    goal_start = None
    min_clearance = float('inf')
    min_physical_clearance = float('inf')
    collision = False
    status = None

    def on_command(msg):
        nonlocal last_command
        command[:] = [msg.linear.x, msg.angular.z]
        last_command = sim_time

    subscription = node.create_subscription(Twist, '/cmd_vel', on_command, 10)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
    # Refuse to mix this test with any existing graph in the selected domain.
    until = time.monotonic() + 1.
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=.05)
    others = [name for name in node.get_node_names() if name != node.get_name()]
    if others:
        raise RuntimeError(f'Test domain is occupied: {others}')
    fixed = TransformStamped()
    fixed.header.frame_id = 'map'
    fixed.child_frame_id = 'odom'
    fixed.transform.rotation.w = 1.
    static_pub.sendTransform(fixed)
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = .05
    grid.info.width = grid.info.height = 120
    grid.info.origin.position.x = grid.info.origin.position.y = -3.
    grid.info.origin.orientation.w = 1.
    grid.data = [100 if abs(-3 + (i // 120 + .5) * .05) >= args.wall or
                 abs(-3 + (i % 120 + .5) * .05) >= 2.8 else 0 for i in range(14400)]
    map_pub.publish(grid)

    def pedestrian():
        if args.person == 'none':
            return None
        # Starts on the centerline, then crosses repeatedly at 0.25 m/s.
        elapsed = max(0., sim_time - (goal_start or sim_time))
        return (0., 0. if args.person == 'standing' else .55 * math.sin(elapsed * .25 / .55))

    def publish_tick():
        nonlocal sim_time, min_clearance, min_physical_clearance, collision
        dt = .02
        sim_time += dt
        for i, (acc, dec) in enumerate(((.4, .5), (.8, 1.0))):
            target = command[i] if sim_time - last_command < .4 else 0.
            velocity[i] += max(-dec * dt, min(acc * dt, target - velocity[i]))
        mid_yaw = pose[2] + velocity[1] * dt / 2
        pose[0] += velocity[0] * math.cos(mid_yaw) * dt
        pose[1] += velocity[0] * math.sin(mid_yaw) * dt
        pose[2] = math.atan2(math.sin(pose[2] + velocity[1] * dt),
                             math.cos(pose[2] + velocity[1] * dt))
        clock = Clock()
        clock.clock.sec = int(sim_time)
        clock.clock.nanosec = int((sim_time % 1) * 1e9)
        clock_pub.publish(clock)
        tf = TransformStamped()
        tf.header.frame_id = 'odom'
        tf.header.stamp = clock.clock
        tf.child_frame_id = 'base_footprint'
        tf.transform.translation.x, tf.transform.translation.y = pose[:2]
        tf.transform.rotation.z = math.sin(pose[2] / 2)
        tf.transform.rotation.w = math.cos(pose[2] / 2)
        tf_pub.sendTransform(tf)
        odom = Odometry()
        odom.header = tf.header
        odom.child_frame_id = tf.child_frame_id
        odom.pose.pose.position.x, odom.pose.pose.position.y = pose[:2]
        odom.pose.pose.orientation = tf.transform.rotation
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = velocity
        odom_pub.publish(odom)
        person = pedestrian()
        c, s = math.cos(pose[2]), math.sin(pose[2])
        def clearance_for(hx, hy):
            distance = args.wall - abs(pose[1]) - abs(s) * hx - abs(c) * hy
            if person:
                px = (person[0] - pose[0]) * c + (person[1] - pose[1]) * s
                py = -(person[0] - pose[0]) * s + (person[1] - pose[1]) * c
                distance = min(distance, math.hypot(max(0., abs(px) - hx),
                                                  max(0., abs(py) - hy)) - .20)
            return distance
        clearance = clearance_for(half_x + padding, half_y + padding)
        physical_clearance = clearance_for(half_x, half_y)
        if goal_start is not None:
            min_clearance = min(min_clearance, clearance)
            min_physical_clearance = min(min_physical_clearance, physical_clearance)
            collision |= physical_clearance < 0.
        if round(sim_time / dt) % 5:
            return
        scan = LaserScan()
        scan.header.frame_id = 'base_footprint'
        scan.header.stamp = clock.clock
        scan.angle_min, scan.angle_max = -math.pi, math.pi - math.pi / 360
        scan.angle_increment = math.pi / 360
        scan.range_min, scan.range_max = .01, 8.
        scan.scan_time = .1
        for i in range(720):
            a = pose[2] + scan.angle_min + i * scan.angle_increment
            dx, dy = math.cos(a), math.sin(a)
            distances = [8.]
            if abs(dx) > 1e-9:
                distances.append(((2.8 if dx > 0 else -2.8) - pose[0]) / dx)
            if abs(dy) > 1e-9:
                distances.append(((args.wall if dy > 0 else -args.wall) - pose[1]) / dy)
            if person:
                rx, ry = person[0] - pose[0], person[1] - pose[1]
                projection = rx * dx + ry * dy
                discriminant = .20 ** 2 - (rx * rx + ry * ry - projection ** 2)
                if discriminant >= 0 and projection > 0:
                    distances.append(projection - math.sqrt(discriminant))
            scan.ranges.append(float(min(d for d in distances if d > .01)))
        scan_pub.publish(scan)
        if goal_start is not None:
            costs = {}
            if latest_costmap and person:
                metadata = latest_costmap.metadata
                for name, xy in [('person', person), ('robot', pose[:2])]:
                    mx = int((xy[0] - metadata.origin.position.x) / metadata.resolution)
                    my = int((xy[1] - metadata.origin.position.y) / metadata.resolution)
                    if 0 <= mx < metadata.size_x and 0 <= my < metadata.size_y:
                        costs[name] = latest_costmap.data[my * metadata.size_x + mx]
                costs['age'] = sim_time - latest_costmap.header.stamp.sec - latest_costmap.header.stamp.nanosec * 1e-9
            samples.append({'t': sim_time - goal_start, 'pose': pose.copy(),
                            'velocity': velocity.copy(), 'command': command.copy(),
                            'clearance': clearance, 'physical_clearance': physical_clearance,
                            'person': person, 'costmap': costs})

    timer = node.create_timer(.02, publish_tick)
    launch_log = output.with_suffix('.launch.log').open('w')
    process = subprocess.Popen(['ros2', 'launch', 'robot_navigation', 'navigation.launch.py',
                                'use_live_map:=true', f'params_file:={params}'], stdout=launch_log,
                               stderr=subprocess.STDOUT, start_new_session=True)
    try:
        until = time.monotonic() + 45.
        active, state_future, next_state_check = False, None, 0.
        while time.monotonic() < until and not (active and client.server_is_ready()):
            rclpy.spin_once(node, timeout_sec=.02)
            if process.poll() is not None:
                raise RuntimeError('Nav2 launch exited; inspect the launch log')
            if state_future is not None and state_future.done():
                active = state_future.result().current_state.id == 3
                state_future = None
            if state_future is None and time.monotonic() >= next_state_check and lifecycle.service_is_ready():
                state_future = lifecycle.call_async(GetState.Request())
                next_state_check = time.monotonic() + .25
        if not (active and client.server_is_ready()):
            raise RuntimeError('Nav2 navigator did not activate')
        # Let initial costmap observations arrive before the first goal.
        ready = sim_time + 2.
        while sim_time < ready:
            rclpy.spin_once(node, timeout_sec=.02)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 1.8
        goal.pose.pose.orientation.z = math.sin(args.goal_yaw / 2)
        goal.pose.pose.orientation.w = math.cos(args.goal_yaw / 2)

        def on_feedback(message):
            feedback['max_recoveries'] = max(feedback['max_recoveries'],
                                              message.feedback.number_of_recoveries)

        goal_start = sim_time
        sent = client.send_goal_async(goal, feedback_callback=on_feedback)
        while not sent.done() and sim_time - goal_start < 10:
            rclpy.spin_once(node, timeout_sec=.02)
        if not sent.done() or not sent.result().accepted:
            raise RuntimeError('Goal was rejected or acceptance timed out')
        handle = sent.result()
        result = handle.get_result_async()
        while not result.done() and sim_time - goal_start < args.timeout and not collision:
            rclpy.spin_once(node, timeout_sec=.02)
        if result.done():
            status = result.result().status
        else:
            cancel = handle.cancel_goal_async()
            until = time.monotonic() + 2.
            while not cancel.done() and time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
    finally:
        command[:] = [0., 0.]
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        launch_log.close()
        report = {'model': 'ideal differential plant + synthetic scans; real Nav2 servers/BT',
                  'person': args.person, 'wall': args.wall, 'status': status,
                  'params_file': str(params),
                  'succeeded': status == 4, 'collision': collision,
                  'min_padded_footprint_clearance': min_clearance if samples else None,
                  'min_physical_footprint_clearance': min_physical_clearance if samples else None,
                  'padding_margin_violated': min_clearance < 0,
                  'sim_seconds': sim_time - goal_start if goal_start is not None else None,
                  'final_pose': pose, **feedback, 'samples': samples}
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'samples'}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    if status != 4 or collision or feedback['max_recoveries'] or min_physical_clearance < .02:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
