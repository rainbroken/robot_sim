#!/usr/bin/env python3
"""Bounded Nav2 action smoke test. Requires Gazebo and localized navigation."""
import argparse
import json
import math
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from gazebo_msgs.msg import ModelStates
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener


def wait(node, future, seconds):
    end = time.monotonic()+seconds
    while not future.done() and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=0.1)
    return future.done()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--goals',default='[[-2.0,0.0,0.0],[2.0,0.0,0.0],[0.0,0.0,0.0]]')
    p.add_argument('--output',required=True)
    p.add_argument('--timeout',type=float,default=150)
    args = p.parse_args()
    rclpy.init()
    node = Node('navigation_sim_test', parameter_overrides=[Parameter('use_sim_time',value=True)])
    client = ActionClient(node,NavigateToPose,'/navigate_to_pose')
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    latest_truth = []
    def on_truth(msg):
        if 'robot' in msg.name:
            latest_truth[:] = [msg.pose[msg.name.index('robot')]]
    truth_subscription = node.create_subscription(
        ModelStates, '/gazebo/model_states', on_truth, qos_profile_sensor_data)

    def terminal_pose(x, y, target_yaw):
        report = {}
        try:
            tf = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            q = tf.transform.rotation
            heading = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            report['final_map_pose'] = [tf.transform.translation.x, tf.transform.translation.y, heading]
            report['final_map_xy_goal_error'] = math.hypot(tf.transform.translation.x-x, tf.transform.translation.y-y)
            report['final_map_yaw_goal_error'] = abs(math.atan2(math.sin(heading-target_yaw), math.cos(heading-target_yaw)))
        except Exception as exc:
            report['final_tf_error'] = str(exc)
        if latest_truth:
            p = latest_truth[0]
            q = p.orientation
            heading = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            report['final_gazebo_pose'] = [p.position.x,p.position.y,heading]
        return report
    results=[]
    try:
        lifecycle = node.create_client(GetState, '/bt_navigator/get_state')
        deadline = time.monotonic()+90
        active = False
        while time.monotonic()<deadline:
            if not lifecycle.wait_for_service(timeout_sec=1):
                continue
            state = lifecycle.call_async(GetState.Request())
            if wait(node,state,2) and state.result().current_state.id==3:
                active = True
                break
            rclpy.spin_once(node,timeout_sec=0.5)
        if not active:
            raise RuntimeError('Nav2 did not activate after localization')
        if not client.wait_for_server(timeout_sec=60):
            raise RuntimeError('Nav2 action server did not become ready')
        for x,y,yaw in json.loads(args.goals):
            goal=NavigateToPose.Goal()
            goal.pose.header.frame_id='map'
            goal.pose.header.stamp=node.get_clock().now().to_msg()
            goal.pose.pose.position.x=float(x);goal.pose.pose.position.y=float(y)
            goal.pose.pose.orientation.z=math.sin(yaw/2);goal.pose.pose.orientation.w=math.cos(yaw/2)
            start=time.monotonic()
            feedback = {'max_recoveries': 0}
            def on_feedback(message):
                f = message.feedback
                feedback['max_recoveries'] = max(feedback['max_recoveries'], int(f.number_of_recoveries))
                feedback['last_distance_remaining'] = float(f.distance_remaining)
            sent=client.send_goal_async(goal, feedback_callback=on_feedback)
            if not wait(node,sent,10) or not sent.result().accepted:
                raise RuntimeError('Navigation goal rejected or acceptance timed out')
            handle=sent.result();done=handle.get_result_async()
            if not wait(node,done,args.timeout):
                cancel=handle.cancel_goal_async();wait(node,cancel,5)
                results.append({'goal':[x,y,yaw],'timeout':True,
                                'wall_seconds':time.monotonic()-start,
                                **feedback, **terminal_pose(x,y,yaw)});break
            status=done.result().status
            results.append({'goal':[x,y,yaw],'status':status,'succeeded':status==4,
                            'wall_seconds':time.monotonic()-start,
                            **feedback, **terminal_pose(x,y,yaw)})
            print(json.dumps(results[-1]),flush=True)
            if status!=4:break
    finally:
        Path(args.output).write_text(json.dumps(results,indent=2))
        node.destroy_node();rclpy.shutdown()
    if len(results)!=len(json.loads(args.goals)) or not all(x.get('succeeded') for x in results):
        raise SystemExit(1)


if __name__=='__main__':
    main()
