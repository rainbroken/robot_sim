#!/usr/bin/env python3
"""Gazebo-only fault injection: stop the localizer heartbeat, verify zero base commands."""
import argparse
import json
import os
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    command = Path(f'/proc/{args.pid}/cmdline').read_bytes()
    if b'/baobao/install/robot_slam/lib/robot_slam/open3d_localizer' not in command:
        raise RuntimeError('Refusing to pause a process outside this simulation workspace')
    rclpy.init()
    node = Node('localization_stop_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
    gate, safe = [], []
    node.create_subscription(Bool,'/navigation/localization_valid',lambda m:gate.append(m.data),10)
    node.create_subscription(Twist,'/cmd_vel_safe',lambda m:safe.append([m.linear.x,m.angular.z]),10)
    pub = node.create_publisher(Twist,'/cmd_vel',10)
    def spin(seconds, command=None):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            if command is not None:pub.publish(command)
            rclpy.spin_once(node,timeout_sec=0.05)
    spin(1)
    if not gate or not gate[-1]:
        raise RuntimeError('Localization must be tracking before this test')
    try:
        os.kill(args.pid,signal.SIGSTOP)
        spin(1.5)
        gate.clear();safe.clear()
        cmd=Twist();cmd.linear.x=0.1
        spin(1.5,cmd)
        report={'heartbeat_paused':True,'gate_samples':len(gate),
                'all_gate_false':bool(gate) and not any(gate),
                'safe_samples':len(safe),
                'max_safe_linear':max((abs(v[0]) for v in safe),default=None),
                'max_safe_angular':max((abs(v[1]) for v in safe),default=None)}
    finally:
        pub.publish(Twist())
        os.kill(args.pid,signal.SIGCONT)
    gate.clear();spin(2,Twist())
    report['recovered']=bool(gate) and gate[-1]
    Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    node.destroy_node();rclpy.shutdown()
    if not (report['all_gate_false'] and report['safe_samples'] and
            report['max_safe_linear']==0 and report['max_safe_angular']==0 and report['recovered']):
        raise SystemExit(1)


if __name__=='__main__':
    main()
