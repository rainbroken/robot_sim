#!/usr/bin/env python3
"""Start Nav2 only after real tracking; stop the base on stale/lost localization."""
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool
from robot_interfaces.msg import LocalizationStatus
from nav2_msgs.srv import ManageLifecycleNodes
from lifecycle_msgs.srv import GetState


class Gate(Node):
    def __init__(self):
        super().__init__('navigation_gate')
        self.status = None
        self.received = 0.0
        self.started = False
        self.pending = None
        self.enabled = False
        self.enable_pending = None
        self.enable_sent = 0.0
        self.startup_sent = 0.0
        self.state_pending = None
        self.state_sent = 0.0
        self.params = self.create_client(SetParameters, '/command_watchdog/set_parameters')
        self.lifecycle = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.state_client = self.create_client(GetState, '/bt_navigator/get_state')
        self.pub = self.create_publisher(Bool, '/navigation/localization_valid', 1)
        self.create_subscription(LocalizationStatus, '/localization/status', self.on_status, 10)
        self.create_timer(0.1, self.tick)

    def on_status(self, msg):
        self.status, self.received = msg, time.monotonic()

    def tick(self):
        stamp = time.monotonic()
        if self.enable_pending is not None and not self.enable_pending.done() and stamp-self.enable_sent > 3.0:
            self.params.remove_pending_request(self.enable_pending)
            self.enable_pending = None
            self.get_logger().warning('Watchdog parameter response timed out; retrying with motion still gated')
        if not self.enabled and self.enable_pending is None and self.params.service_is_ready():
            request = SetParameters.Request(parameters=[Parameter('require_localization', value=True).to_parameter_msg()])
            self.enable_pending = self.params.call_async(request)
            self.enable_sent = stamp
        if self.enable_pending is not None and self.enable_pending.done():
            try:
                results = self.enable_pending.result().results
                self.enabled = bool(results) and all(x.successful for x in results)
            except Exception as exc:
                self.get_logger().error(str(exc))
            self.enable_pending = None
        s = self.status
        valid = bool(s and stamp - self.received < 0.8 and
                     s.state == LocalizationStatus.TRACKING and s.pose_valid and
                     s.map_loaded and s.lidar_data_fresh and s.lidar_timestamp_valid)
        self.pub.publish(Bool(data=valid and self.enabled))
        if self.pending is not None and not self.pending.done() and stamp-self.startup_sent > 30.0:
            self.lifecycle.remove_pending_request(self.pending)
            self.pending = None
            self.get_logger().warning('Nav2 startup response timed out; checking navigator state before retry')
        if self.pending is not None and self.pending.done():
            try:
                self.started = self.pending.result().success
                self.get_logger().info(f'Nav2 lifecycle startup success={self.started}')
            except Exception as exc:
                self.get_logger().error(str(exc))
            self.pending = None
        if self.state_pending is not None and not self.state_pending.done() and stamp-self.state_sent > 3.0:
            self.state_client.remove_pending_request(self.state_pending)
            self.state_pending = None
        if valid and self.enabled and not self.started and self.pending is None:
            # A lost STARTUP response does not mean startup failed. Query first
            # so an already active manager is not asked to configure twice.
            if self.state_pending is not None and self.state_pending.done():
                try:
                    self.started = self.state_pending.result().current_state.id == 3
                    if not self.started and self.lifecycle.service_is_ready():
                        self.pending = self.lifecycle.call_async(ManageLifecycleNodes.Request(command=0))
                        self.startup_sent = stamp
                except Exception as exc:
                    self.get_logger().error(str(exc))
                self.state_pending = None
            elif self.state_pending is None and self.state_client.service_is_ready():
                self.state_pending = self.state_client.call_async(GetState.Request())
                self.state_sent = stamp


def main():
    rclpy.init()
    node = Gate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
