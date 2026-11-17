"""ROS API regression tests on an isolated domain, without Gazebo."""
import os
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.signals import SignalHandlerOptions

from robot_web.bridge import Bridge
from robot_web.core import Backend


@pytest.fixture
def bridge(tmp_path):
    # Constructing a real node catches collisions with rclpy's internal attributes.
    os.environ.setdefault('ROS_DOMAIN_ID', '218')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Bridge(Backend(tmp_path))
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_clock_callback_and_stale_simulation(bridge):
    bridge._on_clock(SimpleNamespace(clock=SimpleNamespace(sec=10, nanosec=0)))
    bridge._touch('odom')
    assert bridge.sim_ready()
    bridge.clock_advanced -= 3
    bridge._on_clock(SimpleNamespace(clock=SimpleNamespace(sec=10, nanosec=0)))
    assert not bridge.sim_ready()  # Repeated /clock messages cannot hide a pause.


def test_out_of_order_release_cannot_resume_old_velocity(bridge):
    bridge._command('release', {'client_id': 'browser-1', 'sequence': 20})
    with pytest.raises(ValueError, match='乱序'):
        bridge._command('velocity', {'client_id': 'browser-1', 'sequence': 19,
                                     'linear': .2, 'angular': 0.})
    assert bridge.lease.owner is None


def test_uninitialized_localization_metrics_are_json_safe(bridge):
    from robot_interfaces.msg import LocalizationStatus
    import json
    message = LocalizationStatus()
    message.registration_error = float('inf')
    message.inlier_ratio = float('nan')
    bridge._localization(message)
    assert bridge.localization['rmse'] is None
    json.dumps(bridge.localization, allow_nan=False)
