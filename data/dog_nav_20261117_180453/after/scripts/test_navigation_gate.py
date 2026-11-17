#!/usr/bin/env python3
"""Regression tests for lost DDS responses; no robot or ROS node is created."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

path = Path(__file__).resolve().parents[1]/'src/robot_slam/scripts/navigation_gate.py'
spec = importlib.util.spec_from_file_location('navigation_gate_under_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Pending:
    def __init__(self, result=None):
        self.value = result

    def done(self):
        return self.value is not None

    def result(self):
        return self.value


class Client:
    def __init__(self):
        self.calls = []
        self.removed = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        future = Pending()
        self.calls.append((request, future))
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


class GateResponseTests(unittest.TestCase):
    def setUp(self):
        # Exercise the actual state machine with deterministic clock/services.
        self.gate = module.Gate.__new__(module.Gate)
        g = self.gate
        g.status = SimpleNamespace(state=module.LocalizationStatus.TRACKING,
                                   pose_valid=True, map_loaded=True,
                                   lidar_data_fresh=True, lidar_timestamp_valid=True)
        g.received = 10.0
        g.started = g.enabled = False
        g.pending = g.enable_pending = g.state_pending = None
        g.enable_sent = g.startup_sent = g.state_sent = 0.0
        g.params, g.lifecycle, g.state_client = Client(), Client(), Client()
        g.pub = Mock()
        g.get_logger = Mock(return_value=Mock())

    def tick(self, stamp=10.0):
        with patch.object(module.time, 'monotonic', return_value=stamp):
            self.gate.tick()

    def test_lost_parameter_response_retries_and_keeps_motion_gated(self):
        g = self.gate
        lost = g.enable_pending = Pending()
        self.tick()
        self.assertEqual(g.params.removed, [lost])
        self.assertEqual(len(g.params.calls), 1)
        self.assertFalse(g.pub.publish.call_args.args[0].data)
        g.enable_pending.value = SimpleNamespace(results=[SimpleNamespace(successful=True)])
        self.tick(10.1)
        self.assertTrue(g.enabled)

    def test_lost_startup_response_checks_active_state_without_duplicate_startup(self):
        g = self.gate
        g.enabled = True
        lost = g.pending = Pending()
        g.startup_sent = -25.0
        self.tick()
        self.assertEqual(g.lifecycle.removed, [lost])
        self.assertEqual(len(g.state_client.calls), 1)
        g.state_pending.value = SimpleNamespace(current_state=SimpleNamespace(id=3))
        self.tick(10.1)
        self.assertTrue(g.started)
        self.assertEqual(g.lifecycle.calls, [])

    def test_lost_state_response_is_retried(self):
        g = self.gate
        g.enabled = True
        lost = g.state_pending = Pending()
        self.tick()
        self.assertEqual(g.state_client.removed, [lost])
        self.assertEqual(len(g.state_client.calls), 1)
        self.assertEqual(g.lifecycle.calls, [])

    def test_empty_parameter_response_does_not_enable_motion(self):
        g = self.gate
        g.enable_pending = Pending(SimpleNamespace(results=[]))
        self.tick()
        self.assertFalse(g.enabled)
        self.assertFalse(g.pub.publish.call_args.args[0].data)


if __name__ == '__main__':
    unittest.main(verbosity=2)
