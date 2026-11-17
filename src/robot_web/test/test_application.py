import pytest

from robot_web.application import Application
from robot_web.core import Backend, MapStore


def test_external_backend_is_rejected_before_changing_watchdog(tmp_path):
    class Bridge:
        def __init__(self):
            self.calls = []

        def call(self, action):
            self.calls.append(action)
            assert action == 'probe'
            return {'sim_ready': True, 'nodes': ['fastlio_frontend']}

    bridge = Bridge()
    backend = Backend(tmp_path / 'logs')
    app = Application(bridge, backend, MapStore(tmp_path / 'maps'))
    try:
        with pytest.raises(RuntimeError, match='外部后端'):
            app._switch('mapping')
        assert bridge.calls == ['probe']
        assert backend.snapshot()['mode'] == 'idle'
    finally:
        app.worker.shutdown()
