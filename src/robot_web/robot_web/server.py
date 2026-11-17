"""Same-origin HTTP API and static UI; ROS stays on its own executor thread."""
from concurrent.futures import TimeoutError
from functools import partial
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import threading
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .application import Application
from .bridge import Bridge
from .core import Backend, MapStore


class Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    def __init__(self, *args, app, root, **kwargs):
        self.app, self.root = app, root
        super().__init__(*args, **kwargs)

    def setup(self):
        super().setup()
        self.connection.settimeout(10)

    def log_message(self, *_):
        pass

    def _reply(self, status, value, mime='application/json; charset=utf-8'):
        body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Content-Security-Policy', "default-src 'self'; style-src 'self'; img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        try:
            path = urlparse(self.path).path
            if path == '/api/state':
                self._reply(200, self.app.snapshot())
            elif path == '/api/map':
                self._reply(200, self.app.bridge.map_snapshot())
            elif path == '/api/maps':
                self._reply(200, {'maps': self.app.maps.list(), 'root': str(self.app.maps.root)})
            elif path == '/api/log':
                self._reply(200, {'text': self.app.backend.tail()})
            elif path in ('/', '/index.html', '/app.js', '/style.css'):
                name = 'index.html' if path == '/' else path[1:]
                mime = {'html': 'text/html', 'js': 'text/javascript', 'css': 'text/css'}[name.split('.')[-1]]
                self._reply(200, (self.root / name).read_bytes(), mime + '; charset=utf-8')
            else:
                self._reply(404, {'error': 'Not found'})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self._reply(500, {'error': str(exc)})

    def do_POST(self):
        try:
            if urlparse(self.path).path != '/api/command':
                self._reply(404, {'error': 'Not found'})
                return
            origin = self.headers.get('Origin')
            if origin and urlparse(origin).netloc != self.headers.get('Host'):
                self._reply(403, {'error': '仅允许同源请求'})
                return
            if self.headers.get_content_type() != 'application/json':
                raise ValueError('需要 application/json')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 4096:
                raise ValueError('请求体必须在 1..4096 字节内')
            data = json.loads(self.rfile.read(length), parse_constant=lambda _: None)
            if not isinstance(data, dict) or not isinstance(data.get('data', {}), dict):
                raise ValueError('请求和 data 必须为对象')
            result = self.app.command(data.get('action'), data.get('data', {}))
            self._reply(202 if result.get('state') == 'running' else 200, result)
        except (ValueError, KeyError, TypeError) as exc:
            self._reply(400, {'error': str(exc)})
        except (RuntimeError, TimeoutError) as exc:
            self._reply(409, {'error': str(exc) or '请求超时；请检查状态后再重试'})
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self._reply(500, {'error': str(exc)})


def main(args=None):
    # Handle SIGINT/SIGTERM ourselves so ROS can park the base and reap children first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    backend = Backend('~/.local/share/baobao/web_logs')
    node = Bridge(backend)
    host = node.declare_parameter('host', '127.0.0.1').value
    port = node.declare_parameter('port', 8080).value
    maps = MapStore(node.declare_parameter('map_root', '~/.local/share/baobao/maps').value)
    app = Application(node, backend, maps)
    root = Path(get_package_share_directory('robot_web')) / 'www'
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, name='robot_web_ros', daemon=True)
    stopping = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.set())
    server = None
    http_thread = None
    try:
        server = Server((host, int(port)), partial(Handler, app=app, root=root))
        ros_thread.start()
        http_thread = threading.Thread(target=server.serve_forever, daemon=True)
        http_thread.start()
        node.get_logger().info(f'Baobao web: http://{host}:{port}; maps={maps.root}')
        while not stopping.wait(0.2) and ros_thread.is_alive():
            pass
        if not stopping.is_set():
            raise RuntimeError('ROS executor stopped unexpectedly; see the traceback above')
    finally:
        if server:
            server.shutdown()
            server.server_close()
        if http_thread:
            http_thread.join(timeout=2)
        if ros_thread.is_alive():
            app.close()
        else:
            backend.stop()
        node.closed = True
        executor.shutdown(timeout_sec=3)
        if ros_thread.is_alive():
            ros_thread.join(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
