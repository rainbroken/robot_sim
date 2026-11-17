from functools import partial
import json
from pathlib import Path
import threading
from types import SimpleNamespace as S
import urllib.error
import urllib.request

import pytest

from robot_web.server import Handler, Server


@pytest.fixture
def api():
    calls = []
    app = S(command=lambda action, data: calls.append((action, data)) or {'message': 'ok'},
            snapshot=lambda: {'sim_ready': False})
    root = Path(__file__).resolve().parents[1] / 'www'
    server = Server(('127.0.0.1', 0), partial(Handler, app=app, root=root))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    url = f'http://127.0.0.1:{server.server_port}'
    try:
        yield opener, url, calls
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


def test_same_origin_and_json_required(api):
    opener, url, calls = api
    for origin, content_type, status in [('http://other.example', 'application/json', 403),
                                       (url, 'text/plain', 400)]:
        request = urllib.request.Request(url+'/api/command', data=b'{"action":"stop"}',
            headers={'Origin': origin, 'Content-Type': content_type})
        with pytest.raises(urllib.error.HTTPError) as error:
            opener.open(request)
        assert error.value.code == status
    assert not calls
    request = urllib.request.Request(url+'/api/command', data=b'{"action":"stop"}',
        headers={'Origin': url, 'Content-Type': 'application/json'})
    assert json.load(opener.open(request))['message'] == 'ok'
    assert calls == [('stop', {})]


def test_static_allowlist_and_security_headers(api):
    opener, url, _ = api
    with opener.open(url+'/') as response:
        assert b'app.js' in response.read()
        assert "frame-ancestors 'none'" in response.headers['Content-Security-Policy']
    for path in ('/../package.xml', '/%2e%2e/package.xml', '/server.py'):
        with pytest.raises(urllib.error.HTTPError) as error:
            opener.open(url+path)
        assert error.value.code == 404


def test_body_size_limit(api):
    opener, url, calls = api
    request = urllib.request.Request(url+'/api/command', data=b' '*4097,
        headers={'Content-Type': 'application/json'})
    with pytest.raises(urllib.error.HTTPError) as error:
        opener.open(request)
    assert error.value.code == 400 and not calls
