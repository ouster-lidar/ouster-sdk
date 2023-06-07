#  type: ignore

import socket
import threading
import time
import pytest
import flask
from werkzeug.serving import make_server
from ouster.client._client import Client


class ServerThread(threading.Thread):

    def __init__(self, app):
        threading.Thread.__init__(self)
        for port in range(8000, 9000):
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                addr = ('127.0.0.1', port)
                self.socket = socket.create_server(addr, family=socket.AF_INET)
                self.server = make_server('127.0.0.1', port, app, fd=self.socket.fileno())
                self.port = port
            except OSError:
                pass

    def run(self):
        self.server.serve_forever()

    def shutdown(self):
        self.server.shutdown()


@pytest.fixture
def mock_server():
    app = flask.Flask('dummy')

    @app.route('/api/v1/system/firmware')
    def firmware():
        time.sleep(2)
        return {}

    server = ServerThread(app)
    server.start()
    return server


@pytest.mark.skip
def test_http_client_timeout(mock_server):
    start_time = time.time()
    try:
        Client(f'localhost:{mock_server.port}', 7502, 7503).get_metadata(timeout_sec=1)
    except RuntimeError as e:
        assert "Timeout was reached" in str(e)
        assert time.time() - start_time < 3
    finally:
        mock_server.shutdown()
