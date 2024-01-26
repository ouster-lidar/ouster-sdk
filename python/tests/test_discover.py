#  type: ignore
"""

 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
"""

import requests
import ouster.cli.plugins.discover
from ouster.cli.plugins.discover import service_info_as_text_str
import socket
from zeroconf import DNSAddress


SOCKET_TIMEOUT = 2
FAKESERVER = 'fakeserver.'


class FakeInfo:
    def __init__(self, fake_server, fake_addresses):
        self.fake_server = fake_server
        self.fake_addresses = fake_addresses

    def dns_addresses(self):
        _type = 0  # not important to us
        _class = 0  # not important to us
        _ttl = 0  # not important to us
        return [
            DNSAddress(
                self.fake_server,
                _type,
                _class,
                _ttl,
                socket.inet_pton(socket.AF_INET6 if ':' in address else socket.AF_INET, address)
            )
            for address in self.fake_addresses
        ]

    def parsed_scoped_addresses(self):
        return self.fake_addresses

    @property
    def server(self):
        return self.fake_server


class MockSocket:
    def __init__(self, af, sock_type, proto):
        print(f"Constructed MockSocket with {af}, {sock_type}")

    def connect(self, *args):
        print(f"Called connect with {args}")

    def settimeout(self, timeout):
        print(f"Called settimeout with {timeout}")

    def close(self):
        pass


def mock_socket(addr_family, sock_type, proto):
    return MockSocket(addr_family, mock_socket, proto)


def test_service_info_as_text_str(monkeypatch):
    """It should format correctly even with no addresses in the info."""
    def mock_get(_, **kwargs):
        raise RuntimeError("err")

    with monkeypatch.context() as m:
        m.setattr(requests, "get", mock_get)
        m.setattr(socket, "socket", mock_socket)

        text, color, error = service_info_as_text_str(FakeInfo(FAKESERVER, []), SOCKET_TIMEOUT)
        server, address, prod_line, dest_ip, lidar_port, imu_port = text.split()
        assert FAKESERVER == server
        assert address == '-'
        assert prod_line == '-'

        fake_addresses = ["192.168.100.200", "200a:aa8::8a2e:370:1337"]
        text, color, error = service_info_as_text_str(FakeInfo(FAKESERVER, fake_addresses), SOCKET_TIMEOUT)
        server, address, prod_line, dest_ip, lidar_port, imu_port = text.split()
        assert FAKESERVER == server
        assert address == fake_addresses[0]
        assert prod_line == '-'


def test_service_info_as_text_str_2(monkeypatch):
    """It should format correctly even when sensor metadata can't be retrieved."""
    def mock_get(_, **kwargs):
        raise RuntimeError("err")

    with monkeypatch.context() as m:
        m.setattr(requests, "get", mock_get)
        m.setattr(socket, "socket", mock_socket)

        fake_addresses = ["192.168.100.200", "200a:aa8::8a2e:370:1337"]
        text, color, error = service_info_as_text_str(FakeInfo(FAKESERVER, fake_addresses), SOCKET_TIMEOUT)
        server, address, prod_line, dest_ip, lidar_port, imu_port = text.split()
        assert FAKESERVER == server
        assert address == fake_addresses[0]
        assert prod_line == '-'


def test_service_info_as_text_str_3(monkeypatch):
    """It should set prod_line when the HTTP response contains it."""
    def mock_get(url, **kwargs):
        class MockResponse:
            def json(self):
                return {'prod_line': 'fake_prod_line'}
        return MockResponse()

    def mock_config(self, *arg):  # NOQA
        return None

    with monkeypatch.context() as m:
        m.setattr("socket.socket", mock_socket)
        m.setattr(requests, "get", mock_get)
        m.setattr(ouster.cli.plugins.discover, "_get_config", mock_config)

        fake_addresses = ["192.168.100.200", "200a:aa8::8a2e:370:1337"]

        text, color, error = service_info_as_text_str(FakeInfo(FAKESERVER, fake_addresses), SOCKET_TIMEOUT)
        server, address, prod_line, dest_ip, lidar_port, imu_port = text.split()
        assert FAKESERVER == server
        assert address == fake_addresses[0]
        assert prod_line == 'fake_prod_line'


def test_service_info_as_text_str_4(monkeypatch):
    """It should not set prod_line when the HTTP response does not contain it."""
    def mock_get(url, **kwargs):
        class MockResponse:
            def json(self):
                return {}
        return MockResponse()

    def mock_config(self, *arg):  # NOQA
        return None

    with monkeypatch.context() as m:
        m.setattr(requests, "get", mock_get)
        m.setattr(socket, "socket", mock_socket)
        m.setattr(ouster.cli.plugins.discover, "_get_config", mock_config)

        fake_addresses = ["192.168.100.200", "200a:aa8::8a2e:370:1337"]
        text, color, error = service_info_as_text_str(FakeInfo(FAKESERVER, fake_addresses), SOCKET_TIMEOUT)
        server, address, prod_line, dest_ip, lidar_port, imu_port = text.split()
        assert FAKESERVER == server
        assert address == fake_addresses[0]
        assert prod_line == '-'


def test_ipv6_url(monkeypatch):
    """It should produce the correct url"""
    test_ipv6_address = "200a:aa8::8a2e:370:1337"
    test_url = f"http://[{test_ipv6_address}]/api/v1/sensor/metadata/sensor_info"

    def mock_get(url, **kwargs):
        class MockResponse:
            def json(self):
                assert test_url == url
                return {}

        return MockResponse()

    def mock_config(self, *arg):  # NOQA
        return None

    with monkeypatch.context() as m:
        m.setattr(requests, "get", mock_get)
        m.setattr(socket, "socket", mock_socket)
        m.setattr(ouster.cli.plugins.discover, "_get_config", mock_config)

        fake_addresses = [test_ipv6_address]
        _, _, _ = service_info_as_text_str(FakeInfo(FAKESERVER, fake_addresses), SOCKET_TIMEOUT)
