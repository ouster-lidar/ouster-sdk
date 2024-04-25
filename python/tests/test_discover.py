#  type: ignore
"""

 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
"""

import packaging  # TODO remove
import importlib  # TODO remove
from sys import version_info
import pytest
import asyncio
from zeroconf import InterfaceChoice, IPVersion
from zeroconf.asyncio import AsyncServiceInfo
from ouster.cli.plugins.discover import\
    parse_scope_id, format_hostname_for_url, \
    get_output_for_sensor, get_text_for_oserror, is_link_local_ipv6_address_and_missing_scope_id, \
    AsyncServiceDiscovery


def test_format_hostname_for_url():
    test_addr_ipv4 = '169.254.169.254'
    test_addr_ipv6 = '200a:aa8::8a2e:370:1337'
    test_hostname = "os-122247000785.local"
    assert format_hostname_for_url(test_addr_ipv4) == test_addr_ipv4
    assert format_hostname_for_url(test_addr_ipv6) == f'[{test_addr_ipv6}]'
    assert format_hostname_for_url(test_hostname) == test_hostname


def test_parse_scope_id():
    # It returns a tuple consisting of the ip address string and an optional integer
    # representing the scope id (if present)
    test_addr_ipv4 = '169.254.169.254'
    test_addr_ipv6 = '200a:aa8::8a2e:370:1337'
    assert parse_scope_id(test_addr_ipv6) == (test_addr_ipv6, None)
    assert parse_scope_id(f"{test_addr_ipv6}%5") == (test_addr_ipv6, 5)
    assert parse_scope_id(f"{test_addr_ipv6}%0") == (test_addr_ipv6, 0)
    assert parse_scope_id(f"{test_addr_ipv6}%0") == (test_addr_ipv6, 0)
    assert parse_scope_id(test_addr_ipv4) == (test_addr_ipv4, None)
    # it raises a ValueError if the scope id is not an integer
    with pytest.raises(ValueError):
        parse_scope_id(f"{test_addr_ipv6}%invalid")


def test_is_link_local_ipv6_address_and_missing_scope_id():
    with pytest.raises(ValueError):
        is_link_local_ipv6_address_and_missing_scope_id("notanaddress")
    assert not is_link_local_ipv6_address_and_missing_scope_id("10.34.80.17")
    assert not is_link_local_ipv6_address_and_missing_scope_id("200a:aa8::8a2e:370:1337")
    assert is_link_local_ipv6_address_and_missing_scope_id("fe80:aa8::8a2e:370:1337")
    assert not is_link_local_ipv6_address_and_missing_scope_id("fe80:aa8::8a2e:370:1337%2")


def test_text_output():
    sensor_json = {
        "active_config": {
            "udp_dest": "10.34.80.17",
            "udp_port_imu": 9503,
            "udp_port_lidar": 9003,
        },
        "addresses": [
            "10.34.26.98"
        ],
        "hostname": "os-992343000025.local.",
        "network": {
            "hostname": "os-992343000025",
            "ipv4": {
                "addr": "10.34.26.98/24",
                "link_local": "169.254.67.1/16",
                "override": None
            },
            "ipv6": {
                "link_local": "fe80::be0f:a7ff:fe00:a992/64"
            },
        },
        "sensor_info": {
            "image_rev": "ousteros-image-prod-aries-v2.5.2+20230714195410",
            "prod_line": "OS-2-128",
            "prod_sn": "992343000025",
        }
    }
    text, color = get_output_for_sensor(sensor_json)
    assert color == 'white'
    assert "OS-2-128 - 992343000025" in text, text
    assert "* UDP destination address: 10.34.80.17" in text, text
    assert "* IPv4 DHCP 10.34.26.98/24" in text


# TODO: remove
def get_text_for_oserror2(error_prefix: str, address: str, e: Exception) -> str:
    if "invalid argument" in str(e).lower() and is_link_local_ipv6_address_and_missing_scope_id(address):
        zeroconf_version = packaging.version.parse(importlib.metadata.version('zeroconf'))
        if version_info < (3, 9):
            return f"{error_prefix} - this version of Python does not support scoped \
link-local IPv6 addresses, which are necessary to retrieve the sensor configuration."
        elif zeroconf_version < packaging.version.parse('0.131.0'):
            return f"{error_prefix} - the installed version of zeroconf ({zeroconf_version}) \
may not be able to provide scoped link-local IPv6 addresses, \
which are necessary to retrieve the sensor configuration.\n" \
                + "Please refer to this GitHub pull request for specifics: \
https://github.com/python-zeroconf/python-zeroconf/pull/1322"
        else:
            return f"{error_prefix} - {e}"
    else:
        return f"{error_prefix} - {e}"


def test_get_text_for_oserror():
    from requests.exceptions import ConnectionError
    e = ConnectionError("Invalid Argument")
    address = "fe80::be0f:a7ff:fe00:a992"
    assert "invalid argument" in str(e).lower()
    assert is_link_local_ipv6_address_and_missing_scope_id(address)
    txt = get_text_for_oserror("prefix", address, e)
    txt2 = get_text_for_oserror2("prefix", address, e)
    assert txt == txt2
    from sys import version_info  # TODO: monkeypatch this or make it a fn parameter
    if version_info < (3, 9):
        assert "this version of Python does not support scoped link-local IPv6 addresses" in txt
    else:
        assert txt == "prefix - Invalid Argument"


async def create_future_task_for_info(asd):
    """a coroutine that creates an AsyncServiceInfo
    and submits it to the provided AsyncServiceDiscovery."""
    while not asd.aiozc:
        await asyncio.sleep(0.5)
    service_type = '_http._tcp.local.'
    name = 'bogus_name.local.'
    info = AsyncServiceInfo(service_type, name)
    info.server = 'server.local.'
    await asyncio.sleep(1)
    await asd.create_future_task_for_info(info)


@pytest.mark.asyncio
async def test_fleetsw_5814():
    """it doesn't raise a RuntimeError due to a future being submitted after executor shutdown"""
    timeout = 0.1
    continuous = False
    show_user_data = False
    asd = AsyncServiceDiscovery(InterfaceChoice.All, IPVersion.All, 'json', timeout, continuous, show_user_data, [])
    asd.async_request_timeout_ms = 0
    await asyncio.gather(asd.async_run(), create_future_task_for_info(asd))
