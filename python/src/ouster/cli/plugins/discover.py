#  type: ignore
"""
ouster-cli discover plugin
Tom Slankard <tom.slankard@ouster.io>
Chris Bayruns <chris.bayruns@ouster.io>
This is adapted from zeroconf's async_browser.py example.
"""

import json
import logging
from typing import Optional, Tuple
import requests
import time
from socket import AddressFamily
import asyncio
import click
from ouster.cli.core import cli
from zeroconf import IPVersion, ServiceStateChange, Zeroconf
from concurrent.futures import ThreadPoolExecutor, as_completed
import zeroconf
from zeroconf.asyncio import (
    AsyncServiceBrowser,
    AsyncServiceInfo,
    AsyncZeroconf,
)
from psutil import net_if_addrs
from sys import version_info
import importlib.metadata
import packaging.version
import ipaddress
import sys

host_interfaces = net_if_addrs()
host_addresses = [
                address.address for iface in host_interfaces.values()
                for address in iface
                if address.family == AddressFamily.AF_INET or address.family == AddressFamily.AF_INET6
            ]

rethrow_exceptions = False


class AsyncServiceDiscovery:
    def __init__(self, interfaces, ip_version: IPVersion, output: str, timeout: int,
        continuous: bool, show_user_data,
        service_names=["_roger._tcp.local.", "_ouster-lidar._tcp.local."],
        pool_size: int = 10, socket_timeout: int = 2) -> None:

        self.interfaces = interfaces
        if type(interfaces) is not zeroconf.InterfaceChoice:
            self.interfaces = list(interfaces)
        self.ip_version = ip_version
        self.output = output
        self.timeout = timeout
        self.continuous = continuous
        self.show_user_data = show_user_data
        self.service_names = service_names
        self.socket_timeout = socket_timeout
        self.aiobrowser: Optional[AsyncServiceBrowser] = None
        self.aiozc: Optional[AsyncZeroconf] = None
        self.start_time = time.time()
        self._executor = ThreadPoolExecutor(max_workers=pool_size)
        self._futures = []
        self._futures_shadow = []
        self._processed_hostnames = []
        self._lock = asyncio.Lock()
        # Note - longer than the default of 3s, but it doesn't affect the overall duration.
        self.async_request_timeout_ms = 10000
        self.shutdown = False

    async def async_run(self) -> None:
        self.aiozc = AsyncZeroconf(self.interfaces, False, self.ip_version)

        self.aiobrowser = AsyncServiceBrowser(
            self.aiozc.zeroconf, self.service_names, handlers=[self.async_on_service_state_change]
        )
        results = []
        while not self.shutdown:
            await asyncio.sleep(0.5)
            async with self._lock:
                self._futures = self._futures_shadow
                self._futures_shadow = []

            for future in as_completed(self._futures):
                result = future.result()
                results.append(result)
                if self.output == 'text':
                    text, color = get_output_for_sensor(result)
                    click.secho(text, color=color)
            self._futures = []

            if not self.continuous and self.timeout:
                if time.time() > self.timeout + self.start_time:
                    if sys.version_info < (3, 9):
                        self._executor.shutdown()
                    else:
                        self._executor.shutdown(cancel_futures=True)
                    self.shutdown = True
        if self.output == 'json':
            print(json.dumps(results, indent=2))

    async def async_close(self) -> None:
        self.shutdown = True
        assert self.aiozc is not None
        assert self.aiobrowser is not None
        await self.aiobrowser.async_cancel()
        await self.aiozc.async_close()

    def async_on_service_state_change(self, zeroconf: Zeroconf, service_type: str,
                                      name: str, state_change: ServiceStateChange) -> None:
        if self.shutdown or state_change is not ServiceStateChange.Added:
            return
        # TODO: handle other state changes
        asyncio.ensure_future(self.async_display_service_info(zeroconf, service_type, name))

    async def create_future_task_for_info(self, info):
        async with self._lock:
            if not self.shutdown and info.server not in self._processed_hostnames:
                self._processed_hostnames.append(info.server)
                f = self._executor.submit(get_all_sensor_info, info, self.socket_timeout, self.show_user_data)
                self._futures_shadow.append(f)

    async def async_display_service_info(self, zeroconf: Zeroconf,
                                         service_type: str, name: str) -> None:
        # try to get service info
        info = AsyncServiceInfo(service_type, name)
        await info.async_request(zeroconf, self.async_request_timeout_ms)
        # submit a task to obtain metadata, etc
        await self.create_future_task_for_info(info)


def parse_scope_id(address: str) -> Tuple[str, Optional[int]]:
    addr_str, _, scope_str = address.partition('%')
    scope_id = int(scope_str) if scope_str else None
    return (addr_str, scope_id)


def is_link_local_ipv6_address_and_missing_scope_id(address: str) -> bool:
    address, scope_id = parse_scope_id(address)
    if scope_id:
        return False
    ip_addr = ipaddress.ip_address(address)
    return type(ip_addr) is ipaddress.IPv6Address and ip_addr.is_link_local


def get_text_for_oserror(error_prefix: str, address: str, e: Exception) -> str:
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


def format_hostname_for_url(hostname_str: str) -> str:
    # if it's an IPv6 address it must be formatted
    try:
        ip_addr = ipaddress.ip_address(hostname_str)
        if type(ip_addr) is ipaddress.IPv6Address:
            return f"[{str(ip_addr)}]"
        return str(ip_addr)
    except ValueError:
        # if it's not an ip address, let's assume it's a hostname
        return hostname_str


def get_sensor_info(hostname_or_address, socket_timeout):
    url = f"http://{format_hostname_for_url(hostname_or_address)}/api/v1/sensor/metadata/sensor_info"
    response = requests.get(url, timeout=socket_timeout)
    return response.json()


def get_sensor_config(hostname_or_address, socket_timeout):
    url = f"http://{format_hostname_for_url(hostname_or_address)}/api/v1/sensor/cmd/get_config_param?args=active"
    response = requests.get(url, timeout=socket_timeout)
    if response.status_code != 200:
        return None
    return response.json()


def get_sensor_network(hostname_or_address, socket_timeout):
    url = f"http://{format_hostname_for_url(hostname_or_address)}/api/v1/system/network"
    response = requests.get(url, timeout=socket_timeout)
    return response.json()


def get_sensor_user_data(hostname_or_address, socket_timeout):
    url = f"http://{format_hostname_for_url(hostname_or_address)}/api/v1/user/data"
    response = requests.get(url, timeout=socket_timeout)
    if response.status_code != 200:
        return None
    return response.json()


def get_output_for_sensor(sensor):
    undefined_value = '-'
    unknown = 'UNKNOWN'
    sensor_hostname = sensor.get('hostname', undefined_value)
    prod_line = unknown
    udp_dest = undefined_value
    udp_port_lidar = undefined_value
    udp_port_imu = undefined_value
    ipv4_address = None
    ipv4_link_local = None
    ipv6_address = None
    ipv6_link_local = None
    # TODO: decide if we need to display the addresses obtained via mDNS.
    # (Probably not.)
    # addresses = sensor.get('addresses')
    sensor_info = sensor.get('sensor_info')
    config = sensor.get('active_config')
    network = sensor.get('network')
    user_data = sensor.get('user_data')
    firmware = sensor_info.get('image_rev', undefined_value) if sensor_info else undefined_value
    sn = sensor_info.get('prod_sn', unknown) if sensor_info else unknown
    warnings = sensor.get('warnings')
    color = 'white'
    if sensor_info:
        prod_line = sensor_info.get('prod_line', unknown)
    if config:
        udp_dest = config.get('udp_dest', undefined_value)
        udp_port_lidar = config.get('udp_port_lidar', undefined_value)
        udp_port_imu = config.get('udp_port_imu', undefined_value)
    if network:

        ipv4 = network.get('ipv4')
        ipv4_address_type = 'static' if ipv4.get('override') else 'DHCP'
        ipv4_address = ipv4.get('override') or ipv4.get('addr')
        ipv4_link_local = ipv4.get('link_local')

        ipv6 = network.get('ipv6')
        ipv6_address_type = 'static' if ipv6.get('override') else 'DHCP'
        ipv6_address = ipv6.get('override') or ipv6.get('addr')
        ipv6_link_local = ipv6.get('link_local')

    if udp_dest in host_addresses:
        color = 'green'
    out = f"{prod_line} - {sn}\n* hostname: {sensor_hostname}\n"
    if warnings:
        out += click.style("* warnings:\n", fg='yellow')
        for warning in warnings:
            out += click.style(f"  * {warning}\n", fg='yellow')
    out += f"* firmware: {firmware}\n* addresses:\n"
    if ipv4_address:
        out += f"  * IPv4 {ipv4_address_type} {ipv4_address}\n"
    if ipv4_link_local:
        out += f"  * IPv4 link-local {ipv4_link_local}\n"
    if ipv6_address:
        out += f"  * IPv6 {ipv6_address_type} {ipv6_address}\n"
    if ipv6_link_local:
        out += f"  * IPv6 link-local {ipv6_link_local}\n"
    out += f"* UDP destination address: {udp_dest}\n"
    out += f"* UDP port lidar, IMU: {udp_port_lidar}, {udp_port_imu}\n"
    if user_data:
        out += f"* user data: {user_data}\n"
    return out, color


def get_all_sensor_info(info, socket_timeout, show_user_data) -> str:
    addresses = info.parsed_scoped_addresses(IPVersion.All)
    sensor_info = None
    config = None
    network = None
    user_data = None
    warnings = []
    for address in addresses:
        try:
            if not sensor_info:
                sensor_info = get_sensor_info(address, socket_timeout)
            if not config:
                config = get_sensor_config(address, socket_timeout)
            if not network:
                network = get_sensor_network(address, socket_timeout)
            if show_user_data and not user_data:
                user_data = get_sensor_user_data(address, socket_timeout)
        except OSError as e:
            warning_prefix = f"Could not connect to {info.server} via {address}"
            warnings.append(
                get_text_for_oserror(
                    warning_prefix,
                    address,
                    e
                )
            )
    return {
        'hostname': info.server,
        'addresses': addresses,
        'active_config': config,
        'sensor_info': sensor_info,
        'network': network,
        'user_data': user_data,
        'warnings': warnings,
    }


@cli.command()
@click.option('-o', '--output', help='Provide the output in the specified format',
    type=click.Choice(['text', 'json'], case_sensitive=False), default='text')
@click.option('-t', '--timeout', help='Run for the specified number of seconds', default=10, type=int)
@click.option('-c', '--continuous', help='Run continuously', is_flag=True, default=False)
@click.option('--http-timeout', help='The timeout for HTTP requests for sensor info', default=2, type=int)
@click.option('--interface', help="Address(es) of interface(s) to listen for mDNS services.", default=[], multiple=True)
@click.option('-u', '--show-user-data', help="Display sensor user data if defined.", is_flag=True, default=False)
@click.pass_context
def discover(ctx, output, timeout, continuous, http_timeout, interface, show_user_data):
    """Perform a one-time or continuous network search for Ouster sensors.
    """
    global rethrow_exceptions
    if output == 'json' and continuous:
        click.secho("Sorry, ouster-cli cannot produce JSON output when running continuously.", fg='yellow', err=True)
        sys.exit(1)
    rethrow_exceptions = ctx.obj.get('TRACEBACK', False)
    loop = asyncio.get_event_loop()
    if not interface:
        interface = zeroconf.InterfaceChoice.All
    logging.getLogger('zeroconf').propagate = False
    runner = AsyncServiceDiscovery(
        interface,
        IPVersion.All,
        output,
        timeout,
        continuous,
        show_user_data,
        socket_timeout=http_timeout
    )
    try:
        loop.run_until_complete(runner.async_run())
    except KeyboardInterrupt:
        pass
    loop.run_until_complete(runner.async_close())
