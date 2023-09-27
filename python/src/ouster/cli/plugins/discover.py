#  type: ignore
"""
ouster-cli discover plugin
Tom Slankard <tom.slankard@ouster.io>
Chris Bayruns <chris.bayruns@ouster.io>
This is adapted from zeroconf's async_browser.py example.
"""

import socket
from typing import Optional
import requests
import time
from socket import AddressFamily
import asyncio
import click
from ouster.cli.core import cli
from ouster.client import get_config
from zeroconf import IPVersion, ServiceStateChange, Zeroconf
from concurrent.futures import ThreadPoolExecutor, as_completed
from zeroconf.asyncio import (
    AsyncServiceBrowser,
    AsyncServiceInfo,
    AsyncZeroconf,
)
from psutil import net_if_addrs
import sys

host_interfaces = net_if_addrs()
host_addresses = [
                address.address for iface in host_interfaces.values()
                for address in iface
                if address.family == AddressFamily.AF_INET or address.family == AddressFamily.AF_INET6
            ]

text_columns = ["HOSTNAME", "ADDRESS", "MODEL", "UDP DESTINATION", "DEST. LIDAR PORT", "DEST. IMU PORT"]
text_column_widths = [30, 20, 16, 45, 20, 20]
mdns_services = ["_roger._tcp.local.", "_ouster-lidar._tcp.local."]
rethrow_exceptions = False


def _get_config(server):
    """ This function is for testing so we can monkeypatch it out """
    return get_config(server)


class AsyncServiceDiscovery:
    def __init__(self, ip_version: IPVersion, timeout: int, continuous: bool, pool_size: int = 10) -> None:
        self.ip_version = ip_version
        self.timeout = timeout
        self.continuous = continuous
        self.aiobrowser: Optional[AsyncServiceBrowser] = None
        self.aiozc: Optional[AsyncZeroconf] = None
        self.start_time = time.time()
        self._executor = ThreadPoolExecutor(max_workers=pool_size)
        self._futures = []
        self._futures_shadow = []
        self._processed_hostnames = []
        self._lock = asyncio.Lock()

    async def async_run(self) -> None:
        self.aiozc = AsyncZeroconf(ip_version=self.ip_version)

        self.aiobrowser = AsyncServiceBrowser(
            self.aiozc.zeroconf, mdns_services, handlers=[self.async_on_service_state_change]
        )
        while True:
            await asyncio.sleep(0.5)
            async with self._lock:
                self._futures = self._futures_shadow
                self._futures_shadow = []

            for future in as_completed(self._futures):
                strs, color, error = future.result()
                if error is not None:
                    click.echo(error)
                click.echo(click.style(''.join(strs), fg=color))
            self._futures = []

            if not self.continuous and self.timeout:
                if time.time() > self.timeout + self.start_time:
                    if sys.version_info < (3, 9):
                        self._executor.shutdown()
                    else:
                        self._executor.shutdown(cancel_futures=True)
                    return

    async def async_close(self) -> None:
        assert self.aiozc is not None
        assert self.aiobrowser is not None
        await self.aiobrowser.async_cancel()
        await self.aiozc.async_close()

    def async_on_service_state_change(self, zeroconf: Zeroconf, service_type: str,
                                      name: str, state_change: ServiceStateChange) -> None:
        if state_change is not ServiceStateChange.Added:
            return
        # TODO: handle other state changes
        asyncio.ensure_future(self.async_display_service_info(zeroconf, service_type, name))

    async def async_display_service_info(self, zeroconf: Zeroconf,
                                         service_type: str, name: str) -> None:
        info = AsyncServiceInfo(service_type, name)
        await info.async_request(zeroconf, 1000)
        if info and info.server:
            async with self._lock:
                if info.server not in self._processed_hostnames:
                    self._processed_hostnames.append(info.server)
                    f = self._executor.submit(service_info_as_text_str, info)
                    self._futures_shadow.append(f)


def address_bytes_to_ip_str(b: bytes) -> str:
    return socket.inet_ntop(
        socket.AF_INET6 if len(b) == 16 else socket.AF_INET, b
    )


def get_address(info) -> str:
    addresses = []
    for address in info.dns_addresses():
        address = address.address
        try:
            addr_type = socket.AF_INET6 if len(address) == 16 else socket.AF_INET

            addr_string = socket.inet_ntop(addr_type, address)
            addresses.append((addr_string, addr_type))
        except IndexError:
            pass

    addresses.append((info.server, socket.AF_INET))

    for addr_string, addr_type in addresses:
        s = socket.socket(addr_type, socket.SOCK_STREAM)
        result = -1
        try:
            if addr_type == socket.AF_INET6:
                result = s.connect_ex((addr_string, 80, 0, 0))
                addr_string = f"[{addr_string}]"
            else:
                result = s.connect_ex((addr_string, 80))
        except socket.gaierror:
            pass
        finally:
            s.close()

        if not result:
            return addr_string

    return None


def service_info_as_text_str(info) -> str:
    addresses = info.dns_addresses()
    ip_addr_string = '-'
    prod_line = '-'
    udp_dest = '-'
    udp_port_lidar = '-'
    udp_port_imu = '-'
    error = None
    color = 'white'

    try:
        ip_addr_string = '-'
        try:
            first_address = addresses[0]
            ip_addr_string = address_bytes_to_ip_str(first_address.address)
        except IndexError:
            pass
        server_addr = get_address(info)
        if server_addr is None:
            color = 'bright_yellow'
            raise Exception(f"Can't Connect To Sensor: Hostname: {info.server} Addresses: {addresses}")
        url = f"http://{server_addr}/api/v1/sensor/metadata/sensor_info"
        response = requests.get(url)
        response_json = response.json()
        prod_line = response_json.get('prod_line', prod_line)
        config = _get_config(server_addr)
        if config:
            if config.udp_dest:
                udp_dest = config.udp_dest
            udp_port_lidar = str(config.udp_port_lidar)
            udp_port_imu = str(config.udp_port_imu)
    except Exception as e:
        if rethrow_exceptions:
            raise
        else:
            error = click.style(e, fg='yellow')

    if udp_dest in host_addresses:
        color = 'green'
    strs = [info.server, ip_addr_string, prod_line, udp_dest, udp_port_lidar, udp_port_imu]
    for i in range(len(strs)):
        strs[i] = strs[i].ljust(text_column_widths[i])
    return ''.join(strs), color, error


@cli.command()
@click.option('-t', '--timeout', help='Run for the specified number of seconds', default=5, type=int)
@click.option('-c', '--continuous', help='Run continuously', is_flag=True, default=False)
@click.pass_context
def discover(ctx, timeout, continuous):
    """Perform a one-time or continuous network search for Ouster sensors.
    """
    global rethrow_exceptions
    rethrow_exceptions = ctx.obj.get('TRACEBACK', False)
    strs = text_columns
    # TODO: extract a text row fmt method
    for i in range(len(strs)):
        strs[i] = strs[i].ljust(text_column_widths[i])
    click.echo(''.join(strs))
    loop = asyncio.get_event_loop()
    runner = AsyncServiceDiscovery(IPVersion.V4Only, timeout, continuous)
    try:
        loop.run_until_complete(runner.async_run())
    except KeyboardInterrupt:
        pass
    loop.run_until_complete(runner.async_close())
