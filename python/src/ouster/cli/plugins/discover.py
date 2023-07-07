#  type: ignore
"""
ouster-cli discover plugin
Tom Slankard <tom.slankard@ouster.io>

This is a POC adapted from zeroconf's async_browser.py example.
"""

from typing import Optional
import requests
import time
from socket import AddressFamily
import asyncio
import click
from ouster.cli.core import cli
from ouster.client import get_config
from zeroconf import IPVersion, ServiceStateChange, Zeroconf
from zeroconf.asyncio import (
    AsyncServiceBrowser,
    AsyncServiceInfo,
    AsyncZeroconf,
)
from psutil import net_if_addrs

host_interfaces = net_if_addrs()
host_addresses = [
                address.address for iface in host_interfaces.values()
                for address in iface
                if address.family == AddressFamily.AF_INET or address.family == AddressFamily.AF_INET6
            ]

text_columns = ["HOSTNAME", "ADDRESS", "MODEL", "UDP DESTINATION", "DEST. LIDAR PORT", "DEST. IMU PORT"]
text_column_widths = [28, 20, 16, 20, 20, 20]
mdns_services = ["_roger._tcp.local."]
rethrow_exceptions = False


class AsyncServiceDiscovery:
    def __init__(self, ip_version: IPVersion, timeout: int, continuous: bool) -> None:
        self.ip_version = ip_version
        self.timeout = timeout
        self.continuous = continuous
        self.aiobrowser: Optional[AsyncServiceBrowser] = None
        self.aiozc: Optional[AsyncZeroconf] = None
        self.start_time = time.time()

    async def async_run(self) -> None:
        self.aiozc = AsyncZeroconf(ip_version=self.ip_version)

        self.aiobrowser = AsyncServiceBrowser(
            self.aiozc.zeroconf, mdns_services, handlers=[async_on_service_state_change]
        )
        while True:
            await asyncio.sleep(0.5)
            if not self.continuous and self.timeout:
                if time.time() > self.timeout + self.start_time:
                    return

    async def async_close(self) -> None:
        assert self.aiozc is not None
        assert self.aiobrowser is not None
        await self.aiobrowser.async_cancel()
        await self.aiozc.async_close()


def async_on_service_state_change(
    zeroconf: Zeroconf, service_type: str, name: str, state_change: ServiceStateChange
) -> None:
    if state_change is not ServiceStateChange.Added:
        return
    # TODO: handle other state changes
    asyncio.ensure_future(async_display_service_info(zeroconf, service_type, name))


def service_info_as_text_str(info) -> str:
    addresses = ["%s" % (addr,) for addr in info.parsed_scoped_addresses()]
    first_address = '-'
    prod_line = '-'
    udp_dest = '-'
    udp_port_lidar = '-'
    udp_port_imu = '-'
    if addresses:
        first_address = addresses[0]
        try:
            url = f"http://{first_address}/api/v1/sensor/metadata/sensor_info"
            response = requests.get(url)
            response_json = response.json()
            prod_line = response_json['prod_line']
            config = get_config(first_address)
            if config:
                if config.udp_dest:
                    udp_dest = config.udp_dest
                udp_port_lidar = str(config.udp_port_lidar)
                udp_port_imu = str(config.udp_port_imu)
        except Exception as e:
            if rethrow_exceptions:
                raise
            else:
                click.echo(click.style(e, fg='yellow'))
    color = 'white'
    if udp_dest in host_addresses:
        color = 'green'
    strs = [info.server, first_address, prod_line, udp_dest, udp_port_lidar, udp_port_imu]
    for i in range(len(strs)):
        strs[i] = strs[i].ljust(text_column_widths[i])
    return ''.join(strs), color


async def async_display_service_info(zeroconf: Zeroconf, service_type: str, name: str) -> None:
    info = AsyncServiceInfo(service_type, name)
    await info.async_request(zeroconf, 1000)
    if info and info.server:
        strs, color = service_info_as_text_str(info)
        click.echo(click.style(''.join(strs), fg=color))


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
        loop.run_until_complete(runner.async_close())
