import json
import click
import requests
import time

import ouster.sdk.core as core

from typing import Optional, List
from ouster.sdk._bindings.client import Sensor as _Sensor
from ouster.sdk import sensor
from ouster.sdk.core import OusterIoType
from ouster.cli.plugins.source import source  # type: ignore
from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand)


@click.group(name="sensor", hidden=True)
def sensor_group() -> None:
    """Commands for working with sensors."""
    pass


@click.command
@click.pass_context
@click.option('--set-static-ip', type=str, default=None,
              help='Set sensor static IP address. A subnet mask should be appended at the end using the /24 form.')
@click.option('--set-gateway', type=str, default=None,
              help='Set sensor gateway IP address.')
@click.option('--clear-static-ip', is_flag=True, help='Clear current static IP address.')
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_network(ctx: SourceCommandContext, click_ctx: click.core.Context,
                   set_static_ip: Optional[str], clear_static_ip: bool,
                   set_gateway: bool) -> None:
    """Manages and queries network settings on the sensor. Run with no arguments to print network details."""
    if (set_static_ip or set_gateway) and clear_static_ip:
        raise click.ClickException("Cannot both set and clear sensor ips.")
    try:
        http = sensor.SensorHttp.create(ctx.source_uri or "")
        if clear_static_ip:
            http.delete_static_ip()
        elif set_static_ip is not None:
            http.set_static_ip(set_static_ip, set_gateway or "")
        elif set_gateway is not None:
            # discover static ip and then do a set
            data = json.loads(http.network())
            override = None
            if "override" in data["ipv4"]:
                override = data["ipv4"]["override"]
            elif "override" in data["ipv6"]:
                override = data["ipv6"]["override"]

            if override is None:
                raise click.ClickException("Sensor must have a static IP in order to set the gateway.")

            # now parse the override, it could just be a string or an object
            if isinstance(override, dict):
                # it is an object
                static_ip = override["addr"]
            else:
                # it is a string
                static_ip = override

            http.set_static_ip(static_ip, set_gateway)
        else:
            click.echo(json.dumps(json.loads(http.network()), indent=4))
    except RuntimeError as e:
        raise click.ClickException(str(e))


@click.command
@click.pass_context
@click.option('--timeout', default=100, show_default=True, type=int,
              help="Diagnostics timeout in seconds.")
@click.argument('filename', metavar='[FILENAME]', type=str, nargs=-1)
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_diagnostics(ctx: SourceCommandContext, click_ctx: click.core.Context,
                       timeout: int, filename: str) -> None:
    """Download diagnostics dump from a sensor."""
    if len(filename) > 1:
        raise click.ClickException("Can only provide at most one filename")
    try:
        uri = ctx.source_uri or ""
        http = sensor.SensorHttp.create(uri)
        click.echo("Starting download. This may take a while....")
        data = http.diagnostics_dump(timeout)
        filename = filename[0] if len(filename) > 0 else uri + "_diagnostics.bin"
        with open(filename, 'wb') as dump_file:
            dump_file.write(data)
        click.echo(f"Saved diagnostic dump to: {filename}")
    except RuntimeError as e:
        raise click.ClickException(str(e))


@click.command
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_metadata(ctx: SourceCommandContext, click_ctx: click.core.Context) -> None:
    """Display sensor metadata about the SOURCE."""  # Implements ouster-cli source <hostname> metadata
    try:
        click.echo(_Sensor(ctx.source_uri or "").fetch_metadata().to_json_string())
    except RuntimeError as e:
        raise click.ClickException(str(e))


def update_fw(sensor, fw, expected_version = None, timeout = 120):
    print("Starting update...")

    # handle IPv6 address escaping
    if sensor.count(':') >= 2:
        sensor = "[" + sensor + "]"

    headers = {}
    headers["Content-Type"] = "application/octet-stream"
    print("Uploading fw...")

    res = requests.post("http://" + sensor + "/api/v1/system/firmware", data=open(fw, 'rb'), headers = headers)
    if res.status_code != 200 and res.status_code != 204:
        print("Upload failed!", res.text)
        return False

    print("Finished uploading fw...")

    # Poll system/firmware until commit is complete
    print("Waiting for apply/commit...")
    start = time.time()
    while True:
        try:
            # check if we should timeout
            if time.time() - start > timeout:
                print("Timeout occurred! Firmware upload may have failed.")
                return False
            res = requests.get("http://" + sensor + "/api/v1/system/firmware")
            data = json.loads(res.text)
            if data["commit_pending"]:
                pass
            else:
                print("Commit complete.")
                new_version = data["fw"]
                print("New FW version: " + new_version)
                break
        except requests.exceptions.ConnectionError:
            pass
        time.sleep(1.0)

    print("Firmware update finished.")
    return True


@click.command
@click.option('--update', metavar='FILENAME', default=None, type=str, help="Update the firmware to the provided file.")
@click.option('-t', '--timeout', default=120, type=int, help='Timeout for the update in seconds.')
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_firmware(ctx: SourceCommandContext, click_ctx: click.core.Context,
                    update: Optional[str], timeout: int) -> None:
    """Retrieve or update the FW version on a sensor."""
    from ouster.sdk.sensor import SensorHttp
    if update is not None:
        try:
            if not update_fw(ctx.source_uri or "", update, timeout = timeout):
                raise RuntimeError("Firmware update failed.")
        except RuntimeError as e:
            raise click.ClickException(str(e))
    else:
        http = SensorHttp.create(ctx.source_uri or "")
        version = http.firmware_version()
        print(f"Version: {version.major}.{version.minor}.{version.patch}")
        print(f"Stage: {version.stage}")
        print(f"Machine: {version.machine}")
        print(f"Build: '{version.build}'")
        print(f"Prerelease: '{version.prerelease}'")


@click.command
@click.option('-s', default=None, type=str, help='Set userdata to the provided string')
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_userdata(ctx: SourceCommandContext, click_ctx: click.core.Context, s: str) -> None:
    """Retrieve or set userdata from the current sensor (if supported by the firmware)"""
    from ouster.sdk.sensor import SensorHttp
    try:
        http = SensorHttp.create(ctx.source_uri or "")
        if s is None:
            click.echo(http.get_user_data())
        else:
            http.set_user_data(s)
    except RuntimeError as e:
        raise click.ClickException(str(e))


@click.command()
@click.argument('keyval', metavar='[KEY VAL]...', type=str, nargs=-1)
@click.option('-d', 'dump', is_flag=True, help='Dump current configuration')
@click.option('-c', 'file', type=click.Path(), help='Read config from file')
@click.option('-u', 'auto', is_flag=True, help='Set automatic udp dest')
@click.option('-p', 'persist', is_flag=True, help='Persist configuration')
@click.option('-s/-n', 'standby', default=None, help='Set STANDBY or NORMAL')
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_config(ctx: SourceCommandContext, click_ctx: click.core.Context,
keyval, dump, file, auto, persist, standby) -> None:
    """Manipulate the sensor configuration.

    Update the sensor configuration or dump it to stdout. The first positional
    argument is the sensor hostname; remaining arguments are interpreted as
    config parameter key/value pairs, for example:

    \b
        $ ouster-cli sensor config os-99xxxxxxxxxx \\
            lidar_mode 2048x10 azimuth_window "[20000, 60000]"

    If no options or config param values are specified, use the default UDP
    ports, automatic UDP destination, full azimuth azimuth window, and set the
    operating mode to NORMAL.
    """
    hostname = ctx.source_uri or ""

    def parse(s):
        """Helper to read cli arg as json value with fallback to string."""
        try:
            return json.loads(s)
        except json.decoder.JSONDecodeError:
            return json.loads(f'"{s}"')

    if dump:
        if file or keyval or auto or persist or standby is not None:
            raise click.ClickException("Cannot use other options with `-d` command")
        cfg = sensor.get_config(hostname)
        click.echo(cfg)
        return
    elif file:
        if keyval:
            raise click.ClickException("Cannot specify extra config keys with `-c`")
        with open(file, 'r') as f:
            click.echo(f"Setting config from file: {file}")
            cfg = core.SensorConfig(f.read())
    elif not keyval and not auto and standby is None:
        auto = True
        cfg = core.SensorConfig()
        cfg.udp_port_lidar = 7502
        cfg.udp_port_imu = 7503
        cfg.azimuth_window = (0, 360000)
        cfg.signal_multiplier = 1
        cfg.operating_mode = core.OperatingMode.NORMAL
        click.echo("No config specified; using defaults and auto UDP dest:")
    else:
        if len(keyval) % 2 != 0:
            raise click.ClickException(f"Unmatched key/value arg: {keyval[-1]}")
        d = dict(zip(keyval[::2], map(parse, keyval[1::2])))
        cfg = core.SensorConfig(json.dumps(d))
        click.echo("Updating configuration:")

    if standby is not None:
        cfg.operating_mode = (core.OperatingMode.STANDBY if standby
                              else core.OperatingMode.NORMAL)

    click.echo(f"{cfg}")
    try:
        sensor.set_config(hostname, cfg, udp_dest_auto=auto, persist=persist)
    except RuntimeError as e:
        raise click.ClickException(str(e))


@click.command()
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_restart(ctx: SourceCommandContext, click_ctx: click.core.Context) -> None:
    """Soft-restart the sensor. Useful to re-apply the persisted config."""
    from ouster.sdk.sensor import SensorHttp
    try:
        http = SensorHttp.create(ctx.source_uri or "")
        http.restart()
    except RuntimeError as e:
        raise click.ClickException(str(e))


@click.command()
@click.pass_context
@click.option('-s', '--set', is_flag=True, help="Set the following zones as active.")
@click.argument('zones', type=int, nargs=-1)
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_live_zones(ctx: SourceCommandContext, click_ctx: click.core.Context, zones: List[int], set: bool) -> None:
    """Query or set the list of zones live on the sensor."""
    from ouster.sdk.sensor import SensorHttp
    try:
        http = SensorHttp.create(ctx.source_uri or "")
        if set:
            http.set_zone_monitor_live_ids(zones)
        else:
            if len(zones) > 0:
                raise click.ClickException("Cannot provide zones if querying. Provide -s option to set.")
            print(http.get_zone_monitor_live_ids())
    except RuntimeError as e:
        raise click.ClickException(str(e))


source.commands[OusterIoType.SENSOR]['config'] = sensor_config
source.commands[OusterIoType.SENSOR]['userdata'] = sensor_userdata
source.commands[OusterIoType.SENSOR]['metadata'] = sensor_metadata
source.commands[OusterIoType.SENSOR]['network'] = sensor_network
source.commands[OusterIoType.SENSOR]['diagnostics'] = sensor_diagnostics
source.commands[OusterIoType.SENSOR]['restart'] = sensor_restart
source.commands[OusterIoType.SENSOR]['firmware'] = sensor_firmware
source.commands[OusterIoType.SENSOR]['live_zones'] = sensor_live_zones
