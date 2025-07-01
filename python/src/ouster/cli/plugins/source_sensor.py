import json

import click

import ouster.sdk.core as core

from typing import Optional
from ouster.sdk._bindings.client import Sensor as _Sensor
from ouster.sdk import sensor

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
@click.option('--clear-static-ip', is_flag=True, help='Clear current static IP address.')
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_network(ctx: SourceCommandContext, click_ctx: click.core.Context,
                   set_static_ip: Optional[str], clear_static_ip: bool) -> None:
    """Manages and queries network settings on the sensor. Run with no arguments to print network details."""
    if set_static_ip and clear_static_ip:
        raise click.ClickException("Cannot both set and clear sensor static ip.")
    try:
        http = sensor.SensorHttp.create(ctx.source_uri or "")
        if clear_static_ip:
            http.delete_static_ip()
        elif set_static_ip is not None:
            http.set_static_ip(set_static_ip)
        else:
            click.echo(json.dumps(json.loads(http.network()), indent=4))
    except RuntimeError as e:
        raise click.ClickException(str(e))


@click.command
@click.pass_context
@click.argument('filename', metavar='[FILENAME]', type=str, nargs=-1)
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_diagnostics(ctx: SourceCommandContext, click_ctx: click.core.Context, filename: str) -> None:
    """Download diagnostics dump from a sensor."""
    if len(filename) > 1:
        raise click.ClickException("Can only provide at most one filename")
    try:
        uri = ctx.source_uri or ""
        http = sensor.SensorHttp.create(uri)
        click.echo("Starting download. This may take a while....")
        data = http.diagnostics_dump(100)
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
        cfg.operating_mode = core.OperatingMode.OPERATING_NORMAL
        click.echo("No config specified; using defaults and auto UDP dest:")
    else:
        if len(keyval) % 2 != 0:
            raise click.ClickException(f"Unmatched key/value arg: {keyval[-1]}")
        d = dict(zip(keyval[::2], map(parse, keyval[1::2])))
        cfg = core.SensorConfig(json.dumps(d))
        click.echo("Updating configuration:")

    if standby is not None:
        cfg.operating_mode = (core.OperatingMode.OPERATING_STANDBY if standby
                              else core.OperatingMode.OPERATING_NORMAL)

    click.echo(f"{cfg}")
    try:
        sensor.set_config(hostname, cfg, udp_dest_auto=auto, persist=persist)
    except RuntimeError as e:
        raise click.ClickException(str(e))
