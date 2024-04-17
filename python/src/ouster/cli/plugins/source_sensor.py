#  type: ignore
import json

import click

import ouster.sdk.client as client

from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand)


@click.group(name="sensor", hidden=True)
def sensor_group() -> None:
    """Commands for working with sensors."""
    pass


@click.command
@click.option('--legacy/--non-legacy',
              default=False,
              help="Use legacy metadata format or not")
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def sensor_metadata(ctx: SourceCommandContext, click_ctx: click.core.Context,
                legacy: bool) -> None:
    """Display sensor metadata about the SOURCE."""  # Implements ouster-cli source <hostname> metadata
    try:
        click.echo(client.Sensor(ctx.source_uri, 7502, 7503,
                                 _legacy_format=legacy)._fetched_meta)
    except RuntimeError as e:
        raise click.ClickException(str(e))


@click.command()
@click.argument('keyval', metavar='[KEY VAL]...', type=str, nargs=-1)
@click.option('-d', 'dump', is_flag=True, help='Dump current configuration')
@click.option('-c', 'file', type=click.File(), help='Read config from file')
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
    hostname = ctx.source_uri

    def parse(s):
        """Helper to read cli arg as json value with fallback to string."""
        try:
            return json.loads(s)
        except json.decoder.JSONDecodeError:
            return json.loads(f'"{s}"')

    if dump:
        if file or keyval or auto or persist or standby is not None:
            raise click.ClickException("Cannot use other options with `-d` command")
        cfg = client.get_config(hostname)
        click.echo(cfg)
        return
    elif file:
        if keyval:
            raise click.ClickException("Cannot specify extra config keys with `-c`")
        cfg = client.SensorConfig(file.read())
        click.echo("Setting config from file:")
    elif not keyval and not auto and standby is None:
        auto = True
        cfg = client.SensorConfig()
        cfg.udp_port_lidar = 7502
        cfg.udp_port_imu = 7503
        cfg.azimuth_window = (0, 360000)
        cfg.signal_multiplier = 1
        cfg.operating_mode = client.OperatingMode.OPERATING_NORMAL
        click.echo("No config specified; using defaults and auto UDP dest:")
    else:
        if len(keyval) % 2 != 0:
            raise click.ClickException(f"Unmatched key/value arg: {keyval[-1]}")
        d = dict(zip(keyval[::2], map(parse, keyval[1::2])))
        cfg = client.SensorConfig(json.dumps(d))
        click.echo("Updating configuration:")

    if standby is not None:
        cfg.operating_mode = (client.OperatingMode.OPERATING_STANDBY if standby
                              else client.OperatingMode.OPERATING_NORMAL)

    click.echo(f"{cfg}")
    try:
        client.set_config(hostname, cfg, udp_dest_auto=auto, persist=persist)
    except RuntimeError as e:
        raise click.ClickException(str(e))
