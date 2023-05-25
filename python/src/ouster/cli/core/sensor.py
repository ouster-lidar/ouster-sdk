#  type: ignore
import json
from typing import Optional, List

import numpy as np

import click
from click import ClickException

import ouster.client as client
from .util import click_ro_file


@click.group(name="sensor")
def sensor_group() -> None:
    """Commands for working with sensors."""
    pass


@sensor_group.command()
@click.argument('hostname', required=True)
@click.option('--legacy/--non-legacy',
              default=False,
              help="Use legacy metadata format or not")
def info(hostname: str, legacy: bool) -> None:
    """Dump sensor metadata to stdout."""
    click.echo(client.Sensor(hostname, 7502, 7503,
                             _legacy_format=legacy)._fetched_meta)


@sensor_group.command()
@click.argument('hostname', type=str, required=True)
@click.argument('keyval', metavar='[KEY VAL]...', type=str, nargs=-1)
@click.option('-d', 'dump', is_flag=True, help='Dump current configuration')
@click.option('-c', 'file', type=click.File(), help='Read config from file')
@click.option('-u', 'auto', is_flag=True, help='Set automatic udp dest')
@click.option('-p', 'persist', is_flag=True, help='Persist configuration')
@click.option('-s/-n', 'standby', default=None, help='Set STANDBY or NORMAL')
def config(hostname, keyval, dump, file, auto, persist, standby) -> None:
    """Manipulate the sensor configuration.

    Update the sensor configuration or dump it to stdout. The first positional
    argument is the sensor hostname; remaining arguments are interpreted as
    config parameter key/value pairs, for example:

    \b
        $ ouster-cli sensor config os-99xxxxxxxxxx \\
            lidar_mode 2048x10 azimuth_window "[20000, 60000]"

    If no options or config param values are specified, use the default UDP
    ports, autoatic UDP destination, full azimuth azimuth window, and set the
    operating mode to NORMAL.
    """

    def parse(s):
        """Helper to read cli arg as json value with fallback to string."""
        try:
            return json.loads(s)
        except json.decoder.JSONDecodeError:
            return json.loads(f'"{s}"')

    if dump:
        if file or keyval or auto or persist or standby is not None:
            raise ClickException("Cannot use other options with `-d` command")
        cfg = client.get_config(hostname)
        click.echo(cfg)
        return
    elif file:
        if keyval:
            raise ClickException("Cannot specify extra config keys with `-c`")
        cfg = client.SensorConfig(file.read())
        click.echo("Setting config from file:")
    elif not keyval and not auto and standby is None:
        auto = True
        cfg = client.SensorConfig()
        cfg.udp_port_lidar = 7502
        cfg.udp_port_imu = 7503
        cfg.azimuth_window = (0, 360000)
        cfg.operating_mode = client.OperatingMode.OPERATING_NORMAL
        click.echo("No config specified; using defaults and auto UDP dest:")
    else:
        if len(keyval) % 2 != 0:
            raise ClickException(f"Unmatched key/value arg: {keyval[-1]}")
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
        raise ClickException(str(e))


@sensor_group.command()
@click.argument('hostname', required=True)
@click.option('-l', '--lidar-port', default=7502, help="Lidar port")
@click.option('-f', '--meta', required=False, type=click_ro_file)
@click.option('-F', '--filter', is_flag=True, help="Drop scans missing data")
@click.option('-b', '--buf-size', default=256, help="Max packets to buffer")
@click.option('-v', '--verbose', is_flag=True, help="Print some debug output")
@click.option('-t', '--timeout', default=1.0, help="Seconds to wait for data")
@click.option('--extrinsics',
              type=float,
              required=False,
              nargs=16,
              help='Lidar sensor extrinsics to use in viz')
@click.option('--soft-id-check',
              is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't "
              "match with metadata")
def viz(hostname: str, lidar_port: int, meta: Optional[str], filter: bool,
        buf_size: int, verbose: bool, timeout: float,
        extrinsics: Optional[List[float]], soft_id_check: bool) -> None:
    """Listen for data on the specified ports and run the visualizer.

    Note: this will currently not configure the sensor or query the sensor for
    the port to listen on. You will need to set the sensor port and destination
    settings separately.
    """
    try:
        from ouster.sdk.viz import SimpleViz
        from ouster.sdkx.viz import ExtendedScanViz
        from ouster.sdkx.parsing import default_scan_fields
    except ImportError as e:
        raise click.ClickException(str(e))

    click.echo("Initializing...")

    # make 0 timeout in the cli mean no timeout
    timeout_ = timeout if timeout > 0 else None

    # override metadata, if provided
    meta_override: Optional[client.SensorInfo] = None
    if meta is not None:
        with open(meta) as json:
            meta_override = client.SensorInfo(json.read())

    source = client.Sensor(hostname,
                           lidar_port,
                           7503,
                           metadata=meta_override,
                           buf_size=buf_size,
                           timeout=timeout_,
                           _soft_id_check=soft_id_check)

    # enable parsing flags field
    fields = default_scan_fields(source.metadata.format.udp_profile_lidar,
                                 flags=True)

    try:
        scans = client.Scans(source,
                            timeout=timeout_,
                            complete=filter,
                            fields=fields,
                            _max_latency=2)

        if extrinsics:
            scans.metadata.extrinsic = np.array(extrinsics).reshape((4, 4))
            print(f"Using sensor extrinsics:\n{scans.metadata.extrinsic}")

        ls_viz = ExtendedScanViz(scans.metadata)
        SimpleViz(ls_viz).run(scans)
    finally:
        if source.id_error_count:
            print(f"WARNING: {source.id_error_count} lidar_packets with "
                  "mismatched init_id/sn were detected.")
            if not soft_id_check:
                print("NOTE: To disable strict init_id/sn checking use "
                      "--soft-id-check option (may lead to parsing "
                      "errors)")

    click.echo("Done")
