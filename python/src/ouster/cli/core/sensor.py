#  type: ignore
import json
from typing import Optional, List


import click
from click import ClickException

import ouster.client as client
from .util import click_ro_file


@click.group(name="sensor", hidden=True)
def sensor_group() -> None:
    """Commands for working with sensors."""
    pass


@sensor_group.command()
@click.argument('hostname', required=True)
@click.option('--legacy/--non-legacy',
              default=False,
              help="Use legacy metadata format or not")
def metadata(hostname: str, legacy: bool) -> None:
    """Dump sensor metadata to stdout."""
    try:
        click.echo(client.Sensor(hostname, 7502, 7503,
                                 _legacy_format=legacy)._fetched_meta)
    except RuntimeError as e:
        raise ClickException(str(e))


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
    ports, automatic UDP destination, full azimuth azimuth window, and set the
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
        cfg.signal_multiplier = 1
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
# TODO: is this needed? can we kill it
@click.option('-b', '--buf-size', default=256, hidden=True, help="Max packets to buffer")
@click.option('-e', '--extrinsics', type=float, nargs=16,
              help='Lidar sensor extrinsics to use in viz')
# TODO: is this a viable option? Can we kill it
@click.option('-m', '--meta', type=click_ro_file,
              help="Provide separate metadata to use with sensor", hidden=True)
@click.option('-F', '--filter', is_flag=True, help="Drop scans missing data")
@click.option('-l', '--lidar-port', type=int, default=None, help="Lidar port")
@click.option('-s', '--soft-id-check', is_flag=True, hidden=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't match with metadata")  # noqa
@click.option('-t', '--timeout', default=1.0, help="Seconds to wait for data")
@click.option('-v', '--verbose', is_flag=True, help="Print some debug output")
@click.option('-x', '--do-not-reinitialize', is_flag=True, default=False,
              help="Do not reinitialize (by default it will reinitialize if needed)")
@click.option('-y', '--no-auto-udp-dest', is_flag=True, default=False,
              help="Do not automatically set udp_dest (by default it will auto set udp_dest")
@click.option("--accum-num",
              default=0,
              help="Integer number of scans to accumulate")
@click.option("--accum-every",
              default=None,
              type=float,
              help="Accumulate every Nth scan")
@click.option("--accum-every-m",
              default=None,
              type=float,
              help="Accumulate scan every M meters traveled")
@click.option("--accum-map",
              is_flag=True,
              help="Enable the overall map accumulation mode")
@click.option("--accum-map-ratio",
              default=0.001,
              help="Ratio of random points of every scan to add to an overall map")
def viz(hostname: str, lidar_port: int, meta: Optional[str], filter: bool,
        buf_size: int, verbose: bool, timeout: float,
        extrinsics: Optional[List[float]], soft_id_check: bool,
        do_not_reinitialize: bool, no_auto_udp_dest: bool, accum_num: int,
        accum_every: Optional[int], accum_every_m: Optional[float],
        accum_map: bool, accum_map_ratio: float) -> None:
    """Listen for data on the specified ports and run the visualizer.

    Note: Please pay attention to your firewall and networking configuration. You
    may have to disable your firewall for packets to reach the visualizer/client.
    """
    try:
        from ouster.viz import SimpleViz, scans_accum_for_cli
    except ImportError as e:
        raise click.ClickException(str(e))

    scan_source = None
    try:
        from ouster.sdkx.open_source import open_source
        scan_source = open_source(hostname, sensor_idx=0,
                                  lidar_port=lidar_port,
                                  imu_port=7503,    # viz doesn't use imu
                                  complete=filter,
                                  flags=True,
                                  do_not_reinitialize=do_not_reinitialize,
                                  no_auto_udp_dest=no_auto_udp_dest,
                                  timeout=timeout,
                                  _soft_id_check=soft_id_check,
                                  extrinsics=extrinsics)

        scans_accum = scans_accum_for_cli(scan_source.metadata,
                                          accum_num=accum_num,
                                          accum_every=accum_every,
                                          accum_every_m=accum_every_m,
                                          accum_map=accum_map,
                                          accum_map_ratio=accum_map_ratio)

        SimpleViz(scan_source.metadata,
                  scans_accum=scans_accum).run(scan_source)
    except Exception as e:
        click.secho(f"{e}", fg='red')
    finally:
        if scan_source:
            scan_source.close()

    click.echo("Done")
