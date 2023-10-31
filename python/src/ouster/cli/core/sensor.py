#  type: ignore
import json
from typing import Optional, List

import numpy as np
import requests

import click
from click import ClickException

import ouster.client as client
from ouster.client._client import SensorConfig
from ouster.sdk.util import firmware_version
from .util import click_ro_file
from copy import copy
from packaging import version


MIN_AUTO_DEST_FW = version.Version("2.3.1")


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


def auto_detected_udp_dest(hostname: str) -> int:
    """
    Function which obtains the udp_dest the sensor would choose when automatically detecting
    without changing anything else about sensor state

    Args:
        hostname: sensor hostname
    Returns:
        udp_dest: the udp_dest the sensor detects automatically
    """
    orig_config = client.get_config(hostname, active=True)

    # get what the possible auto udp_dest is
    config_endpoint = f"http://{hostname}/api/v1/sensor/config"
    response = requests.post(config_endpoint, params={'reinit': False, 'persist': False},
            json={'udp_dest': '@auto'})
    response.raise_for_status()

    # get staged config
    udp_auto_config = client.get_config(hostname, active=False)

    # set staged config back to original
    response = requests.post(config_endpoint, params={'reinit': False, 'persist': False},
            json={'udp_dest': str(orig_config.udp_dest)})
    response.raise_for_status()

    return udp_auto_config.udp_dest


def configure_sensor(hostname: str, lidar_port: int,
        do_not_reinitialize: bool, no_auto_udp_dest) -> SensorConfig:
    """Depending on the args do_not_reinitialize, no_auto_udp_dest,
    possibly reconfigure the sensor. Then, return the configuration that is used."""

    click.echo(f"Contacting sensor {hostname}...")

    fw_version = firmware_version(hostname)

    auto_config_udp_dest = None
    use_set_config_auto = False

    # original config
    orig_config = client.get_config(hostname, active=True)

    if fw_version >= MIN_AUTO_DEST_FW:
        auto_config_udp_dest = auto_detected_udp_dest(hostname)
        if orig_config.udp_dest != auto_config_udp_dest:
            if no_auto_udp_dest or do_not_reinitialize:
                click.echo(f"WARNING: Your sensor's udp destination {orig_config.udp_dest} does "
                           f"not match the detected udp destination {auto_config_udp_dest}. "
                           f"If you get a Timeout error, drop -x and -y from your "
                           f"arguments to allow automatic udp_dest setting.")
    else:
        if no_auto_udp_dest or do_not_reinitialize:
            click.echo("WARNING: You have opted not to allow us to reset your auto UDP dest "
                       "by using either -x or -y. If you get a Timeout error, drop -x and -y "
                       "from  your arguments to allow automatic udp_dest setting.")
        else:
            use_set_config_auto = True

    if do_not_reinitialize:

        if orig_config.operating_mode == client.OperatingMode.OPERATING_STANDBY:
            raise click.ClickException("Your sensor is in STANDBY mode but you have disallowed "
                                       "reinitialization. Drop -x to allow reinitialization or "
                                       "change your sensor's operating mode.")

        if lidar_port is not None and orig_config.udp_port_lidar != lidar_port:
            raise click.ClickException(f"Sensor's lidar port {orig_config.udp_port_lidar} does "
                                       f"not match provided lidar port but you have disallowed "
                                       f"reinitialization. Drop -x to allow reinitialization or "
                                       f"change your specified lidar_port {lidar_port}")
        return orig_config

    new_config = copy(orig_config)
    if lidar_port is not None and orig_config.udp_port_lidar != lidar_port:
        new_config.udp_port_lidar = lidar_port
        click.echo((f"Will change lidar port from {orig_config.udp_port_lidar} to "
                    f"{new_config.udp_port_lidar}..."))
    else:
        # lidar port from arguments is None
        lidar_port = orig_config.udp_port_lidar

    if not no_auto_udp_dest and auto_config_udp_dest and orig_config.udp_dest != auto_config_udp_dest:
        click.echo((f"Will change udp_dest from '{orig_config.udp_dest}' to automatically "
                    f"detected '{auto_config_udp_dest}'..."))
        new_config.udp_dest = auto_config_udp_dest

    if use_set_config_auto:
        click.echo(f"Will change udp_dest from '{orig_config.udp_dest}' to automatically "
            "detected UDP DEST")
        new_config.udp_dest = None

    new_config.operating_mode = client.OperatingMode.OPERATING_NORMAL
    if new_config.operating_mode != orig_config.operating_mode:
        click.echo((f"Will change sensor's operating mode from {orig_config.operating_mode}"
                    f" to {new_config.operating_mode}"))

    if orig_config != new_config or use_set_config_auto:
        click.echo("Setting sensor config...")
        client.set_config(hostname, new_config, persist=False, udp_dest_auto = use_set_config_auto)

        new_config = client.get_config(hostname)

    return new_config


@sensor_group.command()
@click.argument('hostname', required=True)
@click.option('-b', '--buf-size', default=256, hidden=True, help="Max packets to buffer")
@click.option('-e', '--extrinsics', type=float, nargs=16,
              help='Lidar sensor extrinsics to use in viz')
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
        from ouster.sdkx.parsing import default_scan_fields
    except ImportError as e:
        raise click.ClickException(str(e))

    config = configure_sensor(hostname, lidar_port, do_not_reinitialize, no_auto_udp_dest)

    click.echo(f"Initializing connection to sensor {hostname} on "
               f"lidar port {config.udp_port_lidar} with udp dest '{config.udp_dest}'...")

    # make 0 timeout in the cli mean no timeout
    timeout_ = timeout if timeout > 0 else None

    # override metadata, if provided
    meta_override: Optional[client.SensorInfo] = None
    if meta is not None:
        # warn that they should set _soft_id_check if overriding metadata
        if not soft_id_check:
            soft_id_check = True
            click.echo(f"Setting soft_id_check as you have elected to override sensor's metadata "
                       f"with metadata {meta}")

        click.echo(f"Will use {meta} to override sensor metadata...")
        with open(meta) as json:
            meta_override = client.SensorInfo(json.read())

    source = client.Sensor(hostname,
                           config.udp_port_lidar,
                           7503,  # doesn't matter as viz doesn't handle IMU packets
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
            click.echo(f"Using sensor extrinsics:\n{scans.metadata.extrinsic}")

        scans_accum = scans_accum_for_cli(scans.metadata,
                                          accum_num=accum_num,
                                          accum_every=accum_every,
                                          accum_every_m=accum_every_m,
                                          accum_map=accum_map,
                                          accum_map_ratio=accum_map_ratio)

        SimpleViz(scans.metadata, scans_accum=scans_accum).run(scans)

    finally:
        if scans._timed_out:
            click.echo(f"ERROR: Timed out while awaiting new packets from sensor {hostname} "
                       f"using udp destination {config.udp_dest} on port {config.udp_port_lidar}. "
                       f"Check your firewall settings and/or ensure that the lidar port "
                       f"{config.udp_port_lidar} is not being held open.")

        if source.id_error_count:
            click.echo(f"WARNING: {source.id_error_count} lidar_packets with "
                  "mismatched init_id/sn were detected.")
            if not soft_id_check:
                click.echo("NOTE: To disable strict init_id/sn checking use "
                      "--soft-id-check option (may lead to parsing "
                      "errors)")

    click.echo("Done")
