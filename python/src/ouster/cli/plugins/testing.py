import click
from ouster.cli.core import cli  # type: ignore
import time
from typing import IO
from os import path

import ouster.sdk.core as core
import ouster.sdk.sensor as sensor
import ouster.sdk.core._digest as digest
from ouster.sdk.util import resolve_metadata

_click_ro_file = click.Path(exists=True, dir_okay=False, readable=True)

# Pipe logging statements away so we can use output of digest directly as hash
# client.init_logger("info", "ouster-python.log")


@cli.group(name="testing", hidden=True)
def testing_group() -> None:
    pass


@testing_group.command(name="time")
@click.argument('pcap_file', required=True, type=click.Path(exists=True))
@click.argument('json', required=True, type=click.File('r'))
def testing_time(pcap_file: str, json: IO) -> None:
    """Print wall clock execution time for some common pcap operations.

    Useful for rough benchmarking and quick tests for performance regressions
    during development.
    """

    try:
        import ouster.sdk.pcap as pcap
    except ImportError:
        raise click.ClickException("Please verify that libpcap is installed")

    info = core.SensorInfo(json.read())

    click.echo("Initializing replay and client...")
    replay = pcap._replay(pcap_file, info, "127.0.1.1", 7502, 7503)
    src = sensor.SensorPacketSource("test-sensor",
                        sensor_info=[info])
    net_packets = iter(src)
    click.echo("Timing reading data from a socket...")
    start = time.time()
    # keeping one packet "in flight" improves performance quite a bit
    n = 1
    next(replay)
    while next(replay):
        n += 1
        next(net_packets)
    next(net_packets)
    click.echo(f"Got {n} packets")

    dur = time.time() - start
    click.echo(f"Done: took {dur:.3f}s")


@testing_group.command(name="digest")
@click.argument('file', type=click.Path(exists=True))
@click.option('-m', '--meta', required=False, type=click.Path(exists=True),
        help="Metadata file for PCAP, helpful if automatic metadata resolution fails")
@click.option('-s', '--save', is_flag=True, default=False,
              help="Save the digest to file.")
@click.option('-c',
              '--check',
              type=click.Path(exists=True),
              help="Check computed digest")
def compute_digest(file: str, meta: str, check: str, save: bool) -> None:
    """Write out a digest file for the specified lidar data."""

    resolved_meta = resolve_metadata(file, meta)
    if resolved_meta is None:
        raise click.ClickException("Failed to resolve metadata.")

    with open(resolved_meta, 'r') as json:
        info = core.SensorInfo(json.read())

    packets: core.PacketSource
    if path.splitext(file)[1] == ".pcap":
        try:
            from ouster.sdk import pcap
        except ImportError:
            raise click.ClickException(
                "Please verify that libpcap is installed")
        packets = pcap.PcapPacketSource(file,
                            sensor_info=[info])
    else:
        raise click.ClickException(f"File {file} is not supported")

    other = digest.StreamDigest.from_packets(packets)

    if check is not None:
        with open(check, 'r') as c:
            ok = digest.StreamDigest.from_json(c.read())
            ok.check(other)
            click.echo("Ok")
    else:
        click.echo(other.to_json())

    if save:
        filename = file[0:-5] + "_digest.json"
        print("Saving to: ", filename)
        with open(filename, "w") as text_file:
            text_file.write(other.to_json())
