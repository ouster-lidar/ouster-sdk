#  type: ignore
from datetime import datetime
import os

import click
from more_itertools import consume
from prettytable import PrettyTable, PLAIN_COLUMNS  # type: ignore
from textwrap import indent

from ouster.sdk import client
from ouster.sdk.util import (resolve_metadata)
from ouster.cli.core.util import (click_ro_file)

from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand)


@click.group(name="pcap", hidden=False)
def pcap_group() -> None:
    """Commands for working with pcap files."""
    pass


def print_stream_table(all_infos):
    # format output
    table = PrettyTable()
    table.field_names = [
        '', 'Src IP', 'Dst IP', 'Src Port', 'Dst Port', 'AF', 'Frag', 'Size',
        'Count'
    ]

    def stream_sort(k):
        return (list(k)[0].dst_ip, list(k)[0].src_ip, list(k)[0].dst_port)

    for k, v in sorted(all_infos.udp_streams.items(), key=stream_sort):
        frag = 'No' if (len(v.fragment_counts) == 1) and (1 in v.fragment_counts) else 'Yes'

        first = True
        af_count = len(v.payload_size_counts.items())
        for af_key, af_value in v.ip_version_counts.items():
            size_count = len(v.payload_size_counts.items())
            for size_key, size_value in v.payload_size_counts.items():
                cont = ""

                if (size_count > 1 or af_count > 1):
                    cont = 'X' if first else '↳'

                table.add_row([
                    cont, k.src_ip, k.dst_ip, k.src_port, k.dst_port, af_key, frag, size_key, size_value
                ])
                first = False
    table.set_style(PLAIN_COLUMNS)
    table.align = 'r'
    table.align['Src IP'] = 'l'  # type: ignore
    click.echo(click.style(indent(str(table), '  '), fg='yellow'))


@click.command
@click.option('-n', type=int, default=-1, help="Read only INTEGER packets.")
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def pcap_info(ctx: SourceCommandContext, click_ctx: click.core.Context, n: int) -> None:
    """Print information about a pcap file to stdout."""
    file = ctx.source_uri
    try:
        import ouster.sdk.pcap as pcap
    except ImportError:
        raise click.ClickException("Please verify that libpcap is installed")

    pcap_size = os.path.getsize(file)
    # read full pcap with progress bar
    all_infos = None
    with click.progressbar(length=pcap_size, label="Reading pcap:") as bar:
        def progress_callback(current, diff, total):
            bar.update(diff)
        all_infos = pcap._packet_info_stream(file, n, progress_callback, 100)
        bar.update(pcap_size)

    encap = {
        0: '<MULTIPLE>',
        1: 'ETHERNET',
        42: 'SLL'
    }.get(all_infos.encapsulation_protocol, 'UNKNOWN')
    min_datetime = datetime.fromtimestamp(all_infos.timestamp_min)
    max_datetime = datetime.fromtimestamp(all_infos.timestamp_max)
    duration = max_datetime - min_datetime

    click.echo(f"File size:     {pcap_size / (2**20):.2f}M")
    click.echo(f"Packets read:  {all_infos.total_packets}")
    click.echo(f"Encapsulation: {encap}")
    click.echo(f"Capture start: {min_datetime}")
    click.echo(f"Capture end:   {max_datetime}")
    click.echo(f"Duration:      {duration}")
    click.echo("UDP Streams:")
    print_stream_table(all_infos)


@click.command
@click.option('-m', '--meta', required=False, type=click_ro_file,
        help="Metadata for PCAP, helpful if automatic metadata resolution fails")
@click.option('-c', '--cycle', is_flag=True, required=False, type=bool, help='Loop playback')
@click.option('-h', '--host', required=False, type=str, default='127.0.1.1', help='Dest. host of UDP packets')
@click.option('--lidar-port', required=False, type=int, default=7502, help='Dest. port of lidar data')
@click.option('--imu-port', required=False, type=int, default=7503, help='Dest. port of imu data')
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR,
                     retrieve_click_context=True)
def replay(ctx: SourceCommandContext, click_ctx: click.core.Context, meta, cycle, host, lidar_port, imu_port):
    """Replay lidar and IMU packets from a PCAP file to a UDP socket."""
    file = ctx.source_uri
    try:
        import ouster.sdk.pcap as pcap
    except ImportError:
        raise click.ClickException("Please verify that libpcap is installed")
    meta = resolve_metadata(file, meta)
    with open(meta) as json:
        click.echo(f"Reading metadata from: {meta}")
        info = client.SensorInfo(json.read())

    click.echo(f"Sending UDP packets to host {host}, ports {lidar_port} and {imu_port} - Ctrl-C to exit.")

    def replay_once():
        replay = pcap._replay(file, info, host, lidar_port, imu_port)
        consume(replay)

    replay_once()
    while cycle:
        replay_once()
