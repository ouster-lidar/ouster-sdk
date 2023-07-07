# type: ignore

import click
from typing import Optional, List
import ouster.cli.core.osf as osf_cli
from .io_type import OusterIoType
from .source import source, _source_arg_name, _output_file_arg_name, SourceConvertCommand


@click.command
@click.argument(_output_file_arg_name, required=True)
@click.option('-m',
              '--meta',
              required=False,
              type=click.Path(exists=True, dir_okay=False, readable=True))
@click.option('-s', '--chunk-size', default=0, help="Chunk size in bytes")
@click.option('-f',
              '--flags',
              is_flag=True,
              help='Add FLAGS/FLAGS2 to LidarScans')
@click.option('--raw-headers',
              is_flag=True,
              help='Add RAW_HEADERS to LidarScans')
@click.option('--raw-fields',
              is_flag=True,
              help='Add RAW32_WORDs to LidarScans')
@click.option('--extrinsics',
              type=float,
              required=False,
              nargs=16,
              help='Lidar sensor extrinsics to use (single sensor data only)')
@click.option('--multi',
              is_flag=True,
              help='Turn on multi sensor pcap handling and metadata resolutions')
@click.option('--soft-id-check',
              is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't "
              "match with metadata")
@click.pass_context
def osf_from_pcap(ctx, meta: Optional[str], output_file: Optional[str],
                  chunk_size: int, flags: bool, raw_headers: bool,
                  raw_fields: bool, extrinsics: Optional[List[float]],
                  multi: bool, int, soft_id_check: bool) -> None:
    """Convert the source PCAP to OSF"""
    # Implements ouster-cli source <sourcefile>.pcap convert <destfile>.osf
    source = ctx.obj.get(_source_arg_name)
    osf_cli.osf_from_pcap_impl(
        source, meta, output_file,
        chunk_size, flags, raw_headers,
        raw_fields, extrinsics,
        multi, soft_id_check)


@click.command
@click.option('-s', '--short', is_flag=True, help='Print less metadata info')
@click.pass_context
def osf_info(ctx, *args, **kwargs) -> None:
    """Display metadata from the source OSF"""  # Implements ouster-cli source <sourcefile>.osf info
    source = ctx.obj.get(_source_arg_name)
    kwargs['file'] = source
    # TODO refactor
    ctx.forward(osf_cli.osf_info, *args, **kwargs)


# @click.command
# TWS 20230627: '--cycle' is a deprecated option and only hidden to prevent breaking scripts that may be using it
# @click.option("-c", "--cycle", is_flag=True, help="Loop playback", hidden=True)
# @click.option('-e', '--on-eof', default='loop', type=click.Choice(['loop', 'stop', 'exit']),
#    help="Loop, stop, or exit after reaching end of file")
# @click.option("-p", "--pause", is_flag=True, help="Pause at first lidar scan")
# @click.option("--pause-at",
#              default=-1,
#              help="Lidar Scan number where to pause (if --pause is ON)")
# @click.option("--accum-num",
#              default=0,
#              help="Number of lidar scan clouds to accumulate")
# @click.option("--accum-every",
#              default=50,
#              help="Accumulate lidar scan clouds every Nth scan")
# # FIXME! apparently these options were removed but not updated here
# # @click.option("--skip-poses",
# #              is_flag=True,
# #              help="Don't read and don't apply trajectories")
# # @click.option("--alt-traj-file",
# #              required=False,
# #              type=click.Path(exists=True, dir_okay=False),
# #              help="Alternative source of trajectories (OSF with traj msgs)")
# @click.option("-r", "--rate", default=1.0, help="Playback rate")
# @click.option("--extrinsics",
#              type=float,
#              required=False,
#              nargs=16,
#              help="Lidar sensor extrinsics to use in viz (instead possible "
#                   " extrinsics stored in OSF)")
# @click.option("--skip-extrinsics",
#              is_flag=True,
#              help="Don't use any extrinsics (leaves them at Identity)")
# @click.option("-s",
#              "--start-ts",
#              type=int,
#              required=False,
#              default=0,
#              help="Viz from the provided start timestamp (nanosecs)")
# @click.option("--sensor-id",
#              type=int,
#              required=False,
#              default=0,
#              help="Viz only the single sensor by sensor_id")
# @click.pass_context
# def osf_viz(ctx, *args, **kwargs) -> None:
#    """Visualize the OSF data in a 3D viewer"""  # Implements ouster-cli source <sourcefile>.osf viz
#    source = ctx.obj.get(_source_arg_name)
#    kwargs['file'] = source
#    # TODO refactor
#    ctx.forward(osf_cli.osf_viz, *args, **kwargs)


# TODO SW-4407: various OSF convert stories
class OsfConvertCommand(SourceConvertCommand):
    """Implements
    ouster-cli source <sourcefile>.osf convert <otherfile>

    This method delegates to the approrpiate command depending on the file
    extension of the output file argument.
    """
    conversions = {
    }


#  add OSF commands to `source` command
source.commands[OusterIoType.OSF] = {
    # TODO 4407 various stories for OSF conversion
    'convert': OsfConvertCommand(
        'convert',
        context_settings=dict(ignore_unknown_options=True, allow_extra_args=True),
        help="Save point cloud from an OSF file into specific formats"
    ),
    'info': osf_info,
    # 'viz': osf_viz,
}


#  add conversions to OSF from other formats
source.commands[OusterIoType.PCAP]['convert'].conversions[OusterIoType.OSF] = osf_from_pcap
