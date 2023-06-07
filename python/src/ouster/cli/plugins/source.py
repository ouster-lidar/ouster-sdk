#  type: ignore
import click
from ouster.cli.core import cli
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.core.util import click_ro_file
import ouster.cli.core.pcap
import ouster.cli.core.sensor
from typing import List, Optional
from .io_type import extension_from_io_type, io_type_from_extension, io_type, OusterIoType


_source_arg_name = 'source'
_output_file_arg_name = 'output_file'


def _join_with_conjunction(things_to_join: List[str], separator: str = ', ', conjunction: str = 'or') -> str:
    """Given a list of things, return a string like
    'Thing A, Thing B, or Thing C'
    """
    strings = [str(x) for x in things_to_join]
    if len(strings) > 1:
        strings[-1] = conjunction + " " + strings[-1]
    if len(strings) == 2:
        return ' '.join(strings)
    return separator.join(strings)


@click.command()
@click.argument('keyval', metavar='[KEY VAL]...', type=str, nargs=-1)
@click.option('-d', 'dump', is_flag=True, help='Dump current configuration')
@click.option('-c', 'file', type=click.File(), help='Read config from file')
@click.option('-u', 'auto', is_flag=True, help='Set automatic udp dest')
@click.option('-p', 'persist', is_flag=True, help='Persist configuration')
@click.option('-s/-n', 'standby', default=None, help='Set STANDBY or NORMAL')
@click.pass_context
def sensor_config(ctx, *args, **kwargs) -> None:
    """View or modify the sensor configuration"""  # Implements ouster-cli source <hostname> config
    source = ctx.obj.get(_source_arg_name)
    kwargs['hostname'] = source
    # TODO refactor
    ctx.forward(ouster.cli.core.sensor.config, *args, **kwargs)


@click.command
@click.option('--legacy/--non-legacy',
              default=False,
              help="Use legacy metadata format or not")
@click.pass_context
def sensor_info(ctx, *args, **kwargs) -> None:
    """Retrieve the sensor metadata"""  # Implements ouster-cli source <hostname> metadata
    source = ctx.obj.get(_source_arg_name)
    kwargs['hostname'] = source
    # TODO refactor
    ctx.forward(ouster.cli.core.sensor.info, *args, **kwargs)


@click.command
@click.option('-l', '--lidar-port', default=7502, help="default: 7502")
@click.option('-i', '--imu-port', default=7503, help="default: 7503")
@click.option('-n', '--n-frames', type=int, help="number of lidar frames")
@click.option('-s', '--n-seconds', default=0.0, help="max time to record")
@click.option('--chunk-size', default=0, help="split output by size (MB)")
@click.option('-b', '--buf-size', default=640, help="Max packets to buffer")
@click.option('-t', '--timeout', default=1.0, help="Seconds to wait for data")
@click.option('-p', '--prefix', default="", help="Recorded file name prefix")
@click.option('--viz', default=False, is_flag=True, help="Visualize point cloud during recording")
@click.option('--legacy/--non-legacy',
              default=False,
              help="Use legacy metadata format or not")
@click.pass_context
def sensor_record(ctx, *args, **kwargs) -> None:
    """Record a sensor data stream as PCAP"""  # Implements ouster-cli source <hostname> record
    source = ctx.obj.get(_source_arg_name)
    kwargs['hostname'] = source
    # TODO refactor
    ctx.forward(ouster.cli.core.pcap.pcap_record, *args, **kwargs)


@click.command
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
@click.pass_context
def sensor_viz(ctx, *args, **kwargs) -> None:
    """Visualize the sensor data in a 3D viewer"""  # Implements ouster-cli source <hostname> viz
    source = ctx.obj.get(_source_arg_name)
    kwargs['hostname'] = source
    # TODO refactor
    ctx.forward(ouster.cli.core.sensor.viz, *args, **kwargs)


@click.command
@click.argument(_output_file_arg_name, required=True)
@click.option('-m', '--meta', required=False)  # TWS 20230426: changed from -f to -m
@click.option('-l', '--lidar-port', default=None, help="Dest. port of lidar data")
@click.option('-i', '--imu-port', default=None, help="Dest. port of imu data")
@click.option('-o', '--output', required=False, help="BAG output filename")
@click.option('--soft-id-check',
              is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't "
              "match with metadata")
@click.pass_context
def bag_from_pcap(ctx, output_file, meta, lidar_port, imu_port, output, soft_id_check) -> None:
    """Convert the source PCAP to Rosbag"""
    # Implements ouster-cli source <sourcefile>.pcap convert <destfile>.bag
    source = ctx.obj.get(_source_arg_name)
    return ouster.cli.core.pcap.pcap_to_bag_impl(source, meta, lidar_port, imu_port, output_file, soft_id_check)


@click.command
@click.argument(_output_file_arg_name, required=True)
@click.option('-f', '--meta', type=click_ro_file)
@click.option('--start-index', default=0, help="index of scan to start outputting")
@click.option('--num-scans', default=1, help="number of scans to save from pcap to csv files")
@click.pass_context
def csv_from_pcap(ctx,
                output_file: str,
                meta: Optional[str],
                start_index: Optional[int],
                num_scans: Optional[int]) -> None:
    """Convert the source PCAP to CSV"""

    source = ctx.obj.get(_source_arg_name)
    if num_scans is not None and num_scans > 1:
        click.echo("INFO: You've selected to output multiple scans. Your output CSV names will be suffixed with index.")
        csv_base = output_file[0:-4]
        output_paths = [f'{csv_base}_{idx:06d}.csv' for idx in range(0, num_scans)]
    else:
        output_paths = [output_file]

    return ouster.cli.core.pcap.pcap_to_csv_impl(source, meta, start_index, num_scans, output_paths)


class SourceConvertCommand(click.Command):
    """Generalizes ouster-cli source <sourcefile> convert <outputfile>
    """
    def __init__(self, *args, **kwargs):
        kwargs['add_help_option'] = False
        super().__init__(*args, **kwargs)
        click.argument(_output_file_arg_name, required=True)(self)

    def get_output_type_file_extensions_str(self):
        exts = sorted(
            [extension_from_io_type(source_type) for source_type in self.conversions.keys()]
        )
        return _join_with_conjunction(exts)

    def invoke(self, ctx):
        output_type_file_extensions = self.get_output_type_file_extensions_str()
        file_extension_err_text =\
            f"Expected {_output_file_arg_name.upper()} extension to be {output_type_file_extensions}"
        try:
            output_file = ctx.params.get(_output_file_arg_name)
            output_type = io_type_from_extension(output_file)
        except (KeyError, ValueError):
            if CliArgs().has_any_of(ctx.help_option_names):
                click.echo(self.get_help(ctx))
                click.echo(f"\nWhere {_output_file_arg_name.upper()} ends with {output_type_file_extensions}")
                return
            raise click.exceptions.UsageError(file_extension_err_text)
        try:
            convert_command = self.conversions[output_type]
            if CliArgs().has_any_of(ctx.help_option_names):
                click.echo(convert_command.get_help(ctx))
            else:
                convert_command.parse_args(ctx, [output_file] + ctx.args)
                ctx.forward(convert_command, *ctx.args)
        except KeyError:
            raise click.exceptions.UsageError(file_extension_err_text)


class PcapConvertCommand(SourceConvertCommand):
    """Implements ouster-cli source <sourcefile>.pcap convert <otherfile>
    """
#    This method delegates to the appropriate command depending on the file
#    extension of the output file argument.
    conversions = {
        # OusterIoType.ROSBAG: bag_from_pcap,
        OusterIoType.CSV: csv_from_pcap,
    }


@click.command
@click.option('-n', type=int, default=0, help="Read only INTEGER packets.")
@click.pass_context
def pcap_info(ctx, *args, **kwargs) -> None:
    """Implements
    ouster-cli source <sourcefile>.pcap info"""
    source = ctx.obj.get(_source_arg_name)
    kwargs['file'] = source
    ctx.forward(ouster.cli.core.pcap.pcap_info, *args, **kwargs)


@click.command
@click.argument('output')
@click.option('-s', '--start-frame', default=0, help="Start frame index")
@click.option('-n', '--num-frames', default=10, help="Number of frames")
@click.option('-f', '--meta', required=False, type=click_ro_file)
@click.option('-l',
              '--lidar-port',
              default=None,
              help="Dest. port of lidar data")
@click.option('-i', '--imu-port', default=None, help="Dest. port of imu data")
@click.option('--soft-id-check',
              is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't "
              "match with metadata")
@click.pass_context
def pcap_slice(ctx, *args, **kwargs) -> None:
    """Implements
    ouster-cli source <sourcefile>.pcap slice"""
    source = ctx.obj.get(_source_arg_name)
    kwargs['file'] = source
    ctx.forward(ouster.cli.core.pcap.pcap_slice, *args, **kwargs)


@click.command
@click.option('-m', '--meta', required=False, type=click_ro_file)  # TWS 20230426: changed this to `-m` from `-f`.
@click.option('-c', '--cycle', is_flag=True, help="Loop playback")
@click.option('-l', '--lidar-port', default=None, help="Dest. port of lidar data")
@click.option('-i', '--imu-port', default=None, help="Dest. port of imu data")
@click.option('-F', '--filter', is_flag=True, help="Drop scans missing data")
@click.option('-b', '--buf', default=50, help="Scans to buffer for stepping.")
@click.option('-r',
              '--rate',
              default=1.0,
              help="Playback rate. One of 0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0")
@click.option('--extrinsics',
              type=float,
              required=False,
              nargs=16,
              help='Lidar sensor extrinsics to use in viz')
@click.option('--soft-id-check',
              is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't "
              "match with metadata")
@click.option("-p", "--pause", is_flag=True, help="Pause after the first scan")
@click.option("--pause-at",
              default=-1,
              help="Lidar Scan number to pause")
@click.option('--multi',
              is_flag=True,
              help='Turn on multi sensor pcap handling and metadata resolutions')
@click.pass_context
def pcap_viz(ctx, *args, **kwargs) -> None:
    """Visualize the PCAP data in a 3D viewer"""
    source = ctx.obj.get(_source_arg_name)
    kwargs['file'] = source
    # TODO refactor
    ctx.forward(ouster.cli.core.pcap.pcap_viz, *args, **kwargs)


class BagConvertCommand(SourceConvertCommand):
    """Implements
    ouster-cli source <sourcefile>.bag convert <otherfile>

    This method delegates to the approrpiate command depending on the file
    extension of the output file argument.
    """
    conversions = {
        # TODO
    }


class SourceMultiCommand(click.MultiCommand):
    """This class implements the ouster-cli source command group.  It uses the
    `io_type` method to determine the source type and map it to the
    available sub commands for that type.

    The source is also added to the click context so that sub commands that use
    @click.pass_context have access to it."""

    def __init__(self, *args, **kwargs):
        kwargs['no_args_is_help'] = True
        super().__init__(*args, **kwargs)
        self.commands = {
            OusterIoType.SENSOR: {
                'config': sensor_config,
                'metadata': sensor_info,
                'record': sensor_record,
                'viz': sensor_viz,
            },
            OusterIoType.PCAP: {
                # TODO FLEETSW-4407 not MVP
                'convert': PcapConvertCommand('convert',
                    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)),
                'info': pcap_info,
                'slice': pcap_slice,
                'viz': pcap_viz,
            },
            # TODO FLEETSW-4407 not MVP
            OusterIoType.ROSBAG: {
                'convert': BagConvertCommand('convert', hidden=True,
                    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)),
            },
        }

    def get_source_file_extension_str(self):
        exts = sorted(
            [extension_from_io_type(source_type)
                for source_type in self.commands.keys() if extension_from_io_type(source_type)]
        )
        return _join_with_conjunction(exts)

    def list_commands(self, ctx):
        """Get the source type from the click context
        and return the list of appropriate sub command names"""
        source = ctx.params.get(_source_arg_name)
        file_extensions_str = self.get_source_file_extension_str()
        if not source and CliArgs().has_any_of(ctx.help_option_names):
            click.echo(ctx.get_usage())
            click.echo(f"\n{_source_arg_name.upper()} is sensor hostname, or a {file_extensions_str} file.")
            ctx.exit()
        if not source:
            param_decls = [_source_arg_name]
            param = click.core.Argument(param_decls=param_decls)
            raise click.exceptions.MissingParameter(None, ctx, param=param)
        try:
            return self.commands[io_type(source)].keys()
        except (KeyError, ValueError) as e:
            click.echo(ctx.get_usage())
            raise click.exceptions.UsageError(e)

    def get_command(self, ctx, name):
        """Get the click.Command object for the given command name"""
        source = ctx.params.get(_source_arg_name)
        ctx.ensure_object(dict)
        # add source to context so the command can access it
        ctx.obj[_source_arg_name] = source
        if name in self.list_commands(ctx):
            return self.commands[io_type(source)][name]
        else:
            return None

    def invoke(self, ctx):
        """Called when the source command is invoked.
        If called without any args, prints the help.
        Otherwise, the superclass method is called."""
        if not ctx.protected_args:
            print(self.get_help(ctx))
            return
        super().invoke(ctx)


@cli.group(cls=SourceMultiCommand)
@click.argument(_source_arg_name, required=True, type=click.Path())
def source(source):
    """Run a command with the specified source as input.
    """
    pass
