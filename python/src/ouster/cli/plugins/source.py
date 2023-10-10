#  type: ignore
import click
from ouster.cli.core import cli
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.core.util import click_ro_file
import ouster.cli.core.pcap
import ouster.cli.core.sensor
from typing import List, Optional
from .io_type import extension_from_io_type, io_type_from_extension, io_type, OusterIoType


_source_arg_name: str = 'source'
_output_file_arg_name: str = 'output_file'


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
    """
    Manipulate the sensor configuration.

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
    # Implements ouster-cli source <hostname> config
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
    ctx.forward(ouster.cli.core.sensor.metadata, *args, **kwargs)


@click.command
@click.option('-l', '--lidar-port', type=int, default=None, help="Lidar Port")
@click.option('-i', '--imu-port', type=int, default=None, help="default: IMU Port")
@click.option('-n', '--n-frames', type=int, help="number of lidar frames")
@click.option('-s', '--n-seconds', default=0.0, help="max time to record")
@click.option('--chunk-size', default=0, help="split output by size (MB)")
@click.option('-b', '--buf-size', default=640, hidden=True, help="Max packets to buffer")
@click.option('-t', '--timeout', default=1.0, help="Seconds to wait for data")
@click.option('-p', '--prefix', default="", help="Recorded file name prefix")
@click.option('--viz', default=False, is_flag=True, help="Visualize point cloud during recording")
@click.option('--legacy/--non-legacy',
              default=False,
              help="Use legacy metadata format or not")
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
@click.pass_context
def sensor_record(ctx, *args, **kwargs) -> None:
    """Record a sensor data stream as PCAP"""  # Implements ouster-cli source <hostname> record
    source = ctx.obj.get(_source_arg_name)
    kwargs['hostname'] = source
    # TODO refactor
    ctx.forward(ouster.cli.core.pcap.pcap_record, *args, **kwargs)


@click.command
@click.option('-b', '--buf-size', default=256, hidden=True, help="Max packets to buffer")
@click.option('-e', '--extrinsics', type=float, nargs=16,
              help='Lidar sensor extrinsics to use in viz')
@click.option('-m', '--meta', help="Provide separate metadata to use with sensor",
        type=click_ro_file, hidden=True)
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
@click.option('--extrinsics',
              type=float,
              required=False,
              nargs=16,
              help='Lidar sensor extrinsics to use in viz')
@click.option('--soft-id-check',
              is_flag=True,
              hidden=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't match with metadata")  # noqa
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
@click.pass_context
def sensor_viz(ctx, *args, **kwargs) -> None:
    """Visualize the sensor data in a 3D viewer"""  # Implements ouster-cli source <hostname> viz
    source = ctx.obj.get(_source_arg_name)
    kwargs['hostname'] = source
    # TODO refactor
    ctx.forward(ouster.cli.core.sensor.viz, *args, **kwargs)


@click.command
@click.argument(_output_file_arg_name, required=True)
@click.option('-m', '--meta', required=False,
        help="Metadata for PCAP, helpful if automatic metadata resolution fails")
@click.option('-l', '--lidar-port', default=None, type=int, help="Dest port of lidar data")
@click.option('-i', '--imu-port', default=None, type=int, help="Dest port of imu data")
@click.option('-o', '--output', required=False, help="BAG output filename")
@click.option('--soft-id-check',
              is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't match with metadata")  # noqa
@click.pass_context
def bag_from_pcap(ctx, output_file, meta, lidar_port, imu_port, output, soft_id_check) -> None:
    """Convert the source PCAP to Rosbag"""
    # Implements ouster-cli source <sourcefile>.pcap convert <destfile>.bag
    source = ctx.obj.get(_source_arg_name)
    return ouster.cli.core.pcap.pcap_to_bag_impl(source, meta, lidar_port, imu_port, output_file, soft_id_check)


@click.command
@click.argument(_output_file_arg_name, required=True)
@click.option('-m', '--meta', type=click_ro_file,
        help="Metadata for PCAP, helpful if automatic metadata resolution fails")
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
        click.echo("INFO: You've selected to output multiple scans. "
                   "Your output CSV names will be suffixed with index.")
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
        # TODO: hack remove with OSF is allowed
        if '.osf' in exts:
            exts.remove('.osf')
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
                # only print output_file if there's a sample output_file type
                # TODO: remove hack on length and fix Borg
                click.echo(self.get_help(ctx))
                if len(CliArgs().args) > 4:
                    click.echo(f"\nERROR: {file_extension_err_text}")
                return
            raise click.exceptions.UsageError(file_extension_err_text)
        try:
            convert_command = self.conversions[output_type]
            if CliArgs().has_any_of(ctx.help_option_names):
                click.echo(convert_command.get_help(ctx))
            else:
                convert_command.parse_args(ctx, [output_file] + ctx.args)
                try:
                    ctx.forward(convert_command, *ctx.args)
                except TypeError:
                    if len(ctx.args) > 0:
                        raise ouster.cli.core.SourceArgsException(ctx)
        except KeyError:
            raise click.exceptions.UsageError(file_extension_err_text)


class PcapConvertCommand(SourceConvertCommand):
    """Implements ouster-cli source <sourcefile>.pcap convert <otherfile>,
    """
    def __init__(self, *args, **kwargs):
        kwargs['help'] = f"Convert from PCAP to {self.get_output_type_file_extensions_str()}"
        super().__init__(*args, **kwargs)
    # this is a map from output type to a conversion function
    conversions = {
        # OusterIoType.ROSBAG: bag_from_pcap,
        OusterIoType.CSV: csv_from_pcap,
    }


@click.command
@click.option('-n', type=int, default=0, help="Read only INTEGER packets.")
@click.pass_context
def pcap_info(ctx, *args, **kwargs) -> None:
    """Display info about the PCAP file"""
    source = ctx.obj.get(_source_arg_name)
    kwargs['file'] = source
    ctx.forward(ouster.cli.core.pcap.pcap_info, *args, **kwargs)


@click.command
@click.argument('output')
@click.option('-s', '--start-frame', default=0, help="Start frame index")
@click.option('-n', '--num-frames', default=10, help="Number of frames")
@click.option('-m', '--meta', required=False, type=click_ro_file,
        help="Metadata for PCAP, helpful if automatic metadata resolution fails")
@click.option('-l', '--lidar-port', default=None,
              type=int, help="Dest. port of lidar data")
@click.option('-i', '--imu-port', type=int, default=None, help="Dest. port of imu data")
@click.option('--soft-id-check',
              is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't match with metadata")  # noqa
@click.pass_context
def pcap_slice(ctx, *args, **kwargs) -> None:
    """Writes a portion of the input PCAP file to a new file"""
    source = ctx.obj.get(_source_arg_name)
    kwargs['file'] = source
    ctx.forward(ouster.cli.core.pcap.pcap_slice, *args, **kwargs)


@click.command
@click.option('-m', '--meta', required=False, type=click_ro_file,
        help="Metadata for PCAP, helpful if automatic metadata resolution fails")
# TWS 20230627: '--cycle' is a deprecated option and only hidden to prevent breaking scripts that may be using it
@click.option('-c', '--cycle', is_flag=True, help="Loop playback", hidden=True)
@click.option('-e', '--on-eof', default='loop', type=click.Choice(['loop', 'stop', 'exit']),
    help="Loop, stop, or exit after reaching end of file")
@click.option('-l', '--lidar-port', default=None, type=int, help="Dest. port of lidar data")
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
              help="Continue parsing lidar packets even if init_id/sn doesn't match with metadata")  # noqa
@click.option("-p", "--pause", is_flag=True, help="Pause after the first scan")
@click.option("--pause-at",
              default=-1,
              help="Lidar Scan number to pause")
@click.option('--multi',
              is_flag=True,
              help='Turn on multi sensor pcap handling and metadata resolutions')
@click.option('--timeout',
              type=float,
              default=10.0,
              help="Timeout in seconds, after which the script will terminate "
              "if no lidar data is encountered in the PCAP file")
@click.option('--kitti-poses',
              required=False,
              type=click_ro_file,
              help="Poses file in Kitti format, one pose per scan "
              "(can be generated by kiss-icp)")
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

    This method delegates to the appropriate command depending on the file
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
                # TODO SW-4407 not MVP
                'convert': PcapConvertCommand('convert',
                    context_settings=dict(ignore_unknown_options=True, allow_extra_args=True)),
                'info': pcap_info,
                'slice': pcap_slice,
                'viz': pcap_viz,
            },
            # TODO SW-4407 not MVP
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
        # TODO: remove hack when bag is introduced
        exts.remove('.bag')

        return _join_with_conjunction(exts)

    def list_commands(self, ctx):
        """Get the source type from the click context
        and return the list of appropriate sub command names"""
        source = ctx.params.get(_source_arg_name)
        file_extensions_str = self.get_source_file_extension_str()
        if not source and CliArgs().has_any_of(ctx.help_option_names):
            # TODO comment out since it repeats - need to clean this up sometime
            # click.echo(ctx.get_usage())
            # click.echo(f"\nERROR: Please specify a {_source_arg_name.upper()},
            # which should be a " f"sensor hostname, or a {file_extensions_str} file.")
            command_dict_list = []
            for source_type in self.commands.keys():
                command_dict_list = command_dict_list + [{ f"{str(source_type)} {inner_command}":self.commands[source_type][inner_command]  # noqa
                        for inner_command in self.commands[source_type].keys()}]

            all_command_dict = {}
            for command_dict in command_dict_list:
                all_command_dict.update(command_dict)

            return all_command_dict

        if not source:
            param_decls = [_source_arg_name]
            param = click.core.Argument(param_decls=param_decls)
            raise click.exceptions.MissingParameter(None, ctx, param=param)
        try:
            return self.commands[io_type(source)].keys()
        except ValueError as e:  # noqa: F841
            click.echo(ctx.get_usage())
            raise click.exceptions.UsageError("Source type expected to be a sensor hostname, "
                                              f"ip address, or a(n) {file_extensions_str} file. "
                                              "For a sensor source, please check that you can "
                                              "ping the sensor hostname/ip address. For a file "
                                              "source, please check that the file path you have "
                                              "provided exists.")
        except KeyError as e:  # noqa: F841
            click.echo(ctx.get_usage())
            raise click.exceptions.UsageError("Source type expected to be a sensor hostname, "
                                              f"ip address, or a(n) {file_extensions_str} file. "
                                              "For a sensor source, please check that you can "
                                              "ping the sensor hostname/ip address. For a file "
                                              "source, please check that the file path you have "
                                              "provided exists.")

    def get_command(self, ctx, name):
        """Get the click.Command object for the given command name"""
        source = ctx.params.get(_source_arg_name)
        ctx.ensure_object(dict)
        # add source to context so the command can access it
        ctx.obj[_source_arg_name] = source

        list_commands = self.list_commands(ctx)
        if name in list_commands:
            if source:
                return self.commands[io_type(source)][name]
            else:
                return list_commands[name]
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
    """Run a command with the specified source (SENSOR, PCAP, or OSF) as SOURCE.
    For example, a sensor source: ouster-cli source os1-992xxx.local viz
    """
    pass
