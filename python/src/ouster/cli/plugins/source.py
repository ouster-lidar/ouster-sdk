#  type: ignore
import atexit
import click
import re
import threading
import numpy as np
from itertools import islice
from ouster.cli.core import cli
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.core.util import click_ro_file
from ouster.sdk import open_source
from ouster.sdk.client import first_valid_packet_ts
from ouster.sdk.client.core import ClientTimeout
from ouster.sdk.io_type import (extension_from_io_type, io_type, OusterIoType)
import ouster.cli.plugins.source_pcap as pcap_cli
import ouster.cli.plugins.source_osf as osf_cli
import ouster.cli.plugins.source_sensor as sensor_cli
from typing import (List, Optional, Iterable, Tuple, Union)
from .source_save import SourceSaveCommand
from .source_util import (CoupledTee,
                          SourceCommandContext,
                          SourceCommandCallback,
                          SourceCommandType,
                          source_multicommand,
                          _join_with_conjunction)

_source_arg_name: str = 'source'


@click.command()
@click.option("-p", "--pause", is_flag=True, help="Pause at first lidar scan")
@click.option("-e", "--on-eof", default='loop', type=click.Choice(['loop', 'stop', 'exit']),
              help="Loop, stop or exit after reaching end of file")
@click.option('-r',
              '--rate',
              default="1",
              help="Playback rate.",
              type=click.Choice(["0.25", "0.5", "0.75", "1", "1.5", "2", "3", "max"]))
@click.option("--pause-at",
              default=-1,
              help="Lidar Scan number to pause at")
@click.option("--accum-num",
              default=None,
              type=int,
              help="Accumulate up to this number of past scans for visualization. "
                   "Use <= 0 for unlimited. Defaults to 100 if --accum-every or --accum-every-m is set.")
@click.option("--accum-every",
              default=None,
              type=int,
              help="Add a new scan to the accumulator every this number of scans.")
@click.option("--accum-every-m",
              default=None,
              type=float,
              help="Add a new scan to the accumulator after this many meters of travel.")
@click.option("--map", "_map",
              is_flag=True,
              help="If set, add random points from every scan into an overall map for visualization. "
                   "Enabled if either --map-ratio or --map-size are set.")
@click.option("--map-ratio",
              default=None, type=float,
              help="Fraction of random points in every scan to add to overall map (0, 1]. [default: 0.01]")
@click.option("--map-size",
              default=None, type=int,
              help="Maximum number of points in overall map before discarding. [default: 1500000]")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_viz(ctx: SourceCommandContext, pause: bool, on_eof: str, pause_at: int, accum_num: Optional[int],
               accum_every: Optional[int], accum_every_m: Optional[float],
               _map: bool, map_ratio: float, map_size: int, rate: str) -> SourceCommandCallback:
    """Visualize LidarScans in a 3D viewer."""
    try:
        from ouster.sdk.viz import SimpleViz, ScansAccumulator
    except ImportError as e:
        raise click.ClickException(str(e))

    source = ctx.scan_source
    from ouster.sdk.client import ScanSourceAdapter
    if isinstance(source, ScanSourceAdapter):
        source = source._scan_source

    # ugly workarounds ensue
    if on_eof == 'loop':
        # NOTE: setting it here instead of at open_source stage because we do not want to propagate
        #       the flag up to `source` command
        source._cycle = True

    if pause and pause_at == -1:
        pause_at = 0

    # Determine how to set the rate
    if rate == "max":
        rate = 0.0
    else:
        rate = float(rate)
    if source.is_live:
        if rate != 1.0:
            raise click.exceptions.UsageError("Can only set a rate of 1 for live sources")
        rate = None

    ctx.scan_iter, scans = CoupledTee.tee(ctx.scan_iter,
                                          terminate=ctx.terminate_evt)
    metadata = ctx.scan_source.metadata

    # build the accumulator
    if accum_every_m is not None and accum_every is not None:
        raise click.exceptions.UsageError("Can only provide one of --accum-every and --accum-every-m")

    if accum_num is not None and accum_num <= 0:
        accum_num = 1000000

    if accum_every_m is not None or accum_every is not None:
        if accum_num is None:
            accum_num = 100
    elif accum_num is None:
        accum_num = 0

    if accum_every_m is not None:
        accum_every = 0
    elif accum_every is not None:
        accum_every_m = 0.0
    else:
        accum_every = 0 if accum_num is None else 1
        accum_every_m = 0.0

    if map_ratio is not None or map_size is not None:
        _map = True
    map_ratio = 0.01 if map_ratio is None else map_ratio
    map_size = 1500000 if map_size is None else map_size

    if map_ratio > 1.0 or map_ratio <= 0.0:
        raise click.exceptions.UsageError("--map-ratio must be in the range (0, 1]")
    if map_size <= 0:
        raise click.exceptions.UsageError("--map-size must be greater than 0")

    scans_accum = ScansAccumulator(metadata,
                            accum_max_num=accum_num,
                            accum_min_dist_num=accum_every,
                            accum_min_dist_meters=accum_every_m,
                            map_enabled=_map,
                            map_select_ratio=map_ratio,
                            map_max_points=map_size)

    def viz_thread_fn():
        sv = SimpleViz(metadata, scans_accum=scans_accum,
                       rate=rate, pause_at=pause_at, on_eof=on_eof)
        sv.run(scans)
        ctx.terminate_evt.set()

    if ctx.main_thread_fn is not None:
        raise RuntimeError(
            "A main-thread required function has already been set.")
    ctx.main_thread_fn = viz_thread_fn


def extract_slice_indices(click_ctx: Optional[click.core.Context],
                          param: Optional[click.core.Argument], value: str):
    """Validate and extract slice indices of the form [start]:[stop][:step]."""
    index_matches = re.findall(r"^(-?\d*):(-?\d*):?(-?\d*)$", value)  # noqa: W605
    time_index_matches = re.findall(r"^(?:(\d+(?:\.\d+)?)(h|min|s|ms))?"
                                    r":(?:(\d+(?:\.\d+)?)(h|min|s|ms))?(?::(-?\d*))?$", value)  # noqa: W605

    if time_index_matches and len(time_index_matches[0]) == 5:
        # it was a time index, convert everything to float seconds and handle units
        match = time_index_matches[0]
        multipliers = {}
        multipliers['h'] = 60.0 * 60.0
        multipliers['min'] = 60.0
        multipliers['s'] = 1.0
        multipliers['ms'] = 0.001
        start = float(match[0]) * multipliers[match[1]] if match[0] != "" else None
        end = float(match[2]) * multipliers[match[3]] if match[2] != "" else None
        skip = int(match[4]) if match[4] != "" else None
        return start, end, skip

    if not index_matches or len(index_matches[0]) != 3:
        raise click.exceptions.BadParameter(
            "slice indices must be of the form [start]:[stop][:step]")
    parsed_indices = [int(i) if i != "" else None for i in index_matches[0]]
    start, stop, step = parsed_indices[0], parsed_indices[1], parsed_indices[2]
    # Check that indices are non-negative
    if any(i < 0 if i is not None else False for i in parsed_indices):
        raise click.exceptions.BadParameter(
            "slice indices must be non-negative")
    # Check that stop > start
    if (stop is not None) and (not stop > start):
        raise click.exceptions.BadParameter(
            "slice stop index must be greater than start")
    # Check that step > 1
    if (step is not None) and (not step > 0):
        raise click.exceptions.BadParameter(
            "slice step index must be greater than 0")

    return start, stop, step


@click.command()
@click.argument('indices', required=True, callback=extract_slice_indices)
@click.pass_context
@source_multicommand(type=SourceCommandType.PROCESSOR)
def source_slice(ctx: SourceCommandContext,
                 indices: Tuple[Optional[int]]) -> SourceCommandCallback:
    """Slice LidarScans streamed from SOURCE. Use the form [start]:[stop][:step].
    Optionally can specify start and stop as times relative to the start of the file
    using the units h (hours), min (minutes), s (seconds), or ms (milliseconds).
    For example: 10s:20s:2"""
    start, stop, step = indices
    if type(start) is float:
        scans = ctx.scan_iter

        def iter():
            start_time = None
            counter = 0
            for scan in scans:
                scan_time = first_valid_packet_ts(scan)
                if scan_time == 0 or scan_time is None:
                    print("WARNING: Scan missing packet timestamps. Yielding scan in time slice anyways.")
                    yield scan
                    continue
                scan_time = scan_time / 1e9
                if start_time is None:
                    start_time = scan_time
                dt = scan_time - start_time
                if dt >= start:
                    if not stop or dt <= stop:
                        if not step or counter % step == 0:
                            yield scan
                        counter = counter + 1
                    else:
                        return

        ctx.scan_iter = iter()
    else:
        ctx.scan_iter = islice(ctx.scan_iter, start, stop, step)


@click.command()
@click.option("-v", "--verbose", is_flag=True, help="Print out additional stats info.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_stats(ctx: SourceCommandContext, verbose: bool) -> SourceCommandCallback:
    """Calculate and output various statistics about the scans at this point in the pipeline."""
    window = ctx.scan_source.metadata.format.column_window
    scans = ctx.scan_iter
    count = 0
    incomplete_count = 0
    start = None
    end = None
    incomplete_scans = []
    dimensions = {}

    def stats_iter():
        nonlocal count, incomplete_count, window, verbose, start, end, incomplete_scans, dimensions
        ns_to_sec = 1.0 / 1000000000.0
        for scan in scans:
            time = first_valid_packet_ts(scan)
            if time != 0.0:
                if start is None or time < start:
                    start = time
                if end is None or time > end:
                    end = time
            dimensions[(scan.w, scan.h)] = True
            if not scan.complete(window):
                incomplete_count = incomplete_count + 1
                if verbose:
                    incomplete_scans.append(f"    #{count} at {time * ns_to_sec}")
            count = count + 1
            yield scan
    ctx.scan_iter = stats_iter()

    def exit_handler():
        nonlocal count, incomplete_count, start, end, incomplete_scans, dimensions
        ns_to_sec = 1.0 / 1000000000.0
        print("Scan Statistics:")
        print(f"  Count: {count}")
        dstring = ""
        for k in dimensions:
            dstring = dstring + f" {k[0]}x{k[1]}"
        print(f"  Sizes:{dstring}")
        if start is None:
            print("  First Time: No Valid Timestamps")
            print("  Last Time: No Valid Timestamps")
            print("  Duration: Unknown")
        else:
            print(f"  First Time: {start * ns_to_sec}")
            print(f"  Last Time: {end * ns_to_sec}")
            print(f"  Duration: {(end - start) * ns_to_sec} seconds")
        print(f"  Incomplete Scans: {incomplete_count}")
        if verbose:
            for i in incomplete_scans:
                print(i)

    atexit.register(exit_handler)


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
            'ANY': {
                'viz': source_viz,
                'slice': source_slice,
                'stats': source_stats
            },
            OusterIoType.SENSOR: {
                'config': sensor_cli.sensor_config,
                'metadata': sensor_cli.sensor_metadata,
                'save': SourceSaveCommand('save', context_settings=dict(ignore_unknown_options=True,
                                                                        allow_extra_args=True)),
                'userdata': sensor_cli.sensor_userdata,
            },
            OusterIoType.PCAP: {
                'info': pcap_cli.pcap_info,
                'save': SourceSaveCommand('save', context_settings=dict(ignore_unknown_options=True,
                                                                        allow_extra_args=True)),
            },
            OusterIoType.OSF: {
                'dump': osf_cli.osf_dump,
                'info': osf_cli.osf_info,
                'metadata': osf_cli.osf_metadata,
                'parse': osf_cli.osf_parse,
                'save': SourceSaveCommand('save', context_settings=dict(ignore_unknown_options=True,
                                                                        allow_extra_args=True)),
            }
        }

    def get_supported_source_types(self):
        return [iotype for iotype in self.commands.keys() if isinstance(iotype, OusterIoType)]

    def get_source_file_extension_str(self):
        exts = sorted(
            [extension_from_io_type(src_type)
                for src_type in self.commands.keys() if extension_from_io_type(src_type)]
        )
        return _join_with_conjunction(exts)

    def list_commands(self, click_ctx: click.core.Context):
        """Get the source type from the click context
        and return the list of appropriate sub command names"""
        source = click_ctx.params.get(_source_arg_name)

        if not source and CliArgs().has_any_of(click_ctx.help_option_names):
            # Build a map from command name to command
            command_to_types = {}
            for src_type in self.commands.keys():
                for command_name in self.commands[src_type].keys():
                    if command_name not in command_to_types:
                        command_to_types[command_name] = {}
                    if src_type == "ANY":
                        for supported_src_type in self.get_supported_source_types():
                            command_to_types[command_name][supported_src_type] = self.commands[src_type][command_name]
                    else:
                        command_to_types[command_name][src_type] = self.commands[src_type][command_name]

            # Prefix command name with names of supported source types
            command_to_types_renamed = {}
            for key, value in command_to_types.items():
                prefix = _join_with_conjunction(
                    [t.name.upper() for t in value.keys()], separator="|", conjunction="")
                command_to_types_renamed[f"{prefix} {key}"] = value

            return command_to_types_renamed

        file_extensions_str = self.get_source_file_extension_str()
        if not source:
            param_decls = [_source_arg_name]
            param = click.core.Argument(param_decls=param_decls)
            raise click.exceptions.MissingParameter(
                None, click_ctx, param=param)
        try:
            return {**self.commands[io_type(source)], **self.commands["ANY"]}
        except ValueError as e:  # noqa: F841
            click.echo(click_ctx.get_usage())
            raise click.exceptions.UsageError("Source type expected to be a sensor hostname, "
                                              f"ip address, or a(n) {file_extensions_str} file. "
                                              "For a sensor source, please check that you can "
                                              "ping the sensor hostname/ip address. For a file "
                                              "source, please check that the file path you have "
                                              "provided exists.")
        except KeyError as e:  # noqa: F841
            click.echo(click_ctx.get_usage())
            raise click.exceptions.UsageError("Source type expected to be a sensor hostname, "
                                              f"ip address, or a(n) {file_extensions_str} file. "
                                              "For a sensor source, please check that you can "
                                              "ping the sensor hostname/ip address. For a file "
                                              "source, please check that the file path you have "
                                              "provided exists.")

    def get_command(self, click_ctx: click.core.Context, name: str):
        """Get the click.Command object for the given command name"""
        source = click_ctx.params.get(_source_arg_name)
        click_ctx.ensure_object(SourceCommandContext)
        ctx: SourceCommandContext = click_ctx.obj
        # add source to context so the command can access it
        ctx.source_uri = source

        command_list = self.list_commands(click_ctx)
        if name in command_list:
            if not source:
                # If called by --help (without source), return the first implementation of a command
                # NOTE: This results in help printing the docstring of only the first implementation
                return command_list[name][list(command_list[name].keys())[0]]
            else:
                ctx.invoked_command_names.append(name)
                return command_list[name]
        return None

    def invoke(self, click_ctx: click.core.Context):
        """Called when the source command is invoked.
        If called without any args, prints the help.
        Otherwise, the superclass method is called."""
        if not click_ctx.protected_args:
            print(self.get_help(click_ctx))
            return
        super().invoke(click_ctx)


@cli.group(cls=SourceMultiCommand, chain=True)
@click.argument(_source_arg_name, required=True)
@click.option('-m', '--meta', required=False, type=click_ro_file, multiple=True,
              help="Metadata for PCAP, helpful if automatic metadata resolution fails")
@click.option('-l', '--lidar-port', default=None, type=int, help="Dest. port of lidar data")
@click.option('-i', '--imu-port', default=None, type=int, help="Dest. port of imu data")
@click.option('-x', '--do-not-reinitialize', is_flag=True, default=False,
              help="Do not reinitialize (by default it will reinitialize if needed)")
@click.option('-y', '--no-auto-udp-dest', is_flag=True, default=False,
              help="Do not automatically set udp_dest (by default it will auto set udp_dest")
@click.option('-s', '--soft-id-check', is_flag=True,
              help="Continue parsing lidar packets even if init_id/sn doesn't match with metadata")  # noqa
@click.option('-t', '--timeout', default=1.0, help="Seconds to wait for data")
@click.option('-F', '--filter', is_flag=True, help="Drop scans missing data")
@click.option('-e', '--extrinsics', type=float, required=False, nargs=16,
              help="Lidar sensor extrinsics to use in viz (instead possible"
                   " extrinsics stored in OSF). If more than one sensor is"
                   " stored in the osf file and this argument is used then"
                   " the same extrinsics will be applied to all sensors")
@click.option("--extrinsics-file",
              type=click.Path(exists=True, dir_okay=False),
              required=False,
              help="Path to a file containing extrinscs. The parameter would be"
              " superseeded by the --extrinscs parameter if both supplied")
def source(source, meta: Tuple[str, ...], lidar_port: int, imu_port: int, extrinsics: Optional[List[float]],
           extrinsics_file: Optional[str], do_not_reinitialize: bool, no_auto_udp_dest: bool,
           soft_id_check: bool, timeout: int, filter: bool):
    """Run a command with the specified source (SENSOR, PCAP, or OSF) as SOURCE.
    For example, a sensor source: ouster-cli source os1-992xxx.local viz
    """
    pass


@source.result_callback()
@click.pass_context
def process_commands(click_ctx: click.core.Context, callbacks: Iterable[SourceCommandCallback],
                     source: str, meta: Tuple[str, ...], lidar_port: int, imu_port: int,
                     extrinsics: Optional[List[float]], extrinsics_file: Optional[str],
                     do_not_reinitialize: bool, no_auto_udp_dest: bool, soft_id_check: bool,
                     timeout: int, filter: bool) -> None:
    """Process all commands in a SourceMultiCommand, using each command's callback"""

    callbacks = list(callbacks)
    ctx: SourceCommandContext = click_ctx.obj
    command_names = ctx.invoked_command_names

    # ---- Lint commands ----
    # Ensure that no commands are duplicated
    names_duplicate_check = set()
    for name in command_names:
        if name in names_duplicate_check:
            raise click.exceptions.UsageError(f"'{name}' is duplicated in the multi-command chain. "
                                              "Please invoke it only once. ")
        names_duplicate_check.add(name)

    # Ensure that no other commands are present, if a MULTICOMMAND_UNSUPPORTED
    # is present
    multicommand = True
    for idx, c in enumerate(callbacks):
        if c.type is SourceCommandType.MULTICOMMAND_UNSUPPORTED:
            multicommand = False
            if len(callbacks) != 1:
                raise click.exceptions.UsageError(f"'{command_names[idx]}' does not support multi-command chaining. "
                                                  "Please invoke it without other commands. ")

    # Ensure that a consumer is always last
    last_consumer_name, last_consumer_idx = None, None
    last_processor_name, last_processor_idx = None, None
    for idx, c in enumerate(callbacks):
        if c.type is SourceCommandType.PROCESSOR:
            last_processor_idx = idx
            last_processor_name = command_names[idx]
        elif c.type is SourceCommandType.CONSUMER:
            last_consumer_idx = idx
            last_consumer_name = command_names[idx]

    if multicommand:
        if last_consumer_idx is None:
            raise click.exceptions.UsageError("Must have a consumer such as 'viz' or 'save'.")
        if (last_processor_idx is not None) and (last_processor_idx > last_consumer_idx):
            raise click.exceptions.UsageError(f"'{last_processor_name}' must be invoked before "
                                              f"'{last_consumer_name}'. Please reorder the multi-command chain. ")

    if not multicommand:
        # Execute single non-multicommand command
        c = callbacks[0]
        c.callback_fn(ctx)
    else:
        # Execute multicommands
        # Open source

        resolved_extrinsics: Optional[Union[str, np.ndarray]] = None

        if extrinsics_file:
            resolved_extrinsics = extrinsics_file
        if extrinsics:
            resolved_extrinsics = np.array(extrinsics).reshape((4, 4))

        ctx.scan_source = open_source(source,
                                      extrinsics=resolved_extrinsics,
                                      meta=meta,
                                      lidar_port=lidar_port, imu_port=imu_port,
                                      do_not_reinitialize=do_not_reinitialize,
                                      no_auto_udp_dest=no_auto_udp_dest,
                                      soft_id_check=soft_id_check,
                                      timeout=timeout, complete=filter,
                                      buf_size=512)
        # [kk] NOTE: buf_size is in terms of lidar_packets. In 4096x5 mode
        # with 16 columns per packet, 512 packets are needed to fully
        # buffer two frames

        if ctx.scan_source.is_indexed and len(ctx.scan_source) == 0:
            print("WARNING: Source contains no scans.")

        ctx.scan_iter = iter(ctx.scan_source)

        # print any timeout exceptions we get
        scans = ctx.scan_iter

        def catch_iter():
            try:
                for scan in scans:
                    yield scan
            except ClientTimeout as ex:
                print(f"Error: {ex}")
            except FileExistsError as ex:
                print(f"Error: {ex}. Add --overwrite flag to overwrite and continue anyways.")
            return
        ctx.scan_iter = catch_iter()
        try:
            # Execute multicommand callbacks

            # Dataflow between callbacks occur entirely through the scan iterator. Callbacks may additionally
            # register functions in ctx.thread_fns to be run in individual threads
            # Callback invariants:
            #   1. Must leave ctx.scan_iter in a consumable state after invocation.
            #       If a callback consumes the original ctx.scan_iter, it must set ctx.scan_iter
            #       to an unconsumed iterator (via CoupledTee or a new iterator)
            #   2. May add a Callable[None -> None] to ctx.thread_fns. These will be executed in
            #       individual threads.
            #   3. A single callback may also register a function to be run from the main thread in
            #       ctx.main_thread_fn. This is required to support viz on macOS, where openGL
            #       applications must be run from the main thread
            #   4. Any registered thread_fn must support termination of execution via
            #       ctx.terminate_evt.is_set()
            #   5. Every registered thread_fn must request the ouster-cli process terminate by calling
            #       ctx.terminate_evt.set() before terminating

            # Most callbacks will do one of the following:
            #   1. execute immediately, and take no further action (ie, config)
            #   2. map a processing function onto scans, such that processing is implicitly called while iterating
            #   3. set ctx.scan_iter to a new iterator, and register a processing thread in ctx.thread_fns
            #   4. create a CoupledTee from ctx.scan_iter, and re-set ctx.scan_iter to one of the resultant tees

            ctx.thread_fns = []
            ctx.main_thread_fn = None
            ctx.terminate_evt = threading.Event()
            for c in callbacks:
                c.callback_fn(ctx)

            # Create threads from functions registered by the callbacks
            threads = []
            for thread_fn in ctx.thread_fns:
                threads.append(threading.Thread(target=thread_fn))

            # Define a function to consume ctx.scan_iter
            def pipeline_flush():
                try:
                    for _ in ctx.scan_iter:
                        pass
                except Exception as ex:
                    # Terminate everything if we get an unhandled exception
                    ctx.terminate_evt.set()
                    raise ex

            threads.append(threading.Thread(target=pipeline_flush))

            # Start all threads
            for thread in threads:
                thread.start()

            # Execute main thread fn, if set
            if ctx.main_thread_fn is not None:
                ctx.main_thread_fn()
                ctx.terminate_evt.set()

            # Wait for threads to terminate
            for thread in threads:
                thread.join()

            true_source = ctx.scan_source._scan_source
            try:
                if true_source._source._id_error_count > 0:
                    print(f"WARNING: {true_source._source._id_error_count} lidar_packets with "
                          "mismatched init_id/sn were detected.")
                    if not soft_id_check:
                        print("NOTE: To disable strict init_id/sn checking use "
                              "--soft-id-check option (may lead to parsing "
                              "errors)")
            except AttributeError:
                pass  # This source doesnt support _id_error_count
        finally:
            # Attempt to close scansource
            try:
                ctx.scan_source.close()
            except:  # noqa: E722
                pass
