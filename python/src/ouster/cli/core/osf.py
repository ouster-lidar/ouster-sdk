import click

from typing import Iterator, Dict, cast


@click.group(name="osf", hidden=True)
@click.pass_context
def osf_group(ctx) -> None:
    """Commands for working with OSF files and converting data to OSF."""
    try:
        from ouster.osf import _osf
    except ImportError as e:
        raise click.ClickException("Error: " + str(e))
    ctx.ensure_object(dict)
    sdk_log_level = ctx.obj.get('SDK_LOG_LEVEL', None)
    if sdk_log_level:
        _osf.init_logger(sdk_log_level)


@osf_group.command(name='info')  # type: ignore
@click.argument('file', required=True, type=click.Path(exists=True))
@click.option('-s', '--short', is_flag=True, help='Print less metadata info')
@click.pass_context
def osf_info(ctx, file: str, short: bool) -> None:
    """Print information about an OSF file to stdout.

    Parses all metadata entries, output is in JSON format.
    """
    try:
        from ouster.osf import _osf
    except ImportError as e:
        raise click.ClickException("Error: " + str(e))

    if not ctx.obj.get('SDK_LOG_LEVEL', None):
        # If not SDK_LOG_LEVEL passed we set to "error" logging so to ensure
        # that json output is not interferred with other SDK logging messages
        # and thus ruining valid json structure
        _osf.init_logger("error")

    print(_osf.dump_metadata(file, not short))


@osf_group.command(name='parse')  # type: ignore
@click.argument('file',
                required=True,
                type=click.Path(exists=True, dir_okay=False))
@click.option('-d', '--decode', is_flag=True, help="Decode messages")
@click.option('-v',
              '--verbose',
              is_flag=True,
              help="Verbose LidarScan outputs (only when used with -d option)")
@click.option('-r',
              '--check-raw-headers',
              is_flag=True,
              help="Check RAW_HEADERS fields by reconstructing lidar_packets"
              " and batching LidarScan back (without fields data) and compare."
              "(applies only when used with -d option)")
@click.option('-s',
              '--standard',
              is_flag=True,
              help="Show standard layout with chunks")
def osf_parse(file: str, decode: bool, verbose: bool, check_raw_headers: bool,
              standard: bool) -> None:
    """
    Read an OSF file and print messages type, timestamp and counts to stdout.
    Useful to check chunks layout and decoding of all known messages (-d option).
    """
    try:
        from ouster import client
        import ouster.osf as osf
    except ImportError as e:
        raise click.ClickException("Error: " + str(e))

    # NOTE[pb]: Mypy quirks or some of our Python packages structure quirks, idk :(
    from ouster.sdkx.parsing import scan_to_buffers, buffers_to_scan  # type: ignore

    reader = osf.Reader(file)

    orig_layout = "STREAMING" if reader.has_stream_info else "STANDARD"

    print(f"filename: {file}, layout: {orig_layout}")

    # map stream_id to metadata entry
    scan_stream_sensor: Dict[int, osf.LidarSensor]
    scan_stream_sensor = {}
    for scan_stream_id, scan_stream_meta in reader.meta_store.find(
            osf.LidarScanStream).items():
        scan_stream_sensor[scan_stream_id] = reader.meta_store[
            scan_stream_meta.sensor_meta_id]

    ls_cnt = 0
    other_cnt = 0

    def proc_msgs(msgs: Iterator[osf.MessageRef]):
        nonlocal ls_cnt, other_cnt, decode
        for m in msgs:
            if m.of(osf.LidarScanStream):
                prefix = "Ls"
                ls_cnt += 1
            else:
                prefix = "UN"
                other_cnt += 1
            d = ""
            verbose_str = ""
            if decode:
                obj = m.decode()
                d = "[D]" if obj else ""
                if m.of(osf.LidarScanStream):
                    ls = cast(client.LidarScan, obj)

                    if verbose:
                        verbose_str += f"{ls}"

                    if check_raw_headers:
                        d = d + " " if d else ""
                        if client.ChanField.RAW_HEADERS in ls.fields:
                            sinfo = scan_stream_sensor[m.id].info

                            ls_no_fields = osf.slice_and_cast(ls, {})

                            # roundtrip: LidarScan -> buffers -> LidarScan
                            ls_buffers = scan_to_buffers(ls, sinfo)

                            # recovered lidar scan
                            ls_rec = buffers_to_scan(ls_buffers,
                                                     sinfo,
                                                     fields={})

                            assert ls_rec == ls_no_fields, "LidarScan should be" \
                                " equal when recontructed from RAW_HEADERS fields" \
                                " buffers back (no fields included)"

                            d += "[RAW_HEADERS: OK]"
                        else:
                            d += "[RAW_HEADERS: NONE]"

            print(f"  {prefix}\tts: {m.ts}\t\tstream_id: {m.id}\t{d}")
            if verbose_str:
                print(60 * '-')
                print(f"{verbose_str}")
                print(60 * '-')

    if not standard and reader.has_stream_info:
        proc_layout = "STREAMING"
        proc_msgs(reader.messages())
    else:
        proc_layout = "STANDARD"
        for chunk in reader.chunks():
            print(f"Chunk [{chunk.offset}\t\t]: start_ts = {chunk.start_ts}, "
                  f"end_ts = {chunk.end_ts}")
            proc_msgs(iter(chunk))

    showed_as_str = ""
    if orig_layout != proc_layout:
        showed_as_str = f"showed as: {proc_layout}"

    print()
    print(f"SUMMARY: [layout: {orig_layout}] {showed_as_str}")
    print(f"  lidar_scan    (Ls)    count = {ls_cnt}")
    print(f"  other                 count = {other_cnt}")
