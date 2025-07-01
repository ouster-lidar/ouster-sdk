import click

from typing import Any, Iterator, Dict, cast

from ouster.sdk import core
import ouster.sdk._bindings.osf as osf
from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand,
                          _nanos_to_string)


@click.group(name="osf", hidden=True)
@click.pass_context
def osf_group(ctx) -> None:
    """Commands for working with OSF files and converting data to OSF."""
    ctx.ensure_object(dict)
    sdk_log_level = ctx.obj.get('SDK_LOG_LEVEL', None)
    if sdk_log_level:
        osf.init_logger(sdk_log_level)


@click.command
@click.option('-s', '--short', is_flag=True, help='Print less metadata info')
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def osf_dump(ctx: SourceCommandContext, click_ctx: click.core.Context, short: bool) -> None:
    """Print metdata information from an OSF file to stdout.

    Parses all metadata entries, output is in JSON format.
    """
    file = ctx.source_uri or ""

    if not click_ctx.obj.get('SDK_LOG_LEVEL', None):
        # If not SDK_LOG_LEVEL passed we set to "error" logging so to ensure
        # that json output is not interferred with other SDK logging messages
        # and thus ruining valid json structure
        osf.init_logger("error")

    print(osf.dump_metadata(file, not short))


@click.command
@click.option('-v',
              '--verbose',
              is_flag=True,
              help="Print additional information about the file")
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def osf_info(ctx: SourceCommandContext, click_ctx: click.core.Context,
             verbose: bool) -> None:
    """
    Read an OSF file and print messages type, timestamp and counts to stdout.
    Useful to check chunks layout and decoding of all known messages (-d option).
    """
    file = ctx.source_uri or ""

    from ouster.sdk._bindings.osf import LidarScanStream
    import os

    reader = osf.Reader(file)

    orig_layout = "Streaming" if reader.has_stream_info else "Standard"
    if orig_layout == "Streaming" and reader.has_message_idx:
        orig_layout = "Streaming, Indexed"

    # count of messages in each stream
    lidar_streams: Dict[str, Dict[str, Any]]
    lidar_streams = {}
    other_streams: Dict[str, Dict[str, Any]]
    other_streams = {}

    start = 0
    end = 0
    count = 0
    size = os.path.getsize(file)

    sensors = {}
    msensors = reader.meta_store.find(osf.LidarSensor)
    for sensor_id, sensor_meta in msensors.items():
        info = sensor_meta.info
        sensors[sensor_id] = info

    # if we have StreamingStats we can just use those instead of scanning through the file
    obj: Dict[str, Any]
    mstats = reader.meta_store.find(osf.StreamingInfo)
    for sensor_id, meta in mstats.items():
        for id, stats in meta.stream_stats:
            parent = reader.meta_store[id]
            count += stats.message_count
            obj = {}
            obj["count"] = stats.message_count
            obj["start"] = stats.start_ts
            obj["end"] = stats.end_ts
            if not hasattr(parent, "sensor_meta_id"):
                obj["type"] = reader.meta_store[id].type
                other_streams[id] = obj
                continue
            sensor_id = parent.sensor_meta_id
            obj["type"] = reader.meta_store[sensor_id].type
            obj["fields"] = []
            obj["sensor"] = sensors[sensor_id]
            lidar_streams[sensor_id] = obj

            if start == 0:
                start = stats.start_ts
            else:
                start = min(stats.start_ts, start)
            end = max(stats.end_ts, end)

            if obj["type"] == "ouster/v1/os_sensor/LidarSensor":
                # get fields of the first scan
                for msg in reader.messages([id], stats.start_ts, stats.start_ts):
                    if msg.of(LidarScanStream):
                        ls = msg.decode()
                        obj["fields"] = ls.field_types

    # fallback for when there's no index
    for msg in reader.messages():
        if reader.has_message_idx:
            break
        count = count + 1
        if start == 0:
            start = msg.ts
        else:
            start = min(msg.ts, start)
        end = max(msg.ts, end)
        if not msg.of(LidarScanStream):
            if msg.id not in other_streams:
                obj = {}
                obj["count"] = 1
                obj["start"] = obj["end"] = msg.ts
                obj["type"] = reader.meta_store[msg.id].type
                other_streams[msg.id] = obj
            else:
                obj = other_streams[msg.id]
                obj["count"] = obj["count"] + 1
                obj["end"] = max(obj["end"], msg.ts)
        else:
            ls = msg.decode()
            if ls:
                if msg.id not in lidar_streams:
                    # get sensor id
                    obj = {}
                    obj["count"] = 1
                    obj["start"] = msg.ts
                    obj["end"] = msg.ts
                    obj["type"] = reader.meta_store[msg.id].type
                    obj["fields"] = ls.field_types
                    obj["sensor"] = sensors[reader.meta_store[msg.id].sensor_meta_id]
                    lidar_streams[msg.id] = obj
                else:
                    obj = lidar_streams[msg.id]
                    obj["count"] = obj["count"] + 1
                    obj["end"] = max(obj["end"], msg.ts)
                    if obj["fields"] is not None and ls.field_types != obj["fields"]:
                        print("WARNING: fields changed for a sensor over the course of the file!")
                        obj["fields"] = None

    print(f"Filename: {file}\nLayout: {orig_layout}")
    print(f"OSF Version: {reader.version}")
    print(f"Metadata ID: '{reader.metadata_id}'")
    print(f"Size: {size/1000000} MB")
    print(f"Start: {start/1000000000.0} ({_nanos_to_string(start)})")
    print(f"End: {end/1000000000.0} ({_nanos_to_string(end)})")
    print(f"Duration: {(end-start)/1000000000.0}")
    print(f"Messages: {count}\n")

    # print out info about each stream
    for k in lidar_streams:
        stream = lidar_streams[k]
        count = stream["count"]
        start = stream['start'] / 1000000000.0
        end = stream['end'] / 1000000000.0
        sensor = stream["sensor"]
        print(f"Stream {k} {stream['type']}: ")
        print(f"  Scan Count: {count}")
        print(f"  Start: {start} ({_nanos_to_string(stream['start'])})")
        print(f"  End: {end} ({_nanos_to_string(stream['end'])})")
        print(f"  Duration: {end-start} seconds")
        if end == start:
            print("  Rate: N/A")
        else:
            print(f"  Rate: {count/(end-start)} Hz")
        print(f"  Product Line: {sensor.prod_line}")
        print(f"  Sensor Mode: {sensor.config.lidar_mode}")
        if verbose:
            print(f"  Sensor SN: {sensor.sn}")
            print(f"  Sensor FW Rev: {sensor.fw_rev}")
            print("  Fields:")
            if stream["fields"] is None:
                print("  NO CONSISTENT FIELD TYPE")
            else:
                for f in stream["fields"]:
                    print(f"    {f}")

    for k in other_streams:
        stream = other_streams[k]
        count = stream["count"]
        start = stream['start'] / 1000000000.0
        end = stream['end'] / 1000000000.0
        print(f"Stream {k} {stream['type']}: ")
        print(f"  Message Count: {count}")
        print(f"  Start: {start} ({_nanos_to_string(stream['start'])})")
        print(f"  End: {end} ({_nanos_to_string(stream['end'])})")
        print(f"  Duration: {end-start} seconds")
        print(f"  Rate: {count/(end-start)} Hz")


@click.command
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
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def osf_parse(ctx: SourceCommandContext, click_ctx: click.core.Context,
              decode: bool, verbose: bool, check_raw_headers: bool,
              standard: bool) -> None:
    """
    Read an OSF file and print messages type, timestamp and counts to stdout.
    Useful to check chunks layout and decoding of all known messages (-d option).
    """
    file = ctx.source_uri or ""

    # NOTE[pb]: Mypy quirks or some of our Python packages structure quirks, idk :(
    from ouster.sdk.util.parsing import scan_to_packets, packets_to_scan, cut_raw32_words  # type: ignore

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
                    ls = cast(core.LidarScan, obj)

                    d = d + \
                        (" [poses: YES]" if core.poses_present(ls) else "")

                    if verbose:
                        verbose_str += f"{ls}"

                    if check_raw_headers:
                        d = d + " " if d else ""
                        if core.ChanField.RAW_HEADERS in ls.fields:
                            sinfo = scan_stream_sensor[m.id].info

                            # roundtrip: LidarScan -> packets -> LidarScan
                            packets = scan_to_packets(ls, sinfo)

                            # recovered lidar scan
                            field_types = ls.field_types
                            ls_rec = packets_to_scan(
                                packets, sinfo, fields=field_types)

                            ls_no_raw32 = cut_raw32_words(ls)
                            ls_rec_no_raw32 = cut_raw32_words(ls_rec)

                            assert ls_rec_no_raw32 == ls_no_raw32, "LidarScan should be" \
                                " equal when recontructed from RAW_HEADERS fields" \
                                " packets back"

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
