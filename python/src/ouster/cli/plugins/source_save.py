from ouster.cli.core.cli_args import CliArgs
import atexit
import click
import os
import time
from collections import deque
from datetime import datetime
from pathlib import Path
import numpy as np
from typing import (cast, Dict, Union, Tuple, List, Iterator, Iterable, Optional, Set)
from ouster.cli.core import SourceArgsException  # type: ignore[attr-defined]
from ouster.sdk.core import (LidarFrame, FrameSet, ChanField, XYZLut,
                             destagger, SensorInfo, LidarPacket, ImuPacket,
                             PacketSource, ZonePacket, UDPProfileIMU, FieldClass)
from ouster.sdk import osf, open_packet_source
from ouster.sdk.core import (io_type_from_extension, OusterIoType)
from ouster.sdk.osf import OsfDropFrameError
from ouster.sdk.util import frame_to_packets  # type: ignore
from ouster.sdk.pcap.pcap import MTU_SIZE
import ouster.sdk._bindings.pcap as _pcap
from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand,
                          _join_with_conjunction)
from .source_bag import _source_to_bag_iter  # type: ignore[attr-defined]


_file_exists_error = lambda filename: (f"Error: File '{filename}' already exists. Add --overwrite "
                                       "flag to overwrite and continue anyways.")


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.option('--duration', '-D', default=None, type=float, help="Duration to record")
@click.option('--ros2', is_flag=True, default=False, help="If true, save this as a ROS2 bag file.")
@click.option("--split", default=None, type=int,
              help="Split recordings when they approximately surpass this size in megabytes.")
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED)
def source_save_raw(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                    overwrite: bool, duration: Optional[float], ros2: bool,
                    split: Optional[int], **kwargs) -> None:
    """ Save raw packets from the source. Does not support chaining. Supports Bag and PCAP formats."""
    uri = ctx.source_uri
    if uri is None:
        raise RuntimeError("Unexpected condition")
    source_list = [url.strip() for url in uri.split(',') if url.strip()]

    extension = ""
    splitfn = os.path.splitext(filename)
    if splitfn[0][0] == '.' and splitfn[1] == "":
        extension = splitfn[0].replace(".", "")
        filename = ""
    elif splitfn[1] == "":
        click.echo("Error: Must provide a filename with an extension.")
        exit(2)
    else:
        extension = splitfn[1].replace(".", "")

    if extension != "pcap" and extension != "bag":
        raise click.exceptions.BadParameter(f"Cannot save raw file of type {extension}")

    # Finally open the appropriate packet source
    packets: PacketSource
    packets = open_packet_source(source_list, **ctx.source_options)  # type: ignore

    if extension == "pcap":
        save_pcap_impl(packets, filename, prefix, dir, raw=True, overwrite=overwrite,
                       metadata=packets.sensor_info, duration=duration, split=split)
    elif extension == "bag":
        _source_to_bag_iter(packets, raw=True, metadata=packets.sensor_info,
                           prefix=prefix, dir=dir, filename=filename,
                           overwrite=overwrite, duration=duration, ros2=ros2, split=split)


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.option("--split", default=None, type=int,
              help="Split recordings when they approximately surpass this size in megabytes.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_pcap(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                     overwrite: bool, split: Optional[int], **kwargs) -> None:
    """Save source as a PCAP"""
    if ctx.frame_set_iter is None or ctx.frame_set_source is None:
        raise RuntimeError("unexpected condition")

    ctx.frame_set_iter = save_pcap_impl(ctx.frame_set_iter, filename, prefix, dir, False,
                                   overwrite, ctx.frame_set_source.sensor_info, split=split)


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('-c', '--continue-anyways', is_flag=True, default=False, help="Continue saving "
              "frames after an error is encountered, dropping bad data if necessary.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.option("--ts", default='packet', help="Timestamp to use for indexing.", type=click.Choice(['packet', 'lidar']))
@click.option("--compression-level", default=1, help="Specifies the level of compression for OSF files. Higher values "
    "are slower but more space-efficient; lower values are faster but less space-efficient.", type=click.IntRange(0, 9))
@click.option("--png", is_flag=True, default=False, help="Save using PNG compression instead of the default ZPNG. "
              "See --legacy for older SDK compatibility.")
@click.option("--legacy", is_flag=True, default=False, help="Save in a format compatible with older SDKs (0.12-0.15). "
              "Uses PNG compression and drops unsupported field types (e.g. CHAR, ZONE_STATE).")
@click.option("--split", default=None, type=int,
              help="Split recordings when they approximately surpass this size in megabytes.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_osf(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                    overwrite: bool, ts: str, continue_anyways: bool, compression_level: int,
                    png: bool, legacy: bool, split: Optional[int], **kwargs) -> None:
    """Save source as an OSF"""
    from ouster.sdk.core import FrameSetSourceMetadataSet
    if ctx.frame_set_source is None:
        raise RuntimeError("Internal error: expected frame source to be set in context")
    frames = ctx.frame_set_iter
    info = ctx.frame_set_source.sensor_info

    # Automatic file naming
    filename = determine_filename(filename=filename, info=info[0], extension=".osf", prefix=prefix, dir=dir)
    originalfilename = filename
    create_directories_if_missing(filename)

    click.echo(f"Saving OSF file at {filename}")

    if os.path.isfile(filename) and not overwrite:
        click.echo(_file_exists_error(filename))
        exit(1)

    # Initialize osf writer
    # --legacy implies PNG compression
    if png or legacy:
        encoder = osf.Encoder(osf.PngLidarFrameEncoder(compression_level))
    else:
        encoder = osf.Encoder(osf.ZPngLidarFrameEncoder(compression_level))

    osf_writer = osf.AsyncWriter(filename, info, [], 0, encoder)

    # Copy any original frame source metadata entries to the new file
    metadata_copy = FrameSetSourceMetadataSet()
    for key in ctx.frame_set_source.metadata_keys():
        metadata_copy[key] = ctx.frame_set_source.metadata(key)
    osf_writer.save(metadata_copy)

    wrote_frames = False
    dropped_frames = 0
    file_number = 1
    dropped_fields: Set[str] = set()

    use_packet_timestamps = (ts == "packet")
    if not use_packet_timestamps:
        print("WARNING: Saving OSF with lidar timestamps. This option should only be used"
              " to salvage legacy osf recordings with missing packet timestamps.")

    # returns false if we should stop recording
    need_split = False

    # Standard numeric dtype kinds supported by older SDK versions (0.12-0.15)
    LEGACY_DTYPE_KINDS = ("u", "i", "f")  # unsigned int, signed int, float

    future_queue: deque = deque()

    def write_osf(frames: FrameSet):
        nonlocal wrote_frames, dropped_frames, osf_writer, filename, file_number, need_split

        # `save --legacy` is for backwards compatibility with SDK 0.12-0.15.
        # Older versions only support standard numeric ChanFieldTypes (UINT*, INT*, FLOAT*).
        # Drop fields with newer types like CHAR (kind 'S') or ZONE_STATE (kind 'V').
        if legacy:
            for frame in frames:
                if frame is None:
                    continue

                for ft in frame.field_types:
                    if np.dtype(ft.element_type).kind in LEGACY_DTYPE_KINDS:
                        continue

                    name = ft.name
                    if frame.has_field(name):
                        frame.del_field(name)
                        dropped_fields.add(name)

        if need_split:
            need_split = False
            osf_writer.close()
            filename = originalfilename.replace(".osf", "") + f"-{file_number}.osf"
            print(f"Splitting into {filename}")

            if os.path.isfile(filename) and not overwrite:
                click.echo(_file_exists_error(filename))
                exit(1)
            file_number += 1
            osf_writer = osf.AsyncWriter(filename, info, [], 0, encoder)

        try:
            if use_packet_timestamps:
                if ctx.save_collations:
                    future_queue.append(osf_writer.save(frames))
                else:
                    for idx, frame in enumerate(frames):
                        if frame is not None:
                            future_queue.append(osf_writer.save(idx, frame))
                wrote_frames = True
            else:
                # handling lidar timestamps manually
                for index, frame in enumerate(frames):
                    if frame is None:
                        continue
                    else:
                        try:
                            ts = frame.timestamp[frame.get_first_valid_column()]
                        except RuntimeError:
                            if continue_anyways:
                                dropped_frames = dropped_frames + 1
                                continue
                            raise OsfDropFrameError(
                                "Frame has no valid columns") from None
                        if ts == 0:
                            if continue_anyways:
                                dropped_frames = dropped_frames + 1
                            else:
                                raise OsfDropFrameError("Lidar timestamps are zero")
                        else:
                            future_queue.append(osf_writer.save(index, frame, ts))
                            wrote_frames = True

            # check the future queue for any exceptions while saving
            while len(future_queue):
                item = future_queue[0]
                if not item.done():
                    break
                future_queue.popleft()
                item.get()
        except OsfDropFrameError as ex:
            if continue_anyways:
                dropped_frames = dropped_frames + 1
            else:
                osf_writer.close()
                # delete the file if no frames were saved
                if not wrote_frames:
                    os.remove(filename)
                print("ERROR: Cannot save frames because of the following error:")
                print(str(ex))
                print("Try with `--ts lidar` instead or `-c` to continue anyways.")
                return False

        if split is not None:
            if os.path.getsize(filename) / 1000000 > split:
                need_split = True
        return True

    saved = False

    def save_iter():
        nonlocal saved
        try:
            # only save the first loop
            if saved:
                for s in frames():
                    yield s
                return
            for s in frames():
                if not write_osf(s):
                    ctx.terminate_evt.set()
                    return
                yield s
        except (KeyboardInterrupt):
            pass
        finally:
            saved = True
            osf_writer.close()
            if legacy and dropped_fields:
                click.echo(
                    "NOTE: Dropping fields with non-numeric types for backwards compatibility "
                    f"(--legacy): fields={sorted(dropped_fields)}"
                )

    ctx.frame_set_iter = save_iter  # type: ignore

    def handle_termination():
        osf_writer.close()
        if dropped_frames > 0:
            if ts == "lidar":
                click.echo(f"WARNING: Dropped {dropped_frames} frames because missing or decreasing timestamps.")
            else:
                click.echo(f"WARNING: Dropped {dropped_frames} frames because missing or decreasing "
                           "packet timestamps. Try with `--ts lidar` instead.")
        if not wrote_frames:
            click.echo("WARNING: No frames saved.")
    atexit.register(handle_termination)


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_csv(ctx: SourceCommandContext, prefix: str,
                    dir: str, filename: str, overwrite: bool, **kwargs) -> None:
    """Save source as one CSV file per LidarFrame."""
    ctx.frame_set_iter = source_to_csv_iter(ctx.frame_set_iter, ctx.frame_set_source.sensor_info,  # type: ignore
                                       prefix=prefix, dir=dir, filename=filename,
                                       overwrite=overwrite)


# [doc-stag-pcap-to-csv]
def source_to_csv_iter(frame_set_iter: Iterator[FrameSet], infos: List[SensorInfo],
                       prefix: str = "", dir: str = "", overwrite: bool = True,
                       filename: str = "") -> Iterable[FrameSet]:
    """Create a CSV saving iterator from a LidarFrame iterator

    The number of saved lines per csv file is always H x W, which corresponds to
    a full 2D image representation of a lidar frame.

    Pixel fields with shape (H, W) are exported as one column each. Multi-channel
    pixel fields (e.g. with shape (H, W, 3)) are exported with one column per
    channel: field_R, field_G, field_B.

    Each line in a csv file is (for DUAL profile):

        TIMESTAMP (ns), ROW, DESTAGGERED IMAGE COLUMN, MEASUREMENT_ID,
        B_ATTENUATED, B_CLEAR, G_ATTENUATED, G_CLEAR, NEAR_IR (photons),
        RANGE (mm), RANGE2 (mm), SIGNAL (photons),
        SIGNAL2 (photons), REFLECTIVITY (%), REFLECTIVITY2 (%),
        R_ATTENUATED, R_CLEAR, NEAR_IR (photons),
        field_R, field_G, field_B, X (m), Y (m), Z (m), X2 (m), Y2 (m), Z2(m)
    """
    for info in infos:
        if info.num_returns > 1:
            print("Note: You've selected to convert a dual returns pcap to CSV. Each row "
                  "will represent a single pixel, so that both returns for that pixel will "
                  "be on a single row. As this is an example we provide for getting "
                  "started, we realize that you may have conversion needs which are not met "
                  "by this function. You can find the source code on the Python SDK "
                  "documentation website to modify it for your own needs.")
            break

    # Build filenames
    filenames = []
    for info in infos:
        name = determine_filename(filename=filename, info=info, extension=".csv", prefix=prefix, dir=dir)

        name = name[0:-4]  # remove extension
        filenames.append(name)
        click.echo(f"Saving CSV files at {name}_XXX.csv")

    create_directories_if_missing(filenames[0])

    # Construct csv header and data format
    def get_fields_info(frame: LidarFrame) -> Tuple[str, List[str]]:
        field_names = 'TIMESTAMP (ns), ROW, DESTAGGERED IMAGE COLUMN, MEASUREMENT_ID'
        field_fmts = ['%d'] * 4
        dual = ChanField.RANGE2 in frame.fields
        for chan_field in frame.fields:
            if frame.field_class(chan_field) != FieldClass.PIXEL_FIELD:
                continue
            arr = frame.field(chan_field)
            ndim = arr.ndim
            pixel_fmt = '%.4f' if np.issubdtype(arr.dtype, np.floating) else '%d'
            if ndim == 2:
                field_names += f', {chan_field}'
                if chan_field in [ChanField.RANGE, ChanField.RANGE2]:
                    field_names += ' (mm)'
                if chan_field in [ChanField.REFLECTIVITY, ChanField.REFLECTIVITY2]:
                    field_names += ' (%)'
                if chan_field in [ChanField.SIGNAL, ChanField.SIGNAL2,
                        ChanField.NEAR_IR]:
                    field_names += ' (photons)'
                field_fmts.append(pixel_fmt)
            elif ndim == 3:
                n_channels = arr.shape[2]
                if n_channels == 1:
                    suffixes: Tuple[str, ...] = ('',)
                elif n_channels == 3:
                    if chan_field in [ChanField.NORMALS, ChanField.NORMALS2]:
                        suffixes = ('_X', '_Y', '_Z')
                    else:
                        suffixes = ('_R', '_G', '_B')
                else:
                    suffixes = tuple(f'_{i}' for i in range(n_channels))
                for suf in suffixes:
                    field_names += f', {chan_field}{suf}'
                    field_fmts.append(pixel_fmt)
            else:
                continue
        field_names += ', X1 (m), Y1 (m), Z1 (m)'
        field_fmts.extend(3 * ['%.4f'])
        if dual:
            field_names += ', X2 (m), Y2 (m), Z2 (m)'
            field_fmts.extend(3 * ['%.4f'])
        return field_names, field_fmts

    field_names: Dict[int, str] = {}
    field_fmts: Dict[int, List[str]] = {}

    # {recompute xyzlut to save computation in a loop
    xyzlut = []
    row_layer = []
    column_layer_staggered = []
    for info in infos:
        xyzlut.append(XYZLut(info, False))

        row_layer.append(np.fromfunction(lambda i, j: i,
                (info.format.pixels_per_column,
                    info.format.columns_per_frame), dtype=int))
        column_layer = np.fromfunction(lambda i, j: j,
                (info.format.pixels_per_column,
                    info.format.columns_per_frame), dtype=int)
        column_layer_staggered.append(destagger(info, column_layer,
                inverse=True))

    saved = False

    def save_iter():
        nonlocal saved
        try:
            if saved:
                for frame in frame_set_iter():
                    yield frame
                return
            for idx, frames in enumerate(frame_set_iter()):
                for lidar_idx, frame in enumerate(frames):
                    if frame is None:
                        continue

                    # Initialize the field names for csv header
                    if lidar_idx not in field_names or lidar_idx not in field_fmts:
                        field_names[lidar_idx], field_fmts[lidar_idx] = get_fields_info(frame)

                    # Copy per-column timestamps and measurement_ids for each beam
                    timestamps = np.tile(frame.timestamp, (frame.h, 1))
                    measurement_ids = np.tile(frame.measurement_id, (frame.h, 1))

                    # Keep only per-pixel fields for CSV stacking.
                    csv_fields = [field for field in frame.fields
                                 if frame.field_class(field) == FieldClass.PIXEL_FIELD]

                    # Grab channel data
                    fields_values = []
                    for ch in csv_fields:
                        arr = frame.field(ch)
                        if arr.ndim == 2:
                            fields_values.append(arr)
                        elif arr.ndim == 3:
                            for c in range(arr.shape[2]):
                                fields_values.append(arr[:, :, c])
                        else:
                            continue

                    lidar_frame = frame
                    frame = np.dstack((timestamps, row_layer[lidar_idx], column_layer_staggered[lidar_idx],
                        measurement_ids, *fields_values))

                    # Output points in "image" vs. staggered order
                    frame = destagger(info, frame)

                    # Destagger XYZ separately since it has a different type
                    xyz = xyzlut[lidar_idx](lidar_frame.field(ChanField.RANGE))
                    xyz_destaggered = destagger(info, xyz)

                    if ChanField.RANGE2 in csv_fields:
                        xyz2 = xyzlut[lidar_idx](lidar_frame.field(ChanField.RANGE2))
                        xyz2_destaggered = destagger(info, xyz2)

                        # Get all data as one H x W x num fields int64 array for savetxt()
                        frame = np.dstack(tuple(map(lambda x: x.astype(object),
                            (frame, xyz_destaggered, xyz2_destaggered))))
                    else:
                        # Get all data as one H x W x num fields int64 array for savetxt()
                        frame = np.dstack(tuple(map(lambda x: x.astype(object),
                            (frame, xyz_destaggered))))

                    frame_colmajor = np.swapaxes(frame, 0, 1)

                    # Write csv out to file
                    csv_path = f"{filenames[lidar_idx]}_s{lidar_idx}_{idx}.csv"
                    print(f'write frame index #{idx} sensor_idx #{lidar_idx}, to file: {csv_path}')

                    if os.path.isfile(csv_path) and not overwrite:
                        print(_file_exists_error(csv_path))
                        exit(1)

                    header = '\n'.join([f'frame num: {idx}', field_names[lidar_idx]])

                    np.savetxt(csv_path,
                            frame_colmajor.reshape(-1, frame.shape[2]),
                            fmt=field_fmts[lidar_idx],
                            delimiter=',',
                            header=header)

                yield frame
        except (KeyboardInterrupt, StopIteration):
            pass
        finally:
            saved = True

    # type ignored because generators are tricky to mypy
    return save_iter  # type: ignore
# [doc-etag-pcap-to-csv]


@click.command(context_settings=dict(
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_png(ctx: SourceCommandContext, prefix: str, dir: str,
                    filename: str, overwrite: bool, **kwargs) -> None:
    """Save frame source as a series of png files per LidarFrame per field (represented as 8-bit)."""
    from PIL import Image
    from ouster.sdk.core.data import destagger
    frame_set_iter = ctx.frame_set_iter
    filename_no_ext = os.path.splitext(filename)[0]

    def normalize(image):
        min_val = np.min(image)
        max_val = np.max(image)
        if max_val == min_val:
            return np.zeros_like(image)
        return (image - min_val) / (max_val - min_val)

    def compose_path(frame: LidarFrame, dir: str, prefix: str, field_name: str) -> str:
        output_path = f"{frame.sensor_info.sn}_{frame.frame_id}_{field_name}.png"
        if filename_no_ext:
            output_path = f"{filename_no_ext}_{output_path}"
        if prefix:
            output_path = f"{prefix}_{output_path}"
        if dir:
            output_path = os.path.join(dir, output_path)
        return output_path

    def save_field(frame: LidarFrame, f: str):
        field_data = frame.field(f)
        img = destagger(frame.sensor_info, field_data)
        img = normalize(img) * (2**8 - 1)
        img = np.nan_to_num(img, nan=0)
        img = img.astype(np.uint8)
        pil_img = Image.fromarray(img)
        output_path = compose_path(frame, dir, prefix, f)
        create_directories_if_missing(output_path)
        if os.path.isfile(output_path) and not overwrite:
            print(_file_exists_error(output_path))
            exit(1)
        pil_img.save(output_path)

    def png_save_iter():
        for frames in frame_set_iter():
            for frame in frames:
                if frame:
                    for f in frame.fields:
                        save_field(frame, f)
            yield frames

    ctx.frame_set_iter = png_save_iter    # type: ignore


# Determines the filename to use
def determine_filename(prefix: str, dir: str, filename: str, extension: str, info: SensorInfo):
    outpath = Path.cwd()
    if dir:
        outpath = Path(dir)

    time_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = f"{prefix}_" if prefix else prefix

    if filename != "":
        filename = str(outpath / f"{prefix}{filename}")
    else:
        filename = str(outpath / f"{prefix}{info.prod_line}_{info.fw_rev}_"
                       f"{info.config.lidar_mode}_{time_str}{extension}")

    return filename


# Creates path to file if any folders in the chain are missing
def create_directories_if_missing(filename: str):
    outpath = Path(filename).parents[0]
    if not outpath.is_dir():
        outpath.mkdir(parents=True)


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite existing files with the same name.")
@click.option('--ros2', is_flag=True, default=False, help="If true, save this as a ROS2 bag file.")
@click.option("--split", default=None, type=int,
              help="Split recordings when they approximately surpass this size in megabytes.")
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def source_save_bag(ctx: SourceCommandContext, prefix: str, dir: str, filename: str,
                    overwrite: bool, ros2: bool, split: Optional[int], **kwargs) -> None:
    """Save source as a packet rosbag."""
    if ctx.frame_set_iter is None or ctx.frame_set_source is None:
        raise RuntimeError("unexpected condition")

    ctx.frame_set_iter = _source_to_bag_iter(ctx.frame_set_iter, raw=False, split=split,
                                        prefix=prefix, dir=dir, filename=filename,
                                        overwrite=overwrite, metadata=ctx.frame_set_source.sensor_info,
                                        ros2=ros2)


def save_pcap_impl(source: Union[Iterable[FrameSet], PacketSource],
                   filename, prefix, dir, raw, overwrite, metadata, duration = None, split = None):
    # Automatic file naming
    filename = determine_filename(filename=filename, info=metadata[0], extension=".pcap", prefix=prefix, dir=dir)

    create_directories_if_missing(filename)

    filename = filename[0:-5]  # remove extension
    file_number = 1

    # check for existing files
    pcap_filename = f"{filename}.pcap"
    original_filename = pcap_filename
    if os.path.isfile(pcap_filename) and not overwrite:
        click.echo(_file_exists_error(pcap_filename))
        exit(1)

    for i, md in enumerate(metadata):
        fn = f"{filename}_{i}.json"
        if os.path.isfile(fn) and not overwrite:
            click.echo(_file_exists_error(fn))
            exit(1)

    # Save metadata as json
    for i, md in enumerate(metadata):
        fn = f"{filename}_{i}.json"
        with open(fn, 'w') as f:
            click.echo(f"Saving metadata json at {fn}")
            f.write(md.to_json_string())

    click.echo(f"Saving pcap file at {pcap_filename}")

    # Initialize pcap writer
    pcap_record_handle = _pcap.record_initialize(pcap_filename, MTU_SIZE, False)

    def save_packet(idx, packet, port):
        ts = packet.host_timestamp / 1e9 if packet.host_timestamp else time.time()
        _pcap.record_packet(pcap_record_handle, "127.0.0.1",
                            "127.0.0.1", port, port, packet.buf, ts)

    def check_split():
        if split is not None:
            if os.path.getsize(pcap_filename) / 1000000 > split:
                return True
        return False

    def do_split():
        nonlocal pcap_record_handle, pcap_filename, file_number
        pcap_filename = original_filename.replace(".pcap", "") + f"-{file_number}.pcap"
        file_number += 1
        print(f"Splitting into {pcap_filename}")

        if os.path.isfile(pcap_filename) and not overwrite:
            click.echo(_file_exists_error(pcap_filename))
            exit(1)

        _pcap.record_uninitialize(pcap_record_handle)
        pcap_record_handle = _pcap.record_initialize(pcap_filename, MTU_SIZE, False)

    if raw:
        end_time = None
        try:
            source = cast(PacketSource, source)
            last_frame_id = -1
            for idx, packet in source:
                if isinstance(packet, LidarPacket):
                    pfid = packet.frame_id()
                    if pfid != last_frame_id and last_frame_id != -1:
                        if check_split():
                            do_split()
                    last_frame_id = pfid
                    save_packet(idx, packet, metadata[idx].config.udp_port_lidar)
                elif isinstance(packet, ImuPacket):
                    save_packet(idx, packet, metadata[idx].config.udp_port_imu)
                elif isinstance(packet, ZonePacket):
                    save_packet(idx, packet, metadata[idx].config.udp_port_zm)

                if duration is not None:
                    if end_time is None:
                        end_time = packet.host_timestamp + duration * 1e9
                    if packet.host_timestamp > end_time:
                        break
        except (KeyboardInterrupt, StopIteration):
            pass
        finally:
            # Finish pcap_recording when this generator is garbage collected
            _pcap.record_uninitialize(pcap_record_handle)
    else:
        for meta in metadata:
            if meta.config.udp_profile_imu == UDPProfileIMU.LEGACY:
                click.echo("Warning: Saving pcap without save_raw will not save LEGACY IMU packets.")
                break

        def save_iter():
            nonlocal pcap_record_handle
            try:
                # only save the first loop
                if pcap_record_handle is None:
                    for c in source():
                        yield c
                    return
                should_split = False
                for c in source():
                    for idx, frame in enumerate(c):
                        if frame is not None:
                            if should_split:
                                do_split()
                                should_split = False

                            packets = frame_to_packets(frame, metadata[idx])
                            for packet in packets:
                                if isinstance(packet, LidarPacket):
                                    save_packet(idx, packet, metadata[idx].config.udp_port_lidar)
                                elif isinstance(packet, ImuPacket):
                                    save_packet(idx, packet, metadata[idx].config.udp_port_imu)
                                elif isinstance(packet, ZonePacket):
                                    save_packet(idx, packet, metadata[idx].config.udp_port_zm)

                            # only split after saving a whole frame to prevent a frame
                            # ending up between two files
                            should_split = check_split()
                    yield c
            except (KeyboardInterrupt, StopIteration):
                pass
            finally:
                # Finish pcap_recording when this generator is garbage collected
                if pcap_record_handle is not None:
                    _pcap.record_uninitialize(pcap_record_handle)
                    pcap_record_handle = None

        return save_iter


class SourceSaveCommand(click.Command):
    """Generalizes ouster-cli source <> save <outputfile>
    """

    # Map from output type to a save implementation function
    implementations = {
        OusterIoType.OSF: source_save_osf,
        OusterIoType.PCAP: source_save_pcap,
        OusterIoType.BAG: source_save_bag,
        OusterIoType.CSV: source_save_csv,
        OusterIoType.PNG: source_save_png
    }

    def __init__(self, *args, **kwargs):
        kwargs['add_help_option'] = True
        super().__init__(*args, **kwargs)
        self.update_help()
        self.update_params()

    def update_help(self):
        help_str = "Save to an "
        help_str += _join_with_conjunction([k.name.upper() for k in self.implementations.keys()])
        help_str += " with the given filename. If only an extension is provided, the file is named automatically."
        self.help = help_str

    def update_params(self):
        # Add click options/parameters from save implementation commands
        param_mapping = {}
        for (iotype, cmd) in self.implementations.items():
            for p in cmd.params:
                if p.name in param_mapping.keys():
                    param_mapping[p.name][1].append(iotype)
                else:
                    param_mapping[p.name] = (p, [iotype])

        # Prefix options/parameters with name of the output iotype
        self.params = []
        for (_, (param, iotypes)) in param_mapping.items():
            if len(iotypes) < len(self.implementations):
                help_prefix = "|".join([k.name.upper() for k in iotypes])
                help_prefix = f"[{help_prefix}]:"
                # Click calls this init function multiple times on --help.
                # Check that the help string has not already been prepended with param.help
                if help_prefix not in param.help:
                    param.help = f"{help_prefix} {param.help}"
            self.params.append(param)

    def get_help(self, *args, **kwargs):
        # Update help text to capture changes from lazily loaded save implementations
        self.update_help()
        return super().get_help(*args, **kwargs)

    def get_params(self, *args, **kwargs):
        # Update params to capture changes from lazily loaded save implementations
        self.update_params()
        return super().get_params(*args, **kwargs)

    def invoke(self, ctx, *args):
        output_name = ctx.params.get('filename')
        output_format = ""

        split = os.path.splitext(output_name)
        if split[0][0] == '.' and split[1] == "":
            output_format = split[0].replace(".", "")
            ctx.params["filename"] = ""
        elif split[1] == "":
            click.echo("Error: Must provide a filename with an extension.")
            exit(2)
        else:
            output_format = split[1].replace(".", "")

        # Ensure the file extension is present and a valid one
        supported_formats = [iotype.name.upper() for iotype in self.implementations.keys()]
        if output_format.upper() not in supported_formats:
            string = f"Error: Invalid file extension. '.{output_format.lower()}' is not one of "
            string += _join_with_conjunction([f".{x.lower()}" for x in supported_formats])

            click.echo(string + ".")
            exit(2)

        ctx.params["format"] = output_format
        output_type = io_type_from_extension(f" .{output_format}")
        convert_command = self.implementations[output_type]
        if CliArgs().has_any_of(ctx.help_option_names):
            click.echo(convert_command.get_help(ctx))
        else:
            try:
                return ctx.forward(convert_command)
            except TypeError:
                if len(ctx.args) > 0:
                    raise SourceArgsException(ctx)
