from typing import (IO, Any, Callable, Dict, Generator, Iterator, List,
                    Optional, Tuple, cast)

import os
import shutil
import tempfile
import click
import logging
import laspy
import numpy as np
from ouster.sdk.core import OusterIoType
from ouster.cli.plugins.source_save import (SourceSaveCommand,
                                            determine_filename,
                                            create_directories_if_missing,
                                            _file_exists_error)
from ouster.sdk.core import (LocalToneMapper,
                             first_valid_column_pose,
                             dewarp)
from ouster.cli.plugins.source_util import (source_multicommand,
                                            SourceCommandType,
                                            SourceCommandContext)

from ouster.sdk.core import VoxelHashMapXd


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mapping')


FieldInfo = Tuple[str, int]


def is_rgb_field(name: str) -> bool:
    return name.upper() == "RGB"


def is_normal_field(name: str) -> bool:
    return name.upper() == "NORMALS"


def ply_property_names(name: str, count: int) -> List[str]:
    if count == 1:
        return [name.lower()]
    if count == 3 and is_rgb_field(name):
        return ["red", "green", "blue"]
    if count == 3 and is_normal_field(name):
        return ["nx", "ny", "nz"]
    if count == 3:
        base = name.lower()
        return [f"{base}_x", f"{base}_y", f"{base}_z"]
    return [f"{name.lower()}_{i}" for i in range(count)]


def pcd_field_entries(name: str, count: int):
    if count == 1:
        return [name.lower()], [1]
    if count == 3 and is_rgb_field(name):
        # PCD stores rgb as one packed float field.
        return ["rgb"], [1]
    if count == 3 and is_normal_field(name):
        return ["normal_x", "normal_y", "normal_z"], [1, 1, 1]
    if count == 3:
        base = name.lower()
        return [f"{base}_x", f"{base}_y", f"{base}_z"], [1, 1, 1]
    base = name.lower()
    return [f"{base}_{i}" for i in range(count)], [1] * count


def pack_rgb_to_float_array(rgb: np.ndarray) -> np.ndarray:
    rgb_u8 = np.clip(rgb * 255.0, 0, 255).astype(np.uint32)
    rgb_u32 = (rgb_u8[:, 0] << 16) | (rgb_u8[:, 1] << 8) | rgb_u8[:, 2]
    return rgb_u32.view(np.float32)


def scale_rgb_to_uint16(rgb: np.ndarray) -> np.ndarray:
    """Convert normalized [0, 1] float RGB (as produced by the export
    pipeline) into the 16-bit-per-channel range LAS red/green/blue fields
    expect.
    """
    return np.clip(rgb * 65535.0, 0, 65535).astype(np.uint16)


def las_field_column_ranges(in_fields: List[FieldInfo],
                            start_col: int = 3) -> List[Tuple[str, int, int]]:
    """Returns (field_name, start_col, end_col) for each field in the cloud,
    where columns are laid out as XYZ followed by each field's attributes in
    order, matching how `in_fields` columns are concatenated when writing.
    """
    ranges = []
    col = start_col
    for name, count in in_fields:
        ranges.append((name, col, col + count))
        col += count
    return ranges


def pcd_output_cloud(cloud: np.ndarray, in_fields: List[FieldInfo]) -> np.ndarray:
    columns = [cloud[:, 0:3].astype(np.float32, copy=False)]
    idx = 3
    for name, count in in_fields:
        values = cloud[:, idx:idx + count]
        if count == 3 and is_rgb_field(name):
            columns.append(pack_rgb_to_float_array(values).reshape(-1, 1))
        else:
            columns.append(values.astype(np.float32, copy=False))
        idx += count
    return np.concatenate(columns, axis=1) if len(columns) > 1 else columns[0]


def ply_header(point_count: int, in_fields: List[FieldInfo], ascii: bool) -> str:
    header = [
        "ply",
        "format ascii 1.0" if ascii else "format binary_little_endian 1.0",
        f"element vertex {point_count}",
        "property float x",
        "property float y",
        "property float z",
    ]
    for name, count in in_fields:
        for property_name in ply_property_names(name, count):
            header.append(f"property float {property_name}")
    header.append("end_header")
    return "\n".join(header) + "\n"


def pcd_header(point_count: int, in_fields: List[FieldInfo],
               data_mode: str) -> str:
    fields = ["x", "y", "z"]
    sizes = [4, 4, 4]
    types = ["F", "F", "F"]
    counts = [1, 1, 1]

    for name, count in in_fields:
        field_names, field_counts = pcd_field_entries(name, count)
        fields.extend(field_names)
        for field_count in field_counts:
            sizes.append(4)
            types.append("F")
            counts.append(field_count)

    header = [
        f"FIELDS {' '.join(fields)}",
        f"SIZE {' '.join(map(str, sizes))}",
        f"TYPE {' '.join(types)}",
        f"COUNT {' '.join(map(str, counts))}",
        f"WIDTH {point_count}",
        "HEIGHT 1",
        f"POINTS {point_count}",
        f"DATA {data_mode}",
    ]
    return "\n".join(header) + "\n"


class StagedPointCloudWriter:
    """Stage point-cloud chunks while preserving final single-file headers."""

    def __init__(self, output_file_path: str, ascii: bool,
                 in_fields: List[FieldInfo], cols: int,
                 allow_empty_output: bool = False) -> None:
        self.output_file_path = output_file_path
        self.ascii = ascii
        self.in_fields = in_fields
        self.cols = cols
        self.allow_empty_output = allow_empty_output
        self.points_count = 0
        self._closed = False
        _, self.outfile_ext = os.path.splitext(output_file_path)
        output_dir = os.path.dirname(output_file_path) or None
        tmp = tempfile.NamedTemporaryFile(
            prefix=os.path.basename(output_file_path) + ".",
            suffix=".tmp",
            dir=output_dir,
            delete=False)
        self.tmp_path = tmp.name
        tmp.close()
        self._file: IO[Any]
        if ascii and self.outfile_ext in (".ply", ".pcd"):
            self._file = open(self.tmp_path, "w", encoding="utf-8")
        else:
            self._file = open(self.tmp_path, "wb")

    def write(self, cloud: np.ndarray) -> None:
        cloud = np.asarray(cloud, dtype=np.float64)
        if cloud.size == 0:
            return
        if cloud.ndim == 1:
            cloud = cloud.reshape(1, -1)
        if cloud.shape[1] != self.cols:
            raise RuntimeError(
                f"unexpected cloud width {cloud.shape[1]}, expected {self.cols}")

        if self.outfile_ext == ".pcd":
            out = pcd_output_cloud(cloud, self.in_fields)
        else:
            out = cloud.astype(np.float32, copy=False)

        if self.ascii and self.outfile_ext in (".ply", ".pcd"):
            format_string = " ".join(["{}"] * out.shape[1]) + "\n"
            for i in range(out.shape[0]):
                self._file.write(format_string.format(*out[i, :]))
        else:
            self._file.write(out.astype(np.float32, copy=False).tobytes())
        self.points_count += int(cloud.shape[0])

    def _close_temp(self) -> None:
        if not self._closed:
            self._file.close()
            self._closed = True

    def finalize(self) -> None:
        self._close_temp()
        if self.points_count == 0 and not self.allow_empty_output:
            return

        if self.outfile_ext == ".ply":
            if self.ascii:
                with open(self.output_file_path, "w", encoding="utf-8") as out:
                    out.write(ply_header(self.points_count, self.in_fields, self.ascii))
                    with open(self.tmp_path, "r", encoding="utf-8") as tmp:
                        shutil.copyfileobj(tmp, out)
            else:
                with open(self.output_file_path, "wb") as out:
                    out.write(ply_header(
                        self.points_count, self.in_fields, self.ascii).encode("ascii"))
                    with open(self.tmp_path, "rb") as tmp:
                        shutil.copyfileobj(tmp, out)
        elif self.outfile_ext == ".pcd":
            if self.ascii:
                with open(self.output_file_path, "w", encoding="utf-8") as out:
                    out.write(pcd_header(self.points_count, self.in_fields, "ascii"))
                    with open(self.tmp_path, "r", encoding="utf-8") as tmp:
                        shutil.copyfileobj(tmp, out)
            else:
                with open(self.output_file_path, "wb") as out:
                    out.write(pcd_header(
                        self.points_count, self.in_fields, "binary").encode("ascii"))
                    with open(self.tmp_path, "rb") as tmp:
                        shutil.copyfileobj(tmp, out)
        elif self.outfile_ext == ".las" or self.outfile_ext == ".laz":
            cloud = np.memmap(
                self.tmp_path, dtype=np.float32, mode="r",
                shape=(self.points_count, self.cols))
            # laspy's default point format (3) already includes intensity
            # and red/green/blue, so no explicit point_format is needed here.
            LAS_file = laspy.create()
            LAS_file.x = cloud[:, 0]
            LAS_file.y = cloud[:, 1]
            LAS_file.z = cloud[:, 2]
            for name, start, end in las_field_column_ranges(self.in_fields):
                if end - start == 1:
                    LAS_file.intensity = cloud[:, start]
                elif end - start == 3 and is_rgb_field(name):
                    rgb = scale_rgb_to_uint16(cloud[:, start:end])
                    LAS_file.red = rgb[:, 0]
                    LAS_file.green = rgb[:, 1]
                    LAS_file.blue = rgb[:, 2]
            try:
                LAS_file.write(self.output_file_path)
            except laspy.errors.LaspyException as e:
                raise ValueError(f"{e}; if trying to write a laz file make sure to"
                                 " pip install `lazrs` or `laszip` packages") from e

    def cleanup(self) -> None:
        self._close_temp()
        try:
            os.remove(self.tmp_path)
        except OSError:
            pass


def save_pointcloud(filename: str, cloud: np.ndarray, ascii: bool = False,
                    in_fields: List[FieldInfo] = []) -> None:
    """Save an Nx(3+attributes) cloud to PLY or PCD.

    `in_fields` describes the attribute columns after XYZ. Scalar fields map to
    one column. RGB and NORMALS map to three columns with format-specific
    names/packing.
    """
    cloud = np.asarray(cloud, dtype=np.float64)
    if cloud.ndim == 1:
        cloud = cloud.reshape(1, -1)

    writer = StagedPointCloudWriter(
        filename, ascii, in_fields, int(cloud.shape[1]), allow_empty_output=True)
    try:
        writer.write(cloud)
        writer.finalize()
    finally:
        writer.cleanup()


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', default="", help="Output directory.")
@click.option('-v', '--voxel-size', default=0.1, help="Voxel map size for downsampling"
              "The bigger the value, the fewer points it outputs", show_default=True)
@click.option('--max-points-per-voxel', default=10, type=int, help="Max points per voxel")
@click.option('--min-pts-threshold', default=1, type=int, help=("Minimum number of points in a"
                                   "voxel before it is considered in the output"), show_default=True)
@click.option('--field', multiple=True,
              default=["REFLECTIVITY"],
              help=("Chanfield for output pointcloud key values. Use NONE to omit."
                    " Examples include REFLECTIVITY, RANGE, SIGNAL, NEAR_IR."
                    " Can be applied multiple times to add more fields."),
              show_default=True)
@click.option('--decimate', type=bool,
              default=True, help="Downsample the point cloud to output.", show_default=True)
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite "
              "existing files with the same name")
@click.option('-f', '--pts-per-file', default=100000000, type=int, show_default=True,
              help="the number of points per output file.")
@click.option('--ascii', is_flag=True, default=False,
              help="Output files in ASCII rather than binary format")
@click.option('--trim-distance', default=25.0, type=float, show_default=True,
              help=("Distance the sensor pose must move before far voxels are "
                    "extracted from the in-memory voxel map and staged for output."))
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER)
def map_export(ctx: SourceCommandContext, filename: str, prefix: str,
               dir: str, voxel_size: float, max_points_per_voxel: int, min_pts_threshold: int, field: List[str],
               decimate: bool, overwrite: bool, pts_per_file: int, ascii: bool,
               trim_distance: float, **kwargs: Any) -> None:

    if ctx.frame_set_iter is None:
        raise RuntimeError("frame_set_iter is not initialized")
    frames_iter = cast(Callable[[], Iterator[Any]], ctx.frame_set_iter)

    if ctx.frame_set_source is not None:
        infos = ctx.frame_set_source.sensor_info

    outfile_ext = "." + kwargs["format"]
    output_file_path = determine_filename(prefix, dir, filename,
                                          outfile_ext, infos[0])
    create_directories_if_missing(output_file_path)

    # files pre exist check step
    file_wo_ext, outfile_ext = os.path.splitext(output_file_path)
    may_existed_file = file_wo_ext + '-000' + outfile_ext

    if (os.path.isfile(output_file_path) or os.path.isfile(may_existed_file)) and not overwrite:
        print(_file_exists_error(f'{output_file_path} or {may_existed_file}'))
        exit(1)

    already_saved = False
    empty_pose = True

    # Validate there are no duplicate fields and ignore NONE fields
    raw_fields = field
    fields = []
    for name in raw_fields:
        field_name = name.upper()
        if field_name in fields:
            print(f"ERROR: Requested to save duplicate field '{field_name}'.")
            exit(1)
        if field_name == "NONE":
            continue
        fields.append(field_name)

    def map_export_iter() -> Generator[Any, None, None]:
        nonlocal already_saved, empty_pose
        failed = False
        first = True
        field_info: List[FieldInfo] = []
        attribute_count = 0
        voxel_hash_map: Optional[VoxelHashMapXd] = None
        chunk_writer: Optional[StagedPointCloudWriter] = None
        last_trim_position = None
        points_sum = 0
        points_out_range = 0
        points_saved_total = 0
        # Per-field tone mapper state so float16 RGB data is normalized to
        # [0, 1] before downsampling.
        tone_mappers: Dict[str, LocalToneMapper] = {}
        # File rolling when --pts-per-file is exceeded. file_index counts
        # how many chunk files we've finalized; the on-disk filename is
        # composed below.
        file_index = 0

        def open_chunk_writer() -> StagedPointCloudWriter:
            return StagedPointCloudWriter(
                rolling_output_path(), ascii, field_info, 3 + attribute_count)

        def rolling_output_path() -> str:
            # Match the legacy point_cloud_convert behavior: when
            # --pts-per-file is set we suffix outputs with -NNN, otherwise
            # write directly to the requested filename.
            if pts_per_file_active:
                if file_index == 0:
                    return f"{file_wo_ext}{outfile_ext}"
                return f"{file_wo_ext}-{file_index:03d}{outfile_ext}"
            return output_file_path

        def roll_chunk_writer_if_needed() -> None:
            nonlocal chunk_writer, file_index, points_saved_total
            if not pts_per_file_active or chunk_writer is None:
                return
            if chunk_writer.points_count < pts_per_file:
                return
            logger.info(f"Output file: {rolling_output_path()}")
            chunk_writer.finalize()
            points_saved_total += chunk_writer.points_count
            chunk_writer.cleanup()
            file_index += 1
            chunk_writer = open_chunk_writer()

        # pts_per_file <= 0 disables rolling (single output file). Match
        # the legacy convention where the option's default (100M) is
        # effectively "no rolling" for typical inputs.
        pts_per_file_active = pts_per_file is not None and pts_per_file > 0

        def requested_field_count(frame, field_name: str) -> int:
            if field_name in ("SIGNAL", "REFLECTIVITY", "NEAR_IR"):
                return 1
            if field_name in ("NORMALS", "RGB"):
                return 3
            if not frame.has_field(field_name):
                print(f"ERROR: Requested field to save '{field_name}' missing from frame.")
                raise RuntimeError("missing field")
            field_data = frame.field(field_name)
            if field_data.ndim == 2:
                return 1
            if field_data.ndim == 3:
                return int(field_data.shape[-1])
            print(f"ERROR: Requested field to save '{field_name}' must either have 2 or 3 dimensions.")
            raise RuntimeError("unsupported field dimensions")

        def validate_and_extract_field(frame, field_name: str, ret_idx: int,
                                       valid_index: np.ndarray) -> np.ndarray:
            key_name = field_name if ret_idx == 0 else f"{field_name}{ret_idx + 1}"
            if frame.has_field(key_name):
                values = frame.field(key_name)
            elif frame.has_field(field_name):
                values = frame.field(field_name)
            else:
                print(f"ERROR: Requested field to save '{field_name}' missing from frame.")
                raise RuntimeError("missing field")

            if values.ndim < 2 or values.ndim > 3:
                print(f"ERROR: Requested field to save '{field_name}' must either have 2 or 3 dimensions.")
                raise RuntimeError("unsupported field dimensions")
            if values.shape[0] != frame.h or values.shape[1] != frame.w:
                print(f"ERROR: Requested field to save '{field_name}' didn't match frame dimensions.")
                raise RuntimeError("field dimension mismatch")

            # Tone-map float16 RGB data into a 0..1 float range so that
            # downstream packing (and the PLY/PCD writers) see the same
            # input distribution they used to under point_cloud_convert.
            if (values.ndim == 3 and values.shape[-1] == 3 and
                    values.dtype == np.float16 and "RGB" in field_name.upper()):
                values = tone_mappers[field_name].update(values)

            expected_count = dict(field_info)[field_name]
            if expected_count == 1:
                if values.ndim == 3 and values.shape[-1] != 1:
                    print(f"ERROR: Requested field '{field_name}' must map to a single attribute.")
                    raise RuntimeError("unexpected field width")
                extracted = values[valid_index]
                return extracted.reshape(-1, 1)

            if values.ndim != 3 or values.shape[-1] != expected_count:
                print(f"ERROR: Requested field '{field_name}' must have {expected_count} values per point.")
                raise RuntimeError("unexpected field width")
            return values[valid_index].reshape(-1, expected_count)

        def add_points(points: np.ndarray, attributes: np.ndarray) -> None:
            if points.shape[0] == 0:
                return
            if attributes.size == 0:
                points_with_attributes = points.astype(np.float64, copy=False)
            else:
                points_with_attributes = np.concatenate([points, attributes], axis=1).astype(
                    np.float64, copy=False)
            points_with_attributes = np.ascontiguousarray(points_with_attributes)

            if decimate:
                # Default: accumulate through the voxel map, trim/flush as
                # the sensor moves.
                if voxel_hash_map is None:
                    raise RuntimeError("voxel map not initialized")
                voxel_hash_map.add_points(points_with_attributes)
            else:
                # --decimate=False bypasses the voxel map and streams raw
                # points to the chunk writer, matching the legacy
                # point_cloud_convert behavior.
                if chunk_writer is None:
                    raise RuntimeError("chunk writer not initialized")
                chunk_writer.write(points_with_attributes)
                roll_chunk_writer_if_needed()

        def trim_voxel_map(current_position: np.ndarray) -> None:
            nonlocal last_trim_position
            if voxel_hash_map is None or chunk_writer is None:
                return
            if not decimate:
                return
            if trim_distance <= 0:
                return
            if last_trim_position is None:
                last_trim_position = current_position.copy()
                return
            if np.linalg.norm(current_position - last_trim_position) < trim_distance:
                return
            extracted = voxel_hash_map.extract_voxels_far_from_location(
                np.ascontiguousarray(current_position, dtype=np.float64))
            chunk_writer.write(extracted)
            roll_chunk_writer_if_needed()
            last_trim_position = current_position.copy()

        def save_map() -> None:
            if chunk_writer is None:
                logger.info("No points accumulated.")
                return

            # Flush any remaining buffered points from the voxel map.
            if decimate and voxel_hash_map is not None:
                chunk_writer.write(voxel_hash_map.point_cloud())

            total_saved = points_saved_total + chunk_writer.points_count
            if total_saved == 0:
                logger.info("No points accumulated.")
                return

            logger.info(f"Output file: {rolling_output_path()}")
            chunk_writer.finalize()

            if points_sum > 0:
                out_range_pct = (points_out_range / points_sum) * 100
                save_pct = (total_saved / points_sum) * 100
                logger.info(
                    f"Point Cloud status info\n"
                    f"{points_sum} points accumulated,\n"
                    f"{points_out_range} out range points are removed "
                    f"[{out_range_pct:.2f} %],\n"
                    f"{total_saved} points are saved [{save_pct:.2f} %].")

        try:
            for frames in frames_iter():
                if already_saved:
                    yield frames
                    continue

                for frame in frames:
                    if frame is None:
                        continue

                    if first:
                        first = False
                        for field_name in fields:
                            count = requested_field_count(frame, field_name)
                            field_info.append((field_name, count))
                            attribute_count += count
                            # Float16 RGB fields need tone mapping to fit
                            # into 0..1 before packing.
                            if count == 3 and "RGB" in field_name.upper():
                                tone_mappers[field_name] = LocalToneMapper()
                        if outfile_ext == ".las":
                            # LAS only has a single intensity slot and a
                            # single RGB (red/green/blue) triple, so at most
                            # one scalar field and one RGB field are allowed.
                            # Other multi-channel fields (e.g. NORMALS) have
                            # no corresponding LAS attribute.
                            scalar_fields = [n for n, c in field_info if c == 1]
                            rgb_fields = [n for n, c in field_info
                                         if c == 3 and is_rgb_field(n)]
                            unsupported_fields = [
                                n for n, c in field_info
                                if not (c == 1 or (c == 3 and is_rgb_field(n)))
                            ]
                            if (len(scalar_fields) > 1 or len(rgb_fields) > 1 or
                                    unsupported_fields):
                                print("ERROR: LAS format only supports saving a single "
                                      "scalar field (as intensity) and/or a single RGB "
                                      "field.")
                                failed = True
                                exit(1)
                        voxel_hash_map = VoxelHashMapXd(
                            voxel_size, 100.0, max_points_per_voxel, min_pts_threshold, attribute_count)
                        chunk_writer = open_chunk_writer()

                    column_poses = frame.body_to_world
                    if (empty_pose and
                            not np.array_equal(first_valid_column_pose(frame), np.eye(4))):
                        empty_pose = False

                    for ret_idx in range(frame.sensor_info.num_returns):
                        range_name = "RANGE" if ret_idx == 0 else f"RANGE{ret_idx + 1}"
                        if not frame.has_field(range_name):
                            continue

                        rng = frame.field(range_name)
                        valid_index = rng > 0
                        points_sum += rng.size
                        points_out_range += np.count_nonzero(~valid_index)

                        if not np.any(valid_index):
                            continue

                        points = cast(np.ndarray, frame.sensor_info.xyzlut_double(rng))
                        dewarped = cast(
                            np.ndarray,
                            dewarp(points, cast(np.ndarray, column_poses)),
                        )
                        filtered_points = dewarped[valid_index]

                        attribute_chunks = []
                        for field_name in fields:
                            attribute_chunks.append(
                                validate_and_extract_field(frame, field_name, ret_idx, valid_index))
                        if attribute_chunks:
                            attributes = np.concatenate(attribute_chunks, axis=1)
                        else:
                            attributes = np.zeros((filtered_points.shape[0], 0), dtype=np.float64)
                        add_points(filtered_points, attributes)

                    current_pose = first_valid_column_pose(frame)
                    if not np.array_equal(current_pose, np.eye(4)):
                        trim_voxel_map(np.asarray(current_pose[:3, 3], dtype=np.float64))

                yield frames

        except KeyboardInterrupt:
            pass
        except RuntimeError:
            failed = True
            raise
        finally:
            if failed:
                if chunk_writer is not None:
                    chunk_writer.cleanup()
                return
            if empty_pose:
                logger.info(
                    "Warning: Empty lidar frame pose in lidar frame stream!!!\n"
                    "Suggest: Append slam option to ouster-cli command or use a "
                    "SLAM output OSF file as an input.")
            if not already_saved:
                try:
                    save_map()
                    logger.info("Finished point cloud export.")
                    already_saved = True
                finally:
                    if chunk_writer is not None:
                        chunk_writer.cleanup()

    ctx.frame_set_iter = map_export_iter  # type: ignore


SourceSaveCommand.implementations[OusterIoType.PCD] = map_export
SourceSaveCommand.implementations[OusterIoType.LAS] = map_export
SourceSaveCommand.implementations[OusterIoType.LAZ] = map_export
SourceSaveCommand.implementations[OusterIoType.PLY] = map_export
