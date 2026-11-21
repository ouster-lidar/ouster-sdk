from typing import (IO, Any, Callable, Dict, Generator, Iterator, List,
                    NamedTuple, Optional, Sequence, cast)

import os
import shutil
import tempfile
import click
import logging
import laspy
import numpy as np
from ouster.sdk.core import FieldClass, OusterIoType
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

from ouster.sdk.core import (VoxelHashMap, VoxelDownsampleStrategy,
                             AttributeLayout, AttributeType)


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('mapping')


class AttributeField(NamedTuple):
    """One exported field: its name, how many values it carries per point, and
    the type those values keep all the way from the frame to the output file."""
    name: str
    elements: int
    dtype: np.dtype


# Field dtypes the voxel map can store natively, as the bindings define them.
_NATIVE_DTYPES = frozenset(attribute_type.dtype for attribute_type in AttributeType)

# Dtypes no point-cloud format can express directly, mapped to the narrowest
# type that holds every value they can take.
_EXPORT_DTYPE_PROMOTIONS: Dict[np.dtype, np.dtype] = {
    np.dtype(np.float16): np.dtype(np.float32),
    np.dtype(np.bool_): np.dtype(np.uint8),
}

# PLY property type names. PLY has no 64-bit integer property, so those fields
# are written as double (see ply_output_dtype).
_PLY_TYPE_NAMES: Dict[np.dtype, str] = {
    np.dtype(np.int8): "char",
    np.dtype(np.uint8): "uchar",
    np.dtype(np.int16): "short",
    np.dtype(np.uint16): "ushort",
    np.dtype(np.int32): "int",
    np.dtype(np.uint32): "uint",
    np.dtype(np.float32): "float",
    np.dtype(np.float64): "double",
}

_PCD_TYPE_CODES: Dict[str, str] = {"u": "U", "i": "I", "f": "F"}

# --downsample-mode choice -> the VoxelDownsampleStrategy the voxel map uses.
_DOWNSAMPLE_STRATEGIES: Dict[str, VoxelDownsampleStrategy] = {
    'first': VoxelDownsampleStrategy.FIRST_N_POINT,
    'last': VoxelDownsampleStrategy.LAST_N_POINT,
    'average': VoxelDownsampleStrategy.AVERAGE_POINT,
    'random': VoxelDownsampleStrategy.RANDOM,
    'smallest': VoxelDownsampleStrategy.SMALLEST,
    'largest': VoxelDownsampleStrategy.LARGEST,
}


def export_dtype(dtype: Any) -> np.dtype:
    """Return the type a field's values are stored and written in.

    Field dtypes carry over unchanged where the voxel map and the output formats
    can both represent them; the few that they cannot (float16, bool) are
    promoted. Anything else falls back to double, which is what every field used
    to be widened to.
    """
    resolved = np.dtype(dtype)
    resolved = _EXPORT_DTYPE_PROMOTIONS.get(resolved, resolved)
    if resolved not in _NATIVE_DTYPES:
        return np.dtype(np.float64)
    return resolved


def ply_output_dtype(dtype: np.dtype) -> np.dtype:
    """The type a field is written as in a PLY file; 64-bit integers, which PLY
    cannot express, become double."""
    return dtype if dtype in _PLY_TYPE_NAMES else np.dtype(np.float64)


def _as_export_dtype(values: np.ndarray, dtype: np.dtype) -> np.ndarray:
    """Convert `values` to `dtype`, rounding rather than truncating when the
    destination is an integer type."""
    if values.dtype == dtype:
        return values
    if dtype.kind in "ui" and values.dtype.kind == "f":
        info = np.iinfo(dtype)
        return np.clip(np.rint(values), info.min, info.max).astype(dtype)
    return values.astype(dtype, copy=False)


def frame_info(frame: Any) -> Any:
    """Return the :class:`SensorInfo` attached to ``frame``."""
    return frame.sensor_info


def frame_num_returns(frame: Any) -> int:
    """Number of range returns to export for ``frame``."""
    info = frame_info(frame)
    return int(getattr(info, "num_returns", 1)) if info is not None else 1


def frame_reference_pose(frame: Any) -> np.ndarray:
    """A single world pose for ``frame``, used to gate/trim on sensor motion."""
    return np.asarray(first_valid_column_pose(frame), dtype=np.float64)


def project_world_points(frame: Any, rng: np.ndarray) -> Optional[np.ndarray]:
    """Project a RANGE image into world-frame XYZ, shaped ``(H, W, 3)``.

    Returns ``None`` when ``frame`` has no ``sensor_info`` or the sensor cannot
    be projected (for example, a legacy camera without pinhole intrinsics).
    """
    info = frame_info(frame)
    if info is None:
        return None
    try:
        points = np.asarray(info.xyzlut_double(rng), dtype=np.float64).reshape(
            frame.h, frame.w, 3)
    except Exception:
        return None
    return cast(np.ndarray, dewarp(points, np.asarray(frame.body_to_world, dtype=np.float64)))


def is_rgb_field(name: str) -> bool:
    return name.upper() == "RGB"


def is_normal_field(name: str) -> bool:
    return name.upper() == "NORMALS"


# Order in which a scalar/RGB field is auto-selected when --field is not
# given on the command line.
DEFAULT_FIELD_PRIORITY: List[str] = ["RGB", "REFLECTIVITY", "SIGNAL", "NEAR_IR"]


def _select_default_field(frames: Any) -> Optional[str]:
    """Pick the highest-priority field in DEFAULT_FIELD_PRIORITY that is
    present on every currently available (non-None) frame in `frames`.

    Requiring the field on every frame in the batch keeps this safe for
    multi-sensor sets: a field only used by some sensors would otherwise
    crash extraction later when a sensor lacking it is processed.
    """
    usable_frames = [f for f in frames if f is not None]
    if not usable_frames:
        return None
    for candidate in DEFAULT_FIELD_PRIORITY:
        if all(f.has_field(candidate) for f in usable_frames):
            return candidate
    return None


def _base_pixel_fields(frame: Any) -> set:
    """Pixel-per-point field names on `frame`, excluding secondary-return
    variants (e.g. REFLECTIVITY2, RANGE2) whose base field (REFLECTIVITY,
    RANGE) is also present; those are picked up automatically per return
    index elsewhere in the export pipeline.
    """
    pixel_fields = {
        name for name in frame.fields
        if frame.field_class(name) == FieldClass.PIXEL_FIELD
    }
    return {name for name in pixel_fields
            if not (name.rstrip("0123456789") != name and
                    name.rstrip("0123456789") in pixel_fields)}


def _all_available_fields(frames: Any) -> List[str]:
    """Return, sorted, the pixel field names common to every currently
    available (non-None) frame in `frames`.

    As with `_select_default_field`, requiring a field on every frame keeps
    this safe for multi-sensor sets: a field only present on some sensors
    would otherwise crash extraction later when a sensor lacking it comes up.
    """
    usable_frames = [f for f in frames if f is not None]
    if not usable_frames:
        return []
    common = _base_pixel_fields(usable_frames[0])
    for frame in usable_frames[1:]:
        common &= _base_pixel_fields(frame)
    return sorted(common)


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
    return np.clip(np.asarray(rgb, dtype=np.float64) * 65535.0, 0, 65535).astype(np.uint16)


def _dtype_entry(key: str, dtype: np.dtype, count: int):
    return (key, dtype.newbyteorder("<")) if count == 1 else (key, dtype.newbyteorder("<"), count)


class ExportSchema:
    """Column layout of an exported cloud: XYZ followed by each field's values.

    Ties together the two parallel descriptions the pipeline needs:

    * `layout` - what the voxel map stores per point, in the fields' own types.
      The map takes and returns those attributes one array per field, each in
      the type its spec declares.
    * `ply_dtype` / `pcd_dtype` - the structured dtype of one written row for
      each format, byte-identical to what goes on disk.
    """

    def __init__(self, fields: List[AttributeField]) -> None:
        self.fields = list(fields)
        self.keys = [f"a{i}" for i in range(len(self.fields))]
        self.layout = AttributeLayout([(field.dtype, field.elements) for field in self.fields])

        xyz_entries = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
        self.ply_dtype = np.dtype(xyz_entries + [
            _dtype_entry(key, ply_output_dtype(field.dtype), field.elements)
            for key, field in zip(self.keys, self.fields)
        ])
        self.pcd_dtype = np.dtype(xyz_entries + [
            _dtype_entry(key, np.dtype(np.float32), 1)
            if (field.elements == 3 and is_rgb_field(field.name))
            else _dtype_entry(key, field.dtype, field.elements)
            for key, field in zip(self.keys, self.fields)
        ])

    def stage_dtype(self, outfile_ext: str) -> np.dtype:
        """Structured dtype one staged row is written in for `outfile_ext`."""
        return self.pcd_dtype if outfile_ext == ".pcd" else self.ply_dtype


def ply_header(point_count: int, schema: ExportSchema, ascii: bool) -> str:
    header = [
        "ply",
        "format ascii 1.0" if ascii else "format binary_little_endian 1.0",
        f"element vertex {point_count}",
        "property float x",
        "property float y",
        "property float z",
    ]
    for field in schema.fields:
        type_name = _PLY_TYPE_NAMES[ply_output_dtype(field.dtype)]
        for property_name in ply_property_names(field.name, field.elements):
            header.append(f"property {type_name} {property_name}")
    header.append("end_header")
    return "\n".join(header) + "\n"


def pcd_header(point_count: int, schema: ExportSchema, data_mode: str) -> str:
    fields = ["x", "y", "z"]
    sizes = [4, 4, 4]
    types = ["F", "F", "F"]
    counts = [1, 1, 1]

    for field in schema.fields:
        field_names, field_counts = pcd_field_entries(field.name, field.elements)
        # RGB collapses to a single packed float field; everything else keeps
        # the type the field was captured in.
        packed_rgb = field.elements == 3 and is_rgb_field(field.name)
        dtype = np.dtype(np.float32) if packed_rgb else field.dtype
        fields.extend(field_names)
        for field_count in field_counts:
            sizes.append(dtype.itemsize)
            types.append(_PCD_TYPE_CODES[dtype.kind])
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


def _ascii_lines(rows: np.ndarray) -> Iterator[str]:
    """Yield one whitespace-separated text line per row of a structured array,
    expanding multi-value fields into consecutive columns."""
    columns: List[np.ndarray] = []
    for name in rows.dtype.names or ():
        column = rows[name]
        if column.ndim == 1:
            columns.append(column)
        else:
            columns.extend(column[:, i] for i in range(column.shape[1]))
    for values in zip(*columns):
        yield " ".join(str(value) for value in values) + "\n"


class StagedPointCloudWriter:
    """Stage point-cloud chunks while preserving final single-file headers."""

    def __init__(self, output_file_path: str, ascii: bool,
                 schema: ExportSchema,
                 allow_empty_output: bool = False) -> None:
        self.output_file_path = output_file_path
        self.ascii = ascii
        self.schema = schema
        self.allow_empty_output = allow_empty_output
        self.points_count = 0
        self._closed = False
        _, self.outfile_ext = os.path.splitext(output_file_path)
        self.stage_dtype = schema.stage_dtype(self.outfile_ext)
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

    def write(self, xyz: np.ndarray,
              attributes: Optional[Sequence[np.ndarray]] = None) -> None:
        """Append points to the staged output.

        `xyz` is an Nx3 float array; `attributes` holds one array per field of
        the schema, already in the field's own type, as handed to or returned
        by the voxel map. Values are stored as-is; the only cast left is the
        lossless widening to a format's row type (e.g. int64 to a PLY double).
        """
        xyz = np.asarray(xyz)
        if xyz.size == 0:
            return
        if xyz.ndim == 1:
            xyz = xyz.reshape(1, -1)
        if xyz.shape[1] != 3:
            raise RuntimeError(f"unexpected cloud width {xyz.shape[1]}, expected 3")

        rows = int(xyz.shape[0])
        attributes = list(attributes) if attributes is not None else []
        if len(attributes) != len(self.schema.fields):
            raise RuntimeError("expected one attribute array per exported field")

        out = np.empty(rows, dtype=self.stage_dtype)
        out["x"] = xyz[:, 0]
        out["y"] = xyz[:, 1]
        out["z"] = xyz[:, 2]
        for key, field, values in zip(self.schema.keys, self.schema.fields, attributes):
            values = np.asarray(values)
            if values.shape[0] != rows:
                raise RuntimeError("attribute array length does not match the number of points")
            values = (values.reshape(-1) if field.elements == 1
                      else values.reshape(-1, field.elements))
            if self.outfile_ext == ".pcd" and field.elements == 3 and is_rgb_field(field.name):
                out[key] = pack_rgb_to_float_array(np.asarray(values, dtype=np.float64))
            else:
                out[key] = values

        if self.ascii and self.outfile_ext in (".ply", ".pcd"):
            self._file.writelines(_ascii_lines(out))
        else:
            self._file.write(out.tobytes())
        self.points_count += rows

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
                    out.write(ply_header(self.points_count, self.schema, self.ascii))
                    with open(self.tmp_path, "r", encoding="utf-8") as tmp:
                        shutil.copyfileobj(tmp, out)
            else:
                with open(self.output_file_path, "wb") as out:
                    out.write(ply_header(
                        self.points_count, self.schema, self.ascii).encode("ascii"))
                    with open(self.tmp_path, "rb") as tmp:
                        shutil.copyfileobj(tmp, out)
        elif self.outfile_ext == ".pcd":
            if self.ascii:
                with open(self.output_file_path, "w", encoding="utf-8") as out:
                    out.write(pcd_header(self.points_count, self.schema, "ascii"))
                    with open(self.tmp_path, "r", encoding="utf-8") as tmp:
                        shutil.copyfileobj(tmp, out)
            else:
                with open(self.output_file_path, "wb") as out:
                    out.write(pcd_header(
                        self.points_count, self.schema, "binary").encode("ascii"))
                    with open(self.tmp_path, "rb") as tmp:
                        shutil.copyfileobj(tmp, out)
        elif self.outfile_ext == ".las" or self.outfile_ext == ".laz":
            cloud = np.memmap(
                self.tmp_path, dtype=self.stage_dtype, mode="r",
                shape=(self.points_count,))
            # laspy's default point format (3) already includes intensity
            # and red/green/blue, so no explicit point_format is needed here.
            LAS_file = laspy.create()
            LAS_file.x = cloud["x"]
            LAS_file.y = cloud["y"]
            LAS_file.z = cloud["z"]
            for key, field in zip(self.schema.keys, self.schema.fields):
                if field.elements == 1:
                    LAS_file.intensity = cloud[key]
                elif field.elements == 3 and is_rgb_field(field.name):
                    rgb = scale_rgb_to_uint16(cloud[key])
                    LAS_file.red = rgb[:, 0]
                    LAS_file.green = rgb[:, 1]
                    LAS_file.blue = rgb[:, 2]
            try:
                LAS_file.write(self.output_file_path)
            except laspy.errors.LaspyException as e:
                raise ValueError(f"{e}; if trying to write a laz file make sure to"
                                 " pip install 'lazrs' or 'laszip' packages") from e

    def cleanup(self) -> None:
        self._close_temp()
        try:
            os.remove(self.tmp_path)
        except OSError:
            pass


def save_pointcloud(filename: str, cloud: np.ndarray, ascii: bool = False,
                    fields: Optional[List[AttributeField]] = None) -> None:
    """Save an Nx(3+attributes) cloud to PLY, PCD, LAS or LAZ.

    `fields` describes the attribute columns after XYZ; their values are taken
    from `cloud` and converted into each field's declared type. Scalar fields
    take one column, RGB and NORMALS three.
    """
    cloud = np.asarray(cloud, dtype=np.float64)
    if cloud.ndim == 1:
        cloud = cloud.reshape(1, -1)

    schema = ExportSchema(fields or [])
    attributes: List[np.ndarray] = []
    column = 3
    for field in schema.fields:
        attributes.append(_as_export_dtype(cloud[:, column:column + field.elements], field.dtype))
        column += field.elements

    writer = StagedPointCloudWriter(filename, ascii, schema, allow_empty_output=True)
    try:
        writer.write(cloud[:, 0:3], attributes)
        writer.finalize()
    finally:
        writer.cleanup()


@click.command(context_settings=dict(
    ignore_unknown_options=True,
    allow_extra_args=True,
))
@click.argument("filename", required=True)
@click.option('-p', '--prefix', default="", help="Output prefix.")
@click.option('-d', '--dir', "directory", default="", help="Output directory.")
@click.option('-v', '--voxel-size', default=0.1, help="Voxel map size in meters for downsampling."
              " The bigger the value, the fewer points it outputs."
              " Set to <= 0 to disable downsampling.",
              show_default=True)
@click.option('--max-points-per-voxel', default=10, type=int, show_default=True,
              help="Max points per voxel. No effect in average mode.")
@click.option('--min-points-per-voxel', default=1, type=int, show_default=True,
              help="Minimum number of points in a voxel before it is considered in the output.")
@click.option('-f', '--field', multiple=True,
              default=(),
              help=("ChanField for output pointcloud key values."
                    " Examples include REFLECTIVITY, RANGE, SIGNAL, NEAR_IR."
                    " Can be applied multiple times to add more fields."
                    " If not specified, the command will attempt to pick the first"
                    " available field from the set [RGB, REFLECTIVITY, SIGNAL, NEAR_IR]; if none"
                    " of them are available, no scalar field is exported."
                    " Mutually exclusive with --field-none and --field-all."))
@click.option('--field-none', is_flag=True, default=False,
              help=("Export no scalar/attribute field, only XYZ points."
                    " Mutually exclusive with --field and --field-all."))
@click.option('--field-all', is_flag=True, default=False,
              help=("Export every field common to all sensors in the source, instead of just"
                    " one. Mutually exclusive with --field and --field-none."))
@click.option('--downsample-mode',
              type=click.Choice(['average', 'first', 'last', 'random', 'smallest', 'largest']),
              default='random', show_default=True,
              help=("Per-voxel downsampling mode. The 'first' mode keeps the first points admitted"
                    " into each voxel up to 'max-points-per-voxel'. The 'last' mode is the same, but"
                    " keeps the most recently admitted points instead: a full voxel evicts its oldest."
                    " The 'average' mode computes the per-voxel average of all points and their"
                    " attributes. The 'random' mode keeps up to 'max-points-per-voxel'"
                    " uniformly-sampled random points per voxel via reservoir sampling. The"
                    " 'smallest'/'largest' modes keep the points with the smallest/largest value of"
                    " one exported field, selected with --attribute-index."))
@click.option('--attribute-index', default=0, type=int, show_default=True,
              help=("Index into --field (in the order given, or auto-selected/--field-all order when"
                    " --field is omitted) of the field to rank by. Only used, and required to name a"
                    " scalar field, when --downsample-mode is 'smallest' or 'largest'."))
@click.option('--overwrite', is_flag=True, default=False, help="If true, overwrite "
              "existing files with the same name")
@click.option('--pts-per-file', default=100000000, type=int, show_default=True,
              help="Number of points per output file.")
@click.option('--ascii', is_flag=True, default=False,
              help="Output files in ASCII rather than binary format")
@click.option('--trim-distance', default=25.0, type=float, show_default=True,
              help=("Distance the sensor pose must move before far voxels are "
                    "extracted from the in-memory voxel map and staged for output."))
@click.pass_context
@source_multicommand(type=SourceCommandType.CONSUMER, retrieve_click_context=True)
def map_export(ctx: SourceCommandContext, click_ctx, filename: str, prefix: str,
               directory: str, voxel_size: float, max_points_per_voxel: int,
               min_points_per_voxel: int, field: List[str],
               field_none: bool, field_all: bool,
               downsample_mode: str, attribute_index: int, overwrite: bool, pts_per_file: int,
               ascii: bool, trim_distance: float, **kwargs: Any) -> None:
    """Export an accumulated map to a PCD, LAS, LAZ, or PLY file.

    The scalar/attribute field(s) written alongside XYZ are controlled by
    three mutually exclusive options:

    \b
    --field NAME    Export the given field(s) (repeatable). If omitted
                    entirely, the first available field from
                    [RGB, REFLECTIVITY, SIGNAL, NEAR_IR] is used.
    --field-none    Export no scalar/attribute field, only XYZ points.
    --field-all     Export every field common to all sensors in the source.
    """

    if sum([bool(field), field_none, field_all]) > 1:
        print("ERROR: '--field', '--field-none', and '--field-all' are mutually exclusive.")
        exit(1)

    for param in map_export.params:
        if param.name == 'max_points_per_voxel':
            if max_points_per_voxel != param.default and downsample_mode == 'average':
                click.secho("WARNING: '--max-points-per-voxel' is not used when '--downsample-mode' is 'average'.",
                            fg='yellow')

    if ctx.frame_set_iter is None:
        raise RuntimeError("frame_set_iter is not initialized")
    frames_iter = cast(Callable[[], Iterator[Any]], ctx.frame_set_iter)

    # Use the combined sensor_info (lidar and camera) rather than lidar-only, so
    # camera-only sources still yield a naming info. determine_filename handles
    # both LidarInfo and CameraInfo.
    infos: List[Any] = []
    if ctx.frame_set_source is not None:
        infos = list(ctx.frame_set_source.sensor_info)
    if not infos:
        raise RuntimeError("No sensor information available from the source.")

    outfile_ext = "." + kwargs["format"]
    output_file_path = determine_filename(prefix, directory, filename,
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

    # Validate there are no duplicate fields.
    raw_fields = field
    fields: List[str] = []
    for name in raw_fields:
        field_name = name.upper()
        if field_name in fields:
            print(f"ERROR: Requested to save duplicate field '{field_name}'.")
            exit(1)
        fields.append(field_name)

    # --field-none and --field-all are resolved below; with none of the three
    # options given, fall back to auto-selecting a single field once frame
    # data is available.
    resolve_fields_from_frames = not raw_fields and not field_none

    def map_export_iter() -> Generator[Any, None, None]:
        nonlocal already_saved, empty_pose, fields
        failed = False
        first = True
        field_info: List[AttributeField] = []
        schema = ExportSchema([])
        voxel_hash_map: Optional[VoxelHashMap] = None
        chunk_writer: Optional[StagedPointCloudWriter] = None
        last_trim_position = None
        points_in_range = 0
        points_saved_total = 0
        # Per-sensor tone mapper state so float16 RGB data is normalized to
        # [0, 1] before downsampling.
        tone_mappers: Dict[int, LocalToneMapper] = {}
        # File rolling when --pts-per-file is exceeded. file_index counts
        # how many chunk files we've finalized; the on-disk filename is
        # composed below.
        file_index = 0

        def open_chunk_writer() -> StagedPointCloudWriter:
            return StagedPointCloudWriter(rolling_output_path(), ascii, schema)

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

        def requested_field(frame, field_name: str) -> AttributeField:
            """Resolve a requested field's element count and the type its
            values are exported in, from the first frame that carries it."""
            known_counts = {"SIGNAL": 1, "REFLECTIVITY": 1, "NEAR_IR": 1, "NORMALS": 3, "RGB": 3}
            field_data = frame.field(field_name) if frame.has_field(field_name) else None
            if field_name in known_counts:
                count = known_counts[field_name]
            elif field_data is None:
                print(f"ERROR: Requested field to save '{field_name}' missing from frame.")
                raise RuntimeError("missing field")
            else:
                if field_data.ndim == 2:
                    count = 1
                elif field_data.ndim == 3:
                    count = int(field_data.shape[-1])
                else:
                    print(f"ERROR: Requested field to save '{field_name}' must either have 2 or 3 dimensions.")
                    raise RuntimeError("unsupported field dimensions")

            # Fields absent from this frame are reported when extraction runs;
            # double is the historic fallback for anything we cannot type here.
            dtype = (export_dtype(field_data.dtype)
                     if field_data is not None else np.dtype(np.float64))
            return AttributeField(field_name, count, dtype)

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
                sn = frame_info(frame).sn
                if sn not in tone_mappers:
                    tone_mappers[sn] = LocalToneMapper()
                values = tone_mappers[sn].update(values)

            expected = next(f for f in field_info if f.name == field_name)
            expected_count = expected.elements
            if expected_count == 1:
                if values.ndim == 3 and values.shape[-1] != 1:
                    print(f"ERROR: Requested field '{field_name}' must map to a single attribute.")
                    raise RuntimeError("unexpected field width")
                extracted = values[valid_index]
                return _as_export_dtype(extracted.reshape(-1, 1), expected.dtype)

            if values.ndim != 3 or values.shape[-1] != expected_count:
                print(f"ERROR: Requested field '{field_name}' must have {expected_count} values per point.")
                raise RuntimeError("unexpected field width")
            return _as_export_dtype(values[valid_index].reshape(-1, expected_count), expected.dtype)

        def add_points(points: np.ndarray, attributes: List[np.ndarray]) -> None:
            if points.shape[0] == 0:
                return
            points = np.ascontiguousarray(points, dtype=np.float64)
            # Attributes travel one array per field, each in its own type (as
            # validate_and_extract_field produced them), so a uint32 RANGE
            # stays 4 bytes per point instead of being widened to a double on
            # the way into the map and back out again.

            if voxel_hash_map:
                # Default: accumulate through the voxel map, trim/flush as
                # the sensor moves.
                voxel_hash_map.add_points(points, attributes)
            else:
                # otherwise write directly
                if chunk_writer is None:
                    raise RuntimeError("chunk writer not initialized")
                chunk_writer.write(points, attributes)
                roll_chunk_writer_if_needed()

        def trim_voxel_map(current_position: np.ndarray) -> None:
            nonlocal last_trim_position
            if voxel_hash_map is None or chunk_writer is None:
                return
            if trim_distance <= 0:
                return
            if last_trim_position is None:
                last_trim_position = current_position.copy()
                return
            if np.linalg.norm(current_position - last_trim_position) < trim_distance:
                return
            position = np.ascontiguousarray(current_position, dtype=np.float64)
            xyz, attributes = voxel_hash_map.extract_voxels_far_from_location(position)
            chunk_writer.write(xyz, attributes)
            roll_chunk_writer_if_needed()
            last_trim_position = current_position.copy()

        def save_map() -> None:
            if chunk_writer is None:
                logger.info("No points accumulated.")
                return

            # Flush any remaining buffered points from the voxel map.
            if voxel_hash_map is not None:
                xyz, attributes = voxel_hash_map.pointcloud()
                chunk_writer.write(xyz, attributes)

            total_saved = points_saved_total + chunk_writer.points_count
            if total_saved == 0:
                logger.info("No points accumulated.")
                return

            logger.info(f"Output file: {rolling_output_path()}")
            chunk_writer.finalize()

            if points_in_range > 0:
                save_pct = (total_saved / points_in_range) * 100
                if voxel_hash_map is None:
                    logger.info(
                        f"Point Cloud status info\n"
                        f"{points_in_range} points accumulated\n"
                        f"{total_saved} points are saved [{save_pct:.2f} %].")
                else:
                    logger.info(
                        f"Point Cloud status info\n"
                        f"{points_in_range} points accumulated\n"
                        f"{total_saved} points are saved after downsampling [{save_pct:.2f} %].")
        try:
            for frames in frames_iter():
                if already_saved:
                    yield frames
                    continue

                for frame in frames.valid_frames():
                    if frame_info(frame) is None:
                        continue
                    if first:
                        first = False
                        if resolve_fields_from_frames:
                            if field_all:
                                fields = _all_available_fields(frames)
                                if fields:
                                    logger.info(f"Exporting all common fields: {fields}")
                                else:
                                    logger.info("No fields common to all sensors; exporting XYZ only.")
                            else:
                                chosen_field = _select_default_field(frames)
                                fields = [chosen_field] if chosen_field else []
                                if chosen_field:
                                    logger.info(f"Auto-selected export field: {chosen_field}")
                                else:
                                    logger.info("No default scalar/RGB field found; exporting XYZ only.")
                                logger.info("Use --field, --field-none, or --field-all to override this selection.")
                        for field_name in fields:
                            field_info.append(requested_field(frame, field_name))
                        schema = ExportSchema(field_info)
                        for entry in field_info:
                            if ply_output_dtype(entry.dtype) != entry.dtype and \
                                    outfile_ext == ".ply":
                                logger.warning(
                                    f"PLY has no {entry.dtype} property type; "
                                    f"'{entry.name}' is written as double.")
                        if outfile_ext == ".las":
                            # LAS only has a single intensity slot and a
                            # single RGB (red/green/blue) triple, so at most
                            # one scalar field and one RGB field are allowed.
                            # Other multi-channel fields (e.g. NORMALS) have
                            # no corresponding LAS attribute.
                            scalar_fields = [f.name for f in field_info if f.elements == 1]
                            rgb_fields = [f.name for f in field_info
                                          if f.elements == 3 and is_rgb_field(f.name)]
                            unsupported_fields = [
                                f.name for f in field_info
                                if not (f.elements == 1 or
                                        (f.elements == 3 and is_rgb_field(f.name)))
                            ]
                            if (len(scalar_fields) > 1 or len(rgb_fields) > 1 or
                                    unsupported_fields):
                                print("ERROR: LAS format only supports saving a single "
                                      "scalar field (as intensity) and/or a single RGB "
                                      "field.")
                                failed = True
                                exit(1)
                        if voxel_size > 0.0:
                            if downsample_mode in ('smallest', 'largest'):
                                if not (0 <= attribute_index < len(field_info)):
                                    exported = [f.name for f in field_info]
                                    print(f"ERROR: --attribute-index {attribute_index} is out of"
                                          f" range for the {len(field_info)} exported field(s)"
                                          f" {exported}. '--downsample-mode "
                                          f"{downsample_mode}' requires --attribute-index to name"
                                          " one of them.")
                                    failed = True
                                    exit(1)
                                if field_info[attribute_index].elements != 1:
                                    entry = field_info[attribute_index]
                                    print(f"ERROR: --attribute-index {attribute_index} names"
                                          f" '{entry.name}', which has {entry.elements} values per"
                                          f" point; '--downsample-mode {downsample_mode}' needs a"
                                          " scalar field to rank by.")
                                    failed = True
                                    exit(1)
                            voxel_hash_map = VoxelHashMap.create(
                                voxel_size, max_points_per_voxel, min_points_per_voxel,
                                _DOWNSAMPLE_STRATEGIES[downsample_mode], schema.layout,
                                max_distance=100.0, attribute_index=attribute_index)
                        chunk_writer = open_chunk_writer()

                    reference_pose = frame_reference_pose(frame)
                    if empty_pose and not np.array_equal(reference_pose, np.eye(4)):
                        empty_pose = False

                    for ret_idx in range(frame_num_returns(frame)):
                        range_name = "RANGE" if ret_idx == 0 else f"RANGE{ret_idx + 1}"
                        if not frame.has_field(range_name):
                            continue

                        rng = frame.field(range_name)
                        valid_index = rng > 0
                        points_in_range += int(np.count_nonzero(valid_index))

                        if not np.any(valid_index):
                            continue

                        world_points = project_world_points(frame, rng)
                        if world_points is None:
                            continue
                        filtered_points = world_points[valid_index]

                        attribute_chunks = [
                            validate_and_extract_field(frame, field_name, ret_idx, valid_index)
                            for field_name in fields
                        ]
                        add_points(filtered_points, attribute_chunks)

                    if not np.array_equal(reference_pose, np.eye(4)):
                        trim_voxel_map(np.asarray(reference_pose[:3, 3], dtype=np.float64))

                yield frames
        except RuntimeError:
            failed = True
            raise
        finally:
            if failed:
                if chunk_writer is not None:
                    chunk_writer.cleanup()
            else:
                if empty_pose:
                    logger.info(
                        "Warning: Empty frame pose in stream!!!\n"
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
