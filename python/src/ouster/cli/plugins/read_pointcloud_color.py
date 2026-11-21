import numpy as np
from typing import Optional, Tuple, Dict, Callable, List, Union
import laspy
from functools import partial


from ouster.sdk._bindings.viz import (calref_palette, magma_palette)


def _parse_ply(path: str):
    """Parse a PLY file. Returns (prop_names, data, n_vertices)."""
    _PLY_DTYPE: dict = {
        'char': np.dtype('int8'), 'uchar': np.dtype('uint8'),
        'short': np.dtype('int16'), 'ushort': np.dtype('uint16'),
        'int': np.dtype('int32'), 'uint': np.dtype('uint32'),
        'float': np.dtype('float32'), 'double': np.dtype('float64'),
        'int8': np.dtype('int8'), 'uint8': np.dtype('uint8'),
        'int16': np.dtype('int16'), 'uint16': np.dtype('uint16'),
        'int32': np.dtype('int32'), 'uint32': np.dtype('uint32'),
        'float32': np.dtype('float32'), 'float64': np.dtype('float64'),
    }

    data_out: Dict[str, np.ndarray] = {}
    with open(path, 'rb') as fh:
        header_lines = []
        while True:
            line = fh.readline().decode('ascii', errors='replace').strip()
            header_lines.append(line)
            if line == 'end_header':
                break

        fmt = 'ascii'
        n_vertices = 0
        properties: list = []
        in_vertex_block = False

        for line in header_lines:
            parts = line.split()
            if not parts:
                continue
            if parts[0] == 'format':
                fmt = parts[1]
            elif parts[0] == 'element':
                in_vertex_block = (parts[1] == 'vertex')
                if in_vertex_block:
                    n_vertices = int(parts[2])
                else:
                    in_vertex_block = False
            elif parts[0] == 'property' and in_vertex_block:
                if parts[1] == 'list':
                    properties.append((parts[3], None))
                else:
                    properties.append((parts[2], _PLY_DTYPE.get(parts[1])))

        valid_props = [(name, dt) for name, dt in properties if dt is not None]

        if fmt == 'ascii':
            rows = [list(map(float, fh.readline().decode('ascii').split()))
                    for _ in range(n_vertices)]
            data = np.array(rows, dtype=np.float64)
            prop_names = [name for name, _ in valid_props]
            for i in range(len(prop_names)):
                data_out[prop_names[i]] = data[:, i]
        else:
            endian = '<' if fmt == 'binary_little_endian' else '>'
            np_dtype = np.dtype([(name, dt.newbyteorder(endian)) for name, dt in valid_props])
            structured = np.frombuffer(fh.read(n_vertices * np_dtype.itemsize), dtype=np_dtype)
            for name, _ in valid_props:
                data_out[name] = structured[name]

    return data_out, n_vertices


def _parse_pcd(path: str):
    """Parse a PCD file (ascii or binary). Returns (prop_names, data, n_points).

    The packed ``rgb`` field (PCL convention: float32 encoding ARGB bytes) is
    expanded into three separate columns named ``red``, ``green``, ``blue``.
    """
    _PCD_DTYPE = {
        ('F', 4): np.dtype('float32'), ('F', 8): np.dtype('float64'),
        ('U', 1): np.dtype('uint8'), ('U', 2): np.dtype('uint16'),
        ('U', 4): np.dtype('uint32'), ('U', 8): np.dtype('uint64'),
        ('I', 1): np.dtype('int8'), ('I', 2): np.dtype('int16'),
        ('I', 4): np.dtype('int32'), ('I', 8): np.dtype('int64'),
    }

    with open(path, 'rb') as fh:
        fields, sizes, types, counts = [], [], [], []
        n_points = 0
        data_fmt = 'ascii'

        while True:
            raw_line = fh.readline()
            line = raw_line.decode('ascii', errors='replace').strip()
            if line.startswith('#') or not line:
                continue
            parts = line.split()
            key = parts[0].upper()
            if key == 'FIELDS':
                fields = [p.lower() for p in parts[1:]]
            elif key == 'SIZE':
                sizes = [int(p) for p in parts[1:]]
            elif key == 'TYPE':
                types = parts[1:]
            elif key == 'COUNT':
                counts = [int(p) for p in parts[1:]]
            elif key == 'POINTS':
                n_points = int(parts[1])
            elif key == 'DATA':
                data_fmt = parts[1].lower()
                break

        # Build flat column list (expand COUNT > 1 fields)
        col_names = []
        col_dtypes = []
        for fname, fsize, ftype, fcount in zip(fields, sizes, types, counts):
            dt = _PCD_DTYPE.get((ftype.upper(), fsize), np.dtype('float32'))
            for ci in range(fcount):
                col_names.append(fname if fcount == 1 else f"{fname}_{ci}")
                col_dtypes.append(dt)

        raw_data: Union[List[np.ndarray], np.ndarray]
        if data_fmt == 'ascii':
            rows = [list(map(float, fh.readline().decode('ascii').split()))
                    for _ in range(n_points)]
            raw_data = np.array(rows, dtype=np.float64).transpose()
        else:  # binary (binary_compressed not supported)
            row_dtype = np.dtype([(n, dt) for n, dt in zip(col_names, col_dtypes)])
            buf = fh.read(n_points * row_dtype.itemsize)
            structured = np.frombuffer(buf, dtype=row_dtype)
            raw_data = []
            for i, name in enumerate(col_names):
                raw_data.append(structured[name])

    # Expand packed rgb/rgba float field into red/green/blue columns
    data_out: Dict[str, np.ndarray] = {}
    for i, name in enumerate(col_names):
        if name in ('rgb', 'rgba'):
            packed_u32 = raw_data[i].astype(np.float32).view(np.uint32)
            data_out['red'] = ((packed_u32 >> 16) & 0xFF).astype(np.float32)
            data_out['green'] = ((packed_u32 >> 8) & 0xFF).astype(np.float32)
            data_out['blue'] = (packed_u32 & 0xFF).astype(np.float32)
        else:
            data_out[name] = raw_data[i]

    return data_out, n_points


def _parse_las(path: str):
    """Parse a LAS file using laspy. Returns (prop_names, data, n_points).

    XYZ coordinates are automatically scaled by laspy. Intensity is
    included only when at least one point has a non-zero value. RGB channels
    (uint16 in LAS) are down-scaled to [0, 255] and included only when at
    least one channel is non-zero.
    """
    # Read the LAS file
    try:
        las = laspy.read(path)
    except laspy.errors.LaspyException as e:
        raise ValueError(f"{e}; if trying to read a laz file make sure to pip install `lazrs` or `laszip` packages")

    # las.x, las.y, and las.z automatically apply the header's scale and offset
    x = las.x
    y = las.y
    z = las.z

    prop_names = ['x', 'y', 'z']
    columns = [x, y, z]

    # Get available dimensions from the point format
    dim_names = list(las.point_format.dimension_names)

    # Extract other fields if present
    for dim in dim_names:
        if dim.lower() in ('red', 'green', 'blue', 'x', 'y', 'z'):
            continue
        prop_names.append(dim)
        columns.append(las[dim])

    # Extract RGB if present and non-zero
    if all(c in dim_names for c in ('red', 'green', 'blue')):
        scale = 255.0 / 65535.0

        red = np.array(las.red, dtype=np.float32) * scale
        green = np.array(las.green, dtype=np.float32) * scale
        blue = np.array(las.blue, dtype=np.float32) * scale

        if float(red.max()) > 0.0 or float(green.max()) > 0.0 or float(blue.max()) > 0.0:
            prop_names += ['red', 'green', 'blue']
            columns += [red, green, blue]

    n_points = len(las.points)

    data_out: Dict[str, np.ndarray] = {}
    for i in range(len(columns)):
        data_out[prop_names[i]] = columns[i]

    return data_out, n_points


def read_pointcloud(path: str) -> Tuple[Dict[str, np.ndarray], int]:
    ext = path.rsplit('.', 1)[-1].lower() if '.' in path else ''
    if ext == 'pcd':
        data, n_vertices = _parse_pcd(path)
    elif ext == 'las' or ext == 'laz':
        data, n_vertices = _parse_las(path)
    elif ext == 'ply':
        data, n_vertices = _parse_ply(path)
    else:
        raise ValueError(f"Unsupported file extension: {ext}")
    return data, n_vertices


class LoadedCloud:
    def __init__(self, data: Dict[str, np.ndarray]):
        self.lower_names = [n.lower() for n in data]
        self.data = data

    def _col(self, name: str) -> np.ndarray:
        for key in self.data:
            if key.lower() == name:
                return self.data[key]
        raise ValueError(f"Missing column {name} in cloud.")

    def _has(self, *names) -> bool:
        return all(n in self.lower_names for n in names)

    def get_xyz(self) -> np.ndarray:
        # --------------------------------------------------------------- xyz
        if not self._has('x', 'y', 'z'):
            raise ValueError("File does not contain x/y/z vertex properties.")

        return np.stack([self._col('x'), self._col('y'), self._col('z')], dtype=np.float64, axis=1)

    def get_rgb(self, ae=False) -> Optional[Callable[[], np.ndarray]]:
        # Explicit colour channels
        for r_name, g_name, b_name in (
            ('red', 'green', 'blue'),
            ('r', 'g', 'b'),
            ('diffuse_red', 'diffuse_green', 'diffuse_blue'),
        ):
            if self._has(r_name, g_name, b_name):
                def fun():
                    raw_rgb = np.stack([self._col(r_name), self._col(g_name), self._col(b_name)], axis=1)
                    # Normalise uint8 (0-255) or float values > 1 to [0, 1]
                    if raw_rgb.max() > 1.0:
                        raw_rgb = raw_rgb / 255.0

                    # Apply simple auto exposure if requested
                    if ae:
                        p95 = np.percentile(np.max(raw_rgb, axis=1), 95)
                        if p95 != 0:
                            raw_rgb *= 1.0 / p95
                    rgb = np.clip(raw_rgb, 0.0, 1.0)
                    return rgb
                return fun
        return None

    def get_normal_colors(self) -> Optional[Callable[[], np.ndarray]]:
        for nx_n, ny_n, nz_n in (('nx', 'ny', 'nz'), ('normal_x', 'normal_y', 'normal_z')):
            if self._has(nx_n, ny_n, nz_n):
                def fun():
                    normals = np.stack([self._col(nx_n), self._col(ny_n), self._col(nz_n)], axis=1)
                    rgb = np.clip((normals + 1.0) * 0.5, 0.0, 1.0)
                    return rgb
                return fun
        return None

    def get_xyz_colors(self, xyz: np.ndarray) -> Callable[[], np.ndarray]:
        def fun():
            xyz_min = xyz.min(axis=0)
            xyz_max = xyz.max(axis=0)
            xyz_range = np.where((xyz_max - xyz_min) == 0.0, 1.0, xyz_max - xyz_min)
            rgb = (xyz - xyz_min) / xyz_range
            return rgb
        return fun

    def get_all_scalar_colors(self) -> List[Tuple[str, Callable[[], np.ndarray]]]:
        xyz_set = {'x', 'y', 'z'}

        out = []
        for col in self.data:
            if col.lower() in xyz_set:
                continue

            out.append((col, self._scalar_to_colors(col.lower(), self.data[col])))
        return out

    def get_scalar_colors(self) -> Optional[Tuple[str, Callable[[], np.ndarray]]]:
        xyz_set = {'x', 'y', 'z'}
        extra = [n for n in self.lower_names if n not in xyz_set]
        scalar: Optional[np.ndarray] = None
        matched_candidate: Optional[str] = None

        # Named scalar candidates
        for candidate in ('intensity', 'scalar', 'value',
                          'reflectivity', 'signal', 'ambient', 'range'):
            if candidate in extra:
                scalar = self._col(candidate)
                matched_candidate = candidate
                break

        # Fall back: if there is exactly one extra property, treat it as scalar
        if scalar is None and len(extra) == 1:
            scalar = self._col(extra[0])
            matched_candidate = extra[0]

        if scalar is not None and matched_candidate is not None:
            return (matched_candidate, self._scalar_to_colors(matched_candidate, scalar))
        return None

    def _scalar_to_colors(self, name: str, scalar: np.ndarray) -> Callable[[], np.ndarray]:
        def fun(scalar):
            s_min, s_max = float(scalar.min()), float(scalar.max())
            if s_max > s_min:
                s_norm = (scalar - s_min) / (s_max - s_min)
            else:
                s_norm = np.zeros(len(scalar), dtype=np.float32)

            palette = calref_palette if name == 'reflectivity' else magma_palette
            palette_f = palette.astype(np.float32)
            idx = np.clip((s_norm * (len(palette_f) - 1)).astype(np.intp), 0, len(palette_f) - 1)
            rgb = palette_f[idx]
            return rgb
        return partial(fun, scalar)


def read_pointcloud_color(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Read a PLY, PCD, or LAS file and return xyz + color as a 2 (n, 3) arrays.

    The returned columns are [x, y, z, r, g, b] where r/g/b are in [0, 1].

    Color assignment rules (in priority order):
        1. If the file has explicit rgb/red/green/blue properties: use them.
        2. If the file has normal vectors (nx/ny/nz) but no color: map normals
           from [-1, 1] to [0, 1] as a falsecolor.
        3. If the file has a scalar property: apply calref_palette for
           reflectivity, otherwise magma_palette.
        4. If the file only has xyz: colour by normalised xyz coordinates.

    Args:
        path: Path to the PLY (ASCII or binary), PCD (ASCII or binary), or LAS file (binary).

    Returns:
        np.ndarray of shape (n, 3) for xyz and same for rgb
    """
    data, n_vertices = read_pointcloud(path)

    cloud = LoadedCloud(data)

    # --------------------------------------------------------------- xyz
    xyz = cloud.get_xyz()

    # --------------------------------------------------------------- color
    rgb: Optional[np.ndarray] = None

    # Explicit colour channels
    rgb_fun = cloud.get_rgb()
    if rgb_fun is not None:
        rgb = rgb_fun()

    # Use Normal vectors as a color
    if rgb is None:
        rgb_fun = cloud.get_normal_colors()
        if rgb_fun is not None:
            rgb = rgb_fun()

    # Scalar property → palette colour
    if rgb is None:
        scalar_data = cloud.get_scalar_colors()
        if scalar_data is not None:
            _, rgb_fun = scalar_data
            rgb = rgb_fun()

    # xyz only → colour derived from normalised xyz coordinates
    if rgb is None:
        rgb = cloud.get_xyz_colors(xyz)()

    return xyz, rgb
