import numpy as np
from typing import Optional

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
        else:
            endian = '<' if fmt == 'binary_little_endian' else '>'
            np_dtype = np.dtype([(name, dt.newbyteorder(endian)) for name, dt in valid_props])
            structured = np.frombuffer(fh.read(n_vertices * np_dtype.itemsize), dtype=np_dtype)
            prop_names = [name for name, _ in valid_props]
            data = np.zeros((n_vertices, len(valid_props)), dtype=np.float64)
            for i, name in enumerate(prop_names):
                data[:, i] = structured[name].astype(np.float64)

    return prop_names, data, n_vertices


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

        if data_fmt == 'ascii':
            rows = [list(map(float, fh.readline().decode('ascii').split()))
                    for _ in range(n_points)]
            raw_data = np.array(rows, dtype=np.float64)
        else:  # binary (binary_compressed not supported)
            row_dtype = np.dtype([(n, dt) for n, dt in zip(col_names, col_dtypes)])
            buf = fh.read(n_points * row_dtype.itemsize)
            structured = np.frombuffer(buf, dtype=row_dtype)
            raw_data = np.zeros((n_points, len(col_names)), dtype=np.float64)
            for i, name in enumerate(col_names):
                raw_data[:, i] = structured[name].astype(np.float64)

    # Expand packed rgb/rgba float field into red/green/blue columns
    prop_names = []
    columns = []
    for i, name in enumerate(col_names):
        if name in ('rgb', 'rgba'):
            packed_u32 = raw_data[:, i].astype(np.float32).view(np.uint32)
            prop_names += ['red', 'green', 'blue']
            columns.append(((packed_u32 >> 16) & 0xFF).astype(np.float64))
            columns.append(((packed_u32 >> 8) & 0xFF).astype(np.float64))
            columns.append((packed_u32 & 0xFF).astype(np.float64))
        else:
            prop_names.append(name)
            columns.append(raw_data[:, i])

    data = np.column_stack(columns) if columns else np.empty((n_points, 0))
    return prop_names, data, n_points


def read_pointcloud_color(path: str) -> np.ndarray:
    """Read a PLY or PCD file and return xyz + color as an (n, 6) float64 array.

    The returned columns are [x, y, z, r, g, b] where r/g/b are in [0, 1].

    Color assignment rules (in priority order):
        1. If the file has explicit rgb/red/green/blue properties: use them.
        2. If the file has normal vectors (nx/ny/nz) but no color: map normals
           from [-1, 1] to [0, 1] as a falsecolor.
        3. If the file has a scalar property: apply calref_palette for
           reflectivity, otherwise magma_palette.
        4. If the file only has xyz: colour by normalised xyz coordinates.

    Args:
        path: Path to the PLY or PCD file (ASCII or binary).

    Returns:
        np.ndarray of shape (n, 6) and dtype float64.
    """
    ext = path.rsplit('.', 1)[-1].lower() if '.' in path else ''
    if ext == 'pcd':
        prop_names, data, n_vertices = _parse_pcd(path)
    else:
        prop_names, data, n_vertices = _parse_ply(path)

    lower_names = [n.lower() for n in prop_names]

    def _col(name: str) -> np.ndarray:
        return data[:, lower_names.index(name)]

    def _has(*names) -> bool:
        return all(n in lower_names for n in names)

    # --------------------------------------------------------------- xyz
    if not _has('x', 'y', 'z'):
        raise ValueError(f"File '{path}' does not contain x/y/z vertex properties.")

    xyz = data[:, [lower_names.index('x'), lower_names.index('y'), lower_names.index('z')]]

    # --------------------------------------------------------------- color
    rgb: Optional[np.ndarray] = None

    # Explicit colour channels
    for r_name, g_name, b_name in (
        ('red', 'green', 'blue'),
        ('r', 'g', 'b'),
        ('diffuse_red', 'diffuse_green', 'diffuse_blue'),
    ):
        if _has(r_name, g_name, b_name):
            raw_rgb = np.stack([_col(r_name), _col(g_name), _col(b_name)], axis=1)
            # Normalise uint8 (0-255) or float values > 1 to [0, 1]
            if raw_rgb.max() > 1.0:
                raw_rgb = raw_rgb / 255.0
            rgb = np.clip(raw_rgb, 0.0, 1.0)
            break

    # Use Normal vectors as a color
    if rgb is None:
        for nx_n, ny_n, nz_n in (('nx', 'ny', 'nz'), ('normal_x', 'normal_y', 'normal_z')):
            if _has(nx_n, ny_n, nz_n):
                normals = np.stack([_col(nx_n), _col(ny_n), _col(nz_n)], axis=1)
                rgb = np.clip((normals + 1.0) * 0.5, 0.0, 1.0)
                break

    # Scalar property → palette colour
    if rgb is None:
        xyz_set = {'x', 'y', 'z'}
        extra = [n for n in lower_names if n not in xyz_set]
        scalar: Optional[np.ndarray] = None
        matched_candidate: Optional[str] = None

        # Named scalar candidates
        for candidate in ('intensity', 'scalar', 'value',
                          'reflectivity', 'signal', 'ambient', 'range'):
            if candidate in extra:
                scalar = _col(candidate)
                matched_candidate = candidate
                break

        # Fall back: if there is exactly one extra property, treat it as scalar
        if scalar is None and len(extra) == 1:
            scalar = _col(extra[0])
            matched_candidate = extra[0]

        if scalar is not None:
            s_min, s_max = float(scalar.min()), float(scalar.max())
            if s_max > s_min:
                s_norm = (scalar - s_min) / (s_max - s_min)
            else:
                s_norm = np.zeros(n_vertices, dtype=np.float64)

            palette = calref_palette if matched_candidate == 'reflectivity' else magma_palette
            palette_f = palette.astype(np.float64)
            idx = np.clip((s_norm * (len(palette_f) - 1)).astype(np.intp), 0, len(palette_f) - 1)
            rgb = palette_f[idx]

    # xyz only → colour derived from normalised xyz coordinates
    if rgb is None:
        xyz_min = xyz.min(axis=0)
        xyz_max = xyz.max(axis=0)
        xyz_range = np.where((xyz_max - xyz_min) == 0.0, 1.0, xyz_max - xyz_min)
        rgb = (xyz - xyz_min) / xyz_range

    return np.hstack([xyz, rgb], dtype=np.float64)
