import struct
import zlib
from typing import List, Optional, Tuple

import numpy as np
import pytest

from ouster.sdk.io import GifWriter, read_png, write_png


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    return (struct.pack(">I", len(data)) + kind + data +
            struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff))


def _make_png(width: int, height: int, bit_depth: int, color_type: int, raw_rows: List[bytes],
              palette: Optional[bytes] = None, trns: Optional[bytes] = None,
              interlace: int = 0) -> bytes:
    """Builds a PNG from already packed rows (one filter byte of 0 is prepended to each)."""
    ihdr = struct.pack(">IIBBBBB", width, height, bit_depth, color_type, 0, 0, interlace)
    out = b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", ihdr)
    if palette is not None:
        out += _png_chunk(b"PLTE", palette)
    if trns is not None:
        out += _png_chunk(b"tRNS", trns)
    out += _png_chunk(b"IDAT", zlib.compress(b"".join(b"\x00" + row for row in raw_rows)))
    return out + _png_chunk(b"IEND", b"")


def _parse_gif(data: bytes) -> Tuple[int, int, List[int]]:
    """Returns width, height and the delay in hundredths of a second of each frame."""
    assert data[:6] == b"GIF89a"
    width, height, packed = struct.unpack("<HHB", data[6:11])
    pos = 13
    if packed & 0x80:
        pos += 3 * 2 ** ((packed & 0x07) + 1)
    delays: List[int] = []
    frames = 0

    def skip_sub_blocks(pos: int) -> int:
        while data[pos] != 0:
            pos += data[pos] + 1
        return pos + 1

    while data[pos] != 0x3B:
        if data[pos] == 0x21:
            if data[pos + 1] == 0xF9:
                delays.append(struct.unpack("<H", data[pos + 4:pos + 6])[0])
            pos = skip_sub_blocks(pos + 2)
        elif data[pos] == 0x2C:
            frame_packed = data[pos + 9]
            pos += 10
            if frame_packed & 0x80:
                pos += 3 * 2 ** ((frame_packed & 0x07) + 1)
            pos = skip_sub_blocks(pos + 1)
            frames += 1
        else:
            raise AssertionError(f"unexpected GIF block 0x{data[pos]:02x} at {pos}")
    assert pos == len(data) - 1
    assert frames == len(delays)
    return width, height, delays


@pytest.mark.parametrize("shape", [(5, 7), (5, 7, 1), (5, 7, 2), (5, 7, 3), (5, 7, 4)])
def test_png_round_trip(tmp_path, shape):
    image = np.random.default_rng(0).integers(0, 256, size=shape, dtype=np.uint8)
    path = str(tmp_path / "image.png")
    write_png(path, image)
    result = read_png(path)
    expected = image[..., 0] if image.ndim == 3 and image.shape[2] == 1 else image
    assert result.dtype == np.uint8
    assert np.array_equal(result, expected)


def test_write_png_tags_srgb(tmp_path):
    path = tmp_path / "image.png"
    write_png(str(path), np.zeros((2, 3, 3), dtype=np.uint8))
    data = path.read_bytes()
    chunks: List[Tuple[bytes, bytes]] = []
    pos = 8
    while pos < len(data):
        length, kind = struct.unpack(">I4s", data[pos:pos + 8])
        chunks.append((kind, data[pos + 8:pos + 8 + length]))
        pos += 12 + length
    kinds = [kind for kind, _ in chunks]
    assert {b"sRGB", b"gAMA", b"cHRM"} <= set(kinds[:kinds.index(b"IDAT")])
    assert dict(chunks)[b"sRGB"] == b"\x00"  # perceptual rendering intent


def test_write_png_accepts_non_contiguous_arrays(tmp_path):
    image = np.arange(6 * 8, dtype=np.uint8).reshape(6, 8)
    path = str(tmp_path / "image.png")
    write_png(path, image[:, ::2])
    assert np.array_equal(read_png(path), image[:, ::2])


def test_write_png_rejects_invalid_images(tmp_path):
    path = str(tmp_path / "image.png")
    with pytest.raises(TypeError):
        write_png(path, np.zeros((4, 4), dtype=np.float32))
    with pytest.raises(ValueError):
        write_png(path, np.zeros((4, 4, 5), dtype=np.uint8))
    with pytest.raises(ValueError):
        write_png(path, np.zeros((2, 4, 4, 3), dtype=np.uint8))
    with pytest.raises(ValueError):
        write_png(path, np.zeros((0, 4), dtype=np.uint8))


def test_write_png_unwritable_path(tmp_path):
    with pytest.raises(RuntimeError):
        write_png(str(tmp_path / "missing_dir" / "image.png"), np.zeros((4, 4), dtype=np.uint8))


def test_read_png_rejects_missing_and_invalid_files(tmp_path):
    with pytest.raises(RuntimeError, match="failed to open"):
        read_png(str(tmp_path / "missing.png"))
    not_png = tmp_path / "image.jpg"
    not_png.write_bytes(b"\xff\xd8\xff\xe0 jpeg data")
    with pytest.raises(RuntimeError, match="not a PNG"):
        read_png(str(not_png))
    truncated = tmp_path / "truncated.png"
    good = _make_png(4, 4, 8, 0, [bytes(4)] * 4)
    truncated.write_bytes(good[:len(good) // 2])
    with pytest.raises(RuntimeError, match="failed to read"):
        read_png(str(truncated))


def test_read_png_expands_palette_with_transparency(tmp_path):
    palette = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255])
    data = _make_png(3, 1, 8, 3, [bytes([0, 1, 2])], palette=palette, trns=bytes([255, 128]))
    path = tmp_path / "palette.png"
    path.write_bytes(data)
    result = read_png(str(path))
    assert result.shape == (1, 3, 4)
    assert result[0].tolist() == [[255, 0, 0, 255], [0, 255, 0, 128], [0, 0, 255, 255]]


def test_read_png_scales_16_bit_to_8_bit(tmp_path):
    row = struct.pack(">HHH", 0, 0x8080, 0xFFFF)
    path = tmp_path / "gray16.png"
    path.write_bytes(_make_png(3, 1, 16, 0, [row]))
    assert read_png(str(path)).tolist() == [[0, 128, 255]]


def test_read_png_expands_low_bit_depth_gray(tmp_path):
    # 1 bit gray: 0b10100000 -> pixels 1, 0, 1, 0
    path = tmp_path / "gray1.png"
    path.write_bytes(_make_png(4, 1, 1, 0, [bytes([0b10100000])]))
    assert read_png(str(path)).tolist() == [[255, 0, 255, 0]]


def test_read_png_deinterlaces(tmp_path):
    image = np.arange(8 * 8, dtype=np.uint8).reshape(8, 8)
    # Adam7 passes as (row start, col start, row step, col step)
    passes = [(0, 0, 8, 8), (0, 4, 8, 8), (4, 0, 8, 4), (0, 2, 4, 4), (2, 0, 4, 2), (0, 1, 2, 2),
              (1, 0, 2, 1)]
    raw = b"".join(b"\x00" + image[r, c0::cs].tobytes()
                   for r0, c0, rs, cs in passes for r in range(r0, 8, rs))
    ihdr = struct.pack(">IIBBBBB", 8, 8, 8, 0, 0, 0, 1)
    data = (b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", ihdr) +
            _png_chunk(b"IDAT", zlib.compress(raw)) + _png_chunk(b"IEND", b""))
    path = tmp_path / "interlaced.png"
    path.write_bytes(data)
    assert np.array_equal(read_png(str(path)), image)


def test_gif_writer_frames_and_delays(tmp_path):
    path = tmp_path / "anim.gif"
    rng = np.random.default_rng(1)
    with GifWriter(str(path), 16, 8) as writer:
        assert (writer.width, writer.height) == (16, 8)
        writer.add_frame(rng.integers(0, 256, size=(8, 16, 3), dtype=np.uint8), 33.3)
        writer.add_frame(rng.integers(0, 256, size=(8, 16, 4), dtype=np.uint8), 33.3)
        writer.add_frame(rng.integers(0, 256, size=(8, 16, 3), dtype=np.uint8), 33.3)
        writer.add_frame(np.zeros((8, 16, 3), dtype=np.uint8), 0.0)
    width, height, delays = _parse_gif(path.read_bytes())
    assert (width, height) == (16, 8)
    # rounding carries over between frames, and every frame lasts at least 10 ms
    assert delays == [3, 4, 3, 1]


def test_gif_writer_rejects_invalid_input(tmp_path):
    path = str(tmp_path / "anim.gif")
    with pytest.raises(ValueError):
        GifWriter(path, 0, 8)
    with pytest.raises(ValueError):
        GifWriter(path, 70000, 8)
    with pytest.raises(RuntimeError):
        GifWriter(str(tmp_path / "missing_dir" / "anim.gif"), 4, 4)

    writer = GifWriter(path, 4, 4)
    with pytest.raises(ValueError):
        writer.add_frame(np.zeros((4, 5, 3), dtype=np.uint8), 10)
    with pytest.raises(ValueError):
        writer.add_frame(np.zeros((4, 4), dtype=np.uint8), 10)
    with pytest.raises(ValueError):
        writer.add_frame(np.zeros((4, 4, 2), dtype=np.uint8), 10)
    with pytest.raises(TypeError):
        writer.add_frame(np.zeros((4, 4, 3), dtype=np.float32), 10)
    writer.add_frame(np.zeros((4, 4, 3), dtype=np.uint8), 10)
    writer.close()
    writer.close()
    with pytest.raises(RuntimeError):
        writer.add_frame(np.zeros((4, 4, 3), dtype=np.uint8), 10)
    assert _parse_gif((tmp_path / "anim.gif").read_bytes())[2] == [1]
