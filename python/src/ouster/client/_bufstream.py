"""Utility for reading and writing length-delimited binary data."""
from typing import BinaryIO, Generator, Iterable, Optional

from . import BufferT

MAGIC: int = 0xBA615BAD


def readmagic(f: BinaryIO) -> bool:
    """Check magic number in binary data."""
    magic = f.read(4)
    return len(magic) == 4 and int.from_bytes(magic, 'big') == MAGIC


def writemagic(f: BinaryIO) -> None:
    """Write magic number to binary output.

    Stored as big-endian to make it easily identifiable with `od -t x1`.
    """
    f.write(MAGIC.to_bytes(4, 'big'))


def readdelim(f: BinaryIO) -> Optional[bytes]:
    """Read 16-bit length-delimited binary value."""
    delim = f.read(2)
    if len(delim) == 0:
        return None
    elif len(delim) != 2:
        raise IOError("Failed to read delimiter")
    sz = int.from_bytes(delim, 'little')
    buf = f.read(sz)
    if len(buf) != sz:
        raise IOError("Failed to read data")
    return buf


def writedelim(f: BinaryIO, buf: bytes) -> None:
    """Write 16-bit length-delimeted binary value."""
    sz = len(buf)
    try:
        delim = sz.to_bytes(2, 'little')
    except OverflowError:
        raise ValueError('len(buf) >= 2**16') from None

    if f.write(delim) != 2:
        raise IOError("Failed to write delimiter")

    if f.write(buf) != sz:
        raise IOError("Failed to write data")


def read(f: BinaryIO) -> Generator[bytes, None, None]:
    """Read 16-bit length-delimited binary data."""
    if not readmagic(f):
        raise ValueError('Failed to read magic number')

    def g():
        while True:
            buf = readdelim(f)
            if buf is not None:
                yield buf
            else:
                return

    return g()


def write(f: BinaryIO, data: Iterable[BufferT]) -> None:
    """Write 16-bit length-delimited binary data."""
    writemagic(f)
    for buf in data:
        writedelim(f, bytes(buf))
