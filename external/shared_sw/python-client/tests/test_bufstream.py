"""Test simple delimited binary io

Todo:
    - test reading from truncated file

"""

import random
import pytest
from itertools import islice, tee
from ouster.client import _bufstream as bufstream
from typing import Generator
from io import BytesIO

MAX_BUF_SZ = 2**16 - 1
PACKET_BUF_SZ = 12608


def randstream(*, min=0, max=MAX_BUF_SZ) -> Generator[bytes, None, None]:
    """Generate an iterables of random bytes."""
    while True:
        sz = random.randint(min, max)
        yield bytes(random.getrandbits(8) for _ in range(sz))


@pytest.mark.parametrize(
    "n_iter,n_bufs,min_buf_sz,max_buf_sz",
    # yapf: disable
    [
        pytest.param(10, 10, 10, 10, id="fixed_size_10"),
        pytest.param(10, 10, 0, 0, id="fixed_size_0"),
        pytest.param(1, 10, MAX_BUF_SZ, MAX_BUF_SZ, id="fixed_max_buf"),
        pytest.param(10, 10, 0, MAX_BUF_SZ, id="rand_size"),
        pytest.param(10, 0, 10, 10, id="zero_bufs"),
        pytest.param(1, 100, 0, 10, id="100_small_rand_size"),
        pytest.param(1, 64, PACKET_BUF_SZ, PACKET_BUF_SZ, id="64_packet_size"),
    ]
    # yapf: enable
)
def test_write_read(n_iter: int, n_bufs: int, min_buf_sz: int,
                    max_buf_sz: int) -> None:
    """Test that buffers survive a round trip through writing/reading."""
    for _ in range(n_iter):
        b0, b1 = tee(islice(randstream(), n_bufs))
        f = BytesIO()
        bufstream.write(f, b1)
        f.seek(0)
        b2 = bufstream.read(f)
        assert list(b0) == list(b2)


def test_write_too_big() -> None:
    """Test writing a buffer that's too big."""
    f = BytesIO()
    with pytest.raises(ValueError):
        bufstream.write(f, [bytes(2**16)])


def test_read_bad_magic() -> None:
    """Test reading a file with a bad magic number."""
    f = BytesIO(b"badmagic")
    with pytest.raises(ValueError):
        bufstream.read(f)


def test_read_closed() -> None:
    """Test reading a closed file."""
    f = BytesIO()
    f.close()
    with pytest.raises(ValueError):
        bufstream.read(f)


def test_write_closed() -> None:
    """Test writing a closed file."""
    f = BytesIO()
    f.close()
    with pytest.raises(ValueError):
        bufstream.write(f, [bytes()])
