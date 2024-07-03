import os
import tempfile
import pytest
import ouster.cli.core.util


def test_md5file():
    try:
        f = tempfile.NamedTemporaryFile(delete=False)
        f.write(b'foo')
        f.close()
        assert ouster.cli.core.util.md5file(f.name) == "acbd18db4cc2f85cedef654fccc4a4d8"
    finally:
        os.unlink(f.name)


def test_md5file_doesntexist():
    with pytest.raises(FileNotFoundError):
        ouster.cli.core.util.md5file('doesntexist')
