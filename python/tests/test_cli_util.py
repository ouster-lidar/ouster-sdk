import os
import json
import tempfile
import pytest
from ouster.cli import core
import ouster.cli.core.util
from click.testing import CliRunner
from tests.conftest import METADATA_DATA_DIR


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


def test_convert_metadata_to_legacy():
    runner = CliRunner()
    result = runner.invoke(core.cli, ['util', 'convert-metadata-to-legacy'])
    assert result.exit_code == 2
    assert "Error: Missing argument 'META'." in result.output


def test_convert_metadata_to_legacy_2():
    try:
        f = tempfile.NamedTemporaryFile(delete=False)
        f.close()
        runner = CliRunner()
        test_metadata_file = os.path.join(METADATA_DATA_DIR, '3_0_1_os-122246000293-128.json')
        legacy_test_metadata_file = os.path.join(METADATA_DATA_DIR, '3_0_1_os-122246000293-128_legacy.json')
        result = runner.invoke(core.cli, ['util', 'convert-metadata-to-legacy', test_metadata_file])
        assert result.exit_code == 0
        assert json.loads(result.output) == json.load(open(legacy_test_metadata_file))
    finally:
        os.unlink(f.name)
