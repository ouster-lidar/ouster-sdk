import pytest
import tempfile
import os
from tests.conftest import PCAPS_DATA_DIR
from ouster.sdk.client import SensorInfo
from os.path import commonprefix
from pathlib import Path
from ouster.sdk.util.metadata import resolve_metadata, \
    resolve_metadata_multi, data_must_be_a_file_err, meta_must_be_a_file_err


def test_resolve_metadata_when_data_not_a_file():
    """It should raise an exception if the data path is not a file"""
    with pytest.raises(ValueError, match=data_must_be_a_file_err):
        resolve_metadata('')


def test_resolve_metadata_when_data_not_a_file_2():
    """It should raise an exception if the data path is not a file"""
    with pytest.raises(ValueError, match=data_must_be_a_file_err):
        with tempfile.TemporaryDirectory() as directory:
            resolve_metadata(directory)


def test_resolve_metadata_when_metadata_path_provided_is_not_a_file():
    """It should raise an exception if the provided metadata path is
    not a file."""
    with pytest.raises(ValueError, match=meta_must_be_a_file_err):
        with tempfile.NamedTemporaryFile() as f:
            with tempfile.TemporaryDirectory() as directory:
                resolve_metadata(f.name, directory)


def test_resolve_metadata_min_prefix():
    """When there is no JSON file that has a common prefix with the data file,
    resolve_metadata should return None."""
    with tempfile.TemporaryDirectory() as directory:
        dir_path = Path(directory)
        # create some test files (one representing data, one metadata)

        test_data_filename = 'foo'
        test_meta_filename = 'tmpfile'
        assert not commonprefix([test_data_filename, test_meta_filename])
        open(dir_path / test_data_filename, 'a').close()
        open(dir_path / f'{test_meta_filename}.json', 'a').close()

        assert resolve_metadata(dir_path / test_data_filename) is None


def test_resolve_metadata_min_prefix_2():
    """The minimum common prefix between a data path and a resolved metadata
    file should have a length greater than zero."""
    with tempfile.TemporaryDirectory() as directory:
        dir_path = Path(directory)
        # create some test files (one representing data, one metadata)
        test_filename = 'tmpfile'
        open(dir_path / test_filename, 'a').close()
        open(dir_path / f'{test_filename}.json', 'a').close()

        assert resolve_metadata(dir_path / test_filename) == str(dir_path / f'{test_filename}.json')


def test_resolve_metadata_multi():
    """It should return an empty list if no JSON files match the data path."""
    with tempfile.TemporaryDirectory() as directory:
        dir_path = Path(directory)
        # create some test files (one representing data, one metadata)

        test_data_filename = 'foo'
        test_meta_filename = 'tmpfile'
        assert not commonprefix([test_data_filename, test_meta_filename])
        open(dir_path / test_data_filename, 'a').close()
        open(dir_path / f'{test_meta_filename}.json', 'a').close()

        assert resolve_metadata_multi(dir_path / test_data_filename) == []


def test_resolve_metadata_multi_2():
    """It should only files that exist and share a prefix with the data file."""
    # read files with below prefix from PCAPS_DATA_DIR, the serial numbers for metadata on each is different
    test_data_prefix = 'OS-0-128_v3.0.1_1024x10'
    assert set(resolve_metadata_multi(os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.pcap'))) == set(
        [str(os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.2.json')),
         str(os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.json'))])


def test_resolve_metadata_multi_exception_raised():
    # read files with below prefix from PCAPS_DATA_DIR, the serial numbers for metadata on each is same
    test_data_prefix = 'OS-0-128_v3.0.1_1024x10_20240321_125947'
    test_data_filename = os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.pcap')

    with open(os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.json')) as file:
        meta_content = file.read()
        si = SensorInfo(meta_content)
    metas = set([str(os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.2.json')),
                 str(os.path.join(PCAPS_DATA_DIR, f'{test_data_prefix}.json'))])

    with pytest.raises(RuntimeError) as excinfo:
        resolve_metadata_multi(test_data_filename)

    s = ["The following metadata files identified for "
         f"{test_data_filename} contain configuration for the same sensor {si.sn}. Files: "
         f"{metas}",
         "To resolve this, remove the extra metadata file(s) or specify the metadata "
         "files manually using the --meta option."]
    assert str(excinfo.value) == "\n".join(s)
