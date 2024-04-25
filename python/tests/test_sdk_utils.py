import pytest
import tempfile
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
    with tempfile.TemporaryDirectory() as directory:
        dir_path = Path(directory)
        # create some test files (one representing data, one metadata)

        test_data_filename = 'tmpfile'
        test_meta_filename = 'tmpfile'
        open(dir_path / test_data_filename, 'a').close()
        open(dir_path / f'{test_meta_filename}.json', 'a').close()
        open(dir_path / f'{test_meta_filename}.2.json', 'a').close()

        assert set(resolve_metadata_multi(dir_path / test_data_filename)) == set([
            str(Path(dir_path) / f'{test_meta_filename}.json'), str(Path(dir_path) / f'{test_meta_filename}.2.json')
        ])
