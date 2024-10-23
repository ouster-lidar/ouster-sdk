import os
import tempfile
import pytest
from pathlib import Path
from ouster.cli.core import cli  # type: ignore
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.plugins import source, source_osf  # noqa: F401
from ouster.sdk import open_source
from click.testing import CliRunner
from tests.conftest import OSFS_DATA_DIR


# TODO[tws] these tests should be unit tests, not CliRunner-based tests.
# Refactor the underlying code in the process_commands method to enable this.

# TODO[tws] these tests are also using the open_source method with sensor_index=-1
# which enables us to close the file using the close method.
# When using sensor_index > -1, this isn't possible, resulting in an error on Windows
# when attempting to unlink the file in the finally block. This may be confusing
# for API users and may result in bugs.


@pytest.fixture
def test_osf_file() -> str:
    return str(Path(OSFS_DATA_DIR) / 'OS-1-128_v2.3.0_1024x10_lb_n3.osf')


@pytest.fixture
def test_osf_file_new() -> str:
    return str(Path(OSFS_DATA_DIR) / 'OS-0-128_v3.0.1_1024x10_20241017_141645.osf')


def test_osf_slice(test_osf_file) -> None:
    """It should display a warning if slicing out of bounds."""
    num_scans_in_src = 3
    # check precondition
    src = open_source(test_osf_file)
    assert src.scans_num == num_scans_in_src
    src.close()

    try:
        with tempfile.NamedTemporaryFile(suffix='.osf', delete=False) as result_osf:
            result_osf.close()
            runner = CliRunner()
            result = runner.invoke(cli,
                CliArgs([
                    '--traceback',
                    'source', test_osf_file,
                    'slice', f'{num_scans_in_src}:{num_scans_in_src + 20}',  # note - out of bounds
                    'save', '--ts', 'lidar', '--overwrite', result_osf.name
                ]).args, catch_exceptions=False
            )
            print(result.output)
            assert result.exit_code == 0  # FIXME?
            assert os.path.isfile(result_osf.name)

            result_src = open_source(result_osf.name, -1)
            assert result_src.scans_num == []  # FIXME
            result_src.close()
    finally:
        os.unlink(result_osf.name)


def test_osf_slice_2(test_osf_file) -> None:
    """It should slice by indices."""

    expected_num_scans = 3
    slice_start = 1
    slice_end = 3
    # check precondition
    src = open_source(test_osf_file)
    scans = [scan for scan in src]
    assert src.scans_num == expected_num_scans
    src.close()

    try:
        with tempfile.NamedTemporaryFile(suffix='.osf', delete=False) as result_osf:
            result_osf.close()
            runner = CliRunner()
            result = runner.invoke(cli,
                CliArgs([
                    '--traceback',
                    'source', test_osf_file,
                    'slice', f'{slice_start}:{slice_end}',  # note - out of bounds
                    'save', '--ts', 'lidar', '--overwrite', result_osf.name
                ]).args, catch_exceptions=False
            )
            assert result.exit_code == 0  # FIXME?
            assert os.path.isfile(result_osf.name)

            result_src = open_source(result_osf.name, -1)
            result_scans = [scan for scan in result_src]
            assert result_src.scans_num == [slice_end - slice_start]
            assert result_scans[0] == [scans[1]]
            assert result_scans[1] == [scans[2]]
            result_src.close()
    finally:
        os.unlink(result_osf.name)


def test_osf_slice_time(test_osf_file_new) -> None:
    """It should display a warning if slicing out of bounds."""

    expected_num_scans = 3
    # check precondition
    src = open_source(test_osf_file_new)
    assert src.scans_num == expected_num_scans
    src.close()

    try:
        with tempfile.NamedTemporaryFile(suffix='.osf', delete=False) as result_osf:
            result_osf.close()
            runner = CliRunner()
            result = runner.invoke(cli,
                CliArgs([
                    '--traceback',
                    'source', test_osf_file_new,
                    'slice', '60s:120s',
                    'save', '--overwrite', result_osf.name
                ]).args, catch_exceptions=False)
            assert result.exit_code == 0  # FIXME?
            print(result.output)
            assert os.path.isfile(result_osf.name)

            result_src = open_source(result_osf.name, -1)
            assert result_src.scans_num == []
            # TODO[tws] figure out how to capture "WARNING: No scans saved."
            result_src.close()
    finally:
        os.unlink(result_osf.name)


# FIXME[tws]?
# Slicing by time yields scans when using --ts lidar
# Note - test_osf_file has no packet timestamps
def test_osf_slice_time_2(test_osf_file) -> None:
    """It will include all scans if --ts lidar is used."""

    expected_num_scans = 3
    # check precondition
    src = open_source(test_osf_file)
    assert src.scans_num == expected_num_scans
    src.close()

    try:
        with tempfile.NamedTemporaryFile(suffix='.osf', delete=False) as result_osf:
            result_osf.close()
            runner = CliRunner()
            result = runner.invoke(cli,
                CliArgs([
                    '--traceback',
                    'source', test_osf_file,
                    'slice', '0ms:00001ms',  # <-- note, should probably only result in a single scan
                    'save',
                    '--ts', 'lidar',  # <-- NOTE using lidar timestamps
                    '--overwrite', result_osf.name
                ]).args, catch_exceptions=False)
            assert result.exit_code == 0  # FIXME?
            print(result.output)
            assert os.path.isfile(result_osf.name)

            result_src = open_source(result_osf.name, -1)
            assert result_src.scans_num == [3]
            # TODO[tws] figure out how to capture "WARNING: No scans saved."
            result_src.close()
    finally:
        os.unlink(result_osf.name)


# FIXME[tws]?
def test_osf_slice_time_3(test_osf_file_new) -> None:
    """It will allow a time slice stop value equal to zero, which evalutes to False."""

    expected_num_scans = 3
    # check precondition
    src = open_source(test_osf_file_new)
    assert src.scans_num == expected_num_scans
    src.close()

    try:
        with tempfile.NamedTemporaryFile(suffix='.osf', delete=False) as result_osf:
            result_osf.close()
            runner = CliRunner()
            result = runner.invoke(cli,
                CliArgs([
                    '--traceback',
                    'source', test_osf_file_new,
                    'slice', '0.0000ms:0.0000ms',
                    'save', '--overwrite', result_osf.name
                ]).args, catch_exceptions=False)
            assert result.exit_code == 0  # FIXME?
            print(result.output)
            assert os.path.isfile(result_osf.name)

            result_src = open_source(result_osf.name, -1)
            assert result_src.scans_num == [expected_num_scans]  # FIXME?
            result_src.close()
    finally:
        os.unlink(result_osf.name)


def test_osf_slice_time_4(test_osf_file_new) -> None:
    """It will allow a time slice stop value equal to the start value."""

    expected_num_scans = 3
    # check precondition
    src = open_source(test_osf_file_new)
    assert src.scans_num == expected_num_scans
    src.close()

    try:
        with tempfile.NamedTemporaryFile(suffix='.osf', delete=False) as result_osf:
            result_osf.close()
            runner = CliRunner()
            result = runner.invoke(cli,
                CliArgs([
                    '--traceback',
                    'source', test_osf_file_new,
                    'slice', '1.0000ms:1.0000ms',  # TODO[tws] should we force the user to specify stop > start?
                    'save', '--overwrite', result_osf.name
                ]).args, catch_exceptions=False)
            assert result.exit_code == 0  # FIXME?
            print(result.output)
            assert os.path.isfile(result_osf.name)

            result_src = open_source(result_osf.name, -1)
            assert result_src.scans_num == []
            result_src.close()
    finally:
        os.unlink(result_osf.name)


def test_osf_slice_time_5(test_osf_file_new) -> None:
    """It should slice an OSF file by time."""

    scans_in_src = 3
    # check precondition
    src = open_source(test_osf_file_new)
    assert src.scans_num == scans_in_src
    src.close()

    try:
        with tempfile.NamedTemporaryFile(suffix='.osf', delete=False) as result_osf:
            result_osf.close()
            runner = CliRunner()
            result = runner.invoke(cli,
                CliArgs([
                    '--traceback',
                    'source', test_osf_file_new,
                    'slice', '0ms:0.0001ms',   # should only return a single scan
                    'save', '--overwrite', result_osf.name
                ]).args, catch_exceptions=False)
            assert result.exit_code == 0
            print(result.output)
            assert os.path.isfile(result_osf.name)

            result_src = open_source(result_osf.name, -1)
            assert result_src.scans_num == [1]
            result_src.close()
    finally:
        os.unlink(result_osf.name)
