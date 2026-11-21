#  type: ignore
import os
import inspect
from glob import glob
from pathlib import Path
import click
import pytest
import sys
import json
import tempfile
from typing import List
from click.testing import CliRunner

from ouster.cli import core
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.plugins import source, source_osf  # noqa: F401
from ouster.sdk.core import io_type_from_extension, OusterIoType, FrameSet, Object
from ouster.sdk import open_source

from tests.conftest import PCAPS_DATA_DIR, OSFS_DATA_DIR
import ouster.sdk._bindings.osf as osf
import numpy as np


class set_directory(object):
    """Sets the cwd within the context

    Args:
      path (Path): The path to the cwd
    """
    def __init__(self, path: Path):
        self.path = path
        self.origin = Path().absolute()

    def __enter__(self):
        os.chdir(self.path)

    def __exit__(self, exc_type, exc_value, traceback):
        os.chdir(self.origin)


@pytest.fixture
def test_osf_file() -> str:
    return str(Path(OSFS_DATA_DIR) / 'OS-0-128_v3.0.1_1024x10_20241017_141645.osf')


@pytest.fixture
def test_pcap_file() -> str:
    return str(Path(PCAPS_DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')


@pytest.fixture
def test_pcap_file2() -> str:
    return str(Path(PCAPS_DATA_DIR) / 'OS-1-128_v2.3.0_1024x10_lb_n3.pcap')


@pytest.fixture
def test_metadata_file() -> str:
    return str(Path(PCAPS_DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.json')


@pytest.fixture
def test_imu_osf_file() -> str:
    """One frame of a handheld OS-1-256 recording (beam-reduced to 32) with
    the FW 3.2+ IMU profile, RANGE and IMU fields only. The sensor is
    pitched ~16 degrees and actively pitching, so the IMU anchor, deskew,
    and wall capture range all get exercised."""
    return str(Path(OSFS_DATA_DIR) / 'OS-1-256_pitched_imu_n1.osf')


@pytest.fixture
def multisensor_osf_file() -> str:
    """One frame of a dual-sensor no-IMU recording (two OS-1s on poles
    across an intersection, beam-reduced to 16, RANGE only). Exercises the
    walls-only degrade path, per-sensor IMU warnings, and the shared
    correction across sensors."""
    return str(Path(OSFS_DATA_DIR) / 'OS-1-16x2_no_imu_tilted_n1.osf')


@pytest.fixture
def runner():
    return CliRunner()


def read_commands_from_help_text(help_text: str) -> List[str]:
    """Reads the command names (but not their help text)
    from help text that Click generates for click.MultiCommand"""
    command_help_lines = help_text.split("Commands:")[1].splitlines()[1:]
    return set(line.split()[0].strip() for line in command_help_lines)


def test_16x1_extrinsics(test_osf_file, tmp_path):

    runner = CliRunner()
    extrinsics_16x1 = (
        "0.000001,-0.999991,0.004236,-0.080000,-0.999987,-0.000023,-0.005052,-0.200000,"
        "0.005052,-0.004236,-0.999978,0.689000,0.000000,0.000000,0.000000,1.000000"
    )

    tmp_osf_file = str(Path(tmp_path) / "tmp_file.osf")

    result = runner.invoke(core.cli, ['source', '-e', extrinsics_16x1,
                                      test_osf_file, 'save', '--ts', 'lidar', tmp_osf_file])
    expected_extrinsics = np.array([
        0.000001, -0.999991, 0.004236, -0.080000,
        -0.999987, -0.000023, -0.005052, -0.200000,
        0.005052, -0.004236, -0.999978, 0.689000,
        0.000000, 0.000000, 0.000000, 1.000000
    ])

    tmp_osf = osf.OsfFrameSetSource(tmp_osf_file)

    extrinsic_from_osf = np.array(tmp_osf.sensor_info[0].sensor_to_body).flatten()

    assert np.allclose(extrinsic_from_osf, expected_extrinsics, atol=0.0001)
    assert result.exit_code == 0


def test_join_with_conjunction():
    assert source._join_with_conjunction([]) == ''
    assert source._join_with_conjunction(['one thing']) == 'one thing'
    assert source._join_with_conjunction([1, 2, 3]) == '1, 2, or 3'
    assert source._join_with_conjunction([1, 2, 3], conjunction='and') == '1, 2, and 3'
    assert source._join_with_conjunction(['foo', 'bar']) == 'foo or bar'
    assert source._join_with_conjunction(['foo', 'bar', 'baz'], separator='; ') == 'foo; bar; or baz'


def test_cli_args_borg() -> None:
    """If no args are provided to the constructor,
    the `args` attribute of new instances should contain sys.argv[1:]

    Note: see the definition for CliArgs to understand how it works.
    It is used to provide access to all cli args regardless of whether
    the command is run from the command line, or whether the command
    is invoked via click.testing.CliRunner.

    This is necessary because click provides no method to get all cli args
    from the context, which means it's sometimes difficult to determine
    when the user has invoked --help.
    """
    del CliArgs().args
    assert CliArgs().args == sys.argv[1:]
    assert CliArgs().args == sys.argv[1:]


def test_cli_args_borg_2() -> None:
    """If a list of args is provided to the constructor,
    the `args` attribute of new instances should the list."""
    del CliArgs().args
    CliArgs(['a', 'b', 'c'])
    assert CliArgs().args == ['a', 'b', 'c']
    assert CliArgs().has_any_of(['a', 'b'])
    assert not CliArgs().has_any_of(['d', 'e'])

    CliArgs(['d', 'e', 'f'])
    assert CliArgs().args == ['d', 'e', 'f']
    assert CliArgs().has_any_of(['d', 'e'])
    assert not CliArgs().has_any_of(['a', 'b'])


def test_io_type_from_extension() -> None:
    test_osf_name = 'OS1-inters-n5.osf'
    assert io_type_from_extension(test_osf_name) == OusterIoType.OSF
    test_pcap_name = 'data-inters-24784-OS1_128_fw23_legacy_n3.pcap'
    assert io_type_from_extension(test_pcap_name) == OusterIoType.PCAP
    test_bag_name = 'OS1_128_sample_fw23_lb_n3.bag'
    assert io_type_from_extension(test_bag_name) == OusterIoType.BAG


def test_version(runner) -> None:
    result = runner.invoke(core.cli, ['--version'])
    assert "cli, version " in result.output
    assert result.exit_code == 0


def test_help(runner) -> None:
    result = runner.invoke(core.cli, ['--help'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, CliArgs(['util', '--help']).args)
    assert "Usage: cli util [OPTIONS] COMMAND [ARGS]" in result.output
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['--traceback', 'util'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['--sdk-log-level', 'debug', 'util'])
    assert result.exit_code == 0


def test_mapping_help(runner, has_mapping):
    result = runner.invoke(core.cli, ['mapping', '--help'])
    if has_mapping:
        assert result.exit_code == 0


def test_source_help(runner) -> None:
    """It should return 0 if --help is specified."""
    result = runner.invoke(core.cli, CliArgs(['source', '--help']).args)

    # check that a variety of SOURCE commands are in the output
    assert "SENSOR|PCAP|OSF|URL|BAG|MCAP" in result.output
    assert "config" in result.output
    assert "live_zones" in result.output

    # check that general message is there
    assert "Run a command with the specified source" in result.output
    assert result.exit_code == 0


def test_source_no_args(runner) -> None:
    """It should return a usage error if no command is specified."""
    result = runner.invoke(core.cli, CliArgs(['source']).args)
    assert "Error: Missing argument 'SOURCE'." in result.output
    assert result.exit_code == 2


def test_source_sensor(runner, has_mapping) -> None:
    """It should list the correct commands
    in the help depending on source type."""

    # sensor
    result = runner.invoke(core.cli, ['source', '127.0.0.1'])
    assert result.exit_code == 0
    expected_commands = {'config', 'sensor_info', 'viz', 'slice', 'save'}
    if has_mapping:
        expected_commands.add('slam')
    assert set(read_commands_from_help_text(result.output)) >= expected_commands


def test_source_pcap(runner, has_mapping) -> None:
    # pcap
    expected_commands = {'info', 'slice', 'viz'}
    if has_mapping:
        expected_commands.add('slam')
    with tempfile.NamedTemporaryFile(suffix='.pcap') as temp_pcap:
        result = runner.invoke(core.cli, ['source', temp_pcap.name])
        assert result.exit_code == 0
        assert read_commands_from_help_text(result.output) >= expected_commands


@pytest.mark.skip
def test_source_rosbag() -> None:
    # TODO FLEETSW-4407: not MVP
    # rosbag
    # with tempfile.NamedTemporaryFile(suffix='.bag') as temp_pcap:
    #    result = runner.invoke(core.cli, ['source', temp_pcap.name])
    #    assert result.exit_code == 0
    #    commands = result.output.split("Commands:")[1].split()
    #    assert commands == ['convert']
    pass


def test_source_invald(runner) -> None:
    # invalid file type
    with tempfile.NamedTemporaryFile(suffix='.invalid') as temp_pcap:
        result = runner.invoke(core.cli, ['source', temp_pcap.name])
        assert "Source type expected to be a sensor hostname, ip address,"
        "or a .bag, .osf, or .pcap file" in result.output
        assert result.exit_code == 2


def test_source_bad_command(runner):
    """It should exit 2 (see click.exceptions.UsageError) if a bad command
    for the given source was provided."""
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'badcommand'])
    assert "Error: No such command 'badcommand'." in result.output
    assert result.exit_code == 2


def test_source_good_command(runner):
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'config'])
    assert "Error: CurlClient::execute_request failed" in result.output
    assert result.exit_code == 1


def test_source_could_not_resolve(runner):
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', 'badhostname', 'config'])
    # TODO: uncomment when bag is possible source
    # assert ("Error: Source type expected to be a sensor hostname, ip address, "
    # "or a .bag, .osf, or .pcap file.") in result.output
    assert ("Error: Source type expected to be") in result.output
    assert result.exit_code == 2


def test_source_config(runner):
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'config'])
    assert "Error: CurlClient::execute_request failed" in result.output
    assert result.exit_code == 1


def test_source_sensor_info(runner):
    """It should attempt to get sensor_info (and fail when there is no sensor)"""
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'sensor_info'])
    assert "Error: CurlClient::execute_request failed" in result.output
    assert result.exit_code == 1


def test_source_config_help(runner):
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'config', '--help'])
    assert "Usage: cli source SOURCE config [OPTIONS] [KEY VAL]..." in result.output
    assert result.exit_code == 0


def test_source_pcap_info(test_pcap_file, runner):
    """source <pcap> info should work as expected."""
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'info', '-n', 10])
    assert "Packets read:  10" in result.output
    assert result.exit_code == 0


def test_source_pcap_slice_help(test_pcap_file, runner):
    """ouster-cli source <src> slice --help
    should display help"""
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice', '--help'])
    assert "Usage: cli source SOURCE slice [OPTIONS] INDICES" in result.output
    assert result.exit_code == 0


def test_source_pcap_slice_no_arguments(test_pcap_file, runner):
    # help option not provided, but no output file provided
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice'])
    assert "Usage: cli source SOURCE slice [OPTIONS] INDICES" in result.output
    assert "Missing argument 'INDICES'." in result.output
    assert result.exit_code == 2


def test_source_pcap_slice_help_2(test_pcap_file, runner):
    """ouster-cli source <src> slice <output> --help
    should display help"""
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice', 'outputfile.pcap', '--help'])
    assert result.exit_code == 2
    assert "Error: Invalid value for 'INDICES'" in result.output


def test_unrepeatable_command_is_rejected(test_osf_file, runner):
    """Repeating a PROCESSOR_UNREPEATABLE command in a multi-command chain
    should fail with a usage error.

    This tests the duplicate-command lint itself; slice just serves as an
    example unrepeatable command. If the command used here ever becomes
    repeatable, swap in another PROCESSOR_UNREPEATABLE command.
    """
    result = runner.invoke(core.cli,
        CliArgs([
            'source', test_osf_file,
            'slice', '1:3',
            'slice', '0:1',
            'save', '/dev/null.osf'
        ]).args)
    assert result.exit_code != 0
    assert "'slice' is duplicated" in result.output


def source_pcap_slice_impl(test_pcap_file, runner, command, packets, should_fail=False):
    try:
        pcap_filename = None
        with tempfile.NamedTemporaryFile(delete=False) as f:
            pass
        result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice',
                                          command, 'save', '-p', f.name, ".pcap"])
        # FIXME! Written file paths should be logged in output.
        # assert f'Writing: {f.name}' in result.output
        print(result.output)
        if should_fail:
            assert result.exit_code != 0
            return
        assert result.exit_code == 0
        pcaps_generated = glob(f'{f.name}_*.pcap')
        assert len(pcaps_generated) == 1
        pcap_filename = pcaps_generated[0]
        result2 = runner.invoke(core.cli, ['source', pcap_filename, 'info'])
        print(result2.output)
        assert result2.exit_code == 0
        assert "Packets read:  " + packets in result2.output
    finally:
        os.unlink(f'{f.name}')
        if pcap_filename is not None:
            json_filename = pcap_filename[:-5] + '_0.json'
            os.unlink(pcap_filename)
            os.unlink(json_filename)


def test_source_pcap_slice(test_pcap_file, runner):
    """Slicing a pcap should succeed with exit code 0."""
    source_pcap_slice_impl(test_pcap_file, runner, "0:1:1", "64")
    source_pcap_slice_impl(test_pcap_file, runner, "0:1:2", "64")
    source_pcap_slice_impl(test_pcap_file, runner, "0:1:3", "64")
    source_pcap_slice_impl(test_pcap_file, runner, "0:2", "64")
    source_pcap_slice_impl(test_pcap_file, runner, "0:", "64")
    source_pcap_slice_impl(test_pcap_file, runner, "2:", "0")
    source_pcap_slice_impl(test_pcap_file, runner, "1::", "0")
    source_pcap_slice_impl(test_pcap_file, runner, "1::1", "0")
    source_pcap_slice_impl(test_pcap_file, runner, "0s::1", "64")
    source_pcap_slice_impl(test_pcap_file, runner, "0min:1h:1", "64")
    source_pcap_slice_impl(test_pcap_file, runner, "10ms:", "0")
    source_pcap_slice_impl(test_pcap_file, runner, "0s::2", "64")
    source_pcap_slice_impl(test_pcap_file, runner, "1s::1", "0")
    source_pcap_slice_impl(test_pcap_file, runner, "1s::", "0")
    # these cases look legit to me: :1s:1, 0:1s, 1.5:3.5
    # add some bad cases that should error
    fail_cases = ["1s:1:", "'-1s::'", ".s:0.s", "0::0", ".:.:3", "::-1", "3:1"]
    for case in fail_cases:
        source_pcap_slice_impl(test_pcap_file, runner, case, "0", True)


def test_source_pcap_save_no_filename(test_pcap_file, runner, tmp_path):
    """It should save an automatically named file with that extension by default."""
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'save', '.osf']).args)
        assert result.exit_code == 0
        # there's at most one OSF file in output dir
        files = os.listdir(tmp_path)
        assert len(files) == 1
        assert all(filename.endswith('.osf') for filename in files)


def test_source_pcap_save_filename(test_pcap_file, runner, tmp_path):
    """It should save an osf file with the desired name."""
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'save', 'test.osf']).args)
        assert result.exit_code == 0
        # there's at most one OSF file in output dir
        files = os.listdir(tmp_path)
        assert len(files) == 1
        assert all(filename == 'test.osf' for filename in files)


def test_source_pcap_save_filename_prefix(test_pcap_file, runner, tmp_path):
    """It should save an osf file with the desired name."""
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'save', '-p', 'prefix', 'test.osf']).args)
        assert result.exit_code == 0
        # there's at most one OSF file in output dir
        files = os.listdir(tmp_path)
        assert len(files) == 1
        assert all(filename == 'prefix_test.osf' for filename in files)


def test_source_pcap_save_no_extension(test_pcap_file, runner, tmp_path):
    """It should save an osf file with the desired name."""
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'save', 'osf']).args)
        assert result.exit_code == 2
        # there's no OSF file in output dir
        files = os.listdir(tmp_path)
        assert "Must provide a " in result.output
        assert len(files) == 0


def test_source_pcap_legacy_soft_id_failure(runner, tmp_path):
    test_pcap = str(Path(PCAPS_DATA_DIR) / 'same_ports_legacy.pcap')
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', '-s', test_pcap, 'save', 'osf.osf']).args)

        assert "--soft-id-check is not supported for multi-sensor datasets." in result.output

        # there are no files in the output dir
        files = os.listdir(tmp_path)
        assert len(files) == 0


def test_source_pcap_save_raw(test_pcap_file, runner, tmp_path):
    """It should save a pcap file with the desired name."""
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'save_raw', 'test.pcap']).args)
        assert result.exit_code == 0
        # there's a pcap in the output directory
        files = os.listdir(tmp_path)
        assert len(files) == 2
        assert any(filename == 'test.pcap' for filename in files)


def test_source_pcap_save_zm_imu(runner, tmp_path):
    """It should save a pcap file with the desired name."""
    test_pcap_file = str(Path(PCAPS_DATA_DIR) / 'imu_zm_no_lidar.pcap')
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'save', 'test.pcap']).args)
        assert result.exit_code == 0
        # there's a pcap in the output directory
        files = os.listdir(tmp_path)
        assert len(files) == 2
        assert any(filename == 'test.pcap' for filename in files)


def test_source_pcap_save_raw_zm_imu(runner, tmp_path):
    """It should save a pcap file with the desired name."""
    test_pcap_file = str(Path(PCAPS_DATA_DIR) / 'imu_zm_no_lidar.pcap')
    with set_directory(tmp_path):
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'save_raw', 'test.pcap']).args)
        assert result.exit_code == 0
        # there's a pcap in the output directory
        files = os.listdir(tmp_path)
        assert len(files) == 2
        assert any(filename == 'test.pcap' for filename in files)


def test_source_osf_info_help(test_osf_file, runner):
    """ouster-cli source <src>.osf info --help
    should display OSF viz help"""
    result = runner.invoke(core.cli, CliArgs(['source', test_osf_file, 'info', '--help']).args)
    assert "Usage: cli source SOURCE info [OPTIONS]" in result.output

    option_names = [option.name.replace('_', '-') for option in source_osf.osf_info.params]
    assert all([option_name in result.output.lower().replace('_', '-') for option_name in option_names])
    assert result.exit_code == 0


def test_source_osf_viz_help(test_osf_file, runner):
    """ouster-cli source <src>.osf viz --help
    should display OSF viz help"""
    result = runner.invoke(core.cli, ['source', test_osf_file, 'viz', '--help'])
    assert "Usage: cli source SOURCE viz [OPTIONS]" in result.output
    option_names = [option.name.replace('_', '-') for option in source.source_viz.params]
    assert all([option_name in result.output.lower().replace('_', '-') for option_name in option_names])
    assert result.exit_code == 0


def get_stats(monkeypatch, runner, filename, args = []):
    import atexit
    from contextlib import redirect_stdout, redirect_stderr
    import io
    # monkeypatch atexit so and capture the output
    with monkeypatch.context() as m:
        teardown_fn: Callable[[], None] = lambda: None  # noqa

        def set_teardown_fn(fn):
            nonlocal teardown_fn
            teardown_fn = fn

        m.setattr(atexit, "register", set_teardown_fn)
        io_stream = io.StringIO()
        with redirect_stdout(io_stream), redirect_stderr(io_stream):
            all_args = ['source']
            for arg in args:
                all_args.append(arg)
            all_args.append(filename)
            all_args.append('stats')
            result = runner.invoke(core.cli, CliArgs(all_args).args)
            teardown_fn()

    assert result.exit_code == 0
    return io_stream.getvalue()


@pytest.mark.parametrize('args', [('.osf', 10, 1, 1), ('.bag', 10, 1, 1), ('.pcap', 10, 2, 2),
                                  ('.osf', 0, 3, 3), ('.bag', 0, 3, 2), ('.pcap', 0, 4, 4)])
def test_source_split_save(test_pcap_file2, runner, tmp_path, args, monkeypatch):
    """Make sure we save the expected number of files with the expected number of frames."""
    with set_directory(tmp_path):
        ext, split_size, num_files, num_files_weird = args
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file2, 'save', '--split',
                                                  str(split_size), 'test' + ext]).args)
        assert result.exit_code == 0
        # test we got the expected number of files
        files = os.listdir(tmp_path)

        file_number_split = sys.platform.startswith("win")
        file_number_split |= (sys.version_info[0] == 3 and sys.version_info[1] == 14)
        if file_number_split:
            assert len(files) == num_files_weird
        else:
            assert len(files) == num_files
        assert ('test' + ext) in files

        assert "Count: 3" in get_stats(monkeypatch, runner, 'test*' + ext, args=['-g', "--time"])


@pytest.mark.parametrize('args', [('.bag', 10, 1, 1), ('.pcap', 10, 2, 2),
                                  ('.bag', 0, 3, 2), ('.pcap', 0, 4, 4)])
def test_source_split_save_raw(test_pcap_file2, runner, tmp_path, args, monkeypatch):
    """Make sure we save the expected number of files with the expected number of frames."""
    with set_directory(tmp_path):
        ext, split_size, num_files, num_files_weird = args
        assert not os.listdir(tmp_path)  # no files in output dir
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file2, 'save_raw', '--split',
                                                  str(split_size), 'test' + ext]).args)
        print(result.output)
        assert result.exit_code == 0
        # test we got the expected number of files
        files = os.listdir(tmp_path)
        file_number_split = sys.platform.startswith("win")
        file_number_split |= (sys.version_info[0] == 3 and sys.version_info[1] == 14)
        if file_number_split:
            assert len(files) == num_files_weird
        else:
            assert len(files) == num_files
        assert ('test' + ext) in files

        assert "Count: 3" in get_stats(monkeypatch, runner, 'test*' + ext, args=['-g', "--time"])


def test_discover(runner):
    """ouster-cli discover --help
    should display discover plugin help."""
    result = runner.invoke(core.cli, ['discover', '--help'])
    assert "Usage: cli discover [OPTIONS]" in result.output
    assert result.exit_code == 0


def test_source_stats(test_pcap_file, runner, monkeypatch):
    """ouster-cli source ... stats should display correct stats."""
    result = get_stats(monkeypatch, runner, test_pcap_file)
    assert "Count: 1" in result
    assert "First Receive Time: 1650408693" in result
    assert "Sizes: 1024x128" in result
    assert "Incomplete Frames" in result


def test_source_stats_with_objects(tmp_path, test_osf_file, runner, monkeypatch):
    """ouster-cli source ... stats should display correct stats."""
    test_source = open_source(test_osf_file)
    tmp_osf_file = str(Path(tmp_path) / "tmp_file.osf")
    with osf.Writer(tmp_osf_file, test_source.sensor_info) as writer:
        frame_set = next(iter(test_source))
        frame_set.objects['frame_set_objects'] = [Object()]
        frame_set.objects['frame_set_objects_2'] = [Object(), Object()]
        assert type(frame_set) is FrameSet
        assert len(frame_set) == 1
        assert sum([len(v) for v in frame_set.objects.values()]) == 3
        for frame in frame_set:
            frame.status[:] = 1
            frame.timestamp[:] = 1
            frame.receive_timestamp[:] = 1
            frame.objects['frame_objects'] = [Object(), Object(), Object()]
            frame.objects['frame_objects_2'] = [Object(), Object()]
        writer.save(frame_set)
        writer.close()
        try:
            written = open_source(tmp_osf_file)
            frame_set = next(iter(written))  # make sure we can read the file we just wrote
            assert sum([len(v) for v in frame_set.objects.values()]) == 3
            result = get_stats(monkeypatch, runner, tmp_osf_file)
            assert "Objects in Frame Sets: 3" in result
            assert "Objects in Frames: 5" in result
        finally:
            try:
                os.unlink(tmp_osf_file)
            except Exception:
                pass


def test_source_filter_no_lidar(runner, monkeypatch):
    """ouster-cli source --filter on data with no lidar packets should pass through frames"""
    test_pcap_file = str(Path(PCAPS_DATA_DIR) / 'imu_zm_no_lidar.pcap')
    result = get_stats(monkeypatch, runner, test_pcap_file, args = [])
    assert "Count: 1" in result
    result = get_stats(monkeypatch, runner, test_pcap_file, args = ["--filter"])
    assert "Count: 1" in result


def test_source_filter(runner, monkeypatch):
    """ouster-cli source --filter removes incomplete frames"""
    test_pcap_file = str(Path(PCAPS_DATA_DIR) / 'crc_test.pcap')
    result = get_stats(monkeypatch, runner, test_pcap_file, args = [])
    assert "Count: 2" in result
    result = get_stats(monkeypatch, runner, test_pcap_file, args = ["--filter"])
    assert "Count: 1" in result


def test_source_filter_coord_frame_requires_xyz():
    """--coord-frame should only be accepted for X/Y/Z filtering."""
    ctx = source.SourceCommandContext()
    source_filter_fn = inspect.unwrap(source.source_filter.callback)
    with pytest.raises(click.BadParameter, match="X, Y, or Z"):
        source_filter_fn(
            ctx=ctx,
            axis_field="U",
            indices=(None, 10),
            filtered_fields=None,
            invalid_value=0,
            coord_frame="WORLD",
        )


@pytest.mark.parametrize("coord_frame, expect_sensor_frame, expected_dewarp_points", [
    ("SENSOR", True, False),
    ("BODY", False, False),
    ("WORLD", False, True),
])
def test_source_filter_xyz_coord_frame_modes(
    monkeypatch,
    coord_frame,
    expect_sensor_frame,
    expected_dewarp_points,
):
    """XYZ coord frames should map to expected LUT/dewarp behavior."""
    import ouster.sdk.core as sdk_core
    import ouster.sdk.core.frame_ops as so
    source_filter_fn = inspect.unwrap(source.source_filter.callback)

    xyzlut_extrinsics = []
    dewarp_points_args = []
    frame_tokens = []

    sensor_to_body = np.eye(4)
    sensor_to_body[:3, 3] = [1.0, 2.0, 3.0]

    class FakeSensorInfo:
        def __init__(self):
            self.sensor_to_body = sensor_to_body.copy()

    class FakeFrameSetSource:
        sensor_info = [FakeSensorInfo()]

    ctx = source.SourceCommandContext()
    ctx.frame_set_source = FakeFrameSetSource()
    ctx.frame_set_iter = lambda: iter([[{"frame": 1}]])  # type: ignore

    def fake_xyzlut(sensor_info):
        xyzlut_extrinsics.append(np.asarray(sensor_info.sensor_to_body))

        def fake_xyz(range_image):
            h, w = range_image.shape
            return np.zeros((h, w, 3), dtype=np.float64)

        return fake_xyz

    def fake_filter_xyz(frame, xyzlut, axis_idx, lower, upper, invalid,
                        filtered_fields=None, dewarp_points=False):
        frame_tokens.append(frame)
        dewarp_points_args.append(dewarp_points)
        return None

    monkeypatch.setattr(sdk_core, "XYZLut", fake_xyzlut)
    monkeypatch.setattr(so, "filter_xyz", fake_filter_xyz)

    source_filter_fn(
        ctx=ctx,
        axis_field="X",
        indices=(-1000, 1000),
        filtered_fields=None,
        invalid_value=0,
        coord_frame=coord_frame,
    )
    list(ctx.frame_set_iter())  # type: ignore[operator]

    assert frame_tokens
    assert xyzlut_extrinsics
    expected_extrinsics = np.eye(4) if expect_sensor_frame else sensor_to_body
    for extrinsics in xyzlut_extrinsics:
        np.testing.assert_array_equal(extrinsics, expected_extrinsics)
    # Projecting in the sensor frame must not modify the source's metadata.
    np.testing.assert_array_equal(FakeFrameSetSource.sensor_info[0].sensor_to_body, sensor_to_body)
    assert dewarp_points_args
    assert set(dewarp_points_args) == {expected_dewarp_points}


def test_source_plumb_methods(test_imu_osf_file, runner, tmp_path):
    """plumb --method imu/walls/all should each succeed on a source with the
    FW 3.2+ IMU profile, and 'all' should run both stages."""
    with set_directory(tmp_path):
        for method in ('imu', 'walls', 'all'):
            result = runner.invoke(core.cli, CliArgs(
                ['source', test_imu_osf_file, 'plumb', '--method', method,
                 'save', f'{method}.osf']).args)
            assert result.exit_code == 0, result.output
            assert os.path.exists(f'{method}.osf')
            # the frame plumb sampled must still reach downstream consumers
            saved = open_source(f'{method}.osf')
            assert len(saved) == 1
            # the correction lands in the sensor's extrinsic
            if method != 'walls':
                source_before = open_source(test_imu_osf_file)
                delta = (
                    np.asarray(saved.sensor_info[0].sensor_to_body)
                    @ np.linalg.inv(np.asarray(
                        source_before.sensor_info[0].sensor_to_body)))
                assert not np.allclose(delta[:3, :3], np.eye(3), atol=1e-4)
                source_before.close()
            saved.close()


def test_source_plumb_multisensor(multisensor_osf_file, runner, tmp_path):
    """A dual-sensor no-IMU source: plumb degrades to walls only and refines
    each sensor independently from its own wall geometry. One of these
    16-beam sensors sees too sparse a wall to clear
    PLUMB_WALLS_MIN_WALL_POINTS, so that sensor is refused rather than
    guessed at, and the run aborts before anything is written."""
    with set_directory(tmp_path):
        result = runner.invoke(core.cli, CliArgs(
            ['source', multisensor_osf_file, 'plumb',
             'save', 'test.osf']).args)
        assert result.exit_code != 0
        # the sensor with too few wall points is refused, not guessed at
        assert result.output.count('ERROR: could not plumb sensor') == 1
        assert 'sensor extrinsics left unchanged' in result.output
        assert not os.path.exists('test.osf')

        # a sensor that cannot be levelled aborts before anything is written
        result = runner.invoke(core.cli, CliArgs(
            ['source', multisensor_osf_file, 'plumb', '--method', 'imu',
             'save', 'imu.osf']).args)
        assert result.exit_code != 0
        assert result.output.count('ERROR: could not plumb sensor') == 2
        assert 'sensor extrinsics left unchanged' in result.output
        assert not os.path.exists('imu.osf')


def test_source_plumb_legacy_imu(test_pcap_file, runner, tmp_path):
    """Sources without the FW 3.2+ IMU profile: the default 'all' degrades
    to wall geometry with a warning, and '--method imu' errors."""
    with set_directory(tmp_path):
        result = runner.invoke(core.cli, CliArgs(['source', "--sensor-idx", "0",
                                                  test_pcap_file, 'plumb', 'save', 'test.osf']).args)
        assert result.exit_code == 0, result.output
        assert os.path.exists('test.osf')

        result = runner.invoke(core.cli, CliArgs(['source', "--sensor-idx", "0",
                                                  test_pcap_file, 'plumb', '--method', 'imu',
                                                  'save', 'test2.osf']).args)
        assert 'ERROR: could not plumb sensor' in result.output


def test_source_plumb_slice_aware(test_imu_osf_file, runner, tmp_path):
    """A 'slice' before 'plumb' should move the sampling window: slicing
    past the only frame of the test file leaves plumb with nothing to
    sample, while a slice starting at 0 must still succeed."""
    with set_directory(tmp_path):
        result = runner.invoke(core.cli, CliArgs(['source', test_imu_osf_file,
                                                  'slice', '0:',
                                                  'plumb', 'save', 'test.osf']).args)
        assert result.exit_code == 0, result.output

        result = runner.invoke(core.cli, CliArgs(['source', test_imu_osf_file,
                                                  'slice', '1:',
                                                  'plumb', 'save', 'test2.osf']).args)
        assert 'has no frame in the sampled frame set' in result.output


def test_source_osf(runner, has_mapping) -> None:
    """It should list the correct commands
    in the help depending on source type."""
    # osf
    with tempfile.NamedTemporaryFile(suffix='.osf') as temp_osf:
        result = runner.invoke(core.cli, ['source', temp_osf.name])
        assert result.exit_code == 0
        expected_commands = {'info', 'viz', 'slice', 'save'}
        if has_mapping:
            expected_commands.add('slam')
        assert read_commands_from_help_text(result.output) >= expected_commands


def test_source_osf_dump(test_osf_file, runner):
    """ouster-cli source <src>.osf dump
    should display OSF metadata"""
    result = runner.invoke(core.cli, ['--sdk-log-level', 'off', 'source', test_osf_file, 'dump'])
    meta = json.loads(result.output)
    assert len(meta['metadata']['entries']) == 3
    assert 'buffer' in meta['metadata']['entries'][0]
    assert meta['metadata']['entries'][0]['type'] == "ouster/v1/os_sensor/LidarSensor"
    assert result.exit_code == 0


def test_source_osf_dump_short(test_osf_file, runner):
    """ouster-cli source <src>.osf info -s
    should display OSF metadata in short form"""
    result = runner.invoke(core.cli, ['--sdk-log-level', 'off', 'source', test_osf_file, 'dump', '-s'])
    meta = json.loads(result.output)
    assert len(meta['metadata']['entries']) == 3
    assert 'buffer' not in meta['metadata']['entries'][0]
    assert meta['metadata']['entries'][0]['type'] == "ouster/v1/os_sensor/LidarSensor"
    assert result.exit_code == 0


@pytest.mark.perception
@pytest.mark.skipif(sys.platform == "win32", reason="Does not run on Windows")
def test_source_save_collations(test_osf_file, runner, tmp_path):
    """ Verify collations should only be saved in the OSF if they were added or in the OG source"""
    from ouster.cli.plugins.perception import source_detect  # noqa: F401
    OUTPUT_OSF_FILE = str(Path(tmp_path) / "output.osf")
    result = runner.invoke(core.cli, ['source', test_osf_file, 'slice', '2:', 'save', OUTPUT_OSF_FILE])
    print(result.output)
    assert result.exit_code == 0
    assert "Saving OSF file" in result.output

    # validate that it has no collations
    with open_source(OUTPUT_OSF_FILE) as src:
        assert not src.contains_collations

    result = runner.invoke(core.cli, ['source', test_osf_file, 'slice', '2:',
                                      'detect', 'save', "--overwrite", OUTPUT_OSF_FILE])
    print(result.output)
    assert result.exit_code == 0
    assert "Saving OSF file" in result.output

    # validate that it has collations
    with open_source(OUTPUT_OSF_FILE) as src:
        assert src.contains_collations

    # validate that resaving it still has collations
    OUTPUT_OSF_FILE2 = str(Path(tmp_path) / "output2.osf")
    result = runner.invoke(core.cli, ['source', OUTPUT_OSF_FILE,
                                      'save', "--overwrite", OUTPUT_OSF_FILE2])
    print(result.output)
    assert result.exit_code == 0
    assert "Saving OSF file" in result.output

    # validate that it has collations
    with open_source(OUTPUT_OSF_FILE2) as src:
        assert src.contains_collations


@pytest.mark.perception
@pytest.mark.skipif(sys.platform == "win32", reason="Does not run on Windows")
@pytest.mark.parametrize("options, expected", [
    ([], {}),
    (['--no-save-instance-id-fields',
      '--cluster-filter-min-side-length', '0.5',
      '--cluster-filter-max-side-length', '-1',
      '--cluster-filter-min-vertical-size', '0.25',
      '--cluster-filter-max-volume', '100'],
     {'save_instance_id_fields': False,
      'cluster_filter_min_side_length': 0.5,
      'cluster_filter_max_side_length': -1.0,
      'cluster_filter_min_vertical_size': 0.25,
      'cluster_filter_max_volume': 100.0}),
])
def test_source_detect_config_options(test_osf_file, runner, tmp_path, monkeypatch, options, expected):
    """Verify detect command line options are applied to the detection engine config."""
    from ouster.cli.plugins import perception
    from ouster.sdk.perception import ClassicDetectionConfig

    real_engine = perception.DetectionEngine
    captured = []

    class RecordingDetectionEngine:
        @staticmethod
        def create(sensor_infos, config):
            captured.append(config)
            return real_engine.create(sensor_infos=sensor_infos, config=config)

    monkeypatch.setattr(perception, "DetectionEngine", RecordingDetectionEngine)

    output_osf_file = str(Path(tmp_path) / "output.osf")
    result = runner.invoke(core.cli, ['source', test_osf_file, 'slice', '0:2',
                                      'detect', *options, 'save', output_osf_file])
    print(result.output)
    assert result.exit_code == 0
    assert len(captured) == 1

    defaults = ClassicDetectionConfig()
    for name in ('save_instance_id_fields', 'cluster_filter_min_side_length',
                 'cluster_filter_max_side_length', 'cluster_filter_min_vertical_size',
                 'cluster_filter_max_volume'):
        assert getattr(captured[0], name) == pytest.approx(expected.get(name, getattr(defaults, name)))


def test_source_save_frame_set_source_metadata(test_osf_file, tmp_path, runner):
    """Verify that when we save an OSF, we include the frame source metadata."""
    from ouster.sdk.core import ClassMapSet, FrameSetSourceMetadataSet
    src = open_source(test_osf_file)
    src_osf_name = str(Path(tmp_path) / "src.osf")
    dst_osf_name = str(Path(tmp_path) / "dst.osf")
    writer = osf.Writer(src_osf_name, src.sensor_info)
    src.close()
    class_maps = ClassMapSet({
    })
    metadata = FrameSetSourceMetadataSet()
    metadata['class_maps'] = class_maps
    metadata['additional_info'] = "Test additional info"
    writer.save(metadata)
    writer.close()

    assert os.path.exists(src_osf_name)
    assert os.path.getsize(src_osf_name) > 0
    print(f"Created source OSF file at {src_osf_name} with size {os.path.getsize(src_osf_name)} bytes")

    try:
        result = runner.invoke(core.cli, ['source', src_osf_name, 'save', dst_osf_name])
        assert result.exit_code == 0
        with open_source(dst_osf_name) as dst_src:
            dst_metadata_keys = dst_src.metadata_keys()
            assert 'class_maps' in dst_metadata_keys
            assert 'additional_info' in dst_metadata_keys
    finally:
        # allow exceptions here since windows will likely error as the file could be still open
        try:
            os.unlink(src_osf_name)
        except Exception:
            pass
        try:
            os.unlink(dst_osf_name)
        except Exception:
            pass


def test_source_print_field(test_pcap_file, runner):
    import ouster.cli.plugins.source_utils  # noqa force the plugin to get loaded
    result = runner.invoke(
        core.cli,
        ['source', test_pcap_file, 'print', '-f', 'RANGE'],
    )
    assert result.exit_code == 0
    assert 'RANGE' in result.output


def test_source_strip_removes_field(test_pcap_file, runner):
    import ouster.cli.plugins.source_utils  # noqa force the plugin to get loaded
    result = runner.invoke(
        core.cli,
        ['source', str(test_pcap_file), 'strip', '-f', 'RANGE', 'print', '-f', 'RANGE'],
    )
    assert result.exit_code == 0
    assert 'RANGE' not in result.output


def _write_mixed_lidar_camera_osf(lidar_osf_path: str, out_path: str) -> None:
    """Write a mixed OSF with one lidar stream and one camera stream, taking the
    lidar info/frame from an existing lidar-only OSF."""
    from ouster.sdk import core as sdkcore
    from ouster.sdk import osf as sdkosf

    src = open_source(lidar_osf_path)
    lidar_info = src.sensor_info[0]
    lidar_frame = next(iter(src))[0]

    ci = sdkcore.CameraInfo()
    ci.model = sdkcore.CameraModel(100.0, 100.0, 0.0, 0.0)
    ci.format.columns_per_frame, ci.format.pixels_per_column, ci.sn = 4, 3, 999999
    cf = sdkcore.CameraFrame()
    cf.w, cf.h = 4, 3
    cf.add_field('RANGE', np.full((3, 4), 2000, dtype=np.uint32))
    cf.timestamp[0] = lidar_frame.timestamp[0]
    cf.receive_timestamp[0] = lidar_frame.timestamp[0]

    with sdkosf.Writer(out_path, [lidar_info, ci]) as w:
        w.save(sdkcore.FrameSet([lidar_frame, cf]))


@pytest.mark.parametrize("extension", ["pcap", "bag"])
def test_source_save_mixed_camera_warns_and_skips(test_imu_osf_file, runner, tmp_path, extension):
    """Saving a mixed lidar+camera source to a packet format (pcap/bag) warns
    that camera streams are skipped via frame_to_packets, and still saves the
    lidar stream without crashing."""
    import ouster.cli.plugins.source_save  # noqa force the plugin to get loaded
    if extension == "bag":
        pytest.importorskip("rosbags")

    mixed_osf = str(tmp_path / "mixed.osf")
    _write_mixed_lidar_camera_osf(test_imu_osf_file, mixed_osf)

    out_file = str(tmp_path / f"out.{extension}")
    result = runner.invoke(core.cli, ['source', mixed_osf, 'save', out_file],
                           catch_exceptions=False)
    assert result.exit_code == 0
    assert "does not support camera data" in result.output
    assert "1 camera stream(s) will be skipped" in result.output

    produced = list(tmp_path.glob(f"*.{extension}"))
    assert produced, f"expected a .{extension} file to be produced"
    assert any(p.stat().st_size > 0 for p in produced)


def _camera_rgb_image(height: int, width: int, shift: int) -> np.ndarray:
    """A deterministic 8-bit color image, kept well inside 0..255 so that any
    rescaling of it would be obvious."""
    return np.ascontiguousarray(
        ((np.indices((height, width, 3)).sum(0) * 7 + shift) % 200).astype(np.uint8))


def _write_camera_only_osf(out_path: str, frame_count: int = 2) -> None:
    """Write a camera-only OSF whose frames carry an 8-bit RGB image and a RANGE
    image, like a ZED source produces."""
    from ouster.sdk import core as sdkcore
    from ouster.sdk import osf as sdkosf

    width, height = 16, 8
    ci = sdkcore.CameraInfo()
    ci.model = sdkcore.CameraModel(100.0, 100.0, 0.0, 0.0)
    ci.format.columns_per_frame, ci.format.pixels_per_column, ci.sn = width, height, 999999

    with sdkosf.Writer(out_path, [ci]) as w:
        for i in range(frame_count):
            cf = sdkcore.CameraFrame(ci)
            cf.w, cf.h = width, height
            cf.add_field('RGB', _camera_rgb_image(height, width, i * 10))
            cf.add_field('RANGE', np.full((height, width), 2000 + i, dtype=np.uint32))
            cf.timestamp[0] = 100000000 * (i + 1)
            cf.receive_timestamp[0] = cf.timestamp[0]
            cf.status[0] = 1
            w.save(sdkcore.FrameSet([cf]))


def test_source_save_png_camera_only(runner, tmp_path):
    """Camera frames are saved as PNGs, one per frame per image field. Frames have
    no sensor-assigned frame id, so they are named by their order of arrival."""
    import ouster.cli.plugins.source_save  # noqa force the plugin to get loaded
    from ouster.sdk.io import read_png

    camera_osf = str(tmp_path / "camera.osf")
    _write_camera_only_osf(camera_osf, frame_count=2)

    result = runner.invoke(
        core.cli, ['source', camera_osf, 'save', '-d', str(tmp_path), 'out.png'],
        catch_exceptions=False)
    assert result.exit_code == 0

    produced = sorted(p.name for p in tmp_path.glob("*.png"))
    assert produced == [
        'out_999999_0_RANGE.png', 'out_999999_0_RGB.png',
        'out_999999_1_RANGE.png', 'out_999999_1_RGB.png',
    ]

    # An 8-bit color image is already display data and is written unchanged.
    saved = read_png(str(tmp_path / 'out_999999_0_RGB.png'))
    assert saved.shape == (8, 16, 3)
    assert np.array_equal(saved, _camera_rgb_image(8, 16, 0))


def test_source_save_png_mixed_lidar_camera(test_imu_osf_file, runner, tmp_path):
    """A mixed source saves both its lidar and its camera frames, with the lidar
    files still named by frame id."""
    import ouster.cli.plugins.source_save  # noqa force the plugin to get loaded

    mixed_osf = str(tmp_path / "mixed.osf")
    _write_mixed_lidar_camera_osf(test_imu_osf_file, mixed_osf)
    lidar_sn = open_source(mixed_osf).sensor_info[0].sn

    result = runner.invoke(
        core.cli, ['source', mixed_osf, 'save', '-d', str(tmp_path), 'out.png'],
        catch_exceptions=False)
    assert result.exit_code == 0

    produced = [p.name for p in tmp_path.glob("*.png")]
    assert any(f"_{lidar_sn}_" in name for name in produced), produced
    assert 'out_999999_0_RANGE.png' in produced, produced


def test_source_mask_rgb_png_is_converted_and_scaled(test_imu_osf_file, runner, tmp_path):
    """An RGB mask at half the sensor resolution is converted to gray and scaled
    up; pixels that aren't fully white are masked out."""
    from ouster.sdk.core.data import destagger
    from ouster.sdk.io import write_png

    info = open_source(test_imu_osf_file).sensor_info[0]
    h, w = info.h, info.w
    mask = np.zeros((h // 2, w // 2, 3), dtype=np.uint8)
    mask[:, :w // 4] = 255
    mask_png = str(tmp_path / "mask.png")
    write_png(mask_png, mask)

    out_osf = str(tmp_path / "masked.osf")
    result = runner.invoke(core.cli, ['source', test_imu_osf_file, 'mask', '--fields', 'RANGE',
                                      mask_png, 'save', out_osf], catch_exceptions=False)
    assert result.exit_code == 0, result.output
    assert "performing conversion" in result.output
    assert "will scale" in result.output

    frame = next(iter(open_source(out_osf)))[0]
    masked_range = destagger(frame.sensor_info, frame.field('RANGE'))
    assert np.count_nonzero(masked_range[:, :w // 2]) > 0
    assert np.count_nonzero(masked_range[:, w // 2:]) == 0


def test_source_mask_rejects_non_png(test_imu_osf_file, runner, tmp_path):
    mask_file = tmp_path / "mask.jpg"
    mask_file.write_bytes(b"\xff\xd8\xff\xe0 not a png")
    result = runner.invoke(core.cli, ['source', test_imu_osf_file, 'mask', str(mask_file), 'stats'])
    assert result.exit_code != 0
    assert "must be a PNG" in result.output


@pytest.mark.parametrize("command,arg", [("reduce", "32"), ("select", "0,1")])
def test_source_beam_commands_reject_camera_streams(test_imu_osf_file, runner, tmp_path, command, arg):
    """Beam reduction/selection is lidar-only, so a source with a camera stream is
    rejected with a clear message instead of failing on CameraInfo."""
    mixed_osf = str(tmp_path / "mixed.osf")
    _write_mixed_lidar_camera_osf(test_imu_osf_file, mixed_osf)

    result = runner.invoke(core.cli, ['source', mixed_osf, command, arg, 'stats'])
    assert result.exit_code != 0
    assert f"{command} is not compatible with sources containing camera streams" in result.output


@pytest.fixture
def test_zone_monitor_osf_file(test_data_dir) -> str:
    """A single frame OSF whose sensor info has a zone set with four zones,
    each with an STL and a rendered ZRB."""
    return str(test_data_dir / 'zone_monitor' / 'single_frame_zm.osf')


@pytest.fixture
def zone_monitor_output_name(test_zone_monitor_osf_file) -> str:
    """The serial number and sensor index that save_zone_set appends to its output names."""
    return f"{open_source(test_zone_monitor_osf_file).sensor_info[0].sn}_0"


@pytest.fixture
def test_multisensor_zone_monitor_osf_file(test_zone_monitor_osf_file, tmp_path) -> str:
    """Two sensors with zone sets: the single sensor zone monitor OSF plus a
    copy of it with a different serial number and a modified zone."""
    import copy
    src = open_source(test_zone_monitor_osf_file)
    first = src.sensor_info[0]
    second = copy.deepcopy(first)
    second.sn = first.sn + 1
    zone_set = second.zone_set
    zone_set.zones[0].frame_count += 1
    second.zone_set = zone_set
    out = str(tmp_path / 'two_sensors.osf')
    osf.Writer(out, [first, second]).close()
    return out


def test_source_save_zone_set_zip(test_zone_monitor_osf_file, zone_monitor_output_name, runner, tmp_path):
    """By default the zone set is saved as a zip suffixed with the serial number and
    sensor index, containing both STLs and ZRBs, and it round trips to an identical zone set."""
    import zipfile
    from ouster.sdk.core import ZoneSet

    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', str(tmp_path / 'zones.zip')])
    assert result.exit_code == 0, result.output
    assert [p.name for p in tmp_path.iterdir()] == [f'zones_{zone_monitor_output_name}.zip']

    out_zip = tmp_path / f'zones_{zone_monitor_output_name}.zip'
    with zipfile.ZipFile(out_zip) as zf:
        names = set(zf.namelist())
    assert 'metadata.json' in names
    assert {'0.zrb', '1.zrb', '2.zrb', '3.zrb'} <= names
    assert len([n for n in names if n.endswith('.stl')]) == 4

    info = open_source(test_zone_monitor_osf_file).sensor_info[0]
    assert ZoneSet(str(out_zip)) == info.zone_set
    assert f'Saved zone set for sensor idx 0 sn {info.sn} to {out_zip}' in result.output.splitlines()


@pytest.mark.parametrize("use_cwd", [False, True])
def test_source_save_zone_set_zip_into_existing_directory(test_zone_monitor_osf_file, zone_monitor_output_name,
                                                          runner, tmp_path, use_cwd):
    """With zip output, an existing directory as FILENAME (including '.') gets the zips saved inside it,
    named by serial number and sensor index."""
    from ouster.sdk.core import ZoneSet

    out_dir = tmp_path / 'out'
    out_dir.mkdir()
    if use_cwd:
        with set_directory(out_dir):
            result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file, 'save_zone_set', '.'])
        expected_logged_path = os.path.join('.', f'{zone_monitor_output_name}.zip')
    else:
        result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file, 'save_zone_set', str(out_dir)])
        expected_logged_path = str(out_dir / f'{zone_monitor_output_name}.zip')
    assert result.exit_code == 0, result.output
    assert [p.name for p in out_dir.iterdir()] == [f'{zone_monitor_output_name}.zip']
    assert [p.name for p in tmp_path.iterdir()] == ['out']

    info = open_source(test_zone_monitor_osf_file).sensor_info[0]
    assert ZoneSet(str(out_dir / f'{zone_monitor_output_name}.zip')) == info.zone_set
    assert f'Saved zone set for sensor idx 0 sn {info.sn} to {expected_logged_path}' in result.output.splitlines()


def test_source_save_zone_set_zip_into_existing_directory_requires_overwrite(test_zone_monitor_osf_file,
                                                                            zone_monitor_output_name,
                                                                            runner, tmp_path):
    existing = tmp_path / f'{zone_monitor_output_name}.zip'
    existing.write_bytes(b'')
    args = ['source', test_zone_monitor_osf_file, 'save_zone_set']
    result = runner.invoke(core.cli, args + [str(tmp_path)])
    assert result.exit_code == 1
    assert 'already exists' in result.output
    assert existing.stat().st_size == 0

    result = runner.invoke(core.cli, args + ['--overwrite', str(tmp_path)])
    assert result.exit_code == 0, result.output
    assert existing.stat().st_size > 0


def test_source_save_zone_set_zip_without_extension(test_zone_monitor_osf_file, zone_monitor_output_name,
                                                    runner, tmp_path):
    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', str(tmp_path / 'zones')])
    assert result.exit_code == 0, result.output
    assert (tmp_path / f'zones_{zone_monitor_output_name}').is_file()


def test_source_save_zone_set_directory(test_zone_monitor_osf_file, zone_monitor_output_name, runner, tmp_path):
    """With --output-type directory, the zone set is written as loose files in a
    subdirectory named by serial number and sensor index, creating missing parent directories."""
    from ouster.sdk.core import ZoneSet

    out_dir = tmp_path / 'nested' / 'zones'
    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', '--output-type', 'directory', str(out_dir)])
    assert result.exit_code == 0, result.output
    assert [p.name for p in out_dir.iterdir()] == [zone_monitor_output_name]

    sensor_dir = out_dir / zone_monitor_output_name
    assert (sensor_dir / 'metadata.json').is_file()
    assert len(list(sensor_dir.glob('*.stl'))) == 4
    assert len(list(sensor_dir.glob('*.zrb'))) == 4

    info = open_source(test_zone_monitor_osf_file).sensor_info[0]
    assert ZoneSet(str(sensor_dir)) == info.zone_set
    assert f'Saved zone set for sensor idx 0 sn {info.sn} to {sensor_dir}' in result.output.splitlines()


@pytest.mark.parametrize("output_type", ['zip', 'zip_into_directory', 'directory'])
def test_source_save_zone_set_multisensor(test_multisensor_zone_monitor_osf_file, runner, tmp_path,
                                          output_type):
    """Each sensor's zone set is saved to its own output, named by serial number and sensor index,
    and each saved output is logged."""
    from ouster.sdk.core import ZoneSet

    infos = open_source(test_multisensor_zone_monitor_osf_file).sensor_info
    out_dir = tmp_path / 'out'
    out_dir.mkdir()
    out_path = {
        'zip': out_dir / 'zones.zip',
        'zip_into_directory': out_dir,
        'directory': out_dir / 'zones',
    }[output_type]
    result = runner.invoke(core.cli, ['source', test_multisensor_zone_monitor_osf_file, 'save_zone_set',
                                      '--output-type', 'directory' if output_type == 'directory' else 'zip',
                                      str(out_path)])
    assert result.exit_code == 0, result.output

    for idx, info in enumerate(infos):
        if output_type == 'zip':
            saved = out_dir / f'zones_{info.sn}_{idx}.zip'
            assert saved.is_file()
        elif output_type == 'zip_into_directory':
            saved = out_dir / f'{info.sn}_{idx}.zip'
            assert saved.is_file()
        else:
            saved = out_path / f'{info.sn}_{idx}'
            assert saved.is_dir()
        assert ZoneSet(str(saved)) == info.zone_set
        assert f'Saved zone set for sensor idx {idx} sn {info.sn} to {saved}' in result.output.splitlines()
    assert infos[0].zone_set != infos[1].zone_set


@pytest.mark.parametrize("output_type", ['zip', 'directory'])
def test_source_save_zone_set_multisensor_same_serial_number(test_zone_monitor_osf_file, runner, tmp_path,
                                                             output_type):
    """Sensors that share a serial number are saved to separate outputs, told apart by sensor index."""
    import copy
    from ouster.sdk.core import ZoneSet

    first = open_source(test_zone_monitor_osf_file).sensor_info[0]
    second = copy.deepcopy(first)
    zone_set = second.zone_set
    zone_set.zones[0].frame_count += 1
    second.zone_set = zone_set
    src_osf = str(tmp_path / 'same_sn.osf')
    osf.Writer(src_osf, [first, second]).close()
    infos = open_source(src_osf).sensor_info
    assert infos[0].sn == infos[1].sn

    out_dir = tmp_path / 'out'
    out_dir.mkdir()
    out_path = out_dir / ('zones.zip' if output_type == 'zip' else 'zones')
    result = runner.invoke(core.cli, ['source', src_osf, 'save_zone_set', '--output-type', output_type,
                                      str(out_path)])
    assert result.exit_code == 0, result.output

    saved_dir = out_dir if output_type == 'zip' else out_path
    expected_names = ([f'zones_{first.sn}_0.zip', f'zones_{first.sn}_1.zip'] if output_type == 'zip'
                      else [f'{first.sn}_0', f'{first.sn}_1'])
    assert sorted(p.name for p in saved_dir.iterdir()) == expected_names
    for idx, info in enumerate(infos):
        assert ZoneSet(str(saved_dir / expected_names[idx])) == info.zone_set
    assert infos[0].zone_set != infos[1].zone_set


def test_source_save_zone_set_multisensor_conflict_writes_nothing(test_multisensor_zone_monitor_osf_file,
                                                                  runner, tmp_path):
    """If any sensor's output already exists, no zone set is written."""
    infos = open_source(test_multisensor_zone_monitor_osf_file).sensor_info
    out_dir = tmp_path / 'out'
    out_dir.mkdir()
    existing = out_dir / f'zones_{infos[1].sn}_1.zip'
    existing.write_bytes(b'')

    result = runner.invoke(core.cli, ['source', test_multisensor_zone_monitor_osf_file,
                                      'save_zone_set', str(out_dir / 'zones.zip')])
    assert result.exit_code == 1
    assert 'already exists' in result.output
    assert 'Saved zone set' not in result.output
    assert [p.name for p in out_dir.iterdir()] == [existing.name]
    assert existing.stat().st_size == 0


@pytest.mark.parametrize("filter_arg, expect_stl, expect_zrb", [
    ('stl', True, False),
    ('zrb', False, True),
    ('both', True, True),
    ('STL', True, False),
])
def test_source_save_zone_set_filter(test_zone_monitor_osf_file, zone_monitor_output_name, runner, tmp_path,
                                     filter_arg, expect_stl, expect_zrb):
    """--filter controls which zone files are written (case insensitive)."""
    import zipfile

    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', '--filter', filter_arg, str(tmp_path / 'zones.zip')])
    assert result.exit_code == 0, result.output

    with zipfile.ZipFile(tmp_path / f'zones_{zone_monitor_output_name}.zip') as zf:
        names = zf.namelist()
    assert 'metadata.json' in names
    assert any(n.endswith('.stl') for n in names) == expect_stl
    assert any(n.endswith('.zrb') for n in names) == expect_zrb


def test_source_save_zone_set_invalid_filter(test_zone_monitor_osf_file, runner, tmp_path):
    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', '--filter', 'obj', str(tmp_path / 'zones.zip')])
    assert result.exit_code != 0
    assert list(tmp_path.iterdir()) == []


@pytest.mark.parametrize("output_type", ['zip', 'directory'])
def test_source_save_zone_set_requires_overwrite(test_zone_monitor_osf_file, zone_monitor_output_name, runner,
                                                 tmp_path, output_type):
    """An existing output is only replaced when --overwrite is passed."""
    if output_type == 'zip':
        out_arg = tmp_path / 'zones.zip'
        saved = tmp_path / f'zones_{zone_monitor_output_name}.zip'
        saved.write_bytes(b'')
    else:
        out_arg = tmp_path / 'zones'
        saved = out_arg / zone_monitor_output_name
        saved.mkdir(parents=True)

    args = ['source', test_zone_monitor_osf_file, 'save_zone_set', '--output-type', output_type]
    result = runner.invoke(core.cli, args + [str(out_arg)])
    assert result.exit_code == 1
    assert 'already exists' in result.output
    assert '--overwrite' in result.output

    result = runner.invoke(core.cli, args + ['--overwrite', str(out_arg)])
    assert result.exit_code == 0, result.output
    if output_type == 'zip':
        assert saved.stat().st_size > 0
    else:
        assert (saved / 'metadata.json').is_file()


def test_source_save_zone_set_existing_parent_directory(test_zone_monitor_osf_file, zone_monitor_output_name,
                                                        runner, tmp_path):
    """With directory output, FILENAME itself may already exist; only the
    per-sensor subdirectories require --overwrite."""
    out_dir = tmp_path / 'zones'
    out_dir.mkdir()
    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', '--output-type', 'directory', str(out_dir)])
    assert result.exit_code == 0, result.output
    assert (out_dir / zone_monitor_output_name / 'metadata.json').is_file()


def test_source_save_zone_set_zip_onto_directory(test_zone_monitor_osf_file, zone_monitor_output_name,
                                                 runner, tmp_path):
    out_dir = tmp_path / f'zones_{zone_monitor_output_name}.zip'
    out_dir.mkdir()
    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', '--overwrite', str(tmp_path / 'zones.zip')])
    assert result.exit_code == 1
    assert 'is a directory' in result.output
    assert list(out_dir.iterdir()) == []


@pytest.mark.parametrize("conflict", ['filename', 'sensor_subdirectory'])
def test_source_save_zone_set_directory_onto_file(test_zone_monitor_osf_file, zone_monitor_output_name,
                                                  runner, tmp_path, conflict):
    out_arg = tmp_path / 'zones'
    if conflict == 'filename':
        out_file = out_arg
    else:
        out_arg.mkdir()
        out_file = out_arg / zone_monitor_output_name
    out_file.write_bytes(b'')
    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', '--output-type', 'directory',
                                      '--overwrite', str(out_arg)])
    assert result.exit_code == 1
    assert 'is a file' in result.output
    assert out_file.stat().st_size == 0


def test_source_save_zone_set_no_zone_set(test_osf_file, runner, tmp_path):
    """A source without a zone set exits with an error and writes nothing."""
    sn = open_source(test_osf_file).sensor_info[0].sn
    result = runner.invoke(core.cli, ['source', test_osf_file, 'save_zone_set', str(tmp_path / 'zones.zip')])
    assert result.exit_code == 1
    assert f'Sensor idx 0 sn {sn} has no zone set, skipping.' in result.output
    assert 'No zone sets found' in result.output
    assert list(tmp_path.iterdir()) == []


def test_source_save_zone_set_skips_sensor_without_zone_set(test_zone_monitor_osf_file, runner, tmp_path):
    """Sensors without a zone set are reported and skipped; the others are still saved."""
    import copy
    first = open_source(test_zone_monitor_osf_file).sensor_info[0]
    second = copy.deepcopy(first)
    second.sn = first.sn + 1
    second.zone_set = None
    src_osf = str(tmp_path / 'two_sensors.osf')
    osf.Writer(src_osf, [first, second]).close()

    out_dir = tmp_path / 'out'
    out_dir.mkdir()
    result = runner.invoke(core.cli, ['source', src_osf, 'save_zone_set', str(out_dir / 'zones.zip')])
    assert result.exit_code == 0, result.output
    assert f'Sensor idx 1 sn {second.sn} has no zone set, skipping.' in result.output
    assert f'Sensor idx 0 sn {first.sn} has no zone set' not in result.output
    assert [p.name for p in out_dir.iterdir()] == [f'zones_{first.sn}_0.zip']
    saved_lines = [line for line in result.output.splitlines() if line.startswith('Saved zone set')]
    assert saved_lines == [f'Saved zone set for sensor idx 0 sn {first.sn} to {out_dir / f"zones_{first.sn}_0.zip"}']


def test_source_save_zone_set_zip_missing_parent_directory(test_zone_monitor_osf_file, runner, tmp_path):
    """Saving a zip into a directory that doesn't exist reports an error instead of
    silently writing nothing."""
    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file,
                                      'save_zone_set', str(tmp_path / 'missing' / 'zones.zip')])
    assert result.exit_code == 1
    assert isinstance(result.exception, SystemExit)
    assert 'Error: ZoneSet::save: failed to open file for writing' in result.output
    assert list(tmp_path.iterdir()) == []


def test_source_save_zone_set_directory_write_error(test_zone_monitor_osf_file, zone_monitor_output_name,
                                                    runner, tmp_path):
    """A file that save_to_directory can't write is reported as an error, not a traceback."""
    sensor_dir = tmp_path / 'zones' / zone_monitor_output_name
    (sensor_dir / 'metadata.json').mkdir(parents=True)
    result = runner.invoke(core.cli, ['source', test_zone_monitor_osf_file, 'save_zone_set',
                                      '--output-type', 'directory', '--overwrite', str(tmp_path / 'zones')])
    assert result.exit_code == 1
    assert isinstance(result.exception, SystemExit)
    assert 'Error: ZoneSet::save_to_directory: failed to open file for writing' in result.output


def test_source_save_zone_set_directory_overwrite_removes_stale_files(test_zone_monitor_osf_file,
                                                                      zone_monitor_output_name, runner, tmp_path):
    """Overwriting a directory output with a narrower filter removes the zone files the
    previous save wrote, but leaves unrelated files and subdirectories alone."""
    from ouster.sdk.core import ZoneSet

    out_dir = tmp_path / 'zones'
    sensor_dir = out_dir / zone_monitor_output_name
    args = ['source', test_zone_monitor_osf_file, 'save_zone_set', '--output-type', 'directory']
    result = runner.invoke(core.cli, args + [str(out_dir)])
    assert result.exit_code == 0, result.output
    assert len(list(sensor_dir.glob('*.zrb'))) == 4
    (sensor_dir / 'notes.txt').write_text('keep me')
    (sensor_dir / 'extra').mkdir()

    result = runner.invoke(core.cli, args + ['--filter', 'stl', '--overwrite', str(out_dir)])
    assert result.exit_code == 0, result.output
    assert list(sensor_dir.glob('*.zrb')) == []
    assert len(list(sensor_dir.glob('*.stl'))) == 4
    assert (sensor_dir / 'notes.txt').read_text() == 'keep me'
    assert (sensor_dir / 'extra').is_dir()
    assert all(zone.zrb is None for zone in ZoneSet(str(sensor_dir)).zones.values())
