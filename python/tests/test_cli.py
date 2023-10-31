#  type: ignore
import os
from pathlib import Path
import pytest
import sys
import json
import tempfile
from typing import List
from click.testing import CliRunner

from ouster.cli import core
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.plugins.io_type import io_type_from_extension, io_type_from_magic, OusterIoType
from ouster.cli.plugins import source, discover, source_osf  # noqa: F401
import ouster.pcap

from tests.conftest import PCAPS_DATA_DIR, OSFS_DATA_DIR


has_magic = False
try:
    import magic  # noqa: F401
    has_magic = True
except ImportError as e:
    print(e)


@pytest.fixture
def test_osf_file() -> str:
    return str(Path(OSFS_DATA_DIR) / 'OS-1-128_v2.3.0_1024x10_lb_n3.osf')


@pytest.fixture
def test_pcap_file() -> str:
    return str(Path(PCAPS_DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')


@pytest.fixture
def test_metadata_file() -> str:
    return str(Path(PCAPS_DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.json')


@pytest.fixture
def runner():
    return CliRunner()


def read_commands_from_help_text(help_text: str) -> List[str]:
    """Reads the command names (but not their help text)
    from help text that Click generates for click.MultiCommand"""
    command_help_lines = help_text.split("Commands:")[1].splitlines()[1:]
    return [line.split()[0].strip() for line in command_help_lines]


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


@pytest.mark.skipif(not has_magic, reason="didn't have the magic.")
def test_io_type_from_magic(test_osf_file, test_pcap_file) -> None:
    # magic doesn't know OSF
    assert io_type_from_magic(test_osf_file) is None
    assert io_type_from_magic(test_pcap_file) == OusterIoType.PCAP
    # test_bag_file = BAG
    # assert io_type_from_magic(test_bag_file) is None
    with pytest.raises(FileNotFoundError):
        io_type_from_magic('doesntexist')


def test_io_type_from_extension() -> None:
    test_osf_name = 'OS1-inters-n5.osf'
    assert io_type_from_extension(test_osf_name) == OusterIoType.OSF
    test_pcap_name = 'data-inters-24784-OS1_128_fw23_legacy_n3.pcap'
    assert io_type_from_extension(test_pcap_name) == OusterIoType.PCAP
    test_bag_name = 'OS1_128_sample_fw23_lb_n3.bag'
    assert io_type_from_extension(test_bag_name) == OusterIoType.ROSBAG


def test_version(runner) -> None:
    result = runner.invoke(core.cli, ['--version'])
    assert "cli, version " in result.output
    assert result.exit_code == 0


def test_help(runner) -> None:
    result = runner.invoke(core.cli, ['--help'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['pcap', '--help'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['sensor', '--help'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, CliArgs(['util', '--help']).args)
    assert "Usage: cli util [OPTIONS] COMMAND [ARGS]" in result.output
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['osf', '--help'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['--traceback', 'osf'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['--sdk-log-level', 'debug', 'osf'])
    assert result.exit_code == 0


def test_mapping_help(runner, has_mapping):
    result = runner.invoke(core.cli, ['mapping', '--help'])
    if has_mapping:
        assert result.exit_code == 0
    else:
        assert result.exit_code == 2


def test_source_help(runner) -> None:
    """It should return 0 if --help is specified."""
    result = runner.invoke(core.cli, CliArgs(['source', '--help']).args)

    # check that a variety of SOURCE commands are in the output
    assert "PCAP info" in result.output
    assert "SENSOR config" in result.output

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
    expected_commands = ['config', 'metadata', 'record', 'viz']
    if has_mapping:
        expected_commands.append('slam')
    assert read_commands_from_help_text(result.output) == expected_commands


def test_source_pcap(runner, has_mapping) -> None:
    # pcap
    expected_commands = ['convert', 'info', 'slice', 'viz']
    if has_mapping:
        expected_commands.append('slam')
    with tempfile.NamedTemporaryFile(suffix='.pcap') as temp_pcap:
        result = runner.invoke(core.cli, ['source', temp_pcap.name])
        assert result.exit_code == 0
        assert read_commands_from_help_text(result.output) == expected_commands


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
    assert "Error: CurlClient::execute_get failed" in result.output
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
    assert ("Error: Source type expected to be a sensor hostname, ip address, "
    "or a(n) .osf or .pcap file.") in result.output
    assert result.exit_code == 2


def test_source_config(runner):
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'config'])
    assert "Error: CurlClient::execute_get failed" in result.output
    assert result.exit_code == 1


def test_source_metadata():
    """It should attempt to get metadata (and fail when there is no sensor)"""
    runner = CliRunner()
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'metadata'])
    assert "Error: CurlClient::execute_get failed" in result.output
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
    assert "Usage: cli source SOURCE slice [OPTIONS] OUTPUT" in result.output
    assert result.exit_code == 0


def test_source_pcap_slice_no_output(test_pcap_file, runner):
    # help option not provided, but no output file provided
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice'])
    assert "Usage: cli source SOURCE slice [OPTIONS] OUTPUT" in result.output
    assert "Missing argument 'OUTPUT'." in result.output
    assert result.exit_code == 2


def test_source_pcap_slice_help_2(test_pcap_file, runner):
    """ouster-cli source <src> slice <output> --help
    should display help"""
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice', 'outputfile.pcap', '--help'])
    assert result.exit_code == 0
    assert "Usage: cli source SOURCE slice [OPTIONS] OUTPUT" in result.output


def test_source_pcap_slice(test_pcap_file, runner):
    """Slicing a pcap should succeed with exit code 0."""
    try:
        with tempfile.NamedTemporaryFile(suffix='.pcap', delete=False) as f:
            pass
        result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice', '-n', 1, f.name])
        assert f'Writing: {f.name}' in result.output
        assert result.exit_code == 0
        result2 = runner.invoke(core.cli, ['source', f.name, 'info'])
        assert result2.exit_code == 0
        assert "Packets read:  74" in result2.output
    finally:
        os.unlink(f.name)


def test_source_pcap_convert_no_output_file(test_pcap_file, runner):
    result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert']).args)
    assert "Error: Missing argument 'OUTPUT_FILE'." in result.output
    assert result.exit_code == 2


def test_source_pcap_convert_help(test_pcap_file, runner):
    """ouster-cli source <src> convert --help
    should display help"""
    result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert', '--help']).args)
    assert "Usage: cli source SOURCE convert [OPTIONS] OUTPUT_FILE" in result.output
    assert result.exit_code == 0


def test_source_pcap_convert_help_2(test_pcap_file, runner):
    """ouster-cli source <src> convert <output>.osf --help
    should display OSF convert help"""
    with tempfile.NamedTemporaryFile(suffix='.osf') as f:
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert', f.name, '--help']).args)

        assert "Usage: cli source SOURCE convert [OPTIONS] OUTPUT_FILE" in result.output

        option_names = [option.name.replace('_', '-') for option in source_osf.osf_from_pcap.params]
        # check that all the options for the command are present in the help
        assert all([option_name in result.output.lower().replace('_', '-') for option_name in option_names])
        assert result.exit_code == 0


def test_source_osf_info_help(test_osf_file, runner):
    """ouster-cli source <src>.osf info --help
    should display OSF viz help"""
    result = runner.invoke(core.cli, CliArgs(['source', test_osf_file, 'info', '--help']).args)
    assert "Usage: cli source SOURCE info [OPTIONS]" in result.output

    option_names = [option.name.replace('_', '-') for option in source_osf.osf_info.params]
    assert all([option_name in result.output.lower().replace('_', '-') for option_name in option_names])
    assert result.exit_code == 0

# TODO Re-eanble when osf viz is exposed
# def test_source_osf_viz_help(test_osf_file, runner):
#     """ouster-cli source <src>.osf viz --help
#     should display OSF viz help"""
#     result = runner.invoke(core.cli, ['source', test_osf_file, 'viz', '--help'])
#     assert "Usage: cli source SOURCE viz [OPTIONS]" in result.output
#     option_names = [option.name.replace('_', '-') for option in source_osf.osf_viz.params]
#     # 'cycle is deprecated and now hidden
#     option_names.remove('cycle')
#     assert all([option_name in result.output.lower().replace('_', '-') for option_name in option_names])
#     assert result.exit_code == 0


# TODO: Uncomment when bag conversion is re-enabled
# def test_source_pcap_convert_help_3(runner):
#    """ouster-cli source <src>.pcap convert <output>.bag --help
#     should display Rosbag convert help"""
#    with tempfile.NamedTemporaryFile(suffix='.pcap') as pcap_file:
#        with tempfile.NamedTemporaryFile(suffix='.bag') as f:
#            result = runner.invoke(core.cli, CliArgs(['source', pcap_file.name, 'convert', f.name, '--help']).args)
#            assert "Usage: cli source SOURCE convert [OPTIONS] OUTPUT_FILE" in result.output
#            option_names = [option.name.replace('_', '-') for option in source.bag_from_pcap.params]
#
#            # check that all the options for the command are present in the help
#            assert all([option_name in result.output.lower().replace('_', '-') for option_name in option_names])
#            assert result.exit_code == 0


def test_source_pcap_convert_bad_extension(test_pcap_file, runner):
    """ouster-cli source <src>.pcap convert <output>.badextension
    should display an error"""
    with tempfile.NamedTemporaryFile(suffix='.badextension') as f:
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert', f.name]).args)
        assert source.source.commands[OusterIoType.PCAP]['convert'].get_output_type_file_extensions_str(
        ) in result.output
        assert result.exit_code == 2


def test_source_pcap_convert_bad_extension_2(test_pcap_file, runner):
    """ouster-cli source <src>.pcap convert <output>.pcap
    should display an error"""
    with tempfile.NamedTemporaryFile(suffix='.pcap') as f:
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert', f.name]).args)
        assert source.source.commands[OusterIoType.PCAP]['convert'].get_output_type_file_extensions_str(
        ) in result.output
        assert result.exit_code == 2


def test_discover(runner):
    """ouster-cli discover --help
    should display discover plugin help."""
    result = runner.invoke(core.cli, ['discover', '--help'])
    assert "Usage: cli discover [OPTIONS]" in result.output
    assert result.exit_code == 0


def test_match_metadata_with_data_stream(test_pcap_file, test_metadata_file):
    """It should find the data stream with a destination port that matches the metadata file lidar port"""
    all_infos = ouster.pcap._packet_info_stream(test_pcap_file, 0, None, 100)
    meta = ouster.client.SensorInfo(open(test_metadata_file).read())
    matched_stream = ouster.cli.core.pcap.match_metadata_with_data_stream(all_infos, meta)
    assert matched_stream.dst_port == 7502


def test_source_osf(runner, has_mapping) -> None:
    """It should list the correct commands
    in the help depending on source type."""
    # osf
    with tempfile.NamedTemporaryFile(suffix='.osf') as temp_osf:
        result = runner.invoke(core.cli, ['source', temp_osf.name])
        assert result.exit_code == 0
        expected_commands = ['convert', 'info', 'viz']
        if has_mapping:
            expected_commands.append('slam')
        assert read_commands_from_help_text(result.output) == expected_commands


def test_source_osf_info(test_osf_file, runner):
    """ouster-cli source <src>.osf info
    should display OSF metadata"""
    result = runner.invoke(core.cli, ['source', test_osf_file, 'info'])
    meta = json.loads(result.output)
    assert len(meta['metadata']['entries']) == 3
    assert 'buffer' in meta['metadata']['entries'][0]
    assert meta['metadata']['entries'][0]['type'] == "ouster/v1/os_sensor/LidarSensor"
    assert result.exit_code == 0


def test_source_osf_info_short(test_osf_file, runner):
    """ouster-cli source <src>.osf info -s
    should display OSF metadata in short form"""
    result = runner.invoke(core.cli, ['source', test_osf_file, 'info', '-s'])
    meta = json.loads(result.output)
    assert len(meta['metadata']['entries']) == 3
    assert 'buffer' not in meta['metadata']['entries'][0]
    assert meta['metadata']['entries'][0]['type'] == "ouster/v1/os_sensor/LidarSensor"
    assert result.exit_code == 0
