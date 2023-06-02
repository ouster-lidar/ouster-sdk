#  type: ignore
import os
from pathlib import Path
import pytest
import sys
import tempfile
from typing import List
from click.testing import CliRunner

from ouster.cli import core
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.plugins.io_type import io_type_from_extension, io_type_from_magic, OusterIoType
from ouster.cli.plugins import source  # noqa: F401
from ouster.cli.plugins import discover  # noqa: F401

from tests.conftest import DATA_DIR


has_magic = False
try:
    import magic  # noqa: F401
    has_magic = True
except ImportError as e:
    print(e)


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
def test_io_type_from_magic() -> None:
    # test_osf_file = str(test_data_dir / 'lib-osf' / 'OS1_128_sample_n5_standard.osf')
    # assert io_type_from_magic(test_osf_file) is None
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    assert io_type_from_magic(test_pcap_file) == OusterIoType.PCAP
    # test_bag_file = str(test_data_dir / 'lib-osf' / 'OS1_128_sample_fw23_lb_n3.bag')
    # assert io_type_from_magic(test_bag_file) is None
    with pytest.raises(FileNotFoundError):
        io_type_from_magic('doesntexist')


def test_io_type_from_extension() -> None:
    test_osf_file = 'OS1_128_sample_n5_standard.osf'
    assert io_type_from_extension(test_osf_file) == OusterIoType.OSF
    test_pcap_file = 'data-inters-24784-OS1_128_fw23_legacy_n3.pcap'
    assert io_type_from_extension(test_pcap_file) == OusterIoType.PCAP
    test_bag_file = 'OS1_128_sample_fw23_lb_n3.bag'
    assert io_type_from_extension(test_bag_file) == OusterIoType.ROSBAG


def test_version() -> None:
    runner = CliRunner()
    result = runner.invoke(core.cli, ['--version'])
    assert "cli, version " in result.output
    assert result.exit_code == 0


def test_help() -> None:
    runner = CliRunner()

    result = runner.invoke(core.cli, ['--help'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['pcap', '--help'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, ['sensor', '--help'])
    assert result.exit_code == 0

    result = runner.invoke(core.cli, CliArgs(['util', '--help']).args)
    assert "Usage: cli util [OPTIONS] COMMAND [ARGS]" in result.output
    assert result.exit_code == 0


def test_source_help() -> None:
    """It should return 0 if --help is specified."""
    runner = CliRunner()
    result = runner.invoke(core.cli, CliArgs(['source', '--help']).args)
    assert "SOURCE is sensor hostname, or a" in result.output
    assert result.exit_code == 0


def test_source_no_args() -> None:
    """It should return a usage error if no command is specified."""
    runner = CliRunner()
    result = runner.invoke(core.cli, CliArgs(['source']).args)
    assert "Error: Missing argument 'SOURCE'." in result.output
    assert result.exit_code == 2


def test_source() -> None:
    """It should list the correct commands
    in the help depending on source type."""
    runner = CliRunner()

    # sensor
    result = runner.invoke(core.cli, ['source', '127.0.0.1'])
    assert result.exit_code == 0
    assert read_commands_from_help_text(result.output) == ['config', 'metadata', 'record', 'viz']

    # pcap
    with tempfile.NamedTemporaryFile(suffix='.pcap') as temp_pcap:
        result = runner.invoke(core.cli, ['source', temp_pcap.name])
        assert result.exit_code == 0
        assert read_commands_from_help_text(result.output) == ['convert', 'info', 'slice', 'viz']

    # TODO FLEETSW-4407: not MVP
    # rosbag
    # with tempfile.NamedTemporaryFile(suffix='.bag') as temp_pcap:
    #    result = runner.invoke(core.cli, ['source', temp_pcap.name])
    #    assert result.exit_code == 0
    #    commands = result.output.split("Commands:")[1].split()
    #    assert commands == ['convert']

    # invalid file type
    with tempfile.NamedTemporaryFile(suffix='.invalid') as temp_pcap:
        result = runner.invoke(core.cli, ['source', temp_pcap.name])
        assert "Error: Expecting .pcap, .osf, .bag, or .csv." in result.output
        assert result.exit_code == 2


def test_source_bad_command():
    """It should exit 2 (see click.exceptions.UsageError) if a bad command
    for the given source was provided."""
    runner = CliRunner()
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'badcommand'])
    assert "Error: No such command 'badcommand'." in result.output
    assert result.exit_code == 2


def test_source_good_command():
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    runner = CliRunner()
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'config'])
    assert "Error: CurlClient::execute_get failed" in result.output
    assert result.exit_code == 1


def test_source_could_not_resolve():
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    runner = CliRunner()
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', 'badhostname', 'config'])
    assert ("Error: Source type expected to be a sensor hostname, ip address, "
            "or a .pcap, .osf, or .bag file.") in result.output
    assert result.exit_code == 2


def test_source_config():
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    runner = CliRunner()
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'config'])
    assert "Error: CurlClient::execute_get failed" in result.output
    assert result.exit_code == 1


def test_source_config_help():
    """It should not exit 2 (see click.exceptions.UsageError) if a valid command
    for the given source was provided."""
    runner = CliRunner()
    # note - for now any path not considered a file is expected to be a sensor
    # we don't expect this to succeed because there is no such sensor
    # so we should see exit code 1
    result = runner.invoke(core.cli, ['source', '127.0.0.1', 'config', '--help'])
    assert "Usage: cli source SOURCE config [OPTIONS] [KEY VAL]..." in result.output
    assert result.exit_code == 0


def test_source_pcap_info():
    """source <pcap> info should work as expected."""
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'info', '-n', 10])
    assert "Packets read:  10" in result.output
    assert result.exit_code == 0


def test_source_pcap_slice_help():
    """ouster-cli source <src> slice --help
    should display help"""
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice', '--help'])
    assert "Usage: cli source SOURCE slice [OPTIONS] OUTPUT" in result.output
    assert result.exit_code == 0


def test_source_pcap_slice_no_output():
    # help option not provided, but no output file provided
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice'])
    assert "Usage: cli source SOURCE slice [OPTIONS] OUTPUT" in result.output
    assert "Missing argument 'OUTPUT'." in result.output
    assert result.exit_code == 2


def test_source_pcap_slice_help_2():
    """ouster-cli source <src> slice <output> --help
    should display help"""
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
    result = runner.invoke(core.cli, ['source', test_pcap_file, 'slice', 'outputfile.pcap', '--help'])
    assert result.exit_code == 0
    assert "Usage: cli source SOURCE slice [OPTIONS] OUTPUT" in result.output


def test_source_pcap_slice():
    """Slicing a pcap should succeed with exit code 0."""
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
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


def test_source_pcap_convert_no_output_file():
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
    result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert']).args)
    assert "Error: Missing argument 'OUTPUT_FILE'." in result.output
    assert result.exit_code == 2


def test_source_pcap_convert_help():
    """ouster-cli source <src> convert --help
    should display help"""
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
    result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert', '--help']).args)
    assert "Usage: cli source SOURCE convert [OPTIONS] OUTPUT_FILE" in result.output
    assert result.exit_code == 0

# TODO: Uncomment when bag conversion is re-enabled
# def test_source_pcap_convert_help_3():
#    """ouster-cli source <src>.pcap convert <output>.bag --help
#     should display Rosbag convert help"""
#    with tempfile.NamedTemporaryFile(suffix='.pcap') as pcap_file:
#        runner = CliRunner()
#        with tempfile.NamedTemporaryFile(suffix='.bag') as f:
#            result = runner.invoke(core.cli, CliArgs(['source', pcap_file.name, 'convert', f.name, '--help']).args)
#            assert "Usage: cli source SOURCE convert [OPTIONS] OUTPUT_FILE" in result.output
#            option_names = [option.name.replace('_', '-') for option in source.bag_from_pcap.params]
#
#            # check that all the options for the command are present in the help
#            assert all([option_name in result.output.lower().replace('_', '-') for option_name in option_names])
#            assert result.exit_code == 0


def test_source_pcap_convert_bad_extension():
    """ouster-cli source <src>.pcap convert <output>.badextension
    should display an error"""
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
    with tempfile.NamedTemporaryFile(suffix='.badextension') as f:
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert', f.name]).args)
        assert source.source.commands[OusterIoType.PCAP]['convert'].get_output_type_file_extensions_str(
        ) in result.output
        assert result.exit_code == 2


def test_source_pcap_convert_bad_extension_2():
    """ouster-cli source <src>.pcap convert <output>.pcap
    should display an error"""
    test_pcap_file = str(Path(DATA_DIR) / 'OS-0-128-U1_v2.3.0_1024x10.pcap')
    runner = CliRunner()
    with tempfile.NamedTemporaryFile(suffix='.pcap') as f:
        result = runner.invoke(core.cli, CliArgs(['source', test_pcap_file, 'convert', f.name]).args)
        assert source.source.commands[OusterIoType.PCAP]['convert'].get_output_type_file_extensions_str(
        ) in result.output
        assert result.exit_code == 2


def test_discover():
    """ouster-cli discover --help
    should display discover plugin help."""
    runner = CliRunner()
    result = runner.invoke(core.cli, ['discover', '--help'])
    assert "Usage: cli discover [OPTIONS]" in result.output
    assert result.exit_code == 0
