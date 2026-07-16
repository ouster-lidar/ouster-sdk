import os
import sys
import pytest
from unittest.mock import MagicMock, patch
from click.testing import CliRunner

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

import dev_lint  # noqa E402


@pytest.fixture
def mock_context_lint(mock_context):
    """Mocks the Click context object (ctx.obj)."""
    ctx_obj = mock_context

    # Setup confirm_auto_fix to True by default
    ctx_obj.build_libs.confirm_auto_fix.return_value = True
    ctx_obj.test_path = os.path.join("mock_root", "sdk", "python")
    return ctx_obj


# Flake8 Tests
def test_flake8_defaults(mock_context_lint):
    """Test flake8 runs with default arguments and paths."""
    runner = CliRunner()
    result = runner.invoke(dev_lint.flake8, obj=mock_context_lint)

    assert result.exit_code == 0

    # Verify check_for_python_lib was called
    mock_context_lint.build_libs.check_for_python_libs.assert_called_with(["flake8"])

    # Verify RunCommand instantiation and call
    mock_context_lint.build_libs.RunCommand.assert_called_with(tty=True)
    run_instance = mock_context_lint.build_libs.RunCommand.return_value

    # Check the args passed to run_command
    # We expect: [sys.executable, "-m", "flake8", "--config", DEFAULT_CONFIG, PATHS...]
    call_args = run_instance.run_command.call_args[0]
    cmd_list = call_args

    assert "flake8" in cmd_list
    assert "--config" in cmd_list

    # Verify default config path is derived from sdk_dir (not a hardcoded path)
    expected_config = os.path.join(mock_context_lint.sdk_dir, "python", ".flake8")
    assert expected_config in cmd_list, f"Expected config {expected_config!r} not found in {cmd_list}"
    assert run_instance.run_command.call_args[1]['throw_on_error'] is True


def test_flake8_custom_options(mock_context_lint):
    """Test flake8 with custom config and output file."""
    runner = CliRunner()
    result = runner.invoke(dev_lint.flake8,
                           ['--config', 'my.cfg', '--output-file', 'out.txt'],
                           obj=mock_context_lint)

    assert result.exit_code == 0
    run_instance = mock_context_lint.build_libs.RunCommand.return_value
    cmd_list = run_instance.run_command.call_args[0]

    assert "my.cfg" in cmd_list
    assert "--format=junit-xml" in cmd_list
    assert "--output-file" in cmd_list
    assert "out.txt" in cmd_list


def test_flake8_failure(mock_context_lint):
    """Test that flake8 failure (exception) exits cleanly."""
    runner = CliRunner()

    # Make run_command raise an exception
    run_instance = mock_context_lint.build_libs.RunCommand.return_value
    run_instance.run_command.side_effect = Exception("Flake8 violations found")

    result = runner.invoke(dev_lint.flake8, obj=mock_context_lint)

    assert result.exit_code == 1
    assert "flake8 failed" in result.output


# Mypy Tests
def test_mypy_execution(mock_context_lint):
    """Test mypy arguments and CWD."""
    runner = CliRunner()
    result = runner.invoke(dev_lint.mypy, obj=mock_context_lint)

    assert result.exit_code == 0
    run_instance = mock_context_lint.build_libs.RunCommand.return_value

    # Check args
    cmd_list = run_instance.run_command.call_args[0]
    assert "mypy" in cmd_list
    assert "--install-types" in cmd_list
    assert os.path.join(mock_context_lint.sdk_dir, "docs") in cmd_list

    # Check cwd argument
    kwargs = run_instance.run_command.call_args[1]
    assert mock_context_lint.test_path in kwargs['cwd']


def test_mypy_execution_with_output(mock_context_lint):
    """Test mypy arguments and CWD."""
    runner = CliRunner()
    result = runner.invoke(dev_lint.mypy,
                           ['--output-file', 'out.xml'],
                           obj=mock_context_lint)

    assert result.exit_code == 0
    run_instance = mock_context_lint.build_libs.RunCommand.return_value

    # Check args
    cmd_list = run_instance.run_command.call_args[0]
    assert "mypy" in cmd_list
    assert "--install-types" in cmd_list
    assert any("out.xml" in str(arg) for arg in cmd_list)
    assert os.path.join(mock_context_lint.sdk_dir, "docs") in cmd_list

    # Check cwd argument
    kwargs = run_instance.run_command.call_args[1]
    assert mock_context_lint.test_path in kwargs['cwd']


# Mypy Stubs Tests
@patch('subprocess.run')
def test_mypy_stubs_success(mock_subproc, mock_context_lint):
    """Test mypy stubs success (clean output)."""
    runner = CliRunner()

    # Mock subprocess output to be empty or contain ignored lines
    mock_subproc.return_value.stdout = "metaclass differs\n"

    result = runner.invoke(dev_lint.mypy_stubs, obj=mock_context_lint)

    assert result.exit_code == 0
    mock_subproc.assert_called()


@patch('subprocess.run')
def test_mypy_stubs_failure(mock_subproc, mock_context_lint):
    """Test mypy stubs failure on actual errors."""
    runner = CliRunner()

    # Output contains a real error
    mock_subproc.return_value.stdout = "some_file.py: error: inconsistent stub\n"

    result = runner.invoke(dev_lint.mypy_stubs, obj=mock_context_lint)

    assert result.exit_code == 1
    assert "mypy stub check failed" in result.output


# Clang-Format Tests
@patch('dev_lint.files_to_check_fn')
def test_clang_format_no_files(mock_files, mock_context_lint):
    """Test error when no files are found."""
    mock_files.return_value = []
    runner = CliRunner()
    result = runner.invoke(dev_lint.clang_format, obj=mock_context_lint)

    assert result.exit_code != 0
    assert isinstance(result.exception, RuntimeError)
    assert "No files to check" in str(result.exception)


@patch('dev_lint.files_to_check_fn')
@patch('subprocess.run')
def test_clang_format_execution_clean(mock_subproc, mock_files, mock_context_lint):
    """Test clang-format running cleanly on files."""
    runner = CliRunner()

    # Setup inputs
    mock_files.return_value = ["file1.cpp", "file2.hpp"]

    # Subprocess returns no "error" in stdout
    mock_subproc.return_value.stdout = ""

    result = runner.invoke(dev_lint.clang_format, ['--threads', '1'], obj=mock_context_lint)

    assert result.exit_code == 0
    assert "found no issues" in result.output
    # subprocess should be called once per file
    assert mock_subproc.call_count == 2


@patch('dev_lint.files_to_check_fn')
@patch('subprocess.run')
def test_clang_format_execution_errors(mock_subproc, mock_files, mock_context_lint):
    """Test clang-format detecting errors."""
    runner = CliRunner()
    mock_files.return_value = ["bad.cpp"]

    # Simulate clang-format reporting an error
    mock_subproc.return_value.stdout = "error: code should be formatted"

    result = runner.invoke(dev_lint.clang_format, ['--threads', '1'], obj=mock_context_lint)

    assert result.exit_code == 1
    assert "found 1 files with issues" in result.output
    assert "bad.cpp" in result.output


@patch('dev_lint.files_to_check_fn')
@patch('subprocess.run')
def test_clang_format_fix_aborted(mock_subproc, mock_files, mock_context_lint):
    """Test that denying the fix prompt exits the tool."""
    runner = CliRunner()

    mock_files.return_value = ["bad.cpp"]
    mock_subproc.return_value.stdout = "error: code should be formatted"

    # User says "No" to auto-fix
    mock_context_lint.build_libs.confirm_auto_fix.return_value = False

    result = runner.invoke(dev_lint.clang_format, ['--fix'], obj=mock_context_lint)

    assert result.exit_code == 1
    assert "Fix not confirmed" in result.output
    # Should not proceed to find files or run
    mock_files.assert_not_called()


# File Filtering Logic Tests
def test_files_to_check_fn_logic():
    """Unit test for the file filtering logic using mocked Git."""

    # We need to mock 'git.Repo' which is imported inside the function
    # We can do this by patching sys.modules or the module where it is imported.
    # Since 'git' is imported inside the function, we patch 'git' in sys.modules

    mock_git = MagicMock()

    # Setup mock repo structure
    mock_repo = MagicMock()
    mock_blob = MagicMock()
    mock_blob.type = 'blob'
    mock_blob.path = 'src/tracked.cpp'

    # traverse() returns a list of blobs
    mock_repo.head.commit.tree.traverse.return_value = [mock_blob]
    mock_git.Repo.return_value = mock_repo

    with patch.dict(sys.modules, {'git': mock_git}):
        # We also need to mock pathlib.Path.rglob to match our physical files
        # Since we can't easily mock filesystem and imports together cleanly,
        # we will mock the helper functions `get_tracked_files` and `get_source_files`
        # if we wanted to be granular.
        # However, testing the actual fnmatch logic:

        with patch('dev_lint.get_source_files') as mock_glob:
            # Setup: 3 files found on disk
            # 1. Tracked and valid
            # 2. Tracked but excluded (thirdparty)
            # 3. Not tracked (ignored)

            import pathlib
            mock_glob.return_value = [
                pathlib.Path("src/tracked.cpp"),
                pathlib.Path("thirdparty/lib.cpp"),
                pathlib.Path("src/untracked.cpp")
            ]

            # The git mock above says only 'src/tracked.cpp' and 'src/untracked.cpp' are in the tree?
            # Wait, our git mock only returned 'src/tracked.cpp'.

            files = []
            for item in dev_lint.files_to_check_fn("/tmp/sdk"):
                temp_string = str(item)
                temp_string = temp_string.replace("\\\\", "/")
                temp_string = temp_string.replace("\\", "/")
                files.append(temp_string)

            # Expectation:
            # - src/tracked.cpp -> IN (Tracked + Not Excluded)
            # - thirdparty/lib.cpp -> OUT (Excluded)
            # - src/untracked.cpp -> OUT (Not in Tracked list)

            assert "src/tracked.cpp" in [f for f in files]
            assert "thirdparty/lib.cpp" not in [f for f in files]
            assert "src/untracked.cpp" not in [f for f in files]
