import os
import pytest
import sys
from unittest.mock import MagicMock, patch, call
from click.testing import CliRunner

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

import dev_cleanup  # noqa E402


@pytest.fixture
def mock_context_cleanup():
    """Mocks the Click context object (ctx.obj)."""
    ctx_obj = MagicMock()
    # Define standard paths
    ctx_obj.sdk_build_dir = "/tmp/build"
    ctx_obj.sdk_artifact_dir = "/tmp/artifacts"
    ctx_obj._sdk_artifact_dir = "/tmp/_artifacts"
    ctx_obj._dev_persistent_dir = "/tmp/persist"

    return ctx_obj


@pytest.fixture(autouse=True)
def clean_globals():
    """Ensure global lists are empty before and after each test."""
    dev_cleanup.additional_build_cleanup_dirs = []
    dev_cleanup.additional_artifact_cleanup_dirs = []
    yield
    dev_cleanup.additional_build_cleanup_dirs = []
    dev_cleanup.additional_artifact_cleanup_dirs = []


@patch('build_libs.rmtree_readonly')
def test_build_cleanup_standard(mock_rmtree, mock_context_cleanup):
    """Test that build cleanup removes the main build directory."""
    runner = CliRunner()
    result = runner.invoke(dev_cleanup.build_cleanup, obj=mock_context_cleanup)

    assert result.exit_code == 0
    mock_rmtree.assert_called_with("/tmp/build")


@patch('build_libs.rmtree_readonly')
def test_build_cleanup_additional(mock_rmtree, mock_context_cleanup):
    """Test that additional directories are also removed."""
    dev_cleanup.additional_build_cleanup_dirs = ["/tmp/extra1", "/tmp/extra2"]

    runner = CliRunner()
    result = runner.invoke(dev_cleanup.build_cleanup, obj=mock_context_cleanup)

    assert result.exit_code == 0

    expected_calls = [
        call("/tmp/build"),
        call("/tmp/extra1"),
        call("/tmp/extra2")
    ]
    mock_rmtree.assert_has_calls(expected_calls, any_order=True)


@patch('build_libs.rmtree_readonly')
def test_artifacts_cleanup_standard(mock_rmtree, mock_context_cleanup):
    """Test that artifacts cleanup removes the main artifact directory via the private attr."""
    runner = CliRunner()
    result = runner.invoke(dev_cleanup.artifacts_cleanup, obj=mock_context_cleanup)

    assert result.exit_code == 0
    # artifacts_cleanup uses _sdk_artifact_dir (private) to avoid the property
    # side-effect of creating the directory just to delete it.
    mock_rmtree.assert_called_with("/tmp/_artifacts")


@patch('build_libs.rmtree_readonly')
def test_artifacts_cleanup_additional(mock_rmtree, mock_context_cleanup):
    """Test that additional artifact directories are also removed."""
    dev_cleanup.additional_artifact_cleanup_dirs = ["/tmp/art_extra"]

    runner = CliRunner()
    result = runner.invoke(dev_cleanup.artifacts_cleanup, obj=mock_context_cleanup)

    assert result.exit_code == 0
    mock_rmtree.assert_any_call("/tmp/art_extra")


@patch('build_libs.rmtree_readonly')
@patch('os.path.exists')
def test_all_cleanup_everything_exists(mock_exists, mock_rmtree, mock_context_cleanup):
    """Test 'all' cleanup when files exist on disk."""
    mock_exists.return_value = True

    dev_cleanup.additional_build_cleanup_dirs = ["/tmp/build_extra"]
    dev_cleanup.additional_artifact_cleanup_dirs = ["/tmp/art_extra"]

    runner = CliRunner()
    result = runner.invoke(dev_cleanup.all_cleanup, obj=mock_context_cleanup)

    assert result.exit_code == 0
    assert "Cleanup complete" in result.output

    expected_paths = [
        "/tmp/build",
        "/tmp/_artifacts",
        "/tmp/persist",
        "/tmp/build_extra",
        "/tmp/art_extra"
    ]

    for path in expected_paths:
        mock_rmtree.assert_any_call(path)


@patch('shutil.rmtree')
@patch('os.path.exists')
def test_all_cleanup_nothing_exists(mock_exists, mock_rmtree, mock_context_cleanup):
    """Test 'all' cleanup when directories do not exist (should skip rmtree)."""
    # Simulate that NO directories exist
    mock_exists.return_value = False

    runner = CliRunner()
    result = runner.invoke(dev_cleanup.all_cleanup, obj=mock_context_cleanup)

    assert result.exit_code == 0
    assert "Skipping (doesn't exist)" in result.output

    # rmtree should NOT be called
    mock_rmtree.assert_not_called()


@patch('build_libs.rmtree_readonly')
@patch('os.path.exists')
def test_all_cleanup_mixed_existence(mock_exists, mock_rmtree, mock_context_cleanup):
    """Test 'all' cleanup with a mix of existing and missing directories."""

    def side_effect(path):
        if path == "/tmp/build":
            return True
        if path == "/tmp/persist":
            return False
        return False

    mock_exists.side_effect = side_effect

    runner = CliRunner()
    result = runner.invoke(dev_cleanup.all_cleanup, obj=mock_context_cleanup)

    assert result.exit_code == 0

    mock_rmtree.assert_any_call("/tmp/build")

    deleted_paths = [args[0] for args, _ in mock_rmtree.call_args_list]
    assert "/tmp/persist" not in deleted_paths
