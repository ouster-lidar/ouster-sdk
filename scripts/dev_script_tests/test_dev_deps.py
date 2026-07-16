import os
import sys
import pytest
from unittest.mock import MagicMock, patch
from click.testing import CliRunner

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

import dev_dependencies  # noqa E402


@pytest.mark.parametrize("binary, expected", [
    ("apt-get", "apt"),
    ("yum", "yum"),
    ("dnf", "dnf"),
    ("pacman", "pacman"),
    ("brew", "brew"),
    ("apk", "alpine"),
])
def test_detect_package_manager(binary, expected):
    """Test that the correct package manager string is returned based on available binary."""
    with patch("shutil.which") as mock_which:
        # Simulate that only the specific binary exists
        mock_which.side_effect = lambda x: "/bin/" + x if x == binary else None

        assert dev_dependencies.detect_package_manager() == expected


def test_detect_package_manager_none():
    """Test that None is returned if no known package manager is found."""
    with patch("shutil.which", return_value=None):
        assert dev_dependencies.detect_package_manager() is None


def test_enable_local_vcpkg(mock_context):
    """Test vcpkg initialization."""
    runner = CliRunner()
    result = runner.invoke(dev_dependencies.enable_local_vcpkg, ['--reinit'], obj=mock_context)

    assert result.exit_code == 0
    # Verify initialize_vcpkg called with correct args
    mock_context.build_libs.initialize_vcpkg.assert_called()


def test_enable_local_vcpkg_callback(mock_context):
    """Test that the additional functionality callback is triggered."""
    mock_callback = MagicMock()
    # Temporarily set the global callback in the module
    original_callback = dev_dependencies.additional_functionality_callback
    dev_dependencies.additional_functionality_callback = mock_callback

    try:
        runner = CliRunner()
        result = runner.invoke(dev_dependencies.enable_local_vcpkg, obj=mock_context)
        assert result.exit_code == 0
        mock_callback.assert_called_once()
    finally:
        # Restore state
        dev_dependencies.additional_functionality_callback = original_callback


def test_install_vcpkg_reqs_windows(mock_context):
    """Test that Windows immediately exits successfully (no deps required)."""
    runner = CliRunner()
    with patch("os.name", "nt"):
        result = runner.invoke(dev_dependencies.install_vcpkg_package_requirements, obj=mock_context)
        assert result.exit_code == 0
        assert "No vcpkg dependencies required" in result.output


def test_install_vcpkg_reqs_apt(mock_context):
    """Test apt-get commands are generated on Linux/Apt."""
    runner = CliRunner()
    with patch("os.name", "posix"), \
         patch("dev_dependencies.detect_package_manager", return_value="apt"):

        result = runner.invoke(dev_dependencies.install_vcpkg_package_requirements, obj=mock_context)
        assert result.exit_code == 0

        run_instance = mock_context.build_libs.RunCommand.return_value
        # Check for update call
        run_instance.run_command.assert_any_call("apt-get", "update")
        # Check for install call containing essential packages
        args, _ = run_instance.run_command.call_args
        assert args[0] == "apt-get"
        assert "build-essential" in args


def test_install_vcpkg_reqs_brew(mock_context):
    """Test brew commands are generated on Mac/Brew."""
    runner = CliRunner()
    with patch("os.name", "posix"), \
         patch("dev_dependencies.detect_package_manager", return_value="brew"):

        result = runner.invoke(dev_dependencies.install_vcpkg_package_requirements, obj=mock_context)
        assert result.exit_code == 0

        run_instance = mock_context.build_libs.RunCommand.return_value
        run_instance.run_command.assert_any_call("brew", "update")
        # Check args of the last call
        args, _ = run_instance.run_command.call_args
        assert "cmake" in args


def test_install_system_packages_windows_fail(mock_context):
    """Windows is not supported for system packages, should fail."""
    runner = CliRunner()
    with patch("os.name", "nt"):
        result = runner.invoke(dev_dependencies.install_system_packages, obj=mock_context)
        assert result.exit_code == 1
        assert "Windows is detected" in result.output


def test_install_system_packages_unsupported_linux(mock_context):
    """Arch/Alpine etc are not officially supported in this specific function."""
    runner = CliRunner()
    with patch("os.name", "posix"), \
         patch("dev_dependencies.detect_package_manager", return_value="pacman"):

        result = runner.invoke(dev_dependencies.install_system_packages, obj=mock_context)
        assert result.exit_code == 1
        assert "not officially supported" in result.output


def test_install_system_packages_apt_basic(mock_context):
    """Test standard APT installation."""
    runner = CliRunner()
    with patch("os.name", "posix"), \
         patch("dev_dependencies.detect_package_manager", return_value="apt"):

        result = runner.invoke(dev_dependencies.install_system_packages, obj=mock_context)
        assert result.exit_code == 0

        run_instance = mock_context.build_libs.RunCommand.return_value
        # Verify libeigen3-dev is in the install list
        found_lib = False
        for call_args in run_instance.run_command.call_args_list:
            if "libeigen3-dev" in call_args[0]:
                found_lib = True
        assert found_lib


def test_install_vcpkg_reqs_unsupported_pm_is_noop(mock_context):
    """An unsupported package manager must print a message and exit cleanly."""
    runner = CliRunner()
    with patch("os.name", "posix"), \
         patch("dev_dependencies.detect_package_manager", return_value="pacman"):
        result = runner.invoke(dev_dependencies.install_vcpkg_package_requirements, obj=mock_context)
    assert result.exit_code == 0
    assert "No vcpkg dependencies required" in result.output


def test_install_system_packages_brew_full(mock_context):
    """Test Brew installation with all flags."""
    runner = CliRunner()
    with patch("os.name", "posix"), \
         patch("dev_dependencies.detect_package_manager", return_value="brew"):

        result = runner.invoke(dev_dependencies.install_system_packages,
                               ['--doxygen', '--clangformat'],
                               obj=mock_context)
        assert result.exit_code == 0

        run_instance = mock_context.build_libs.RunCommand.return_value
        # Get the args of the last call (brew install ...)
        args, _ = run_instance.run_command.call_args

        assert "doxygen" in args
        assert "clang-format" in args
        assert "eigen" in args
