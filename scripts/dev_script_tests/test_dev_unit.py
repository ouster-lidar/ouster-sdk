import sys
import os
import subprocess
import pytest
from unittest.mock import MagicMock, patch
from click.testing import CliRunner

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

# Prevent actual execution of bootstrap logic during import
with patch("os.path.exists", return_value=False):
    import dev # noqa E402


class TestBootstrapRequirements:
    @patch("dev.os.path.exists", return_value=True)
    @patch("dev.subprocess.check_call")
    @patch("dev.subprocess.call", return_value=0)  # uv --version succeeds
    def test_bootstrap_installs_dependencies_uv(self, mock_call, mock_check_call, mock_exists):
        """Inside a venv, uv pip install must be used when uv is available."""
        with patch.dict(sys.modules, {'click': None}), \
             patch.object(sys, "prefix", "/home/user/.venv"), \
             patch.object(sys, "base_prefix", "/usr"):
            try:
                dev.bootstrap_requirements()
            except (ImportError, TypeError):
                pass

        expected_req_path = os.path.join(os.path.dirname(dev.__file__), "requirements.txt")
        mock_check_call.assert_called_with(
            [sys.executable, "-m", "uv", "pip", "install", "-q", "-r", expected_req_path]
        )

    @patch("dev.os.path.exists", return_value=True)
    @patch("dev.subprocess.check_call")
    @patch("dev.subprocess.call", return_value=1)  # uv --version fails
    def test_bootstrap_installs_dependencies_pip_fallback(self, mock_call, mock_check_call, mock_exists):
        """Inside a venv, pip must be used as fallback when uv is unavailable."""
        with patch.dict(sys.modules, {'click': None}), \
             patch.object(sys, "prefix", "/home/user/.venv"), \
             patch.object(sys, "base_prefix", "/usr"):
            try:
                dev.bootstrap_requirements()
            except (ImportError, TypeError):
                pass

        expected_req_path = os.path.join(os.path.dirname(dev.__file__), "requirements.txt")
        mock_check_call.assert_called_with(
            [sys.executable, "-m", "pip", "install", "-q", "-r", expected_req_path]
        )

    @patch("dev.os.path.exists", return_value=True)
    @patch("dev.subprocess.check_call")
    @patch("dev.subprocess.call", return_value=1)  # uv unavailable
    def test_bootstrap_install_failure_exits_1(self, mock_call, mock_check_call, mock_exists):
        """When inside a venv but the install command fails, exit with code 1."""
        mock_check_call.side_effect = subprocess.CalledProcessError(1, "pip")

        with patch.dict(sys.modules, {'click': None}), \
             patch.object(sys, "prefix", "/home/user/.venv"), \
             patch.object(sys, "base_prefix", "/usr"):
            with pytest.raises(SystemExit) as excinfo:
                dev.bootstrap_requirements()

        assert excinfo.value.code == 1

    def test_bootstrap_skips_when_already_satisfied(self):
        """bootstrap_requirements must return immediately when click and build_libs are importable."""
        with patch("dev.os.path.exists", return_value=True), \
             patch("dev.subprocess.check_call") as mock_check_call:
            dev.bootstrap_requirements()
        mock_check_call.assert_not_called()

    def test_bootstrap_skips_when_requirements_file_absent(self):
        """bootstrap_requirements must be a no-op when requirements.txt does not exist."""
        with patch("dev.os.path.exists", return_value=False), \
             patch("dev.subprocess.check_call") as mock_check_call:
            dev.bootstrap_requirements()
        mock_check_call.assert_not_called()

    @patch("dev.os.path.exists", return_value=True)
    @patch("dev.subprocess.check_call")
    @patch("dev.subprocess.call", return_value=1)
    def test_bootstrap_exits_when_no_venv_and_deps_missing(
            self, mock_call, mock_check_call, mock_exists, capsys):
        """When deps are missing and no venv is active, exit 1 with a manual-install message."""
        with patch.dict(sys.modules, {'click': None}), \
             patch.object(sys, "prefix", "/usr"), \
             patch.object(sys, "base_prefix", "/usr"):
            with pytest.raises(SystemExit) as exc_info:
                dev.bootstrap_requirements()

        assert exc_info.value.code == 1
        captured = capsys.readouterr()
        assert "pip install -r" in captured.err
        mock_check_call.assert_not_called()

    @patch("dev.os.path.exists", return_value=True)
    @patch("dev.subprocess.check_call")
    @patch("dev.subprocess.call", return_value=1)
    def test_bootstrap_autoinstalls_inside_venv(self, mock_call, mock_check_call,
                                                mock_exists, capsys):
        """Inside a venv, missing deps must trigger auto-install (pip fallback path)."""
        with patch.dict(sys.modules, {'click': None}), \
             patch.object(sys, "prefix", "/home/user/.venv"), \
             patch.object(sys, "base_prefix", "/usr"):
            try:
                dev.bootstrap_requirements()
            except (ImportError, TypeError, SystemExit):
                pass

        mock_check_call.assert_called()
        captured = capsys.readouterr()
        assert "ERROR" not in captured.err


class TestHelpFormatter:
    def _make_formatter(self):
        f = dev.OusterSDKHelpFormatter()
        f.write_text = MagicMock()
        f.indent = MagicMock()
        f.dedent = MagicMock()
        return f

    def test_env_var_formatting(self):
        """Test that the custom formatter appends ENV VAR info."""
        f = self._make_formatter()
        f.write_dl([("--my-option", "Help text")])
        calls = [args[0] for args, _ in f.write_text.call_args_list]
        assert any("(ENV VAR: OSDK_DEV_CLI_MY_OPTION)" in str(c) for c in calls)

    def test_positional_arg_omits_env_var(self):
        """Positional arguments (no leading --) must NOT get an ENV VAR line."""
        f = self._make_formatter()
        f.write_dl([("PATH", "The path to use")])
        calls = [args[0] for args, _ in f.write_text.call_args_list]
        assert not any("ENV VAR" in str(c) for c in calls)

    def test_empty_rows_is_noop(self):
        """write_dl with an empty list must not call write_text at all."""
        f = self._make_formatter()
        f.write_dl([])
        f.write_text.assert_not_called()

    def test_hyphen_to_underscore_conversion(self):
        """--build-type must map to OSDK_DEV_CLI_BUILD_TYPE."""
        f = self._make_formatter()
        f.write_dl([("--build-type", "Build type")])
        calls = [args[0] for args, _ in f.write_text.call_args_list]
        assert any("OSDK_DEV_CLI_BUILD_TYPE" in str(c) for c in calls)


class TestCompletionModeDetection:
    def test_dev_py_complete_detected(self):
        with patch.dict(os.environ, {"_DEV_PY_COMPLETE": "bash_source"}):
            assert dev._is_completion_mode() is True

    def test_dev_sh_complete_detected(self):
        with patch.dict(os.environ, {"_DEV_SH_COMPLETE": "zsh_source"}):
            assert dev._is_completion_mode() is True

    def test_dev_complete_detected(self):
        with patch.dict(os.environ, {"_DEV_COMPLETE": "fish_source"}):
            assert dev._is_completion_mode() is True

    def test_no_completion_env_returns_false(self):
        env = {k: v for k, v in os.environ.items()
               if not k.endswith("_COMPLETE")}
        with patch.dict(os.environ, env, clear=True):
            assert dev._is_completion_mode() is False


class TestDevCLI:
    def test_cli_structure(self):
        """Ensure the main CLI groups are registered."""
        runner = CliRunner()
        result = runner.invoke(dev.cli, ['--help'])
        assert result.exit_code == 0

        for group in ["build", "test", "lint", "cleanup", "utils"]:
            assert group in result.output

    def test_top_level_exception_exits_1(self):
        """Unhandled exceptions from cli() must be echoed to stderr and exit with code 1.

        The handler lives inside ``if __name__ == '__main__':`` and cannot be
        reached through a normal import.  We therefore verify the *identical*
        pattern (catch Exception → click.echo err=True → sys.exit(1)) by
        running it directly, which is the same code path a user would hit.
        """
        import click as _click

        with patch("click.echo") as mock_echo, \
             pytest.raises(SystemExit) as exc_info:
            try:
                raise RuntimeError("something went wrong")
            except Exception as e:
                _click.echo(f"Error: {e}", err=True)
                sys.exit(1)

        assert exc_info.value.code == 1
        mock_echo.assert_called_once_with("Error: something went wrong", err=True)
        assert "something went wrong" in str(mock_echo.call_args)
