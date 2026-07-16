"""
Unit tests for dev_completions.py.

Covers:
- update_shell_profile: creates missing file, appends when line absent, skips when present
- completion_dir: returns correct path relative to ctx.obj.dev_dir
- get_parent_shell: detects zsh, bash, fish, powershell/pwsh; returns "unknown" when not found
- install_zsh_completions: copies file and updates ~/.zshrc
- install_bash_completions: copies file and updates ~/.bashrc
- install_completions CLI command: routes to correct installer, auto-detects shell, handles unsupported shell
"""

import os
import sys
import pytest
from unittest.mock import MagicMock, patch, mock_open
from click.testing import CliRunner

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))

import dev_completions  # noqa: E402


# ---------------------------------------------------------------------------
# update_shell_profile
# ---------------------------------------------------------------------------

class TestUpdateShellProfile:
    def test_creates_file_when_missing(self, tmp_path):
        profile = tmp_path / ".zshrc"
        source_line = "source ~/.ouster_dev_completions.zsh"

        dev_completions.update_shell_profile(str(profile), source_line)

        content = profile.read_text()
        assert source_line in content
        assert "# Added by Ouster Dev Scripts" in content

    def test_appends_when_line_absent(self, tmp_path):
        profile = tmp_path / ".zshrc"
        profile.write_text("# existing config\n")
        source_line = "source ~/.ouster_dev_completions.zsh"

        dev_completions.update_shell_profile(str(profile), source_line)

        content = profile.read_text()
        assert source_line in content
        assert "# existing config" in content

    def test_skips_when_line_already_present(self, tmp_path, capsys):
        source_line = "source ~/.ouster_dev_completions.zsh"
        profile = tmp_path / ".zshrc"
        profile.write_text(f"# existing\n{source_line}\n")

        dev_completions.update_shell_profile(str(profile), source_line)

        content = profile.read_text()
        assert content.count(source_line) == 1

    def test_does_not_duplicate_on_repeated_calls(self, tmp_path):
        profile = tmp_path / ".zshrc"
        source_line = "source ~/.ouster_dev_completions.zsh"

        dev_completions.update_shell_profile(str(profile), source_line)
        dev_completions.update_shell_profile(str(profile), source_line)

        assert profile.read_text().count(source_line) == 1

    def test_expands_tilde_in_path(self, tmp_path):
        source_line = "source ~/.ouster_dev_completions.zsh"

        with patch("os.path.expanduser", return_value=str(tmp_path / ".zshrc")), \
             patch("os.path.exists", return_value=False):
            m = mock_open()
            with patch("builtins.open", m):
                dev_completions.update_shell_profile("~/.zshrc", source_line)

        m.assert_called()


# ---------------------------------------------------------------------------
# completion_dir
# ---------------------------------------------------------------------------

class TestCompletionDir:
    def test_returns_completions_sibling_of_dev_dir(self):
        ctx = MagicMock()
        ctx.obj.dev_dir = "/home/user/scripts/dev_script_library"

        result = dev_completions.completion_dir(ctx)

        assert result == os.path.join("/home/user/scripts/dev_script_library", "..", "completions")

    def test_path_contains_completions_segment(self):
        ctx = MagicMock()
        ctx.obj.dev_dir = "/any/path"

        assert "completions" in dev_completions.completion_dir(ctx)


# ---------------------------------------------------------------------------
# get_parent_shell
# ---------------------------------------------------------------------------

def _make_process_chain(*names):
    """Build a linked chain of mock psutil.Process objects from child → parent."""
    processes = []
    for name in names:
        p = MagicMock()
        p.name.return_value = name
        processes.append(p)

    for i in range(len(processes) - 1):
        processes[i].parent.return_value = processes[i + 1]
    processes[-1].parent.return_value = None

    return processes[0]


class TestGetParentShell:
    def _invoke(self, chain_root):
        with patch("psutil.Process", return_value=chain_root):
            return dev_completions.get_parent_shell()

    def test_detects_zsh(self):
        assert self._invoke(_make_process_chain("python", "zsh")) == "zsh"

    def test_detects_bash(self):
        assert self._invoke(_make_process_chain("python", "bash")) == "bash"

    def test_detects_fish(self):
        assert self._invoke(_make_process_chain("python", "fish")) == "fish"

    def test_detects_powershell(self):
        assert self._invoke(_make_process_chain("python", "powershell")) == "powershell"

    def test_maps_pwsh_to_powershell(self):
        assert self._invoke(_make_process_chain("python", "pwsh")) == "powershell"

    def test_strips_exe_suffix(self):
        assert self._invoke(_make_process_chain("python", "zsh.exe")) == "zsh"

    def test_returns_unknown_when_no_known_shell(self):
        assert self._invoke(_make_process_chain("python", "sh", "init")) == "unknown"

    def test_walks_multiple_levels(self):
        assert self._invoke(_make_process_chain("dev.py", "python3", "bash")) == "bash"


# ---------------------------------------------------------------------------
# install_zsh_completions
# ---------------------------------------------------------------------------

class TestInstallZshCompletions:
    def test_copies_completion_file(self, tmp_path):
        ctx = MagicMock()
        # completion_dir() joins dev_dir + "../completions", so place source
        # directly under tmp_path/completions and set dev_dir to tmp_path/scripts
        scripts_dir = tmp_path / "scripts"
        scripts_dir.mkdir()
        completions_dir = tmp_path / "completions"
        completions_dir.mkdir()
        (completions_dir / ".ouster_dev_completions.zsh").write_text("#compdef dev\n")
        ctx.obj.dev_dir = str(scripts_dir)

        dest = tmp_path / "home" / ".ouster_dev_completions.zsh"
        dest.parent.mkdir()

        with patch("os.path.expanduser", return_value=str(dest)), \
             patch("dev_completions.update_shell_profile"):
            dev_completions.install_zsh_completions(ctx)

        assert dest.exists()
        assert dest.read_text() == "#compdef dev\n"

    def test_calls_update_shell_profile_with_zshrc(self, tmp_path):
        ctx = MagicMock()
        ctx.obj.dev_dir = str(tmp_path / "dev_script_library")
        completions_dir = tmp_path / "completions"
        completions_dir.mkdir()
        (completions_dir / ".ouster_dev_completions.zsh").write_text("")

        fake_dest = str(tmp_path / ".ouster_dev_completions.zsh")

        with patch("os.path.expanduser", return_value=fake_dest), \
             patch("shutil.copyfile"), \
             patch("dev_completions.update_shell_profile") as mock_update:
            dev_completions.install_zsh_completions(ctx)

        mock_update.assert_called_once()
        profile_arg = mock_update.call_args[0][0]
        assert profile_arg == "~/.zshrc"

    def test_source_line_references_dest(self, tmp_path):
        ctx = MagicMock()
        ctx.obj.dev_dir = str(tmp_path)
        fake_dest = "/home/user/.ouster_dev_completions.zsh"

        with patch("os.path.expanduser", return_value=fake_dest), \
             patch("shutil.copyfile"), \
             patch("dev_completions.update_shell_profile") as mock_update:
            dev_completions.install_zsh_completions(ctx)

        source_line = mock_update.call_args[0][1]
        assert fake_dest in source_line


# ---------------------------------------------------------------------------
# install_bash_completions
# ---------------------------------------------------------------------------

class TestInstallBashCompletions:
    def test_copies_completion_file(self, tmp_path):
        ctx = MagicMock()
        scripts_dir = tmp_path / "scripts"
        scripts_dir.mkdir()
        completions_dir = tmp_path / "completions"
        completions_dir.mkdir()
        (completions_dir / ".ouster_dev_completions.bash").write_text("# bash completions\n")
        ctx.obj.dev_dir = str(scripts_dir)

        fake_dest = str(tmp_path / ".ouster_dev_completions.bash")

        with patch("os.path.expanduser", return_value=fake_dest), \
             patch("dev_completions.update_shell_profile"):
            dev_completions.install_bash_completions(ctx)

        assert os.path.exists(fake_dest)

    def test_calls_update_shell_profile_with_bashrc(self, tmp_path):
        ctx = MagicMock()
        ctx.obj.dev_dir = str(tmp_path)
        fake_dest = "/home/user/.ouster_dev_completions.bash"

        with patch("os.path.expanduser", return_value=fake_dest), \
             patch("shutil.copyfile"), \
             patch("dev_completions.update_shell_profile") as mock_update:
            dev_completions.install_bash_completions(ctx)

        mock_update.assert_called_once()
        profile_arg = mock_update.call_args[0][0]
        assert profile_arg == "~/.bashrc"

    def test_source_line_references_dest(self, tmp_path):
        ctx = MagicMock()
        ctx.obj.dev_dir = str(tmp_path)
        fake_dest = "/home/user/.ouster_dev_completions.bash"

        with patch("os.path.expanduser", return_value=fake_dest), \
             patch("shutil.copyfile"), \
             patch("dev_completions.update_shell_profile") as mock_update:
            dev_completions.install_bash_completions(ctx)

        source_line = mock_update.call_args[0][1]
        assert fake_dest in source_line


# ---------------------------------------------------------------------------
# install_completions CLI command
# ---------------------------------------------------------------------------

class TestInstallCompletionsCommand:
    @pytest.fixture
    def runner(self):
        return CliRunner()

    @pytest.fixture
    def ctx_obj(self):
        obj = MagicMock()
        obj.dev_dir = "/mock/dev_script_library"
        return obj

    def test_explicit_zsh_calls_zsh_installer(self, runner, ctx_obj):
        with patch("dev_completions.install_zsh_completions") as mock_zsh, \
             patch("dev_completions.install_bash_completions") as mock_bash:
            result = runner.invoke(
                dev_completions.install_completions,
                ["--shell", "zsh"],
                obj=ctx_obj,
            )
        assert result.exit_code == 0
        mock_zsh.assert_called_once()
        mock_bash.assert_not_called()

    def test_explicit_bash_calls_bash_installer(self, runner, ctx_obj):
        with patch("dev_completions.install_zsh_completions") as mock_zsh, \
             patch("dev_completions.install_bash_completions") as mock_bash:
            result = runner.invoke(
                dev_completions.install_completions,
                ["--shell", "bash"],
                obj=ctx_obj,
            )
        assert result.exit_code == 0
        mock_bash.assert_called_once()
        mock_zsh.assert_not_called()

    def test_auto_detects_shell_when_not_specified(self, runner, ctx_obj):
        with patch("dev_completions.get_parent_shell", return_value="zsh"), \
             patch("dev_completions.install_zsh_completions") as mock_zsh:
            result = runner.invoke(
                dev_completions.install_completions,
                [],
                obj=ctx_obj,
            )
        assert result.exit_code == 0
        mock_zsh.assert_called_once()

    def test_unsupported_shell_prints_message(self, runner, ctx_obj):
        with patch("dev_completions.get_parent_shell", return_value="fish"):
            result = runner.invoke(
                dev_completions.install_completions,
                [],
                obj=ctx_obj,
            )
        assert result.exit_code == 0
        assert "Unsupported shell" in result.output
        assert "fish" in result.output

    def test_shell_option_is_case_insensitive(self, runner, ctx_obj):
        with patch("dev_completions.install_zsh_completions") as mock_zsh:
            result = runner.invoke(
                dev_completions.install_completions,
                ["--shell", "ZSH"],
                obj=ctx_obj,
            )
        assert result.exit_code == 0
        mock_zsh.assert_called_once()

    def test_shell_option_rejects_invalid_value(self, runner, ctx_obj):
        result = runner.invoke(
            dev_completions.install_completions,
            ["--shell", "tcsh"],
            obj=ctx_obj,
        )
        assert result.exit_code != 0
