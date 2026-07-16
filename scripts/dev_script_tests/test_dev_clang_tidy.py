import os
import sys
import pytest
from unittest.mock import MagicMock, patch, mock_open
from click.testing import CliRunner
from multiprocessing.pool import AsyncResult

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "dev_script_library")))


import dev_lint_clang_tidy as target_module  # noqa E402


class TestClangBinVersion:
    @patch("shutil.which")
    @patch("subprocess.run")
    def test_version_parsing_success(self, mock_run, mock_which):
        """Test that we correctly parse LLVM version output."""
        mock_which.return_value = "/usr/bin/clang-tidy"
        mock_run.return_value.stdout = "LLVM (http://llvm.org/):\\n  LLVM version 16.0.0 optimized build."
        mock_run.return_value.returncode = 0

        ver = target_module.ClangBinVersion("clang-tidy")

        assert ver.get_major_version() == 16
        assert ver.is_clang() is True
        assert ver.is_clangd() is False

    @patch("shutil.which")
    @patch("subprocess.run")
    def test_clangd_version_parsing(self, mock_run, mock_which):
        """Test detection of clangd-tidy."""
        mock_which.return_value = "/usr/bin/clangd-tidy"
        mock_run.return_value.stdout = "clangd-tidy version 17.0.1"
        mock_run.return_value.returncode = 0

        ver = target_module.ClangBinVersion("clangd-tidy")

        assert ver.get_major_version() == 17
        assert ver.is_clangd() is True
        assert ver.is_clang() is False

    @patch("shutil.which", return_value=None)
    def test_binary_not_found(self, mock_which):
        with pytest.raises(FileNotFoundError):
            target_module.ClangBinVersion("missing-binary")


class TestAbstractClangTidyParsing:
    """Test the regex parsing logic in the abstract base class."""

    class ConcreteTidy(target_module.AbstractClangTidy):
        def _execute_tidy(self):
            return "", ""

    def test_regex_parsing(self):
        instance = self.ConcreteTidy([], 1, False, "/base")

        # A standard clang-tidy output line
        log_line = "/base/src/main.cpp:10:5: warning: function"
        log_line += " is explicitly marked 'void' [readability-redundant-declaration]"

        match = instance._clang_message_regex.match(log_line)
        assert match is not None
        assert match.group('path') == "/base/src/main.cpp"
        assert match.group('line_number') == "10"
        assert match.group('msg_level') == "warning"
        assert match.group('name') == "readability-redundant-declaration"


class TestClangdTidySuppression:
    """Tests the complex NOLINT/BEGINNOLINT logic in ClangdTidy."""

    @pytest.fixture
    def tidier(self):
        return target_module.ClangdTidy(
            "clangd-tidy", [], 1, None, None, False, "/base"
        )

    def test_nolint_suppression(self, tidier):
        # file content simulation
        file_content = [
            "int x = 0; // NOLINT(bugprone-too-small)",   # Line 1
            "int y = 0;",                                 # Line 2
            "int z = 0; // NOLINT"                        # Line 3 (suppress all)
        ]

        with patch.object(tidier, '_get_file_lines', return_value=file_content):
            # Case 1: Specific suppression matches
            _ = target_module.AbstractClangTidy.ClangTidyEntry(
                path="test.cpp", line_number=1, column_number=1, msg_level="warning",
                msg="err", name="bugprone-too-small", split_names=False
            )
            # Case 2: Specific suppression does NOT match
            _ = target_module.AbstractClangTidy.ClangTidyEntry(
                path="test.cpp", line_number=1, column_number=1, msg_level="warning",
                msg="err", name="other-check", split_names=False
            )
            # Case 3: Global suppression
            _ = target_module.AbstractClangTidy.ClangTidyEntry(
                path="test.cpp", line_number=3, column_number=1, msg_level="warning",
                msg="err", name="any-check", split_names=False
            )

            # Manually trigger processing logic (usually done in _process_lines)
            suppressed_map = tidier._get_suppressed_lines("test.cpp")

            # Line 1 has specific check
            assert "bugprone-too-small" in suppressed_map[1]
            # Line 3 has global suppression (represented by empty list usually or implied logic)
            # Based on implementation: if "NOLINT" with no args, it returns []
            assert suppressed_map[3] == []

    def test_begin_end_nolint_block(self, tidier):
        file_content = [
            "// BEGINNOLINT(check-one)",  # Line 1
            "bad_code();",                # Line 2
            "// ENDNOLINT",               # Line 3
            "bad_code_again();"           # Line 4
        ]

        with patch.object(tidier, '_get_file_lines', return_value=file_content):
            suppressed_map = tidier._get_suppressed_lines("test.cpp")

            # Line 2 should have 'check-one'
            assert 2 in suppressed_map
            assert "check-one" in suppressed_map[2]

            # Line 4 should NOT have it
            assert 4 not in suppressed_map

    def test_nolintnextline_block(self, tidier):
        file_content = [
            "// NOLINTNEXTLINE(check-two)",  # Line 1
            "bad_code();",                   # Line 2
            "good_code();",                  # Line 3
            "bad_code_again();"              # Line 4
        ]

        with patch.object(tidier, '_get_file_lines', return_value=file_content):
            suppressed_map = tidier._get_suppressed_lines("test.cpp")

            # Line 2 should have 'check-one'
            assert 2 in suppressed_map
            assert "check-two" in suppressed_map[2]

            # Line 4 should NOT have it
            assert 4 not in suppressed_map


class TestGitDiffLogic:
    def test_check_new_warnings_in_diff(self):
        """Ensure we flag warnings on added lines but ignore existing ones."""

        # Mock Git Diff Object
        mock_diff = MagicMock()
        mock_diff.a_path = "src/changed.cpp"
        mock_diff.b_path = "src/changed.cpp"
        # Mocking the blobs to simulate a change at line 20
        # This is complex to mock via difflib, so we mock get_added_line_numbers instead

        entries = [
            # Old warning (line 5)
            target_module.AbstractClangTidy.ClangTidyEntry(
                "src/changed.cpp", 5, 1, "Warning", "msg", "check", False
            ),
            # New warning (line 20)
            target_module.AbstractClangTidy.ClangTidyEntry(
                "src/changed.cpp", 20, 1, "Warning", "msg", "check", False
            )
        ]

        # Patch the helper function that calculates line numbers
        with patch(f"{target_module.__name__}.get_added_line_numbers") as mock_get_lines:
            mock_get_lines.return_value = [20, 21, 22]  # Lines added in the diff

            # Run check
            with patch("builtins.open", mock_open()):  # Suppress file writing
                result = target_module.check_for_new_warnings_in_diff(
                    entries, [mock_diff], log_new_warnings_file="dummy.json"
                )

            # Result should be False because a regression was found at line 20
            assert result is False


class TestClangTidyMultiprocessing:
    """
    Tests for the ClangTidy._execute_tidy method.
    """

    @pytest.fixture
    def tidy_instance(self):
        tidy = target_module.ClangTidy(
            clang_tidy_bin="clang-tidy",
            clang_apply_bin="clang-apply",
            paths=["file1.cpp", "file2.cpp", "file3.cpp"],
            threads=2,
            compile_commands="compile_commands.json",
            clang_tidy_config=".clang-tidy",
            build_dir="build",
            split_names=False,
            base_dir="/root",
            progress_bar=True,
            quiet=True
        )
        return tidy

    @patch(f"{target_module.__name__}.Pool")
    @patch("tqdm.tqdm")
    def test_pool_flushing_and_result_aggregation(self, mock_tqdm, mock_pool_cls, tidy_instance):
        # 1. Setup the Mock Pool Instance
        mock_pool_instance = mock_pool_cls.return_value

        # CRITICAL FIX: Ensure the 'with Pool() as pool' variable is this same instance
        mock_pool_instance.__enter__.return_value = mock_pool_instance

        # 2. Setup Mock AsyncResults
        result1 = MagicMock(spec=AsyncResult)
        result1.get.return_value = "Output from file1\n"
        result1.ready.return_value = True

        result2 = MagicMock(spec=AsyncResult)
        result2.get.return_value = "Output from file2\n"
        result2.ready.return_value = True

        result3 = MagicMock(spec=AsyncResult)
        result3.get.return_value = "Output from file3\n"
        result3.ready.return_value = True

        # Configure apply_async to return our specific results in order
        mock_pool_instance.apply_async.side_effect = [result1, result2, result3]

        # 3. Run method under test
        with patch.object(tidy_instance, '_tidy_thread'):
            raw_output, stderr_output = tidy_instance._execute_tidy()

        # 4. Assertions
        mock_pool_cls.assert_called_once_with(processes=2)

        # Verify apply_async calls
        assert mock_pool_instance.apply_async.call_count == 3
        calls = mock_pool_instance.apply_async.call_args_list
        assert calls[0][1]['args'][0] == "file1.cpp"

        # Verify Output
        assert "Output from file1" in raw_output
        assert "Output from file2" in raw_output
        assert "Output from file3" in raw_output

    @patch(f"{target_module.__name__}.Pool")
    def test_pool_waiting_logic(self, mock_pool_cls, tidy_instance):
        mock_pool_instance = mock_pool_cls.return_value

        # CRITICAL FIX
        mock_pool_instance.__enter__.return_value = mock_pool_instance

        delayed_result = MagicMock(spec=AsyncResult)
        # Simulate not ready, then not ready, then ready
        delayed_result.ready.side_effect = [False, False, True]
        delayed_result.get.return_value = "Delayed Output"

        # Return this specific result when apply_async is called
        mock_pool_instance.apply_async.return_value = delayed_result

        tidy_instance._files = ["slow_file.cpp"]

        with patch("time.sleep") as mock_sleep:
            raw_output, _ = tidy_instance._execute_tidy()

            # Assert that the loop actually waited
            assert mock_sleep.call_count >= 1
            assert "Delayed Output" in raw_output

    @patch(f"{target_module.__name__}.Pool")
    def test_no_tqdm_installed(self, mock_pool_cls, tidy_instance):
        mock_pool_instance = mock_pool_cls.return_value

        # CRITICAL FIX
        mock_pool_instance.__enter__.return_value = mock_pool_instance

        mock_res = MagicMock(spec=AsyncResult)
        mock_res.ready.return_value = True
        mock_res.get.return_value = ""
        mock_pool_instance.apply_async.return_value = mock_res

        # Patch sys.modules to simulate missing tqdm
        with patch.dict(sys.modules, {'tqdm': None}):
            with patch("sys.stdout", new_callable=MagicMock):
                tidy_instance._execute_tidy()

                # Check that flag was disabled due to ImportError
                assert tidy_instance._progress_bar is False


@pytest.fixture
def cli_runner():
    return CliRunner()


def test_clang_tidy_cli_happy_path(cli_runner, mock_context):
    """
    Test the main click command with mocks injected.
    This validates argument parsing and flow control.
    """

    # 1. Mock the ClangBinVersion check
    with patch(f"{target_module.__name__}.ClangBinVersion") as MockVersion:
        inst = MockVersion.return_value
        inst.is_clang.return_value = True
        inst.get_major_version.return_value = 16

        # 2. Mock shutil.which (used for tool checks)
        with patch("shutil.which", return_value="/usr/bin/clang-tidy"):

            # 3. Mock the actual ClangTidy class execution
            with patch(f"{target_module.__name__}.ClangTidy") as MockClangTidy:
                tidy_instance = MockClangTidy.return_value
                # It returns a set of entries
                tidy_instance.run_tidy.return_value = set()

                # 4. Invoke the command
                # We need to inject the mock_context into the invoke command
                result = cli_runner.invoke(
                    target_module.clang_tidy,
                    ['--clang-tidy-bin', 'clang-tidy', 'src/'],
                    obj=mock_context
                )

                # 5. Assertions
                if result.exit_code != 0:
                    print(result.output)

                assert result.exit_code == 0
                assert "Running clang-tidy..." in result.output

                # Verify arguments passed to ClangTidy constructor
                MockClangTidy.assert_called_once()
                call_args = MockClangTidy.call_args
                # Check build dir was passed (index 6 in constructor based on script)
                assert call_args[0][6] == mock_context.cmake_build_dir


def test_clang_tidy_diff_gate_failure(cli_runner, mock_context):
    """Test that the CLI fails if new warnings are found against a diff."""

    with patch(f"{target_module.__name__}.ClangBinVersion") as MockVersion, \
         patch("shutil.which", return_value="/usr/bin/clang-tidy"), \
         patch("git.Repo") as _, \
         patch(f"{target_module.__name__}._get_changed_files", return_value=["file.cpp"]), \
         patch(f"{target_module.__name__}.check_for_new_warnings_in_diff", return_value=False), \
         patch(f"{target_module.__name__}.ClangTidy") as MockClangTidy:

        # Setup version
        MockVersion.return_value.is_clang.return_value = True
        MockVersion.return_value.get_major_version.return_value = 16

        # Setup Tidy execution to return some dummy entry
        MockClangTidy.return_value.run_tidy.return_value = {MagicMock()}

        # Run command with --diff-against
        result = cli_runner.invoke(
            target_module.clang_tidy,
            ['--diff-against', 'origin/main', 'src/'],
            obj=mock_context
        )

        # Should fail because check_for_new_warnings_in_diff returned False
        assert result.exit_code != 0
        assert "Checkin gate failed" in result.output


class TestCLIArguments:
    """
    Tests specific to CLI argument parsing and flag propagation.
    Ensures that flags like --fix, --threads, --quiet, etc., actually reach
    the backend classes.
    """

    @pytest.fixture
    def mock_deps(self):
        """
        Setup common mocks for CLI tests to avoid repetitive patching.
        Returns a dictionary of mocks for assertion checking.
        """
        # We need to patch the classes that the CLI instantiates to check their args
        with patch(f"{target_module.__name__}.ClangTidy") as MockClangTidy, \
             patch(f"{target_module.__name__}.ClangdTidy") as MockClangdTidy, \
             patch(f"{target_module.__name__}.ClangBinVersion") as MockVersion, \
             patch("shutil.which") as MockWhich:

            # Setup defaults so the script doesn't crash early
            MockWhich.return_value = "/usr/bin/clang-tidy"

            # Mock version check to always pass (version 16+)
            version_instance = MockVersion.return_value
            version_instance.is_clang.return_value = True
            version_instance.is_clangd.return_value = False
            version_instance.get_major_version.return_value = 16

            # Ensure constructors return something usable
            MockClangTidy.return_value.run_tidy.return_value = set()
            MockClangdTidy.return_value.run_tidy.return_value = set()

            yield {
                "ClangTidy": MockClangTidy,
                "ClangdTidy": MockClangdTidy,
                "Version": MockVersion,
                "Which": MockWhich
            }

    def test_flag_propagation_build_options(self, cli_runner, mock_context, mock_deps):
        """
        Verify generic build flags (--threads, --use-system-libs, --no-manifest-mode)
        are passed to ctx.obj.build_options.process_args.
        """
        result = cli_runner.invoke(
            target_module.clang_tidy,
            [
                '--threads', '8',
                '--use-system-libs',
                '--no-manifest-mode',
                'src/'
            ],
            obj=mock_context
        )

        assert result.exit_code == 0

        # Verify calls to build_options
        mock_context.build_options.process_args.assert_called_once()
        _, kwargs = mock_context.build_options.process_args.call_args

        assert kwargs['threads'] == '8'
        assert kwargs['use_system_libs'] is True
        assert kwargs['manifest_mode'] is False

    def test_output_file_flags(self, cli_runner, mock_context, mock_deps):
        """
        Verify that --raw-output, --json-output, etc. are passed correctly
        to the ClangTidy constructor.
        """
        result = cli_runner.invoke(
            target_module.clang_tidy,
            [
                '--raw-output', 'custom_raw.txt',
                '--json-output', 'custom.json',
                '--json-summary-output', 'summary.json',
                '--timing-json', 'timing.json',
                'src/'
            ],
            obj=mock_context
        )

        assert result.exit_code == 0

        args = mock_deps['ClangTidy'].call_args[0]

        assert 'custom_raw.txt' in args
        assert 'custom.json' in args
        assert 'summary.json' in args
        assert 'timing.json' in args

    def test_fix_flag_confirmed(self, cli_runner, mock_context, mock_deps):
        """
        Verify --fix triggers the confirmation prompt and passes True to ClangTidy.
        """
        # Mock the user confirming the fix
        mock_context.build_libs.confirm_auto_fix.return_value = True

        result = cli_runner.invoke(
            target_module.clang_tidy,
            ['--fix', 'src/'],
            obj=mock_context
        )

        assert result.exit_code == 0
        mock_context.build_libs.confirm_auto_fix.assert_called_once()

        args = mock_deps['ClangTidy'].call_args[0]
        assert args[9] is True

    def test_fix_flag_aborted(self, cli_runner, mock_context, mock_deps):
        """
        Verify --fix aborts execution if user declines confirmation.
        """
        # Mock the user modifying/declining the fix
        mock_context.build_libs.confirm_auto_fix.return_value = False

        result = cli_runner.invoke(
            target_module.clang_tidy,
            ['--fix', 'src/'],
            obj=mock_context
        )

        # Should fail with UsageError
        assert result.exit_code != 0
        assert "Fix not confirmed" in result.output

        # ClangTidy should NOT have been run
        mock_deps['ClangTidy'].assert_not_called()

    def test_quiet_flag(self, cli_runner, mock_context, mock_deps):
        """Verify --quiet sets the quiet parameter."""
        result = cli_runner.invoke(
            target_module.clang_tidy,
            ['--quiet', 'src/'],
            obj=mock_context
        )
        assert result.exit_code == 0

        # Check that quiet=True was passed
        call = mock_deps['ClangTidy'].call_args_list[0]
        assert call.kwargs["quiet"] is True

    def test_clangd_tidy_fallback(self, cli_runner, mock_context, mock_deps):
        """
        Verify logic: if no binary specified, and clangd-tidy exists, use it.
        """
        # Simulate clang-tidy NOT found, but clangd-tidy FOUND
        def which_side_effect(cmd):
            if cmd == "clangd-tidy":
                return "/usr/bin/clangd-tidy"
            return None

        mock_deps['Which'].side_effect = which_side_effect

        # Setup version check for clangd logic
        mock_deps['Version'].return_value.is_clang.return_value = False
        mock_deps['Version'].return_value.is_clangd.return_value = True

        result = cli_runner.invoke(target_module.clang_tidy, ['src/'], obj=mock_context)

        assert result.exit_code == 0
        assert "Using clangd-tidy" in result.output

        # Should instantiate ClangdTidy, NOT ClangTidy
        mock_deps['ClangdTidy'].assert_called_once()
        mock_deps['ClangTidy'].assert_not_called()

    def test_custom_bin_override(self, cli_runner, mock_context, mock_deps):
        """
        Verify --clang-tidy-bin overrides auto-detection.
        """
        custom_bin = "/opt/custom/clang-tidy"
        mock_deps['Which'].return_value = custom_bin

        result = cli_runner.invoke(
            target_module.clang_tidy,
            ['--clang-tidy-bin', custom_bin, 'src/'],
            obj=mock_context
        )

        assert result.exit_code == 0
        # Verify we checked version of the CUSTOM binary
        mock_deps['Version'].assert_called_with(custom_bin)

        # Verify ClangTidy was called with this binary
        args = mock_deps['ClangTidy'].call_args[0]
        assert args[0] == custom_bin

    def test_check_for_tool_failure(self, cli_runner, mock_context, mock_deps):
        """
        Verify script exits if the requested tool is missing.
        """
        # Mock tool check returning False
        mock_context.build_libs.check_for_tool.return_value = False

        result = cli_runner.invoke(target_module.clang_tidy, ['src/'], obj=mock_context)

        # Should raise an exception or exit
        assert result.exit_code != 0
        # Verify it tried to check for the tool
        mock_context.build_libs.check_for_tool.assert_called()
