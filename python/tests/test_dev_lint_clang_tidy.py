import unittest
from unittest.mock import patch, Mock, MagicMock, mock_open
import os
import sys
import json

lint_script_path = os.path.join(os.path.dirname(__file__), "..",
                                "..", "scripts", "dev_script_library")
lint_script_path = os.path.abspath(lint_script_path)
sys.path.insert(0, lint_script_path)
import dev_lint_clang_tidy  # type: ignore # noqa


class TestClangTidyEntry(unittest.TestCase):
    """Tests for the ClangTidyEntry inner class."""

    def test_initialization(self):
        entry = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "path/to/file.cpp", "10", "5", "warning", "Test message", "test.check", False)
        self.assertEqual(entry.path, "path/to/file.cpp")
        self.assertEqual(entry.line_number, 10)
        self.assertEqual(entry.column_number, 5)
        self.assertEqual(entry.msg_level, "warning")
        self.assertEqual(entry.msg, "Test message")
        self.assertEqual(entry.name, ["test.check"])

    def test_initialization_with_split_names(self):
        entry = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "1", "1", "error", "msg", "check1,check2", True)
        self.assertEqual(entry.name, ["check1", "check2"])

    def test_equality(self):
        entry1 = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "10", "5", "warning", "msg", "check", False)
        entry2 = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "10", "5", "warning", "msg", "check", False)
        self.assertEqual(entry1, entry2)
        self.assertEqual(hash(entry1), hash(entry2))

    def test_inequality(self):
        entry1 = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "10", "5", "warning", "msg", "check", False)
        entry2 = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "11", "5", "warning", "msg", "check", False)
        self.assertNotEqual(entry1, entry2)
        self.assertNotEqual(hash(entry1), hash(entry2))

    def test_less_than(self):
        entry1 = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "10", "5", "warning", "msg", "check", False)
        entry2 = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "11", "1", "warning", "msg", "check", False)
        self.assertLess(entry1, entry2)


class TestAbstractClangTidy(unittest.TestCase):
    """Tests for the AbstractClangTidy base class."""

    class ConcreteTidy(dev_lint_clang_tidy.AbstractClangTidy):
        """A concrete implementation for testing purposes."""
        def _execute_tidy(self):
            return "", ""

    def setUp(self):
        self.tidy = self.ConcreteTidy(paths=[], threads=1, split_names=False, base_dir="/base")

    def test_flush_entry(self):
        line = "path/to/file.cpp:10:5: warning: Test message [test.check]"
        test_match = self.tidy._clang_message_regex.match(line)
        self.tidy._flush_entry(test_match)

        self.assertEqual(len(self.tidy._entries), 1)
        self.assertEqual(self.tidy._count, 1)

        entry = list(self.tidy._entries)[0]
        # os.path.relpath behavior can vary, so let's make the assertion robust
        self.assertTrue(entry.path.replace("\\", "/").endswith("path/to/file.cpp"))
        self.assertEqual(entry.line_number, 10)
        self.assertEqual(entry.column_number, 5)
        self.assertEqual(entry.msg_level, "warning")
        self.assertEqual(entry.msg, "Test message")
        self.assertEqual(entry.name, ["test.check"])

    def test_process_lines(self):
        lines = [
            "/base/path/to/file.cpp:10:5: warning: Test message [test.check]",
            "some other log line",
            "/base/path/to/another.cpp:20:1: error: Another message [another.check]"
        ]
        self.tidy._process_lines(lines)

        self.assertEqual(len(self.tidy._entries), 2)
        self.assertEqual(self.tidy._count, 2)

        temp_entries = sorted(list(self.tidy._entries))

        self.assertEqual('path/to/another.cpp', temp_entries[0].path.replace('\\', '/'))
        self.assertEqual(20, temp_entries[0].line_number)
        self.assertEqual(1, temp_entries[0].column_number)
        self.assertEqual('error', temp_entries[0].msg_level)
        self.assertEqual('Another message', temp_entries[0].msg)
        self.assertEqual(['another.check'], temp_entries[0].name)

        self.assertEqual('path/to/file.cpp', temp_entries[1].path.replace('\\', '/'))
        self.assertEqual(10, temp_entries[1].line_number)
        self.assertEqual(5, temp_entries[1].column_number)
        self.assertEqual('warning', temp_entries[1].msg_level)
        self.assertEqual('Test message', temp_entries[1].msg)
        self.assertEqual(['test.check'], temp_entries[1].name)

    def test_flush_entry_with_complex_message(self):
        line = "ouster_mapping/src/trajectory.cpp:444:45: Error: No viable " + \
            "conversion from 'Array<[...], 0, [2 * ...]>' to 'Array<[...], " +\
            "Eigen::Dynamic aka -1, [2 * ...]>' [typecheck_nonviable_condition]"
        test_match = self.tidy._clang_message_regex.match(line)
        self.tidy._flush_entry(test_match)

        self.assertEqual(len(self.tidy._entries), 1)
        self.assertEqual(self.tidy._count, 1)

        entry = list(self.tidy._entries)[0]
        self.assertTrue(entry.path.replace("\\", "/").endswith("ouster_mapping/src/trajectory.cpp"))
        self.assertEqual(entry.line_number, 444)
        self.assertEqual(entry.column_number, 45)
        self.assertEqual(entry.msg_level, "Error")
        self.assertEqual(entry.msg,
                         "No viable conversion from 'Array<[...], 0, [2 * ...]>' " +
                         "to 'Array<[...], Eigen::Dynamic aka -1, [2 * ...]>'")
        self.assertEqual(entry.name, ["typecheck_nonviable_condition"])


class TestClangTidy(unittest.TestCase):
    """Tests for the standard ClangTidy class."""

    @patch('subprocess.run')
    def test_tidy_thread(self, mock_subprocess_run):
        # Setup
        tidy = dev_lint_clang_tidy.ClangTidy(
            clang_tidy_bin='clang-tidy',
            clang_apply_bin='clang-apply-replacements',
            paths=['/path/to/file.cpp'],
            threads=1,
            compile_commands='/path/to/build',
            clang_tidy_config='/path/to/.clang-tidy',
            build_dir='/path/to/build',
            split_names=False,
            base_dir='/path/to'
        )

        # Action
        tidy._tidy_thread('file.cpp')

        # Assert
        mock_subprocess_run.assert_called_once()
        args = mock_subprocess_run.call_args[0][0]
        self.assertIn('clang-tidy', args)
        self.assertIn('file.cpp', args)
        self.assertIn('-p', args)
        self.assertIn('/path/to/build', args)
        self.assertIn('--config-file=/path/to/.clang-tidy', args)

    @patch('shutil.which', return_value='clang-tidy')
    @patch('time.sleep')
    @patch('subprocess.run')
    @patch('dev_lint_clang_tidy.Pool')
    def test_execute_tidy_parallel(self, mock_pool, mock_subprocess_run, mock_sleep, mock_which):
        # Setup mock for subprocess.run
        mock_subprocess_run.return_value.stdout = "output from clang-tidy"
        mock_subprocess_run.return_value.stderr = ""

        # Setup mock pool
        mock_async_result1 = MagicMock()
        mock_async_result1.ready.return_value = True
        mock_async_result1.get.return_value = "pool output 1"
        mock_async_result2 = MagicMock()
        mock_async_result2.ready.return_value = True
        mock_async_result2.get.return_value = "pool output 2\noutput from clang-tidy"

        mock_pool_instance = mock_pool.return_value.__enter__.return_value
        mock_pool_instance.apply_async.side_effect = [mock_async_result1, mock_async_result2]

        # Setup ClangTidy instance
        tidy = dev_lint_clang_tidy.ClangTidy(
            clang_tidy_bin='ct', clang_apply_bin='car',
            paths=['file1.cpp', 'file2.cpp'], threads=2,
            compile_commands='.', clang_tidy_config='.', build_dir='.',
            split_names=False, base_dir='.', progress_bar=False
        )

        # Action
        output, errors = tidy._execute_tidy()

        # Assert
        mock_pool.assert_called_once_with(processes=2)
        self.assertEqual(mock_pool_instance.apply_async.call_count, 2)
        mock_pool_instance.apply_async.assert_any_call(tidy._tidy_thread, args=('file1.cpp',))
        mock_pool_instance.apply_async.assert_any_call(tidy._tidy_thread, args=('file2.cpp',))

        self.assertIn("pool output 1", output)
        self.assertIn("pool output 2", output)
        self.assertIn("output from clang-tidy", output)
        self.assertEqual(errors, "")


class TestClangdTidy(unittest.TestCase):
    """Tests for the ClangdTidy class, focusing on NOLINT parsing."""

    def setUp(self):
        self.tidy = dev_lint_clang_tidy.ClangdTidy(
            clang_tidy_bin='clangd-tidy',
            paths=[],
            threads=1,
            compile_commands=None,
            clang_tidy_config=None,
            split_names=False,
            base_dir='/project'
        )

    def test_parse_directive_checks(self):
        # Test specific checks
        line1 = "// NOLINT(check1, check2)"
        self.assertEqual(self.tidy._parse_directive_checks("NOLINT", line1), ["check1", "check2"])

        # Test suppress all
        line2 = "// NOLINT"
        self.assertEqual(self.tidy._parse_directive_checks("NOLINT", line2), [])

        # Test suppress all with empty parens
        line3 = "// NOLINT()"
        self.assertEqual(self.tidy._parse_directive_checks("NOLINT", line3), [])

        # Test multi-line
        line4 = """/* BEGINNOLINT(
 check1,
 check2
) */"""
        self.assertEqual(self.tidy._parse_directive_checks("BEGINNOLINT", line4), ["check1", "check2"])

    @patch('builtins.open', new_callable=mock_open)
    @patch('os.path.exists', return_value=True)
    def test_get_suppressed_lines(self, mock_exists, mock_file):
        file_content = [
            "line 1\n",
            "line 2 // NOLINT(check-a)\n",
            "line 3\n",
            "// NOLINTNEXTLINE(check-b)\n",
            "line 5\n",
            "// BEGINNOLINT(check-c)\n",
            "line 7\n",
            "line 8\n",
            "// BEGINNOLINT(check-d)\n",
            "line 10\n",
            "// ENDNOLINT\n",
            "// ENDNOLINT\n",
            "line 13 //NOLINT\n",
            "line 14\n"
        ]
        mock_file.return_value.readlines.return_value = file_content

        suppressed = self.tidy._get_suppressed_lines("test.cpp")

        self.assertNotIn(1, suppressed)
        self.assertEqual(suppressed[2], ["check-a"])
        self.assertNotIn(3, suppressed)
        self.assertNotIn(4, suppressed)
        self.assertEqual(suppressed[5], ["check-b"])
        self.assertNotIn(6, suppressed)
        self.assertEqual(suppressed[7], ["check-c"])
        self.assertEqual(suppressed[8], ["check-c"])
        self.assertNotIn(9, suppressed)
        self.assertEqual(sorted(suppressed[10]), sorted(["check-c", "check-d"]))
        self.assertNotIn(11, suppressed)
        self.assertNotIn(12, suppressed)
        self.assertEqual(suppressed[13], [])  # Empty list means all checks
        self.assertNotIn(14, suppressed)

    @patch('dev_lint_clang_tidy.ClangdTidy._get_suppressed_lines')
    def test_process_lines_filters_suppressed(self, mock_get_suppressed):
        mock_get_suppressed.return_value = {
            10: ['test.check'],  # Line 10 suppresses 'test.check'
            20: []  # Line 20 suppresses everything
        }

        lines = [
            "test.cpp:10:5: warning: Suppressed message [test.check]",
            "test.cpp:15:5: warning: Unsuppressed message [another.check]",
            "test.cpp:20:1: error: Suppressed all message [some.check]"
        ]
        self.tidy._process_lines(lines)

        self.assertEqual(len(self.tidy._entries), 1)
        self.assertEqual(self.tidy._count, 1)
        self.assertEqual(len(self.tidy._suppressed_warnings), 2)

        entry = list(self.tidy._entries)[0]
        self.assertEqual(entry.line_number, 15)
        self.assertEqual(entry.name, ["another.check"])


class TestClangBinVersion(unittest.TestCase):
    """Tests for ClangBinVersion class."""

    @patch('shutil.which', return_value='clang-tidy')
    @patch('subprocess.run')
    def test_clang_tidy_version_parsing(self, mock_run, mock_which):
        mock_run.return_value.stdout = "LLVM version 17.0.1\nSome other info"
        version_checker = dev_lint_clang_tidy.ClangBinVersion("clang-tidy")

        self.assertEqual(version_checker.get_major_version(), 17)
        self.assertTrue(version_checker.is_clang())
        self.assertFalse(version_checker.is_clangd())

    @patch('shutil.which', return_value='clangd-tidy')
    @patch('subprocess.run')
    def test_clangd_tidy_version_parsing(self, mock_run, mock_which):
        mock_run.return_value.stdout = "clangd-tidy version 18.0.0\nSome other info"
        version_checker = dev_lint_clang_tidy.ClangBinVersion("clangd-tidy")

        self.assertEqual(version_checker.get_major_version(), 18)
        self.assertFalse(version_checker.is_clang())
        self.assertTrue(version_checker.is_clangd())

    @patch('shutil.which', return_value=None)
    def test_raises_file_not_found(self, mock_which):
        with self.assertRaises(FileNotFoundError):
            dev_lint_clang_tidy.ClangBinVersion("non_existent_binary")


class TestDiffLogic(unittest.TestCase):
    """Tests for git diff processing functions."""

    @patch('difflib.SequenceMatcher')
    def test_get_added_line_numbers(self, mock_sequence_matcher):
        # Mocking git.Diff object and its blobs
        mock_diff = Mock()
        mock_diff.a_blob.data_stream.read.return_value = b'line one\nline two\nline three\n'
        mock_diff.b_blob.data_stream.read.return_value = b'line one\nline three\n'

        # Configure the mock to return opcodes that indicate an insertion
        mock_matcher_instance = mock_sequence_matcher.return_value
        mock_matcher_instance.get_opcodes.return_value = [
            ('equal', 0, 1, 0, 1),
            ('delete', 1, 2, 1, 1),
            ('insert', 2, 2, 1, 2),  # Inserting 'line three' at line 2
            ('equal', 2, 3, 2, 3)
        ]

        added_lines = dev_lint_clang_tidy.get_added_line_numbers(mock_diff)
        self.assertEqual(added_lines, [2])

    def test_check_for_new_warnings_in_diff(self):
        # Mocks
        entry_new = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "25", "1", "Warning", "new warning", "new.check", False)
        entry_old = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "file.cpp", "50", "1", "Warning", "old warning", "old.check", False)

        mock_diff = Mock()
        mock_diff.b_path = "file.cpp"
        diffs = [mock_diff]

        # Patch the helper function that calculates added lines
        with patch('dev_lint_clang_tidy.get_added_line_numbers', return_value=[25, 26]):
            result = dev_lint_clang_tidy.check_for_new_warnings_in_diff([entry_new, entry_old], diffs)

            # The function should return False because a new warning was found
            self.assertFalse(result)

            # Test again with no new warnings
            result_ok = dev_lint_clang_tidy.check_for_new_warnings_in_diff([entry_old], diffs)
            self.assertTrue(result_ok)

    def test_get_added_line_numbers_new_file(self):
        mock_diff = Mock()
        mock_diff.b_blob = None
        mock_diff.a_blob.data_stream.read.return_value = b'line 1\nline 2\n'
        added_lines = dev_lint_clang_tidy.get_added_line_numbers(mock_diff)
        self.assertEqual(added_lines, [1, 2])

    def test_get_added_line_numbers_deleted_file(self):
        mock_diff = Mock()
        mock_diff.a_blob = None
        mock_diff.b_blob.data_stream.read.return_value = b'line 1\n'
        added_lines = dev_lint_clang_tidy.get_added_line_numbers(mock_diff)
        self.assertEqual(added_lines, [])

    def test_get_added_line_numbers_no_changes(self):
        mock_diff = Mock()
        mock_diff.a_blob.data_stream.read.return_value = b'line 1\n'
        mock_diff.b_blob.data_stream.read.return_value = b'line 1\n'
        added_lines = dev_lint_clang_tidy.get_added_line_numbers(mock_diff)
        self.assertEqual(added_lines, [])

    @patch('builtins.open', new_callable=mock_open)
    def test_check_for_new_warnings_in_diff_skipped_log(self, mock_file):
        entry = dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
            "unrelated.cpp", "10", "1", "warning", "msg", "check", False)
        diffs = []  # No diffs, so all files with warnings will be skipped
        dev_lint_clang_tidy.check_for_new_warnings_in_diff([entry], diffs, log_skipped_file="skipped.txt")
        mock_file.assert_called_once_with("skipped.txt", "w")
        handle = mock_file()
        written_content = "".join(call[0][0] for call in handle.write.call_args_list)
        data = json.loads(written_content)
        self.assertIn(str(entry), data["prev_warnings"])


class TestAbstractClangTidyReports(unittest.TestCase):
    """Tests for the report generation of AbstractClangTidy."""

    class ConcreteTidy(dev_lint_clang_tidy.AbstractClangTidy):
        def _execute_tidy(self):
            self._raw_out = "raw output"
            self._entries = {
                dev_lint_clang_tidy.AbstractClangTidy.ClangTidyEntry(
                    "file.cpp", "10", "5", "warning", "msg", "check", False)
            }
            return "stdout", "stderr"

    @patch("builtins.open", new_callable=mock_open)
    def test_generate_reports(self, mock_open_):
        tidy = self.ConcreteTidy(paths=[], threads=1, split_names=False, base_dir="/base",
                                 raw_output_file="raw.txt", json_output_file="raw.json",
                                 json_summary_output_file="summary.json")
        tidy.run_tidy()  # This will call _generate_reports

        # Check that the files were opened for writing
        self.assertEqual(mock_open_.call_count, 3)
        mock_open_.assert_any_call("raw.txt", "w")
        mock_open_.assert_any_call("raw.json", "w")
        mock_open_.assert_any_call("summary.json", "w")


class TestClangTidyExecution(unittest.TestCase):
    """Tests for the execution logic of ClangTidy."""

    @patch('shutil.which', return_value='ct')
    @patch("builtins.open", new_callable=mock_open)
    def test_generate_reports_with_timing(self, mock_open_, mock_which):
        tidy = dev_lint_clang_tidy.ClangTidy(
            clang_tidy_bin='ct', clang_apply_bin='car', paths=[], threads=1,
            compile_commands='.', clang_tidy_config='.', build_dir='.', split_names=False, base_dir='.',
            timing_json="timing.json")

        tidy._timing = {"check1": {"user_time": 0.1, "wall_time": 0.2}}
        tidy._generate_reports()

        mock_open_.assert_any_call("timing.json", "w")
        handle = mock_open_()
        write_calls = handle.write.call_args_list
        timing_content = json.loads(write_calls[0][0][0])
        self.assertEqual(timing_content["check1"]["user_time"], 0.1)


class TestClangdTidyComplex(unittest.TestCase):
    """More complex tests for ClangdTidy."""

    def setUp(self):
        self.tidy = dev_lint_clang_tidy.ClangdTidy(
            clang_tidy_bin='clangd-tidy', paths=[], threads=1,
            compile_commands=None, clang_tidy_config=None, split_names=False, base_dir='/project')

    @patch('builtins.open', new_callable=mock_open)
    @patch('os.path.exists', return_value=True)
    def test_get_suppressed_lines_complex(self, mock_exists, mock_file):
        file_content = [
            "// BEGINNOLINT(check-a)\n",
            "line 2\n",
            "/* BEGINNOLINT(check-b) */\n",
            "line 4 // NOLINT(check-c)\n",
            "// ENDNOLINT\n",
            "line 6\n",
            "// ENDNOLINT\n",
            "line 8 \r\n",
        ]
        mock_file.return_value.readlines.return_value = file_content
        suppressed = self.tidy._get_suppressed_lines("test.cpp")

        self.assertEqual(suppressed[2], ["check-a"])
        self.assertCountEqual(suppressed[4], ["check-a", "check-b", "check-c"])
        self.assertEqual(suppressed[6], ["check-a"])
        self.assertNotIn(8, suppressed)


class TestGetChangedFiles(unittest.TestCase):
    """Tests for the _get_changed_files function."""

    @patch('os.path.exists', return_value=True)
    def test_get_changed_files(self, mock_exists):
        mock_diff_add = MagicMock()
        mock_diff_add.b_path = "new_file.cpp"

        mock_diff_mod = MagicMock()
        mock_diff_mod.b_path = "modified_file.cpp"

        mock_diff_del = MagicMock()
        mock_diff_del.b_path = None

        diffs = [mock_diff_add, mock_diff_mod, mock_diff_del]
        changed_files = dev_lint_clang_tidy._get_changed_files(diffs)

        self.assertEqual(set(changed_files), {"new_file.cpp", "modified_file.cpp"})

    def test_get_changed_files_empty(self):
        changed_files = dev_lint_clang_tidy._get_changed_files([])
        self.assertEqual(changed_files, [])


if __name__ == '__main__':
    unittest.main()
