import click
import sys
import os
import re
import glob
from multiprocessing import Pool
import time
import json
import subprocess
import difflib
import abc
import shutil


class AbstractClangTidy(abc.ABC):
    class ClangTidyEntry:
        # This inner class is identical for both and belongs here.
        def __init__(self, path, line_number, column_number,
                     msg_level, msg, name, split_names):
            self.path = path
            self.line_number = int(line_number)
            self.column_number = int(column_number)
            self.msg_level = msg_level
            self.msg = msg
            if name:
                self.name = name.split(',') if split_names else [name]
            else:
                self.name = ""

        def __str__(self):
            return str(self.__dict__)

        def __eq__(self, other):
            return (self.path == other.path and
                    self.line_number == other.line_number and
                    self.column_number == other.column_number and
                    self.msg_level == other.msg_level and
                    self.msg == other.msg and self.name == other.name)

        def __lt__(self, other):
            return (self.path, self.line_number, self.column_number,
                    self.msg_level, self.msg, tuple(self.name)) < \
                    (other.path, other.line_number, other.column_number,
                     other.msg_level, other.msg, tuple(other.name))

        def __hash__(self):
            return hash((self.path, self.line_number, self.column_number,
                         self.msg_level, self.msg, tuple(self.name)))

    def __init__(self, paths, threads, split_names, base_dir,
                 raw_output_file=None, json_output_file=None, json_summary_output_file=None,
                 progress_bar=True, quiet=True):
        # Common initialization logic
        self._files = list(paths)
        self._threads = int(threads) if threads else os.cpu_count()
        self._split_names = split_names
        self._base_dir = base_dir
        self._raw_output_file = raw_output_file
        self._json_output_file = json_output_file
        self._json_summary_output_file = json_summary_output_file
        self._progress_bar = progress_bar
        self._quiet = quiet

        self._clang_message_regex = re.compile(r"^(?P<path>.+):(?P<line_number>\d+):(?P<column_number>\d+): "
                                               r"(?P<msg_level>\S+): (?P<msg>.*?)( \[(?P<name>[^]]*)\])?$")

        self._entries = set()
        self._raw_output = ""
        self._count = 0

    def _flush_entry(self, current_result):
        # Common parsing logic for a single diagnostic line
        path = current_result.group('path')
        if self._base_dir:
            path = os.path.relpath(os.path.abspath(path), self._base_dir)
        entry = self.ClangTidyEntry(path, current_result.group('line_number'),
                                    current_result.group('column_number'),
                                    current_result.group('msg_level'),
                                    current_result.group('msg'),
                                    current_result.group('name'), self._split_names)
        if entry not in self._entries:
            self._entries.add(entry)
        self._count += 1

    @abc.abstractmethod
    def _execute_tidy(self):
        """
        Abstract method for executing the specific tidy tool.
        Subclasses MUST implement this.
        It should return a tuple: (raw_stdout, raw_stderr)
        """
        pass

    def _generate_reports(self):
        if self._raw_output_file:
            with open(self._raw_output_file, 'w') as f:
                f.write(self._raw_output)

        if not self._quiet:
            print("Raw Output:")
            print(self._raw_output)

        print(f"Number of Unique entries: {len(self._entries)}")
        print(f"Number of Entries Encountered: {self._count}")

        if self._json_output_file:
            json_data = json.dumps([e.__dict__ for e in self._entries], indent=4)
            with open(self._json_output_file, 'w') as f:
                f.write(json_data)

        buckets = {}
        for item in self._entries:
            for name in item.name:
                buckets[name] = buckets.get(name, 0) + 1

        issue_categories = dict(sorted(buckets.items(), key=lambda item: item[1], reverse=True))
        summary = {
            "unique_entries": len(self._entries),
            "total_entries": self._count,
            "issue_categories": issue_categories
        }

        if self._json_summary_output_file:
            with open(self._json_summary_output_file, 'w') as f:
                f.write(json.dumps(summary, indent=4))

        print("Most Frequent Issue Categories:")
        print(json.dumps(issue_categories, indent=4))

    def _non_entry_log_line(self, line):
        pass

    def _process_lines(self, lines):
        for line in lines:
            result = self._clang_message_regex.match(line)
            if result:
                self._flush_entry(result)
            else:
                self._non_entry_log_line(line)

    def run_tidy(self):
        self._entries = set()
        self._count = 0

        self._raw_output, raw_errors = self._execute_tidy()

        if raw_errors and not self._quiet:
            print(raw_errors, file=sys.stderr)

        self._process_lines(self._raw_output.splitlines())

        self._generate_reports()

        return self._entries


class ClangTidy(AbstractClangTidy):
    def __init__(self, clang_tidy_bin, clang_apply_bin, paths, threads,
                 compile_commands, clang_tidy_config, build_dir, split_names,
                 base_dir, fix=False, timing_json=None, raw_output_file=None,
                 json_output_file=None, json_summary_output_file=None,
                 progress_bar=True, quiet=True):
        super().__init__(paths, threads, split_names, base_dir, raw_output_file=raw_output_file,
                         json_output_file=json_output_file, json_summary_output_file=json_summary_output_file,
                         progress_bar=progress_bar, quiet=quiet)
        self._clang_tidy_bin = clang_tidy_bin
        self._clang_apply_bin = clang_apply_bin
        self._compile_commands = compile_commands
        self._clang_tidy_config = clang_tidy_config
        self._build_dir = build_dir
        self._fix = fix
        self._timing_json = timing_json
        self._timing = {}

        self._clang_time_profile_regex = re.compile(r"\s*(?P<user_time>\d*\.[\d]*)\s*\(\s*(\d*\.\d*%)\)"
                                                    r"\s*(?P<system_time>\d*\.[\d]*)\s*\(\s*(\d*\.\d*%)\)"
                                                    r"\s*(?P<user_system_time>\d*\.[\d]*)\s*\(\s*(\d*\.\d*%)\)"
                                                    r"\s*(?P<wall_time>\d*\.[\d]*)\s*\(\s*(\d*\.\d*%)\)"
                                                    r"\s*(?P<name>.*)")

    def _tidy_thread(self, file_path):
        # This helper method is specific to this class's execution strategy
        args = [self._clang_tidy_bin, file_path, "-p", self._compile_commands,
                f"--config-file={self._clang_tidy_config}"]
        if self._timing_json is not None:
            args.append("--enable-check-profile")
        if self._fix:
            args.append(
                f"--export-fixes={self._build_dir}/clang-tidy-fixes-{str(hash(file_path))}.yaml")
        result = subprocess.run(args, capture_output=True, text=True)
        return result.stdout + result.stderr

    def _execute_tidy(self, progress_bar=True):
        if not self._files:
            return "", "No files to process."

        bar = None
        if self._progress_bar:
            try:
                from tqdm import tqdm
                bar = tqdm(total=len(self._files))
            except ImportError:
                print("tqdm not found, cant display progress bar. please install tqdm")
                self._progress_bar = False
        else:
            print(f"Files Remaining: {len(self._files)}")
        with Pool(processes=self._threads) as pool:
            results = [pool.apply_async(self._tidy_thread, args=(f,)) for f in self._files]
            raw_outputs = []
            while len(results) > 0:
                if bar is not None:
                    bar.refresh()
                for item in results:
                    if item.ready():
                        results.remove(item)
                        raw_outputs.append(item.get())
                        if bar is not None:
                            bar.update()
                            bar.refresh()
                        else:
                            print(f"Files Remaining: {len(results)}")
                        break
                time.sleep(0.1)
        if bar is not None:
            bar.clear()
            bar.close()
        if self._fix:
            print("Applying Fixes:")
            subprocess.check_output(
                [self._clang_apply_bin, "--ignore-insert-conflict", self._build_dir])
        return "\n".join(raw_outputs), ""

    def _non_entry_log_line(self, line):
        timing = self._clang_time_profile_regex.match(line)
        if timing:
            timing_name = timing.group("name")
            user_time = timing.group("user_time")
            system_time = timing.group("system_time")
            wall_time = timing.group("wall_time")
            if timing_name not in self._timing:
                self._timing[timing_name] = {
                    "user_time": 0.0,
                    "system_time": 0.0,
                    "wall_time": 0.0
                }
                self._timing[timing_name]["user_time"] += float(
                    user_time)
                self._timing[timing_name]["system_time"] += float(
                    system_time)
                self._timing[timing_name]["wall_time"] += float(
                    wall_time)

    def _generate_reports(self):
        super()._generate_reports()
        if self._timing_json is not None and self._timing is not None:
            with open(self._timing_json, 'w') as f:
                data_out = dict(sorted(self._timing.items(),
                                       key=lambda item: item[1]["wall_time"], reverse=True))
                f.write(json.dumps(data_out, indent=4))


class ClangdTidy(AbstractClangTidy):
    def __init__(self, clang_tidy_bin, paths, threads,
                 compile_commands, clang_tidy_config, split_names,
                 base_dir, fix=False, timing_json=None, raw_output_file=None,
                 json_output_file=None, json_summary_output_file=None,
                 progress_bar=True, quiet=True):
        super().__init__(paths, threads, split_names, base_dir, raw_output_file=raw_output_file,
                         json_output_file=json_output_file, json_summary_output_file=json_summary_output_file,
                         progress_bar=progress_bar, quiet=quiet)
        self._clangd_tidy_bin = clang_tidy_bin

        if compile_commands and os.path.isfile(compile_commands):
            self._compile_commands_dir = os.path.dirname(compile_commands)
        else:
            self._compile_commands_dir = compile_commands

        self._file_cache = {}
        self._suppressed_lines_cache = {}
        self._suppressed_warnings = []

    def _get_file_lines(self, path):
        if path in self._file_cache:
            return self._file_cache[path]

        full_path = os.path.join(self._base_dir, path)
        if not os.path.exists(full_path):
            return None

        try:
            with open(full_path, 'r', encoding='utf-8', errors='ignore') as f:
                lines = f.readlines()
                self._file_cache[path] = lines
                return lines
        except FileNotFoundError:
            return None

    def _parse_directive_checks(self, directive, text):
        """
        Parses a directive line (like NOLINT or BEGINNOLINT) to extract check names.
        """
        # This regex is designed to find a directive and capture the content within its parentheses.
        # It handles cases where the content might be multi-line, thanks to re.DOTALL.
        match = re.search(rf'{directive}.*?\((.*)\)', text, re.DOTALL)
        if match:
            content = match.group(1).strip()
            # If the parentheses are empty, it means suppress all checks.
            if not content:
                return []  # An empty list signifies suppression of all checks.
            # Split the captured content by commas to get individual check names.
            return [c.strip() for c in content.split(',')]
        # If no parentheses are found (e.g., a plain "NOLINT"), suppress all checks.
        return []

    def _get_suppressed_lines(self, path):
        if path in self._suppressed_lines_cache:
            return self._suppressed_lines_cache[path]

        lines = self._get_file_lines(path)
        if lines is None:
            return {}

        suppressed_lines = {}
        nolint_block_stack = []  # Stack to handle nested BEGINNOLINT/ENDNOLINT

        i = 0
        while i < len(lines):
            line = lines[i]
            line_num = i + 1

            # Heuristic for BEGINNOLINT:
            # Handles cases where the BEGINNOLINT directive and its categories span
            # multiple lines. This logic also supports nested blocks by using a stack.
            if "BEGINNOLINT" in line:
                full_directive_text = line
                directive_start_index = line.find('BEGINNOLINT(')
                # Check if the directive opens a parenthesis but doesn't close it on the same line.
                if directive_start_index != -1 and ')' not in line[directive_start_index:]:
                    # If so, start buffering subsequent lines until the closing parenthesis is found.
                    directive_lines = [line[directive_start_index:]]
                    j = i + 1
                    while j < len(lines):
                        next_line_text = lines[j]
                        directive_lines.append(next_line_text)
                        if ')' in next_line_text:
                            i = j  # Move the outer loop's counter forward to avoid re-processing these lines.
                            break
                        j += 1
                    # Join the buffered lines into a single string for parsing.
                    full_directive_text = " ".join(l.strip() for l in directive_lines)

                block_checks = self._parse_directive_checks("BEGINNOLINT", full_directive_text)
                nolint_block_stack.append(block_checks)
                i += 1
                continue

            # Handle ENDNOLINT
            if "ENDNOLINT" in line:
                if nolint_block_stack:
                    nolint_block_stack.pop()
                i += 1
                continue

            # Apply suppression from all active BEGINNOLINT blocks on the stack
            if nolint_block_stack:
                # Union all checks from the stack for the current line
                active_block_checks = set()
                for checks in nolint_block_stack:
                    # If any block is a general suppression (empty list), the result is a general suppression
                    if not checks:
                        active_block_checks = set()  # Representing all checks
                        break
                    active_block_checks.update(checks)

                current_checks = suppressed_lines.get(line_num)
                if current_checks is not None:
                    if not current_checks or not active_block_checks:
                        suppressed_lines[line_num] = []
                    else:
                        suppressed_lines[line_num] = list(set(current_checks) | set(active_block_checks))
                else:
                    suppressed_lines[line_num] = list(active_block_checks) if active_block_checks else []

            # Heuristic for NOLINT:
            # Handles cases where the NOLINT directive and its categories span
            # multiple lines. This is similar to the BEGINNOLINT heuristic.
            if "NOLINT" in line:
                nolint_start_line_num = line_num
                is_next_line = "NOLINTNEXTLINE" in line

                full_nolint_text = line
                nolint_start_index = line.find('NOLINT(')
                # Check for an unclosed parenthesis, indicating a multi-line directive.
                if nolint_start_index != -1 and ')' not in line[nolint_start_index:]:
                    # Buffer subsequent lines to reconstruct the full directive.
                    directive_lines = [line[nolint_start_index:]]
                    j = i + 1
                    while j < len(lines):
                        next_line_text = lines[j]
                        directive_lines.append(next_line_text)
                        if ')' in next_line_text:
                            i = j  # Move the outer loop's counter forward to avoid re-processing these lines.
                            break
                        j += 1
                    full_nolint_text = " ".join(l.strip() for l in directive_lines)

                checks = self._parse_directive_checks("NOLINT", full_nolint_text)

                target_line = nolint_start_line_num + 1 if is_next_line else nolint_start_line_num
                if target_line <= len(lines):
                    # Combine with any existing block suppressions for the target line
                    existing_checks = suppressed_lines.get(target_line)

                    if existing_checks is not None:
                        if not existing_checks or not checks:
                            suppressed_lines[target_line] = []
                        else:
                            suppressed_lines[target_line] = list(set(existing_checks) | set(checks))
                    else:
                        suppressed_lines[target_line] = checks

            i += 1

        self._suppressed_lines_cache[path] = suppressed_lines
        return suppressed_lines

    def _process_lines(self, lines):
        for line in lines:
            result = self._clang_message_regex.match(line)
            if result:
                path = result.group('path')
                entry = self.ClangTidyEntry(path, result.group('line_number'),
                                            result.group('column_number'),
                                            result.group('msg_level'),
                                            result.group('msg'),
                                            result.group('name'), self._split_names)

                suppressed_lines = self._get_suppressed_lines(entry.path)
                is_suppressed = False

                # Check current line
                suppressed_checks = suppressed_lines.get(entry.line_number)
                if suppressed_checks is not None:
                    if not suppressed_checks or any(c in suppressed_checks for c in entry.name):
                        is_suppressed = True

                # Heuristic for multi-line statements
                if not is_suppressed:
                    file_lines = self._get_file_lines(entry.path)
                    if file_lines and entry.line_number < len(file_lines):
                        line_content = file_lines[entry.line_number - 1].strip()
                        if line_content.endswith(',') or line_content.endswith('(') or line_content.endswith('\\'):
                            suppressed_checks_next = suppressed_lines.get(entry.line_number + 1)
                            if suppressed_checks_next is not None:
                                if not suppressed_checks_next or any(c in suppressed_checks_next for c in entry.name):
                                    is_suppressed = True

                if is_suppressed:
                    self._suppressed_warnings.append(entry)
                else:
                    if entry not in self._entries:
                        self._entries.add(entry)
                    self._count += 1
            else:
                self._non_entry_log_line(line)

    def _execute_tidy(self, progress_bar=True, timing_enabled=False):
        if not self._files:
            return "", "No files to process."

        args = [self._clangd_tidy_bin]
        if self._compile_commands_dir:
            args.extend(["-p", self._compile_commands_dir])
        args.extend(["-j", str(self._threads)])
        if progress_bar:
            args.append("--tqdm")
        args.extend(self._files)

        process = subprocess.run(args, capture_output=True, text=True)
        return process.stdout, process.stderr

    def _generate_reports(self):
        super()._generate_reports()
        if self._json_output_file:
            suppressed_json_path = self._json_output_file.replace(".json", "_suppressed.json")
            suppressed_warnings_dict = [e.__dict__ for e in self._suppressed_warnings]
            with open(suppressed_json_path, 'w') as f:
                json.dump(suppressed_warnings_dict, f, indent=4)
            print(f"Suppressed warnings written to: {suppressed_json_path}")
            suppressed_json_path = self._json_output_file.replace(".json", "_suppressed_debug.json")
            with open(suppressed_json_path, 'w') as f:
                json.dump(self._suppressed_lines_cache, f, indent=4)
            print(f"Suppressed warnings debug written to: {suppressed_json_path}")


class ClangBinVersion:
    def __init__(self, clang_bin):
        self.clang_bin = clang_bin
        self._version_output = ""
        self._major_version = None

        if not shutil.which(self.clang_bin):
            raise FileNotFoundError(f"Binary not found at the specified path: {self.clang_bin}")

        try:
            # Execute the command to get the version string
            result = subprocess.run(
                [self.clang_bin, "--version"],
                capture_output=True,
                text=True,
                check=True  # Raise an exception if the command returns a non-zero exit code
            )
            self._version_output = result.stdout.strip()
        except (subprocess.CalledProcessError, OSError) as e:
            # Raise an error if the binary cannot be executed or fails
            raise ValueError(f"Failed to get version from '{self.clang_bin}': {e}")

        # Pre-parse the major version number from the output
        self._parse_major_version()

    def _parse_major_version(self):
        # Regex to find a version string like X.Y.Z and capture the first number (X)
        match = re.search(r'(\d+)\.\d+\.\d+', self._version_output)
        if match:
            self._major_version = int(match.group(1))

    def get_major_version(self):
        return self._major_version

    def is_clangd(self):
        return "clangd-tidy" in self._version_output

    def is_clang(self):
        # Standard clang-tidy output includes "LLVM version"
        return "LLVM version" in self._version_output and not self.is_clangd()


def help_for_missing_clang_tidy(flag):
    print("")
    pass


def _get_changed_files(diffs):
    paths = set()
    for diff in diffs:
        if diff.b_path:
            paths.add(diff.b_path)
    return [p for p in paths if p and os.path.exists(p)]


def get_added_line_numbers(diff):
    """
    Gets added line numbers for a single git.Diff object using difflib.
    Handles cases where the file is newly created or deleted.
    """
    # Use an empty list if a blob is None (e.g., a new file has no a_blob)
    b_content = diff.a_blob.data_stream.read().decode('utf-8', 'ignore').splitlines() if diff.a_blob else []

    # Use an empty list if b_blob is None (e.g., a deleted file)
    a_content = diff.b_blob.data_stream.read().decode('utf-8', 'ignore').splitlines() if diff.b_blob else []

    added_lines = set()

    # autojunk=False is important to get a more literal diff
    matcher = difflib.SequenceMatcher(None, a_content, b_content, autojunk=False)

    # The opcodes describe how to turn a_content into b_content.
    # We look for 'insert' or 'replace' tags.
    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag == 'insert' or tag == 'replace':
            # The j1 and j2 indices refer to the b_content (new file).
            # We add the line numbers from this range to our set.
            for line_num in range(j1 + 1, j2 + 1):
                added_lines.add(line_num)

    return sorted(added_lines)


def check_for_new_warnings_in_diff(entries, diffs, log_skipped_file=None,
                                   log_new_warnings_file="clang-tidy-new-warnings.json"):
    """Checks for new clang-tidy warnings in the changed lines of a diff."""
    new_warnings = []
    added_lines_by_file = {}
    skipped_warnings = []
    prev_warnings = []
    # Pre-calculate all added lines for each changed file
    for diff in diffs:
        if diff.b_path:
            added_lines_by_file[diff.b_path] = list(get_added_line_numbers(diff))

    # Now, check all entries against our map
    for entry in entries:
        if entry.path in added_lines_by_file:
            added_lines = added_lines_by_file[entry.path]
            if entry.line_number in added_lines and entry.msg_level == "Warning":
                new_warnings.append(entry)
            else:
                skipped_warnings.append(str(entry))
        else:
            prev_warnings.append(str(entry))

    if log_skipped_file is not None:
        with open(log_skipped_file, 'w') as f:
            json.dump({'added_lines': added_lines_by_file,
                       'skipped_warnings': skipped_warnings,
                       'prev_warnings': prev_warnings}, f, indent=4)
            print(f"Skipped warnings debug written to: {log_skipped_file}")

    if new_warnings:
        print("New clang-tidy regressions were found:")
        with open(log_new_warnings_file, 'w') as f:
            for warning in new_warnings:
                print(json.dumps(warning.__dict__, indent=4))
                f.write(json.dumps(warning.__dict__, indent=4))
        return False
    return True


@click.command()
@click.pass_context
@click.option('--clang-tidy-bin',
              default=None,
              help='Pass in alternative clang-tidy binary.')
@click.option('--clang-apply-replacements-bin',
              default="clang-apply-replacements",
              help='Pass in alternative clang-apply-replacements binary.')
@click.option('--fix',
              default=False,
              is_flag=True,
              help='Apply automatic fixes to selected issues.')
@click.option('--raw-output',
              default=None,
              help='Raw output file for the results.')
@click.option('--json-output',
              default=None,
              help='Json output file for the results.')
@click.option('--json-summary-output',
              default=None,
              help='Json output file for the results.')
@click.option('--timing-json',
              default=None,
              help='Json output file for the clang-tidy utility timing.')
@click.option('--quiet',
              default=False,
              is_flag=True,
              help='Do not emit stdout messages.')
@click.option('--cmake-bin',
              default="cmake",
              help='Pass in alternative cmakes binary.')
@click.option('--vcpkg-toolchain',
              default=None,
              help='Path to alternative vcpkg toolchain file.')
@click.option('--vcpkg-triplet',
              default=None,
              help='Vcpkg triplet to use with the alternative toolchain file.')
@click.option('--use-system-libs',
              default=False,
              is_flag=True,
              help='Use system libraries instead of vcpkg.')
@click.option('--threads',
              default=None,
              help='Number of threads to use.')
@click.option('--no-manifest-mode',
              default=False,
              is_flag=True,
              help='Disable vcpkg manifest mode.')
@click.option('--diff-against',
              default=None,
              help='Git ref to diff against for line number alignment.'
              'You may run into issues without using the full origin/<branch name> (e.g., origin/develop).')
@click.argument('paths',
                default=None,
                nargs=-1)
def clang_tidy(ctx, clang_tidy_bin, clang_apply_replacements_bin,
               fix, raw_output, json_output, json_summary_output,
               timing_json, quiet, cmake_bin, vcpkg_toolchain,
               vcpkg_triplet, use_system_libs, threads,
               no_manifest_mode, diff_against, paths):
    """Run clang-tidy."""
    manifest_mode = not no_manifest_mode
    ctx.obj.build_options.process_args(
        cmake_bin=cmake_bin, vcpkg_toolchain=vcpkg_toolchain,
        vcpkg_triplet=vcpkg_triplet, threads=threads,
        manifest_mode=manifest_mode, use_system_libs=use_system_libs)
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)

    ctx.obj.build_options.run_vcpkg_initialized_check()

    config_file = os.path.join(ctx.obj.sdk_dir, ".clang-tidy")

    ctx.obj.build_libs.check_for_python_lib("pybind11")
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)
    if diff_against:
        ctx.obj.build_libs.check_for_python_lib("git", display_name="gitpython")

    is_clang_tidy = None
    if clang_tidy_bin is None:
        if shutil.which("clangd-tidy"):
            print("Using clangd-tidy")
            clang_tidy_bin = "clangd-tidy"
            is_clang_tidy = False
        else:
            print("Using clang-tidy")
            clang_tidy_bin = "clang-tidy"
            is_clang_tidy = True

    if not ctx.obj.build_libs.check_for_tool(clang_tidy_bin):
        help_for_missing_clang_tidy("--clang-tidy-bin")
        raise

    clang_tidy_version = ClangBinVersion(clang_tidy_bin)
    if clang_tidy_version.is_clang():
        clang_tidy_major_version = clang_tidy_version.get_major_version()
        if clang_tidy_major_version < 16:
            print("ERROR: clang-tidy needs to be updated, minimum version is 16")
            print(f"       current clang-tidy version is {clang_tidy_major_version}")
            help_for_missing_clang_tidy("--clang-tidy-bin")
            raise
        is_clang_tidy = True
    else:
        is_clang_tidy = False

    fix_confirmed = False
    if fix:
        fix_confirmed = ctx.obj.build_libs.confirm_auto_fix()
        if not fix_confirmed:
            raise click.UsageError("Fix not confirmed, exiting.")
        else:
            config_file = f"{config_file}-autofix"
        if not ctx.obj.build_libs.check_for_tool(clang_apply_replacements_bin):
            help_for_missing_clang_tidy("--clang-apply-replacements-bin")
            raise
        clang_tidy_apply_version = ClangBinVersion(clang_apply_replacements_bin)
        clang_tidy_apply_major_version = clang_tidy_apply_version.get_major_version()
        if clang_tidy_apply_major_version < 16:
            print("ERROR: clang-apply-replacements needs to be updated, minimum version is 16")
            print(f"       current clang-apply-replacements version is {clang_tidy_apply_major_version}")
            help_for_missing_clang_tidy("--clang-apply-replacements-bin")
            raise

    print(f"Manifest Mode: {ctx.obj.build_options.manifest_mode}")

    clang_tidy_commands_dir = os.path.join(ctx.obj.cmake_build_dir,
                                           "clang_tidy")
    compile_commands_json = os.path.join(clang_tidy_commands_dir,
                                         "compile_commands.json")
    try:
        toolchain = None
        triplet = None
        if not ctx.obj.build_options.use_system_libs:
            toolchain = ctx.obj.build_options.vcpkg_toolchain
            triplet = ctx.obj.build_options.vcpkg_triplet
        ctx.obj.build_libs.generate_compile_commands(compile_commands_json,
                                                     ctx.obj.sdk_dir,
                                                     ctx.obj.sdk_artifact_dir,
                                                     clang_tidy_commands_dir,
                                                     toolchain=toolchain,
                                                     triplet=triplet,
                                                     cmake_path=ctx.obj.build_options.cmake_bin,
                                                     manifest_mode=ctx.obj.build_options.manifest_mode,
                                                     env=ctx.obj.build_options.build_env)
    except Exception:
        print("Error generating compile_commands.json")
        raise click.ClickException("Please check the output for details.")

    if len(paths) == 0:
        paths = []
        for item in glob.glob(os.path.join(ctx.obj.sdk_dir, "ouster_*")):
            paths.append(item)
        paths.append(os.path.join(ctx.obj.sdk_dir, "python"))
        paths.append(os.path.join(ctx.obj.sdk_dir, "examples"))
        if not quiet:
            print(f"Paths not specified, using default paths: {paths}")

    _paths = []
    for item in paths:
        if os.path.isdir(item):
            _paths.extend(
                glob.glob(item + "/**/*.cpp", recursive=True))
        else:
            _paths.append(item)
    paths = _paths

    if raw_output is None:
        raw_output = os.path.join(ctx.obj.cmake_build_dir, "clang-tidy-output.txt")
    if json_output is None:
        json_output = os.path.join(ctx.obj.cmake_build_dir, "clang-tidy-output.json")
    if timing_json is None:
        timing_json = os.path.join(ctx.obj.cmake_build_dir, "clang-tidy-timing.json")
    if json_summary_output is None:
        json_summary_output = os.path.join(ctx.obj.cmake_build_dir,
                                           "clang-tidy-output-summary.json")

    diffs = None
    if diff_against:
        import git
        print(f"Limiting to changes since git ref: {diff_against}")
        try:
            repo = git.Repo(search_parent_directories=True)

            # Get the diff between the current state and the given ref
            diffs = repo.index.diff(repo.commit(diff_against))
            # Filter the paths to only include changed files
            changed_files = _get_changed_files(diffs)

            _paths = []
            for item in changed_files:
                for item2 in paths:
                    if item in item2:
                        _paths.append(item2)
            paths = _paths
            print(paths)
        except git.exc.InvalidGitRepositoryError:
            print("Warning: Not a git repository. Cannot perform diff.")
            diffs = None
        except git.exc.GitCommandError as e:
            print(f"Warning: Git command failed: {e}")
            diffs = None

    tidy = None
    if is_clang_tidy:
        tidy = ClangTidy(clang_tidy_bin, clang_apply_replacements_bin,
                         paths, ctx.obj.build_options.threads,
                         compile_commands_json, config_file,
                         ctx.obj.cmake_build_dir, False, ctx.obj.sdk_dir,
                         fix and fix_confirmed, timing_json, raw_output,
                         json_output, json_summary_output,
                         sys.stdout.isatty(), quiet)
        print("Running clang-tidy...")
    else:
        if fix:
            raise click.ClickException("Error: --fix is not supported with clangd-tidy.")
        if timing_json:
            print("Warning: --timing-json is is not supported with clangd-tidy. Ignoring.")
        tidy = ClangdTidy(clang_tidy_bin, paths,
                          ctx.obj.build_options.threads,
                          compile_commands_json,
                          config_file,
                          split_names=False,
                          base_dir=ctx.obj.sdk_dir,
                          raw_output_file=raw_output,
                          json_output_file=json_output,
                          json_summary_output_file=json_summary_output,
                          progress_bar=sys.stdout.isatty(),
                          quiet=quiet)
        print("Running clangd-tidy...")
    entries = tidy.run_tidy()

    if diffs:
        skipped_json_path = json_output.replace(".json", "_skipped_debug.json")

        if not check_for_new_warnings_in_diff(entries, diffs, skipped_json_path):
            raise click.ClickException("Checkin gate failed: new clang-tidy warnings detected.")
        else:
            print("Checkin gate passed: no new clang-tidy warnings.")


def import_module(click_context):
    click_context.lint_group.add_command(clang_tidy)


def finalize(click_context):
    pass
