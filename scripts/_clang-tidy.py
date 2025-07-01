"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.
"""
import argparse
import re
import glob
import os
import subprocess
from multiprocessing import Pool, cpu_count
import time
import json
import sys


class ClangTidy:
    class ClangTidyEntry:
        def __init__(self, path, line_number, column_number,
                     msg_level, msg, name, split_names):
            self.path = path
            self.line_number = line_number
            self.column_number = column_number
            self.msg_level = msg_level
            self.msg = msg
            if name:
                if split_names:
                    self.name = name.split(',')
                else:
                    self.name = [name]
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

        def __hash__(self):
            return hash((self.path, self.line_number, self.column_number,
                         self.msg_level, self.msg, tuple(self.name)))

    def __init__(self, clang_tidy_bin, clang_apply_bin, paths, threads,
                 compile_commands, clang_tidy_config, build_dir,
                 split_names, fix=False):
        self._clang_tidy_bin = clang_tidy_bin
        self._clang_apply_bin = clang_apply_bin
        self._files = []
        self._split_names = split_names
        self._fix = fix
        for item in paths:
            if os.path.isdir(item):
                self._files.extend(
                    glob.glob(item + "/**/*.cpp", recursive=True))
                # Only in the fixing case do you need to include the headers
                if fix:
                    self._files.extend(
                        glob.glob(item + "/**/*.h", recursive=True))
            else:
                self._files.append(item)

        self._threads = threads
        self._compile_commands = compile_commands
        self._clang_tidy_config = clang_tidy_config
        self._build_dir = build_dir
        self._clang_message_regex = re.compile(r"^(?P<path>.+):(?P<line_number>\d+):(?P<column_number>\d+): "
                                               r"(?P<msg_level>\S+): (?P<msg>.*?)( \[(?P<name>.*)\])?$")

        self._clang_time_profile_regex = re.compile(r"\s*(?P<user_time>\d*\.[\d]*)\s*\(\s*(\d*\.\d*%)\)"
                                                    r"\s*(?P<system_time>\d*\.[\d]*)\s*\(\s*(\d*\.\d*%)\)"
                                                    r"\s*(?P<user_system_time>\d*\.[\d]*)\s*\(\s*(\d*\.\d*%)\)"
                                                    r"\s*(?P<wall_time>\d*\.[\d]*)\s*\(\s*(\d*\.\d*%)\)"
                                                    r"\s*(?P<name>.*)")
        self._timing = {}
        self._entries = set()
        self._count = 0

    def _tidy_thread(self, file_path, enable_timing):
        clang_tidy_args = [self._clang_tidy_bin,
                           "-extra-arg=-ferror-limit=0"]
        if enable_timing:
            clang_tidy_args.append("--enable-check-profile")
        if self._fix:
            clang_tidy_args.append(
                f"--export-fixes={self._build_dir}/clang-tidy-fixes-{str(hash(file_path))}.yaml")
        clang_tidy_args.extend(["-p", self._compile_commands,
                                f"--config-file={self._clang_tidy_config}",
                                file_path])
        return subprocess.run(clang_tidy_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT).stdout

    def _flush_entry(self, current_result):
        entry = self.ClangTidyEntry(current_result.group('path'), current_result.group('line_number'),
                                    current_result.group(
                                        'column_number'), current_result.group('msg_level'),
                                    current_result.group('msg'), current_result.group('name'), self._split_names)
        self._entries.add(entry)
        self._count += 1

    def run_tidy(self, raw_output_file=None, json_output_file=None, json_summary_output_file=None,
                 timing_json=None, progress_bar=True, quiet=True):
        self._timing = {}
        self._entries = set()
        self._count = 0
        if progress_bar:
            try:
                from tqdm import tqdm
            except ImportError:
                print("tqdm not found, cant display progress bar. please install tqdm")
                progress_bar = False
        _pool = Pool(processes=self._threads)
        results = [_pool.apply_async(self._tidy_thread, args=(
            item, timing_json is not None)) for item in self._files]
        if self._fix:
            print("Scanning for Fixes")
        else:
            print("Processing Files")
        if raw_output_file:
            if os.path.exists(raw_output_file):
                os.remove(raw_output_file)
        bar = None
        if progress_bar:
            bar = tqdm(total=len(self._files))
        while len(results) > 0:
            if progress_bar:
                bar.refresh()
            time.sleep(1)
            for item in results:
                if item.ready():
                    results.remove(item)
                    raw_output = item.get().decode('utf-8')
                    if raw_output_file:
                        with open(raw_output_file, 'a') as f:
                            temp_output = raw_output.split(")  Total", 2)
                            if len(temp_output) > 1:
                                f.write(temp_output[1])
                            else:
                                f.write(raw_output)
                    for line in raw_output.split("\n"):
                        result = self._clang_message_regex.match(line)
                        if result:
                            self._flush_entry(result)
                        else:
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

                    if progress_bar:
                        bar.update()
                        bar.refresh()
                    break
        if progress_bar:
            bar.clear()
            bar.close()
        if self._fix:
            print("Applying Fixes:")
            subprocess.check_output(
                [self._clang_apply_bin, "--ignore-insert-conflict", self._build_dir])
        temp_json = json.dumps(
            [x.__dict__ for x in list(self._entries)], indent=4)
        if not quiet:
            print("Raw Output:")
            with open(raw_output_file, 'r') as f:
                print(f.read())
        print(f"Number of Unique entries: {len(self._entries)}")
        print(f"Number of Entries Encountered: {self._count}")
        if json_output_file:
            with open(json_output_file, 'w') as f:
                f.write(temp_json)
        buckets = {}
        for item in list(self._entries):
            for name in item.name:
                if name not in buckets:
                    buckets[name] = 0
                buckets[name] += 1

        issue_categories = dict(
            sorted(buckets.items(), key=lambda item: item[1], reverse=True))
        summary = {
            "unique_entries": len(self._entries),
            "total_entries": self._count,
            "issue_categories": issue_categories
        }
        if timing_json is not None:
            with open(timing_json, 'w') as f:
                data_out = dict(sorted(self._timing.items(
                ), key=lambda item: item[1]["wall_time"], reverse=True))
                f.write(json.dumps(data_out, indent=4))
        if json_summary_output_file:
            with open(json_summary_output_file, 'w') as f:
                f.write(json.dumps(summary, indent=4))

        print("Most Frequent Issue Categories:")
        print(issue_categories)

        return self._entries


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run Clang Tidy')
    parser.add_argument("--clang-tidy-bin",
                        help='Path to clang-tidy binary',
                        required=True,
                        type=str)
    parser.add_argument("--clang-apply-replacement-bin",
                        help='Path to clang-apply-replacement binary',
                        required=True,
                        type=str)
    parser.add_argument("-j", "--threads-to-use",
                        help='Number Of Threads To Use',
                        default=cpu_count(),
                        type=int)
    parser.add_argument("--compile-commands-json",
                        help='Path to compile_commands.json file',
                        required=True,
                        type=str)
    parser.add_argument("--clang-tidy-config",
                        help='Path to .clang-tidy file',
                        required=True,
                        type=str)
    parser.add_argument("--build-dir",
                        help='Path to the build directory',
                        required=True,
                        type=str)
    parser.add_argument('-p', "--paths", nargs='+',
                        help='Paths to Scan',
                        required=True)
    parser.add_argument("--json-output", type=str, default=None,
                        help='Json Output File')
    parser.add_argument("--timing-json", type=str, default=None,
                        help='Time Profiling Json File')
    parser.add_argument("--json-summary-output", type=str, default=None,
                        help='Json Summary Output File')
    parser.add_argument("--raw-output", type=str, default=None,
                        help='Raw Output File')

    parser.add_argument("--split-names", default=False, action='store_true',
                        help='Split the names into their own buckets')

    parser.add_argument("--quiet", default=True, action='store_true',
                        help='Do not emit stdout messages')
    parser.add_argument('--fix', default=False, action='store_true',
                        help=argparse.SUPPRESS)
    args = parser.parse_args()

    tidy = ClangTidy(args.clang_tidy_bin, args.clang_apply_replacement_bin,
                     args.paths, args.threads_to_use, args.compile_commands_json,
                     args.clang_tidy_config, args.build_dir, args.split_names,
                     args.fix)

    tidy.run_tidy(args.raw_output, args.json_output, args.json_summary_output,
                  args.timing_json, sys.stdout.isatty(), args.quiet)
