import argparse
import re
import shutil
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
            return (self.path == other.path and self.line_number == other.line_number and
                    self.column_number == other.column_number and self.msg_level == other.msg_level and
                    self.msg == other.msg and self.name == other.name)

        def __hash__(self):
            return hash((self.path, self.line_number, self.column_number,
                         self.msg_level, self.msg, tuple(self.name)))


    def __init__(self, clang_tidy_bin, paths, threads, compile_commands, clang_tidy_config, build_dir, split_names):
        self._clang_tidy_bin = clang_tidy_bin
        self._files = []
        self._split_names = split_names
        for item in paths:
            if os.path.isdir(item):
                self._files.extend(glob.glob(item + "/*.h"))
                self._files.extend(glob.glob(item + "/*.cpp"))
                self._files.extend(glob.glob(item + "/**/*.h"))
                self._files.extend(glob.glob(item + "/**/*.cpp"))
            else:
                self._files.append(item)
        self._threads = threads
        self._compile_commands = compile_commands
        self._clang_tidy_config = clang_tidy_config
        self._build_dir = build_dir
        self._clang_message_regex = re.compile(r"^(?P<path>.+):(?P<line_number>\d+):(?P<column_number>\d+): "
                                               r"(?P<msg_level>\S+): (?P<msg>.*?)( \[(?P<name>.*)\])?$")
        self._entries = set()
        self._count = 0

    def _tidy_thread(self, file_path):
        clang_tidy_args = [self._clang_tidy_bin,
                           "-extra-arg=-ferror-limit=0",
                           "-p", self._compile_commands,
                           f"--config-file={self._clang_tidy_config}",
                           file_path
                           ]
        return subprocess.run(clang_tidy_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT).stdout

    def _flush_entry(self, current_result):
        entry = self.ClangTidyEntry(current_result.group('path'), current_result.group('line_number'),
                                    current_result.group('column_number'), current_result.group('msg_level'),
                                    current_result.group('msg'), current_result.group('name'), self._split_names)
        self._entries.add(entry)
        self._count += 1

    def run_tidy(self, raw_output_file=None, json_output_file=None, json_summary_output_file=None, progress_bar=True,  quiet=True):
        if progress_bar:
            try:
                from tqdm import tqdm
            except:
                print("tqdm not found, cant display progress bar. please install tqdm")
                progress_bar = False
        _pool = Pool(processes=self._threads)
        results = [_pool.apply_async(self._tidy_thread, args=(item, )) for item in self._files]
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
                            f.write(raw_output)
                    for line in raw_output.split("\n"):
                        result = self._clang_message_regex.match(line)
                        if result:
                            self._flush_entry(result)
                    if progress_bar:
                        bar.update()
                        bar.refresh()
                    break
        if progress_bar:
            bar.clear()
            bar.close()
        temp_json = json.dumps([x.__dict__ for x in list(self._entries)], indent=4)
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

        issue_categories = dict(sorted(buckets.items(), key=lambda item: item[1], reverse=True))
        summary = {
            "unique_entries": len(self._entries),
            "total_entries": self._count,
            "issue_categories": issue_categories
        }
        if json_summary_output_file:
            with open(json_summary_output_file, 'w') as f:
                f.write(json.dumps(summary, indent=4))

        print("Most Frequent Issue Categories:")
        print(issue_categories)
        print ("Summary")
        print(summary)

        return self._entries

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run Clang Tidy')
    parser.add_argument("--clang-tidy-bin",
                        help='Path to clang-tidy binary',
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
    parser.add_argument("--json-summary-output", type=str, default=None,
                        help='Json Summary Output File')
    parser.add_argument("--raw-output", type=str, default=None,
                        help='Raw Output File')

    parser.add_argument("--split-names", default=False, action='store_true',
                        help='Split the names into their own buckets')

    parser.add_argument("--quiet", default=True, action='store_true',
                        help='Do not emit stdout messages')

    args = parser.parse_args()

    tidy = ClangTidy(args.clang_tidy_bin, args.paths,
                     args.threads_to_use, args.compile_commands_json,
                     args.clang_tidy_config, args.build_dir, args.split_names)

    tidy.run_tidy(args.raw_output, args.json_output, args.json_summary_output, sys.stdout.isatty(), args.quiet)
