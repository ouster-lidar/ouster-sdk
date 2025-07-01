"""
Copyright (c) 2024, Ouster, Inc.
All rights reserved.
"""
import sys
import subprocess
import xmltodict
import xml.parsers.expat
import json
import os
import glob
import msparser
from collections import Counter

# Due to the fact that argparse atomatically associates any arguments
# containing dashes with itself, we have to manually parse.
# Without this, trying to run something like:
# python3 check_valgrind.py --memory-check python3 -m pytest .
# will fail with argparse complaining about the -m flag
# This utility is trying to mostly match the valgrind cli usage.


class ValgrindScriptArguments:
    def __init__(self):
        self.memory_profile = False
        self.memory_check = False
        self.generate_summary = False
        self.stack = False
        self.file_prefix = ""
        self.output = os.path.join(os.getcwd(), "check_valgrind_output")
        self.command = []

    def help(self):
        print("Usage: check_valgrind.py [--memory-profile] [--memory-check] "
              "[--output <output dir>] <command>")
        print("Must supply at least one of the --memory commands")
        print("Multiple --memory commands may be specified")
        print(" --memory-profile: Profile memory usage")
        print(" --memory-check: Check for memory issues")
        print(" --generate-summary: Summarize memory check results")
        print(" --file-prefix: Prefix to filename")
        print(" --stack: Enable Stack Memory Usage")
        print("--output <output dir>: The output directory to write to")
        print("<command>: The command to run under valgrind"
              "(will consume the rest of the args)")

    def parse_args(self):
        optional_parsing = True
        output_parsing = False
        assign_file_prefix = False
        for item in sys.argv[1:]:
            if assign_file_prefix:
                self.file_prefix = item
                assign_file_prefix = False
                continue
            if output_parsing:
                self.output = item
                output_parsing = False
                continue
            elif optional_parsing:
                if "--memory-profile" in item:
                    self.memory_profile = True
                elif "--memory-check" in item:
                    self.memory_check = True
                elif "--stack" in item:
                    self.stack = True
                elif "--generate-summary" in item:
                    self.generate_summary = True
                elif "--file-prefix" in item:
                    assign_file_prefix = True
                elif item == "--output":
                    output_parsing = True
                else:
                    optional_parsing = False
                    self.command.append(item)
            else:
                self.command.append(item)

        if (not self.memory_check) and (not self.memory_profile):
            print("ERROR: Script needs 1 or more of the following specified:")
            print("--memory-profile")
            print("--memory-check")
            self.help()
            return False

        if os.path.exists(self.output):
            if not os.path.isdir(self.output):
                print(
                    f"ERROR: specified --output path: \"{self.output}\" is not a directory")
                self.help()
                return False
        return True


def check_valgrind_output(valgrind_output):
    if valgrind_output.returncode != 0:
        print("ERROR: valgrind execution failed")
        print(valgrind_output.stderr)
        sys.exit(2)


if __name__ == '__main__':
    args = ValgrindScriptArguments()
    if not args.parse_args():
        sys.exit(1)

    memory_check = args.memory_check
    memory_profile = args.memory_profile
    generate_summary = args.generate_summary
    stack = args.stack
    file_prefix = args.file_prefix
    output = args.output
    command = args.command
    os.makedirs(output, exist_ok=True)
    memory_check_data_out = {}
    memory_profile_data_out = {}
    if memory_check:
        full_command = ["valgrind",
                        "--leak-check=full",
                        "--show-leak-kinds=all",
                        "--trace-children=yes",
                        "--xml=yes",
                        f"--xml-file={output}/memcheck.xml.%p"]
        full_command.extend(command)

        valgrind_output = subprocess.run(full_command,
                                         capture_output=True, text=True)
        check_valgrind_output(valgrind_output)

        memory_check_data_out["memory_check"] = []
        for item in glob.glob(f"{output}/memcheck.xml.*"):
            try:
                with open(item, 'r') as f:
                    memory_check_data_out["memory_check"].append(
                        xmltodict.parse(f.read()))
            except xml.parsers.expat.ExpatError:
                continue
        with open(os.path.join(output, f"{file_prefix}memory_check_valgrind.json"), 'w') as f:
            f.write(json.dumps(memory_check_data_out))

    if generate_summary:
        final_data = {}
        error_summary = []
        for data in memory_check_data_out["memory_check"]:
            if "error" in data.get("valgrindoutput", ""):
                errors_full = data["valgrindoutput"]["error"]
                for error in errors_full:
                    error_local = {}
                    error_local["unique"] = error["unique"]
                    error_local["kind"] = error["kind"]
                    # Parse the error details
                    if "what" in error:
                        error_local["issue_text"] = error["what"]
                    elif "xwhat" in error:
                        error_local["issue_text"] = error["xwhat"]["text"]
                    # add to error_summary
                    error_summary.append(error_local)

                # Count occurences of unique issues
                kind_counts = Counter(item["kind"] for item in error_summary)
                unique_kinds_json = json.dumps(
                    {"unique_count": dict(kind_counts)}, indent=4)

                # add to final output json
                final_data["total_count"] = len(error_summary)
                final_data["unique_count"] = len(
                    set(item["kind"] for item in error_summary))
                final_data["unique_categories"] = unique_kinds_json
                final_data["error_list"] = error_summary
            else:
                # Cpp test results do not output the error stack with memory check output
                print("No error stack to summarize. Run without generate summary.")

        if final_data:
            with open(os.path.join(output, f"{file_prefix}memory_check_summary_valgrind.json"), 'w') as f:
                f.write(json.dumps(final_data))

    if memory_profile:
        full_command = ["valgrind",
                        "--tool=massif"]
        if stack:
            full_command.append("--stacks=yes")
        full_command.extend([f"--massif-out-file={output}/massif.out.%p",
                            "--trace-children=yes"])
        full_command.extend(command)

        for item in glob.glob(f"{output}/massif.out.*"):
            os.remove(item)

        valgrind_output = subprocess.run(full_command,
                                         capture_output=True, text=True)
        check_valgrind_output(valgrind_output)

        memory_profile_data_out["memory_profile"] = []
        for item in glob.glob(f"{output}/massif.out.*"):
            profile_data_out = {}
            data = msparser.parse_file(item)
            profile_data_out["cmd"] = data['cmd']

            profile_data_out["peak"] = data['snapshots'][data['peak_snapshot_index']]
            profile_data_out["peak"].pop("heap_tree", None)

            profile_data_out["snapshots"] = []
            for item in data['snapshots']:

                profile_data_out["snapshots"].append(item)
                profile_data_out["snapshots"][-1].pop("heap_tree", None)
            memory_profile_data_out["memory_profile"].append(profile_data_out)
        with open(os.path.join(output, f"{file_prefix}memory_profile_valgrind.json"), 'w') as f:
            f.write(json.dumps(memory_profile_data_out))
