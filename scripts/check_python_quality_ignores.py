# type: ignore

"""
Copyright (c) 2025, Ouster, Inc.
All rights reserved.
"""
import os
import glob
import re
import json

UNCONDITIONAL_TYPE = 'unconditional'
TYPE_PARSE_ISSUE = 'Issue parsing type'


def mypy_whole_file_check(lines):
    result = None
    # See if the whole file is ignored for mypy
    if re.search("# type: ignore.*", lines[0]) is not None:
        result = [UNCONDITIONAL_TYPE]
    # First alternative for mypy ignoring whole file
    if re.search("# mypy: ignore-errors.*", lines[0]) is not None:
        result = [UNCONDITIONAL_TYPE]
    # Second alternative for mypy ignoring whole file with conditions
    search = re.search(r"# mypy: disable-error-code=\"(.*)\".*", lines[0])
    if search is not None:
        result = []
        for item in search.group(1).split(","):
            result.append(item.strip())

    return result


def process_line_regex(regex, group, line_number, data_out):
    if regex is not None:
        if 'per_line' not in data_out:
            data_out['per_line'] = []
        data_out['per_line'].append(
            {'line_number': line_number, 'type': [UNCONDITIONAL_TYPE]})

        if regex.group(group) is not None:
            data_out['per_line'][-1]['type'] = []
            for temp_type in regex.group(group).split(","):
                temp_type = temp_type.strip()
                if len(temp_type) == 0:
                    temp_type = TYPE_PARSE_ISSUE
                data_out['per_line'][-1]['type'].append(temp_type)


def process_file(file_path, data_out):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        file_result = {}
        if len(lines) > 0:
            mypy_result = {}
            flake8_result = {}

            # MYPY WHOLE FILE CHECKS
            mypy_check = mypy_whole_file_check(lines)
            if mypy_check is not None:
                mypy_result['whole_file'] = mypy_check

            # FLAKE8 WHOLE FILE CHECKS
            if re.search(r"#[ ]*flake8:[ ]*noqa(:[ ]*)*", lines[0]) is not None:
                flake8_result['whole_file'] = [UNCONDITIONAL_TYPE]

            # PER LINE CHECKS
            i = 1
            for line in lines:
                # MYPY PER LINE CHECK
                process_line_regex(
                    re.search(r"#[ ]*type:[ ]*ignore(\[(.*)\])*", line), 2, i, mypy_result)
                # FLAKE8 PER LINE CHECK
                process_line_regex(
                    re.search(r"#[ ]*noqa(:[ ]*(([CFWE]\d*[ ,]*)*))*", line), 2, i, flake8_result)
                i += 1
            if mypy_result:
                file_result['mypy'] = mypy_result
            if flake8_result:
                file_result['flake8'] = flake8_result

        if file_result:
            data_out[file_path] = file_result


if __name__ == "__main__":
    results = {}

    script_dir = os.path.dirname(os.path.realpath(__file__))
    root_dir = os.path.realpath(os.path.join(script_dir, ".."))

    for item in glob.glob(f"{root_dir}/**/*.py", recursive=True):
        process_file(item, results)

    print(json.dumps(results, indent=4))
