import click
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


def process_line_regex(regex, group, line_number, filename):
    entries = []
    if regex is not None:
        entry = {
            "filename": filename,
            "line_number": line_number,
            "type": [UNCONDITIONAL_TYPE]
        }

        if regex.group(group) is not None:
            entry["type"] = []
            for temp_type in regex.group(group).split(","):
                temp_type = temp_type.strip()
                if len(temp_type) == 0:
                    temp_type = TYPE_PARSE_ISSUE
                entry["type"].append(temp_type)
        entries.append(entry)
    return entries


def process_file(file_path, results):
    with open(file_path, 'r') as f:
        lines = f.readlines()
        if len(lines) > 0:
            # MYPY WHOLE FILE CHECKS
            mypy_check = mypy_whole_file_check(lines)
            if mypy_check:
                results['mypy'].append({
                    "filename": file_path,
                    "line_number": -1,
                    "type": mypy_check
                })

            # FLAKE8 WHOLE FILE CHECKS
            if re.search(r"#[ ]*flake8:[ ]*noqa(:[ ]*)*", lines[0]):
                results['flake8'].append({
                    "filename": file_path,
                    "line_number": -1,
                    "type": [UNCONDITIONAL_TYPE]
                })

            # PER LINE CHECKS
            i = 1
            for line in lines:
                # MYPY PER LINE CHECK
                mypy_entries = process_line_regex(
                    re.search(r"#[ ]*type:[ ]*ignore(\[(.*)\])*", line),
                    2, i, file_path)
                results['mypy'].extend(mypy_entries)
                # FLAKE8 PER LINE CHECK
                flake8_entries = process_line_regex(
                    re.search(r"#[ ]*noqa(:[ ]*(([CFWE]\d*[ ,]*)*))*", line),
                    2, i, file_path)
                results['flake8'].extend(flake8_entries)
                i += 1


def summarize_data(results):
    results["mypy_count"] = len(results["mypy"]) if "mypy" in results else 0
    results["flake8_count"] = len(results["flake8"]) if "flake8" in results else 0


@click.command()
@click.pass_context
@click.option('--quiet', default=False,
              is_flag=True, help='Suppress output.')
@click.option('--output-dir', default=os.getcwd(),
              help=(
                  "Output directory for python ignores results. "
                  "Default is current working directory. "
                  "The final location will be output-dir/output-file."
              ))
@click.option('--output-file', default="mypy_ignores.json",
              help='Output file for python ignores results. Default is mypy_ignores.json')
def python_ignores(ctx, quiet, output_dir, output_file):
    """Check for unneeded lint ignores  in the python code."""
    results = {"mypy": [], "flake8": []}
    print(f"Checking for python ignores in {ctx.obj.sdk_dir}")
    python_dir = os.path.join(ctx.obj.sdk_dir, "python")

    for item in glob.glob(os.path.join(python_dir, "**", "*.py"),
                          recursive=True):
        process_file(item, results)
    summarize_data(results)
    if not quiet:
        print(json.dumps(results, indent=4))

    if (output_dir is not None) and (output_file is not None):
        with open(os.path.join(output_dir, output_file), 'w') as f:
            json.dump(results, f, indent=4)


def import_module(click_context):
    click_context.lint_group.add_command(python_ignores)


def finalize(click_context):
    pass
