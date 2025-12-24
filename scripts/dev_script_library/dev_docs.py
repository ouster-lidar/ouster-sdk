import click
import os
import glob
import re
from packaging.version import Version
import subprocess


def extended_doc_check(file_path):
    import xml.etree.ElementTree as ET
    inline_param_issues = []
    # The regex matches parameter direction tags like [in], [out], [in,out].
    # The pattern expects:
    #   - Optional leading whitespace
    #   - A square-bracketed direction list containing one or more of 'in' or 'out', comma-separated
    #   - At least one space after the bracket
    #   - Followed by any text
    # Examples matched:
    #   "[in] Description", "[out,in] Description"
    # Invalid if missing brackets or spacing, e.g., "in] Missing bracket", "[in]NoSpace", @param TextwithoutDirection
    inline_param_direction_check = re.compile(r"^\s*\[(?:in|out)(?:,(?:in|out))*\]\s+.*")
    block_param_issues = []
    tree = ET.parse(file_path)
    root = tree.getroot()
    for member in root.findall('.//memberdef[@kind="function"]'):
        # Extract full function definition
        func_type = member.findtext('type') or ''
        func_name = member.findtext('name') or ''
        func_args = member.findtext('argsstring') or ''
        function_def = f"{func_type} {func_name}{func_args}".strip()

        loc = member.find('location')
        function = {
            'function_def': function_def,
            'file': loc.attrib['file'],
            'line': loc.attrib['line'],
            'column': loc.attrib['column']}
        for child in member.findall("param"):
            try:
                param_name = child.find("declname").text
                para_elem = child.find("briefdescription").find("para")
                param_desc = child.find("briefdescription").find("para").text
                param_desc = ''.join(para_elem.itertext()).strip() if para_elem is not None else ''
                if not inline_param_direction_check.search(param_desc):
                    inline_param_issues.append((function, param_name))
            except AttributeError:
                pass
        for child in member.findall('.//parameterlist[@kind="param"]/parameteritem'
                                    '/parameternamelist/parametername'):
            if "direction" not in child.attrib:
                param_name = child.text
                block_param_issues.append((function, param_name))
    return inline_param_issues, block_param_issues


@click.command()
@click.pass_context
@click.option('--doxygen-bin',
              default="doxygen",
              help='Pass in alternative doxygen binary.')
@click.option('--output-dir',
              default=None,
              help='Pass in doxygen output directory.')
@click.option('--param-log',
              default=None,
              help="Path to write parameter direction warnings.")
@click.option('--no-werror',
              default=False,
              is_flag=True,
              help="Don't error on doxygen warnings.")
@click.option('--warning-log',
              default=None,
              help="Pass in the path to the warning log.")
@click.option('--no-inline-param-check',
              default=False,
              is_flag=True,
              help="Don't check inline parameter directions.")
def build_doxygen_docs(ctx, doxygen_bin, output_dir,
                       no_werror, warning_log,
                       no_inline_param_check, param_log):
    """Check the status of doxygen commenting in the cpp code."""
    ctx.obj.build_libs.check_for_tool(doxygen_bin)
    ctx.obj.build_libs.check_for_python_lib("xml.etree.ElementTree")
    doxygen_version = subprocess.run([doxygen_bin, "-v"],
                                     stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT).stdout
    doxygen_version = doxygen_version.decode('utf-8')
    doxygen_version = doxygen_version.split(" ")[0]
    ci_version = "1.11.0"
    fail = False
    if Version(doxygen_version) < Version(ci_version):
        raise RuntimeError(f"ERROR: doxygen needs to be updated, CI version is {ci_version}" +
                           f"       current doxygen version is {doxygen_version}")

    if Version(doxygen_version) > Version(ci_version):
        print(f"WARNING: doxygen version is greater than {ci_version}, results might differ from CI output")
        print(f"         current doxygen version is {doxygen_version}")

    if output_dir is None:
        output_dir = ctx.obj.docs_build_dir

    doxygen = ctx.obj.build_libs.Doxygen("Ouster-SDK",
                                         ctx.obj.build_libs.parse_version(ctx.obj.sdk_dir),
                                         output_dir, ctx.obj.sdk_dir, True, True,
                                         doxygen_bin=doxygen_bin, warning_log=warning_log)
    doxygen.generate_doxygen()
    warnings = doxygen.get_warnings()
    filtered_warnings = []
    exclusions = ["doxygen could be confused by a macro call",
                  "ouster::impl"]
    for line in warnings:
        found = False
        for exclusion in exclusions:
            if exclusion in line:
                found = True
                break
        if not found:
            filtered_warnings.append(line)

    if filtered_warnings:
        print(f"=============== {len(filtered_warnings)} Doxygen Warnings ===============")
        for item in filtered_warnings:
            print(item)
        print("================================================")
        if not no_werror:
            print(f"ERROR: {len(filtered_warnings)} Doxygen Warnings Exist: PLEASE FIX")
            fail = True
    else:
        print("No Doxygen Warnings Found")

    param_issues = []
    for item in glob.glob(os.path.join(output_dir, "xml", "*.xml")):
        inline, block = extended_doc_check(item)
        if not no_inline_param_check and len(inline) > 0:
            param_issues.extend(inline)
        param_issues.extend(block)

    param_lines = []
    if len(param_issues) > 0:
        print("=============== Parameter Direction Warnings ===============")
        for function, param_name in param_issues:
            warning_line = (
                f"{function['file']}:{function['line']} "
                f"Function: {function['function_def']} "
                f"Parameter: {param_name}"
            )
            print(warning_line)
            param_lines.append(warning_line)
        print("============================================================")
        if not no_werror:
            print(f"ERROR: {len(param_issues)} Parameter Direction Warnings Exist: PLEASE FIX")
            fail = True
    else:
        param_lines.append("No Parameter Direction Warnings Found")

    if param_log:
        with open(param_log, 'w') as f:
            f.write('\n'.join(param_lines) + '\n')
        print(f"Parameter direction warnings written to {param_log}")
        if len(param_issues) > 0 and not no_werror:
            fail = True

    if fail:
        raise RuntimeError("Doxygen build failed due to warnings or parameter issues.")


def import_module(click_context):
    click_context.docs_group.add_command(build_doxygen_docs)


def finalize(click_context):
    pass
