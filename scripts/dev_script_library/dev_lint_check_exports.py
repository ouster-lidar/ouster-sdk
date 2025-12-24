import click
import sys
import glob
import os
import time
from multiprocessing import Pool


def find_annotation(node):
    import clang.cindex
    for child in node.get_children():
        if child.kind == clang.cindex.CursorKind.ANNOTATE_ATTR:
            return child.spelling


def get_full_name(item):
    stack = [item.displayname]
    filename = item.location.file
    current_node = item.lexical_parent
    while current_node is not None and current_node.spelling != filename:
        stack.append(current_node.spelling)
        current_node = current_node.lexical_parent

    return "::".join(reversed(stack[:-1]))


def process_ast(node):
    import clang.cindex
    # Output arrays
    wrong_annotation = []
    correct_annotation = []
    missing_annotation = []

    # Internal Processing Queues
    initial_queue = [node]
    ouster_queue = []

    # Dictionary to specify the "code type" to "decorator type"
    ast_type_match = {
        clang.cindex.CursorKind.CLASS_DECL: "OUSTER_API_CLASS",
        clang.cindex.CursorKind.CONSTRUCTOR: "OUSTER_API_FUNCTION",
        clang.cindex.CursorKind.DESTRUCTOR: "OUSTER_API_FUNCTION",
        clang.cindex.CursorKind.CXX_METHOD: "OUSTER_API_FUNCTION",
        clang.cindex.CursorKind.STRUCT_DECL: "OUSTER_API_CLASS",
        clang.cindex.CursorKind.FUNCTION_DECL: "OUSTER_API_FUNCTION",

    }
    # Type to use when we want to ignore an item
    ouster_api_ignore = "OUSTER_API_IGNORE"

    # Breadth first search through the nodes for items under the ouster namespace
    # We only care about items under the ouster namespace, ignore everything else
    while len(initial_queue) > 0:
        current_node = initial_queue.pop(0)
        if current_node.kind == clang.cindex.CursorKind.NAMESPACE and current_node.spelling == "ouster":
            ouster_queue.append(current_node)
        else:
            initial_queue.extend([x for x in current_node.get_children()])

    # Breadth first traversal of all of the nodes under the ouster namespace
    while len(ouster_queue) > 0:
        current_node = ouster_queue.pop(0)
        kind = current_node.kind

        # Skip if is an impl namespace
        if current_node.kind == clang.cindex.CursorKind.NAMESPACE and current_node.spelling == "impl":
            continue

        in_class = current_node.lexical_parent and (
            current_node.lexical_parent.kind == clang.cindex.CursorKind.CLASS_DECL or
            current_node.lexical_parent.kind == clang.cindex.CursorKind.STRUCT_DECL)
        in_class_template = (current_node.lexical_parent and
                             current_node.lexical_parent.kind == clang.cindex.CursorKind.CLASS_TEMPLATE)

        # Class templates do not need to be exposed in the shared library
        # due to them just being compiled from the header in the code
        # using the shared library.
        if (kind == clang.cindex.CursorKind.CXX_METHOD or
            kind == clang.cindex.CursorKind.CONSTRUCTOR or
                kind == clang.cindex.CursorKind.DESTRUCTOR) and in_class_template:
            continue

        if in_class:
            # When a function is inside of a class, this is normally a friend function
            # decleration, we dont need to expose this
            if kind == clang.cindex.CursorKind.FUNCTION_DECL:
                continue

            # We dont need to expose deleted members
            if current_node.is_deleted_method():
                continue

            # We dont need to expose any class members that are not marked public
            if current_node.access_specifier != clang.cindex.AccessSpecifier.PUBLIC:
                if kind == clang.cindex.CursorKind.CONSTRUCTOR:
                    continue
                if kind == clang.cindex.CursorKind.CXX_METHOD:
                    continue
                if kind == clang.cindex.CursorKind.STRUCT_DECL:
                    continue
                if kind == clang.cindex.CursorKind.FUNCTION_TEMPLATE:
                    continue

        # If this is a forward class/struct decl, we dont need to expose here
        if kind == clang.cindex.CursorKind.CLASS_DECL or kind == clang.cindex.CursorKind.STRUCT_DECL:
            if len([x for x in current_node.get_children()]) == 0:
                continue

        if kind == clang.cindex.CursorKind.CLASS_TEMPLATE:
            template_count = 0
            total_count = 0
            for item in current_node.get_children():
                if item.kind == clang.cindex.CursorKind.TEMPLATE_TYPE_PARAMETER:
                    template_count += 1
                if item.kind == clang.cindex.CursorKind.TEMPLATE_NON_TYPE_PARAMETER:
                    template_count += 1
                total_count += 1
            if total_count == template_count:
                continue

        if kind in ast_type_match:
            annotation = find_annotation(current_node)
            if annotation:
                if annotation == ouster_api_ignore:
                    continue
                if annotation != ast_type_match[kind]:
                    wrong_annotation.append(
                        (current_node, ast_type_match[kind], annotation))
                else:
                    correct_annotation.append(
                        (current_node, ast_type_match[kind], annotation))
            else:
                missing_annotation.append(
                    (current_node, ast_type_match[kind], annotation))
        ouster_queue.extend([x for x in current_node.get_children()])

    wrong_annotation.sort(key=lambda x: str(list(x)[0].location.file))
    wrong_annotation = [item_to_string(*x) for x in wrong_annotation]
    correct_annotation.sort(key=lambda x: str(list(x)[0].location.file))
    correct_annotation = [item_to_string(*x) for x in correct_annotation]
    missing_annotation.sort(key=lambda x: str(list(x)[0].location.file))
    missing_annotation = [item_to_string(*x) for x in missing_annotation]

    return (wrong_annotation, correct_annotation, missing_annotation)


def item_to_string(item, expected, actual):
    result = f"\t{item.location.file}:[{item.location.line}:{item.location.column}] "
    result += f"{get_full_name(item)} Kind: {item.kind}: Expected: {expected} Actual: {actual}"
    return result


def process(file_to_check, tempdir):
    import clang.cindex
    index = clang.cindex.Index.create()
    compdb = clang.cindex.CompilationDatabase.fromDirectory(tempdir)
    commands = compdb.getCompileCommands(file_to_check)

    file_args = ["-D OUSTER_CHECK_EXPORTS=1"]
    for command in commands:
        for argument in command.arguments:
            file_args.append(argument)
    tu = index.parse(file_to_check, file_args[:-2])
    wrong_annotation, correct_annotation, missing_annotation = process_ast(
        tu.cursor)

    return missing_annotation, wrong_annotation, file_to_check


@click.command()
@click.pass_context
@click.argument('paths',
                default=None,
                nargs=-1)
@click.option('--verbose',
              default=False,
              is_flag=True,
              help='Enable verbose output.')
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
def check_cpp_exports(ctx, paths, verbose, cmake_bin, vcpkg_toolchain,
                      vcpkg_triplet, threads, use_system_libs, no_manifest_mode):
    """Check if the C++ SDK shared library exports are correct labeled in code."""
    manifest_mode = not no_manifest_mode
    ctx.obj.build_options.process_args(
        cmake_bin=cmake_bin, vcpkg_toolchain=vcpkg_toolchain,
        vcpkg_triplet=vcpkg_triplet, threads=threads,
        manifest_mode=manifest_mode, use_system_libs=use_system_libs)
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)

    ctx.obj.build_options.run_vcpkg_initialized_check()

    ctx.obj.build_libs.check_for_python_lib("clang.cindex", display_name="libclang")
    ctx.obj.build_libs.check_for_tool(ctx.obj.build_options.cmake_bin)
    # if vcpkg_toolchain is None:
    #     ctx.obj.build_libs.initialize_vcpkg(ctx.obj.vcpkg_dir)
    import clang.cindex
    if "clang_CXXMethod_isDeleted" not in clang.cindex.conf.lib.__dict__:
        print("ERROR: Please use python3 -m pip install libclang")
        raise click.UsageError("ERROR: libclang >= 16 is needed to run this script.")

    check_exports_commands_dir = os.path.join(ctx.obj.cmake_build_dir,
                                              "check_exports")
    compile_commands_json = os.path.join(check_exports_commands_dir,
                                         "compile_commands.json")
    print(f"Manifest Mode: {ctx.obj.build_options.manifest_mode}")
    try:
        toolchain = None
        triplet = None
        if not ctx.obj.build_options.use_system_libs:
            toolchain = ctx.obj.build_options.vcpkg_toolchain
            triplet = ctx.obj.build_options.vcpkg_triplet
        ctx.obj.build_libs.generate_compile_commands(compile_commands_json,
                                                     ctx.obj.sdk_dir,
                                                     ctx.obj.sdk_artifact_dir,
                                                     check_exports_commands_dir,
                                                     toolchain=toolchain,
                                                     triplet=triplet,
                                                     cmake_path=ctx.obj.build_options.cmake_bin,
                                                     manifest_mode=ctx.obj.build_options.manifest_mode,
                                                     env=ctx.obj.build_options.build_env)
    except Exception:
        print("Error generating compile_commands.json")
        raise RuntimeError("Please check the output for details.")

    if len(paths) == 0:
        paths = [f"{ctx.obj.sdk_dir}/ouster_client/include/ouster",
                 f"{ctx.obj.sdk_dir}/ouster_osf/include/ouster/osf",
                 f"{ctx.obj.sdk_dir}/ouster_pcap/include/ouster",
                 f"{ctx.obj.sdk_dir}/ouster_viz/include/ouster",
                 f"{ctx.obj.sdk_dir}/ouster_sensor/include/ouster",
                 f"{ctx.obj.sdk_dir}/ouster_mapping/include/ouster"]

    files = []
    for item in paths:
        if os.path.isdir(item):
            for sub_item in glob.glob(item + "/*.h"):
                if "/impl/" not in sub_item and "/src/" not in sub_item:
                    files.append(sub_item)
            for sub_item in glob.glob(item + "/**/*.h", recursive=True):
                if "/impl/" not in sub_item and "/src/" not in sub_item:
                    files.append(sub_item)
        else:
            files.append(item)

    pool = Pool(processes=ctx.obj.build_options.threads)

    progress_bar = sys.stdout.isatty()
    bar = None
    if progress_bar:
        try:
            from tqdm import tqdm
        except ImportError:
            print("tqdm not found, cant display progress bar. please install tqdm")
    if progress_bar:
        bar = tqdm(total=len(files))

    results = [pool.apply_async(process, args=(file_to_check,
                                               check_exports_commands_dir))
               for file_to_check in files]

    missing = []
    wrong = []
    files_complete = []
    while len(results) > 0:
        if progress_bar:
            bar.refresh()
            time.sleep(0.1)
        for item in results:
            if item.ready():
                results.remove(item)
                missing_annotation, wrong_annotation, file_to_check = item.get()
                missing.extend(missing_annotation)
                wrong.extend(wrong_annotation)
                files_complete.append(file_to_check)
                if progress_bar:
                    if verbose:
                        tqdm.write(f"Finished File: {file_to_check}")
                        tqdm.write("Remaining Files:")
                        for remaining in list(set(files) - set(files_complete)):
                            tqdm.write(f"    {remaining}")
                    bar.update()
                    bar.refresh()
                break
    if progress_bar:
        bar.clear()
        bar.close()
    issues = False
    if len(wrong) > 0:
        issues = True
        print("Wrong:")
        for item in set(wrong):
            print(item)
    if len(missing) > 0:
        issues = True
        print("Missing:")
        for item in set(missing):
            print(item)

    if issues:
        click.ClickException("ERROR: Issues found in C++ SDK exports." +
                             "Please fix the above issues.")
    else:
        print("No issues detected")


def import_module(click_context):
    click_context.lint_group.add_command(check_cpp_exports)


def finalize(click_context):
    pass
