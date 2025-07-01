import sys
import clang.cindex
import glob
import os
import atexit
import shutil
import subprocess
import tempfile
import argparse
import time
from multiprocessing import Pool, cpu_count

if "clang_CXXMethod_isDeleted" not in clang.cindex.conf.lib.__dict__:
    print("ERROR: libclang >= 16 is needed to run this script.")
    print("ERROR: Please use python3 -m pip install libclang")
    sys.exit(1)


def find_annotation(node):
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


if __name__ == '__main__':
    source_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

    parser = argparse.ArgumentParser(
        prog='check_exports.py',
        description='Check the linker exports for the project')

    parser.add_argument('--vcpkg-toolchain', default=None)
    parser.add_argument('--vcpkg-triplet', default=None)
    parser.add_argument("-j", "--threads-to-use",
                        help='Number Of Threads To Use',
                        default=cpu_count(),
                        type=int)
    parser.add_argument('-v', '--verbose', action='store_true', default=False)
    parser.add_argument('--paths', required=False, nargs='*',
                        help='Optional paths to restrict scan to', default=[
                            f"{source_dir}/ouster_client/include/ouster",
                            f"{source_dir}/ouster_osf/include/ouster/osf",
                            f"{source_dir}/ouster_pcap/include/ouster",
                            f"{source_dir}/ouster_viz/include/ouster"
                        ])

    args = parser.parse_args()
    vcpkg_toolchain = args.vcpkg_toolchain
    vcpkg_triplet = args.vcpkg_triplet
    threads = args.threads_to_use
    verbose = args.verbose

    wrong = []
    missing = []

    paths = []
    for item in args.paths:
        paths.append(item)

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

    tempdir = tempfile.mkdtemp()

    def cleanup():
        shutil.rmtree(tempdir)

    atexit.register(cleanup)

    if vcpkg_toolchain and vcpkg_triplet:
        subprocess.check_output(['cmake', source_dir,
                                 f"-DCMAKE_TOOLCHAIN_FILE={vcpkg_toolchain}",
                                 f"-DVCPKG_DEFAULT_TRIPLET={vcpkg_triplet}",
                                 "-DVCPKG_MANIFEST_MODE=OFF"],
                                cwd=tempdir)
    else:
        subprocess.check_output(
            ['cmake', '-B', tempdir, source_dir], cwd=tempdir)

    pool = Pool(processes=threads)

    progress_bar = sys.stdout.isatty()
    bar = None
    if progress_bar:
        try:
            from tqdm import tqdm
        except ImportError:
            print("tqdm not found, cant display progress bar. please install tqdm")
            progress_bar = False
    if progress_bar:
        bar = tqdm(total=len(files))

    results = [pool.apply_async(process, args=(file_to_check, tempdir))
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
        sys.exit(1)
    else:
        print("No issues detected")
