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

def find_annotation(node):
    for child in node.get_children():
        if child.kind == clang.cindex.CursorKind.ANNOTATE_ATTR:
            return child.spelling

CLASS_TEMPLATE_METHOD = ""

def process_ast(node):
    wrong_annotation = []
    correct_annotation = []
    missing_annotation = []
    initial_queue = [node]
    ouster_queue = []
    ast_type_match = {
        clang.cindex.CursorKind.CLASS_DECL: "OUSTER_API_CLASS",
        clang.cindex.CursorKind.CONSTRUCTOR: "OUSTER_API_FUNCTION",
        clang.cindex.CursorKind.DESTRUCTOR: "OUSTER_API_FUNCTION",
        clang.cindex.CursorKind.CXX_METHOD: "OUSTER_API_FUNCTION",
        clang.cindex.CursorKind.STRUCT_DECL: "OUSTER_API_CLASS",
        clang.cindex.CursorKind.FUNCTION_DECL: "OUSTER_API_FUNCTION",

    }
    ouster_api_ignore = "OUSTER_API_IGNORE"
    while len(initial_queue) > 0:
        current_node = initial_queue.pop(0)
        if current_node.kind == clang.cindex.CursorKind.NAMESPACE and current_node.spelling == "ouster":
            ouster_queue.append(current_node)
        else:
            initial_queue.extend([x for x in current_node.get_children()])

    while len(ouster_queue) > 0:
        current_node = ouster_queue.pop(0)
        kind = current_node.kind

        in_class = current_node.lexical_parent and (current_node.lexical_parent.kind == clang.cindex.CursorKind.CLASS_DECL or
                                                    current_node.lexical_parent.kind == clang.cindex.CursorKind.STRUCT_DECL)
        in_class_template = current_node.lexical_parent and current_node.lexical_parent.kind == clang.cindex.CursorKind.CLASS_TEMPLATE

        # make sure it actually is a class method if template
        if kind == clang.cindex.CursorKind.FUNCTION_TEMPLATE:
            if in_class or in_class_template:
                kind = CLASS_TEMPLATE_METHOD
                if current_node.access_specifier != clang.cindex.AccessSpecifier.PUBLIC:
                    continue

        if (kind == clang.cindex.CursorKind.CXX_METHOD or
            kind == clang.cindex.CursorKind.CONSTRUCTOR or 
            kind == clang.cindex.CursorKind.DESTRUCTOR) and in_class_template:
            kind = CLASS_TEMPLATE_METHOD
        if current_node.access_specifier != clang.cindex.AccessSpecifier.PUBLIC:
            if kind == clang.cindex.CursorKind.CONSTRUCTOR:
                continue
            if kind == clang.cindex.CursorKind.CXX_METHOD:
                continue
            if kind == clang.cindex.CursorKind.STRUCT_DECL:
                continue
            if kind == clang.cindex.CursorKind.FUNCTION_DECL:
                continue
            if kind == clang.cindex.CursorKind.FUNCTION_TEMPLATE:
                if len([x for x in current_node.get_children()]) == 0:
                    continue

        if kind == clang.cindex.CursorKind.FUNCTION_DECL and in_class:
            continue

        if kind == clang.cindex.CursorKind.CLASS_DECL:
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

        if kind == clang.cindex.CursorKind.STRUCT_DECL:
            if len([x for x in current_node.get_children()]) == 0:
                continue

        if kind in ast_type_match:
            annotation = find_annotation(current_node)
            if annotation:
                if annotation == ouster_api_ignore:
                    continue
                if annotation != ast_type_match[kind]:
                    wrong_annotation.append((current_node, ast_type_match[kind], annotation))
                else:
                    correct_annotation.append((current_node, ast_type_match[kind], annotation))
            else:
                missing_annotation.append((current_node, ast_type_match[kind], annotation))
        ouster_queue.extend([x for x in current_node.get_children()])

    wrong_annotation.sort(key=lambda x: str(list(x)[0].location.file))
    wrong_annotation = [item_to_string(*x) for x in wrong_annotation]
    correct_annotation.sort(key=lambda x: str(list(x)[0].location.file))
    correct_annotation = [item_to_string(*x) for x in correct_annotation]
    missing_annotation.sort(key=lambda x: str(list(x)[0].location.file))
    missing_annotation = [item_to_string(*x) for x in missing_annotation]

    return (wrong_annotation, correct_annotation, missing_annotation)


def get_full_name(item):
    output = ""
    stack = [item.displayname]
    filename = item.location.file
    current_node = item.lexical_parent
    while current_node is not None and current_node.spelling != filename:
        stack.append(current_node.spelling)
        current_node = current_node.lexical_parent

    return "::".join(reversed(stack[:-1]))


def item_to_string(item, expected, actual):
    return f"\t{item.location.file}:[{item.location.line}:{item.location.column}] {get_full_name(item)} Kind: {item.kind}: Expected: {expected} Actual: {actual}"


def process(file_to_check, tempdir):
    index = clang.cindex.Index.create()
    token_dict = {}
    compdb = clang.cindex.CompilationDatabase.fromDirectory(tempdir)
    commands = compdb.getCompileCommands(file_to_check)

    file_args = ["-D OUSTER_CHECK_EXPORTS=1"]
    for command in commands:
        for argument in command.arguments:
            file_args.append(argument)
    tu = index.parse(file_to_check, file_args[:-2])
    wrong_annotation, correct_annotation, missing_annotation = process_ast(tu.cursor)
    
    return missing_annotation, wrong_annotation, file_to_check

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                        prog='check_exports.py',
                        description='Check the linker exports for the project')


    parser.add_argument('--vcpkg-toolchain', default=None)
    parser.add_argument('--vcpkg-triplet', default=None)
    parser.add_argument("-j", "--threads-to-use",
                        help='Number Of Threads To Use',
                        default=cpu_count(),
                        type=int)
    parser.add_argument('-v', '--verbose',  action='store_true', default=False)

    args = parser.parse_args()

    vcpkg_toolchain = args.vcpkg_toolchain
    vcpkg_triplet = args.vcpkg_triplet
    threads = args.threads_to_use
    verbose = args.verbose

    wrong = []
    missing = []
    source_dir = os.path.dirname(os.path.abspath(__file__))
    files = []
    files.extend(glob.glob(f"{source_dir}/ouster_client/include/ouster/*.h"))
    files.extend(glob.glob(f"{source_dir}/ouster_osf/include/ouster/osf/*.h"))
    files.extend(glob.glob(f"{source_dir}/ouster_pcap/include/ouster/*.h"))
    files.extend(glob.glob(f"{source_dir}/ouster_viz/include/ouster/*.h"))

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
        subprocess.check_output(['cmake', '-B', tempdir, source_dir], cwd=tempdir)

    pool = Pool(processes=threads)

    progress_bar = sys.stdout.isatty()
    bar = None
    if progress_bar:
        try:
            from tqdm import tqdm
        except:
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
                        tqdm.write(f"Remaining Files:")
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
        print(f"Wrong:")
        for item in set(wrong):
            print(item)
    if len(missing) > 0:
        issues = True
        print(f"Missing:")
        for item in set(missing):
            print(item)

    if issues:
        sys.exit(1)
    else:
        print("No issues detected")
