import click
import sys
import os
import fnmatch
import pathlib
import subprocess


additional_python_flake8_dirs = []


@click.command()
@click.pass_context
@click.option('--config',
              default=None,
              help='Pass in alternative flake8 config file.')
@click.option('--output-file',
              default=None,
              help='Output file for flake8 results.')
def flake8(ctx, config, output_file):
    """Run python flake8 linter."""
    ctx.obj.build_libs.check_for_python_lib("flake8")

    if config is None:
        config = os.path.join(ctx.obj.sdk_dir, "python", ".flake8")

    args = [sys.executable, "-m", "flake8", "--config", config]
    if output_file is not None:
        args.extend(["--format=junit-xml", "--output-file", output_file])
    args.extend([os.path.join(ctx.obj.sdk_dir, "python", "src"),
                 os.path.join(ctx.obj.sdk_dir, "python", "tests"),
                 os.path.join(ctx.obj.sdk_dir, "scripts")])
    args.extend(additional_python_flake8_dirs)

    print("Running flake8 with args: ", args)
    run = ctx.obj.build_libs.RunCommand(tty=True)
    try:
        run.run_command(*args, throw_on_error=True)
    except Exception as e:
        print("flake8 failed, please check the output.")
        print(e.__str__())
        sys.exit(1)


@click.command()
@click.pass_context
@click.option('--output-file',
              default=None,
              help='Output file for flake8 results.')
def mypy(ctx, output_file):
    """Run python mypy linter."""
    ctx.obj.build_libs.check_for_python_lib("mypy")

    args = [sys.executable, "-m", "mypy", "--install-types", "--non-interactive"]
    if output_file is not None:
        args.extend(["--junit-xml", output_file])
    args.extend([os.path.join(ctx.obj.sdk_dir, "python", "src"),
                 os.path.join(ctx.obj.sdk_dir, "python", "tests"),
                 os.path.join(ctx.obj.sdk_dir, "tests", "integration"),
                 os.path.join(ctx.obj.sdk_dir, "tests", "hil")])

    run = ctx.obj.build_libs.RunCommand(tty=True)
    try:
        run.run_command(*args, cwd=os.path.join(ctx.obj.sdk_dir, "python"),
                        throw_on_error=True)
    except Exception as e:
        print("mypy failed, please check the output.")
        print(e.__str__())
        sys.exit(1)


@click.command()
@click.pass_context
def mypy_stubs(ctx):
    """Run python mypy stub linter."""
    ctx.obj.build_libs.check_for_python_lib("mypy")

    args = [sys.executable, "-m", "mypy.stubtest", "ouster.sdk._bindings", "--concise"]

    result = subprocess.run(args,
                            text=True,
                            cwd=ctx.obj.sdk_dir,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT).stdout
    errors = []
    for line in result.split("\n"):
        if len(line) == 0:
            continue

        # ignore missing operator overloads
        if "__" in line:
            continue

        # ignore metaclass differing
        if "metaclass differs" in line:
            continue
        errors.append(line)
        print(line)

    if len(errors) > 0:
        print("mypy stub check failed, please check the output.")
        sys.exit(1)


def get_tracked_files(repo):
    # returns a set of Paths of all files in the repo
    return {
        pathlib.Path(path.path)
        for path in repo.head.commit.tree.traverse()
        if path.type == 'blob'
    }


def get_source_files(sdk_dir):
    for extension in ['h', 'hpp', 'c', 'cpp']:
        for path in pathlib.Path(sdk_dir).rglob(f"*.{extension}"):
            yield path.relative_to(sdk_dir)


def files_to_check_fn(sdk_dir):
    import git
    repo = git.Repo(sdk_dir)
    tracked_files = get_tracked_files(repo)
    excluded = [
        "thirdparty/*", "sdk-extensions/*",
        "*/optional-lite/nonstd/optional.hpp",
        "examples/cxxopts.hpp"
    ]

    files_to_check = []
    for rel_path in get_source_files(sdk_dir):
        if rel_path in tracked_files and not any([fnmatch.fnmatch(rel_path, x) for x in excluded]):
            files_to_check.append(rel_path)

    return files_to_check


@click.command()
@click.pass_context
@click.option('--clang-format-bin',
              default="clang-format",
              help='Pass in alternative clang-tidy binary.')
@click.option('--fix',
              default=False,
              is_flag=True,
              help='Apply automatic fixes to selected issues.')
@click.option('--threads',
              default=None,
              type=int,
              help='Number of threads to use.')
def clang_format(ctx, clang_format_bin, fix, threads):
    """Run clang-format linter."""
    ctx.obj.build_libs.check_for_tool(clang_format_bin)

    def iter_wrap_function(iterable, total):
        for item in iterable:
            yield item

    iter_wrap = iter_wrap_function
    if sys.stdout.isatty():
        try:
            import tqdm
            iter_wrap = tqdm.tqdm
        except ImportError:
            print("tqdm not installed, not using status bar.")

    fix_confirmed = False
    if fix:
        fix_confirmed = ctx.obj.build_libs.confirm_auto_fix()
        if not fix_confirmed:
            print("Fix not confirmed, exiting.")
            sys.exit(1)

    files_to_check = files_to_check_fn(ctx.obj.sdk_dir)
    errors = []
    if len(files_to_check) < 1:
        raise RuntimeError("No files to check!")

    def task(item):
        args = [clang_format_bin, '-style=file']
        if not fix or not fix_confirmed:
            args.extend(["--Werror", "--dry-run"])
        args.extend(["-i", item])
        # TODO: look at using RunCommand class
        result = subprocess.run(args,
                                text=True,
                                cwd=ctx.obj.sdk_dir,
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT,).stdout
        if "error" in result:
            return (item, result)
        else:
            return None

    from multiprocessing.pool import ThreadPool
    with ThreadPool(processes=threads) as pool:
        results = pool.imap_unordered(task, files_to_check)
        for result in iter_wrap(results, total=len(files_to_check)):
            if result is not None:
                errors.append(result)

    if len(errors) > 0:
        print(f"clang-format found {len(errors)} files with issues.")
        for item, out in errors:
            print(f"  {item}: {out}")
        sys.exit(1)
    else:
        print("clang-format found no issues.")


def import_module(click_context):
    click_context.lint_group.add_command(flake8)
    click_context.lint_group.add_command(mypy)
    click_context.lint_group.add_command(mypy_stubs)
    click_context.lint_group.add_command(clang_format)


def finalize(click_context):
    pass
