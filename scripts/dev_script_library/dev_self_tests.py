import click
import sys
import os


additional_test_dirs = []


@click.command(name="self-tests")
@click.pass_context
@click.option('--threads',
              default=None,
              help='Number of threads to use.')
def self_tests(ctx, threads):
    """Run Self tests."""
    ctx.obj.build_libs.check_for_python_libs([("xdist", "pytest-xdist")])
    ctx.obj.build_options.process_args(threads=threads)

    run = ctx.obj.build_libs.RunCommand(tty=True)
    tests = [
        os.path.join(ctx.obj.sdk_dir, "scripts", "dev_script_tests")
    ] + additional_test_dirs
    failures = []
    for python_test_dir in tests:
        print(f"Running dev script self tests in: {python_test_dir}")
        args = [sys.executable, "-m", "pytest", '-n',
                str(ctx.obj.build_options.threads)]
        try:
            run.run_command(*args, cwd=python_test_dir)
        except Exception as e:
            print(f"Error running dev script self tests in: {python_test_dir}: {e}")
            failures.append((python_test_dir, e))
    if failures:
        details = "\n".join(
            f"- {path}: {type(err).__name__}: {err}" for path, err in failures)
        raise RuntimeError(f"One or more self-test targets failed:\n{details}")


def import_module(click_context):
    click_context.test_group.add_command(self_tests)


def finalize(click_context):
    pass
