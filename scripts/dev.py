import glob
import os
import sys
import subprocess


def bootstrap_requirements():
    """Bootstrap pip requirements."""
    requirements_file = os.path.join(os.path.dirname(__file__),
                                     "requirements.txt")
    if not os.path.exists(requirements_file):
        return
    # Check if we need to install requirements
    try:
        import click  # noqa: F401
        dev_script_library_dir = os.path.abspath(os.path.join(
            os.path.dirname(__file__),
            "dev_script_library"))
        sys.path.append(dev_script_library_dir)
        import build_libs  # noqa: E402, F401
        return
    except ImportError:
        pass
    print("Installing required dependencies...")
    try:
        subprocess.check_call([
            sys.executable, "-m", "pip", "install", "-q", "-r",
            requirements_file
        ])
        print("Dependencies installed successfully!")
    except subprocess.CalledProcessError as e:
        print(f"Failed to install requirements: {e}", file=sys.stderr)
        print("Please run manually: pip3 install -r ./scripts/requirements.txt",
              file=sys.stderr)
        sys.exit(1)
    except FileNotFoundError:
        print("pip not found. Please install pip and try again.",
              file=sys.stderr)
        sys.exit(1)


bootstrap_requirements()
import click # noqa

dev_script_library_dir = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    "dev_script_library"))
sys.path.append(dev_script_library_dir)
import build_libs  # noqa: E402
import context  # noqa: E402
click_context = context.ClickContext(build_libs)


@click.group(no_args_is_help=True,)
@click.pass_context
def cli(ctx):
    ctx.obj = click_context


# initialize common groups
@cli.group()
@click.pass_context
def build(ctx):
    """Code building commands."""
    pass


@cli.group()
@click.pass_context
def test(ctx):
    """Code testing commands."""
    pass


@cli.group()
@click.pass_context
def utils(ctx):
    """Various utility functions."""
    pass


@cli.group()
@click.pass_context
def cleanup(ctx):
    """Various cleanup functions."""
    pass


@cli.group()
@click.pass_context
def lint(ctx):
    """Linting commands for the project."""
    pass


@cli.group()
@click.pass_context
def docs(ctx):
    """Documentation commands for the project."""
    pass


class OusterSDKHelpFormatter(click.HelpFormatter):
    auto_envvar_prefix = "OSDK_DEV_CLI"

    def __init__(self, **kwargs):
        super().__init__(self, **kwargs)
        self.indent_increment = 4

    def write_dl(self, rows):
        """Write a description list."""
        if not rows:
            return
        for key, value in rows:
            is_option = key.startswith("--")
            name_for_env = key.split(" ")[0]
            name_for_env = name_for_env.replace("--", "")
            name_for_env = name_for_env.replace("-", "_").upper()
            self.write_text(f"{key}: ")
            self.indent()
            self.write_text(f"{value}")
            if is_option:
                self.indent()
                self.write_text(f"(ENV VAR: {self.auto_envvar_prefix}_{name_for_env})")
                self.dedent()
            self.dedent()


if __name__ == '__main__':
    click.Context.formatter_class = OusterSDKHelpFormatter
    click_context.top_level_group = cli
    click_context.build_group = build
    click_context.test_group = test
    click_context.utils_group = utils
    click_context.lint_group = lint
    click_context.docs_group = docs
    click_context.cleanup_group = cleanup

    for item in glob.glob(os.path.join(click_context.dev_dir,
                                       dev_script_library_dir,
                                       'dev_*.py')):
        click_context.add_module(item)
    for item in glob.glob(os.path.join(click_context.sdkx_dev_dir,
                                       'dev_*.py')):
        click_context.add_module(item)

    click_context.finalize()
    try:
        cli(auto_envvar_prefix=OusterSDKHelpFormatter.auto_envvar_prefix)
    except Exception as e:
        click.echo(f"Error: {e}", err=True)
        sys.exit(1)
