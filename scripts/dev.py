import importlib
import glob
import os
import shlex
import site
import sys
import subprocess


def _is_command_utils_version_sdk(argv):
    """True when dev.py utils version-sdk."""
    for i in range(1, len(argv) - 1):
        if argv[i] == "utils" and argv[i + 1] == "version-sdk":
            return True
    return False


def _bootstrap_version_sdk_requirements():
    """Ensure click, GitPython, flufl.lock via build_libs; optionally pip install."""
    dev_script_library_dir = os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        "dev_script_library"))
    if dev_script_library_dir not in sys.path:
        sys.path.insert(0, dev_script_library_dir)

    version_sdk_python_libs = [
        "click",
        ("git", "GitPython"),
        ("flufl.lock", "flufl.lock"),
    ]

    try:
        import build_libs as bl  # noqa: F401
    except ImportError:
        bl = None

    install_var = os.environ.get("INSTALL_OUSTER_SDK_VERSION_DEPS")
    if install_var is not None and install_var.strip() == "0":
        if bl is None:
            print(
                "Cannot import build_libs (e.g. flufl.lock). "
                "Install click, GitPython, and flufl.lock, "
                "or unset INSTALL_OUSTER_SDK_VERSION_DEPS.",
                file=sys.stderr,
            )
            sys.exit(1)
        bl.check_for_python_libs(version_sdk_python_libs,
                                 fail_on_missing=True)
        return

    if bl is not None and bl.check_for_python_libs(
            version_sdk_python_libs, fail_on_missing=False):
        return

    print("Installing version-sdk Python dependencies...")
    try:
        install_cmd = [
            sys.executable, "-m", "pip", "install", "-q",
            "click>=8.1.3,<8.1.8", "GitPython", "flufl.lock"
        ]
        subprocess.check_call(install_cmd)
        local_bin = os.path.join(os.path.expanduser("~"), ".local", "bin")
        if os.path.exists(local_bin) and local_bin not in os.environ["PATH"]:
            os.environ["PATH"] = local_bin + os.pathsep + os.environ["PATH"]
        importlib.reload(site)
    except subprocess.CalledProcessError as e:
        print(
            f"Failed to install version-sdk dependencies: {e}",
            file=sys.stderr,
        )
        print(
            f"Please run manually: {shlex.join(install_cmd)}",
            file=sys.stderr,
        )
        sys.exit(1)
    except FileNotFoundError:
        print(
            "pip not found. Please install pip and try again.",
            file=sys.stderr,
        )
        sys.exit(1)

    importlib.invalidate_caches()
    site.main()

    try:
        import build_libs as bl  # noqa: F401
    except ImportError:
        print(
            "build_libs still unavailable after pip install.",
            file=sys.stderr,
        )
        sys.exit(1)

    bl.check_for_python_libs(version_sdk_python_libs, fail_on_missing=True)


def bootstrap_requirements():
    """Bootstrap pip requirements, installing them if not already present."""
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

    in_venv = sys.prefix != sys.base_prefix
    if not in_venv:
        print("ERROR: Required dependencies are missing and no virtual environment "
              "is active.", file=sys.stderr)
        print(f"Please install them manually: pip install -r {requirements_file}",
              file=sys.stderr)
        sys.exit(1)

    print(f"Installing required dependencies from {requirements_file}...")

    # Prefer uv for faster, reproducible installs; fall back to pip.
    def _try_install(installer_args):
        try:
            subprocess.check_call(
                [sys.executable] + installer_args + ["-r", requirements_file])
            return True
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

    uv_available = subprocess.call(
        [sys.executable, "-m", "uv", "--version"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0

    if uv_available:
        success = _try_install(["-m", "uv", "pip", "install", "-q"])
    else:
        # Upgrade pip first — old versions (e.g. 19.x) lack PEP 517 support.
        subprocess.call([sys.executable, "-m", "pip", "install", "-q",
                         "--upgrade", "pip"])
        success = _try_install(["-m", "pip", "install", "-q"])

    if success:
        print("Dependencies installed successfully.")
    else:
        print("Failed to install dependencies automatically.", file=sys.stderr)
        print(f"Please run manually: pip install -r {requirements_file}",
              file=sys.stderr)
        sys.exit(1)


def _is_completion_mode():
    """Return True when the shell is requesting tab-completion source/results.

    Click signals completion mode via ``_{PROG}_COMPLETE`` environment
    variables (e.g. ``_DEV_PY_COMPLETE=bash_source``).  We check all
    plausible names so the early-exit works regardless of whether the
    user invokes the script as ``dev.py``, ``dev.sh``, or ``dev``.
    """
    for prog in ("dev_py", "dev_sh", "dev"):
        if os.environ.get(f"_{prog.upper()}_COMPLETE"):
            return True
    return False


if _is_command_utils_version_sdk(sys.argv):
    _bootstrap_version_sdk_requirements()
else:
    bootstrap_requirements()
import click # noqa

dev_script_library_dir = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    "dev_script_library"))
sys.path.append(dev_script_library_dir)

if _is_completion_mode():
    # Lightweight stub used only during shell completion.  Only the path
    # attributes needed to locate modules are set explicitly; everything else
    # (group handles, plugin-specific attributes like wasm_src_dir, etc.)
    # is returned as None on first read and stored on assignment — exactly
    # like MagicMock, but without the test-library dependency.
    # finalize() is a no-op because several modules read files or probe build
    # state there, none of which is needed just to enumerate commands.
    class _CompletionContext:
        def __init__(self):
            self.submodules = {}
            self.dev_dir = dev_script_library_dir
            self.sdk_dir = os.path.abspath(
                os.path.join(os.path.dirname(__file__), ".."))
            sdk_private_dir = os.path.abspath(
                os.path.join(self.sdk_dir, "_private"))
            self.sdkx_dir = os.path.abspath(
                os.path.join(self.sdk_dir, "sdk-extensions"))
            self.sdkx_dev_dir = os.path.abspath(
                os.path.join(self.sdkx_dir, "scripts", "dev_script_library"))
            self.sdkx_private_dev_dir = os.path.abspath(
                os.path.join(sdk_private_dir, "scripts", "dev_script_library"))

        def __getattr__(self, name):
            # Return a no-op stub so that callers like
            # ``click_context.some_group.add_command(...)`` don't crash if a
            # group attribute has not been set yet (e.g. in tests or when a
            # plugin is loaded before the main block assigns the groups).
            class _Noop:
                def __getattr__(self, _):
                    return lambda *a, **kw: None
            return _Noop()

        def add_module(self, module_file):
            import importlib
            module_dir = os.path.dirname(module_file)
            module_name = os.path.splitext(os.path.basename(module_file))[0]
            if module_dir not in sys.path:
                sys.path.append(module_dir)
            module = importlib.import_module(module_name)
            self.submodules[module_name] = module
            module.import_module(self)

        def finalize(self):
            pass

    click_context = _CompletionContext()
    build_libs = None
else:
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


AUTO_ENVVAR_PREFIX = "OSDK_DEV_CLI"


class OusterSDKHelpFormatter(click.HelpFormatter):
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
                self.write_text(f"(ENV VAR: {AUTO_ENVVAR_PREFIX}_{name_for_env})")
                self.dedent()
            self.dedent()


if __name__ == '__main__':
    click.Context.formatter_class = OusterSDKHelpFormatter
    click_context.top_level_group = cli
    click_context.build_group = build
    click_context.test_group = test
    click_context.utils_group = utils
    click_context.lint_group = lint
    click_context.cleanup_group = cleanup

    for item in glob.glob(os.path.join(dev_script_library_dir, 'dev_*.py')):
        click_context.add_module(item)
    for item in glob.glob(os.path.join(click_context.sdkx_dev_dir,
                                       'dev_*.py')):
        click_context.add_module(item)
    for item in glob.glob(os.path.join(click_context.sdkx_private_dev_dir,
                                       'dev_*.py')):
        click_context.add_module(item)

    click_context.finalize()

    if _is_completion_mode():
        # Let Click handle completion and exit — no formatter, no heavy init.
        # DEV_PROG_NAME is set by dev.sh when the user invokes completion via
        # a name other than dev.py (e.g. "dev" or "dev.sh"), so the generated
        # script references the right executable.
        prog_name = os.environ.get("DEV_PROG_NAME")
        cli.main(standalone_mode=True, prog_name=prog_name)
    else:
        click.Context.formatter_class = OusterSDKHelpFormatter
        try:
            cli(auto_envvar_prefix=AUTO_ENVVAR_PREFIX)
        except Exception as e:
            click.echo(f"Error: {e}", err=True)
            sys.exit(1)
