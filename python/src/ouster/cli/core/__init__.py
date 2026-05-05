"""Ouster command-line tool top-level module."""
import sys
import traceback
import click
import logging
from logging.handlers import RotatingFileHandler
import platform
import os
from typing import Optional

from ouster.sdk import __version__
from ouster.sdk.core import init_logger
from ouster.sdk.sensor import ClientError

from .util import util_group

from threadpoolctl import threadpool_limits

APP_NAME = 'ouster'
TRACEBACK = False
TRACEBACK_FLAG = '--traceback'

logger = logging.getLogger("cli-args-logger")


def log_packages():
    """Log installed packages for debugging import/dependency issues."""
    try:
        from importlib.metadata import distributions
        package_list = sorted([f"{dist.metadata['Name']}=={dist.version}" for dist in distributions()])
        logger.debug(str(package_list))
    except Exception as e:
        logger.debug(f"Failed to log packages: {e}")


def is_package_related_error(exception: Exception) -> bool:
    """Check if an exception is likely related to package/dependency issues."""
    # Direct import/module errors
    if isinstance(exception, (ImportError, ModuleNotFoundError, AttributeError)):
        return True

    # Check if error message suggests dependency issues
    error_msg = str(exception).lower()
    package_keywords = ['version', 'compatibility', 'incompatible', 'requires',
                       'dependency', 'not installed', 'missing']
    return any(keyword in error_msg for keyword in package_keywords)


class SourceArgsException(Exception):
    def __init__(self, context_object):
        self._context_object = context_object
        super().__init__("Incorrect Args Supplied")

    def get_usage(self):
        return self._context_object.get_usage()

    def get_unexpected_args(self):
        return self._context_object.args


def print_version(ctx, param, value):
    if not value or ctx.resilient_parsing:
        return
    click.echo(f"ouster-cli, version {__version__}")

    click.echo('\nPlugins provided:')
    for plugin in find_plugins():
        click.echo(plugin.name)
    ctx.exit()


@click.group()
@click.option('-v', '--version', is_flag=True, callback=print_version,
              expose_value=False, is_eager=True)
@click.option(TRACEBACK_FLAG,
              "trace",
              is_flag=True,
              default=False,
              help="Turn on the tracebacks on errors")
@click.option(
    '--sdk-log-level',
    type=click.Choice(
        ["trace", "debug", "info", "warning", "error", "critical", "off"],
        case_sensitive=False),
    help="Set Ouster SDK logging level")
@click.pass_context
def cli(ctx, trace: bool, sdk_log_level: Optional[str]) -> None:
    """Experimental sensor utilities CLI."""
    global TRACEBACK
    # we keep TRACEBACK global because it's used in the main click
    # runner below on the exception hanling path, where context is not
    # available
    TRACEBACK = trace

    ctx.ensure_object(dict)
    ctx.obj['TRACEBACK'] = TRACEBACK
    if sdk_log_level:
        ctx.obj['SDK_LOG_LEVEL'] = sdk_log_level.lower()
        init_logger(ctx.obj['SDK_LOG_LEVEL'])


# util commands
cli.add_command(util_group)


def find_plugins(show_traceback: bool = False):
    import ouster.cli.plugins
    import pkgutil
    import importlib
    submodules = []
    for module in pkgutil.iter_modules(
        ouster.cli.plugins.__path__, ouster.cli.plugins.__name__ + "."
    ):
        try:
            if module.ispkg:
                subpkg = importlib.import_module(module.name)
                for submodule in pkgutil.iter_modules(subpkg.__path__, subpkg.__name__ + "."):
                    submodules.append(submodule)
                    importlib.import_module(submodule.name)
            else:
                submodules.append(module)
                importlib.import_module(module.name)
        except Exception as e:
            logger.debug(f"Failed to load plugin {module.name} due to an error.")
            click.echo(click.style(
                f"Failed to load plugin {module.name} due to an error: {e}",
                fg="yellow"
            ), err=True)
            if show_traceback:
                click.echo(click.style(
                    traceback.format_exc(),
                    fg="yellow"
                ), err=True)
            else:
                click.echo(click.style(
                    f"Run {os.path.basename(sys.argv[0])} {TRACEBACK_FLAG} for debug output.",
                    fg="yellow"
                ), err=True)

            # Log packages only for import/dependency errors
            if is_package_related_error(e):
                logger.debug("Plugin load failed due to package-related error, listing packages")
                log_packages()

    return submodules


def run(args=None) -> None:
    exit_code = None

    # Set openblas and friends to only use 1 thread to avoid threadpool busywaiting issues
    # The busywaiting in openblas was taking a viz from less than a half a core to 3
    # The small matrices/vectors we are using with BLAS are generally not conducive to
    # multithreading anyways
    threadpool_limits(limits=1, user_api='blas')

    if platform.system() == "Windows":
        client_log_dir = os.getenv("LOCALAPPDATA")

        if not client_log_dir:
            client_log_dir = os.getenv("TMP")
            if not client_log_dir:
                client_log_dir = "C:"
        client_log_dir = os.path.join(client_log_dir, "ouster-cli")
        client_log_location = os.path.join(client_log_dir, "cli.log")
    else:
        client_log_dir = os.path.join(os.path.expanduser("~"), ".ouster-cli")
        client_log_location = os.path.join(client_log_dir, "cli.log")

    handler: Optional[logging.Handler] = None
    if not os.path.exists(client_log_dir):
        try:
            os.makedirs(client_log_dir)
        except Exception as e:
            click.echo(f"Can't enable logging: {e}")
            handler = logging.NullHandler()

    if not os.access(client_log_dir, os.W_OK):
        click.echo("Can't enable logging")
        handler = logging.NullHandler()

    if not handler:
        handler = RotatingFileHandler(client_log_location, maxBytes=(5 * 1024 * 1024), backupCount=10)
        handler.setFormatter(logging.Formatter('%(asctime)s %(message)s'))
    logger.setLevel(logging.DEBUG)
    logger.addHandler(handler)

    # if this is not set, it will continue on to display the log to console
    logger.propagate = False

    logger.debug(platform.python_version() + " : " + " ".join(sys.argv))

    try:
        exit_code = 0
        find_plugins(TRACEBACK_FLAG in sys.argv)
        cli.main(args=args, standalone_mode=False)
    except click.Abort:
        print('Aborted!')
        logger.debug('Aborted!')
        exit_code = 1
    except click.ClickException as e:
        e.show(file=sys.stderr)
        exit_code = e.exit_code
        logger.debug(e)
    except ClientError as e:
        print(f'Client error: {e}', file=sys.stderr)
        logger.debug(e)
        exit_code = 2
    except SourceArgsException as e:
        print(e.get_usage())
        print("")
        print(f"Error: Got unexpected extra arguments ({' '.join(e.get_unexpected_args())})")
        exit_code = 3
        logger.debug(e.get_unexpected_args())
    except Exception as e:
        click.secho(f"ERROR: {e}", fg="red")
        if TRACEBACK:
            print("-" * 70)
            traceback.print_exc(file=sys.stderr)
            print("-" * 70)
            print(f'Internal error: {e}', file=sys.stderr)
            exit_code = 4
            logger.debug(e)
        else:
            print("Add the --traceback option after ouster-cli for more information.")

        # Log packages for unexpected exceptions that might be package-related
        if is_package_related_error(e):
            logger.debug("Unexpected exception with package indicators, listing packages")
            log_packages()
        else:
            logger.debug("Unexpected exception (probably not package-related)")
            logger.debug(f"Exception type: {type(e).__name__}, message: {str(e)}")

    logger.debug("return code: " + str(exit_code))
    sys.exit(exit_code)


if __name__ == '__main__':
    run()
