#  type: ignore
"""Ouster command-line tool top-level module."""
import sys
import traceback
import collections
import click
import logging
from logging.handlers import RotatingFileHandler
import platform
import os
from importlib_metadata import distributions, version
from more_itertools import always_iterable
import inspect

from typing import Optional, List, Mapping

from ouster.client import ClientError, init_logger

from .pcap import pcap_group
from .sensor import sensor_group
from .util import util_group
from .osf import osf_group


this_package_name = 'ouster-sdk'
APP_NAME = 'ouster'
TRACEBACK = False
TRACEBACK_FLAG = '--traceback'


logger = logging.getLogger("cli-args-logger")


def log_packages():
    import pkg_resources
    package_list = sorted([f"{item.key}=={item.version}" for item in pkg_resources.working_set])
    logger.debug(str(package_list))


class SourceArgsException(Exception):
    def __init__(self, context_object):
        self._context_object = context_object
        super().__init__("Incorrect Args Supplied")

    def get_usage(self):
        return self._context_object.get_usage()

    def get_unexpected_args(self):
        return self._context_object.args


def get_packages_and_versions():
    result = set()
    for dist_name in set(packages_distributions().get('ouster', [])):
        if dist_name == this_package_name:
            continue
        result.add((dist_name, version(dist_name)))
    return result


def print_version(ctx, param, value):
    if not value or ctx.resilient_parsing:
        return
    click.echo(f"ouster-cli, version {version(this_package_name)}")

    packages_and_versions = get_packages_and_versions()
    if packages_and_versions:
        click.echo('\nOuster Python packages:')
        for dist_name, dist_version in packages_and_versions:
            click.echo(f"{dist_name}, {dist_version}")

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
        if sdk_log_level != "off":
            ctx.obj['SDK_LOG_LEVEL'] = sdk_log_level.lower()
            init_logger(ctx.obj['SDK_LOG_LEVEL'])


# pcap commands
cli.add_command(pcap_group)

# sensor commands
cli.add_command(sensor_group)

# util commands
cli.add_command(util_group)

# osf commands
cli.add_command(osf_group)


# from https://github.com/python/importlib_metadata, Apache 2.0 license
def packages_distributions() -> Mapping[str, List[str]]:
    pkg_to_dist = collections.defaultdict(list)
    for dist in distributions():
        for pkg in _top_level_declared(dist) or _top_level_inferred(dist):
            pkg_to_dist[pkg].append(dist.metadata['Name'])
    return dict(pkg_to_dist)


# from https://github.com/python/importlib_metadata, Apache 2.0 license
def _top_level_declared(dist):
    return (dist.read_text('top_level.txt') or '').split()


# from https://github.com/python/importlib_metadata, Apache 2.0 license
def _top_level_inferred(dist):
    opt_names = {
        f.parts[0] if len(f.parts) > 1 else inspect.getmodulename(f)
        for f in always_iterable(dist.files)
    }
    return list(filter(lambda name: name is not None and '.' not in name, opt_names))


def find_plugins(show_traceback: bool = False):
    import ouster.cli.plugins
    import pkgutil
    import importlib
    submodules = []
    load_fail = False
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
            load_fail = True
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
                    f"Run {os.path.basename(os.path.sys.argv[0])} {TRACEBACK_FLAG} for debug output.",
                    fg="yellow"
                ), err=True)
    if load_fail:
        log_packages()

    return submodules


def run(args=None) -> None:
    exit_code = None

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

    handler = None
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
        find_plugins(TRACEBACK_FLAG in sys.argv)
        exit_code = cli.main(args=args, standalone_mode=False)
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
        print(e)
        if TRACEBACK:
            print("-" * 70)
            traceback.print_exc(file=sys.stderr)
            print("-" * 70)
            print(f'Internal error: {e}', file=sys.stderr)
            exit_code = 4
            logger.debug(e)
    logger.debug("return code: " + str(exit_code))
    if exit_code != 0:
        logger.debug("error detected, listing packages")
        log_packages()
    sys.exit(exit_code)


if __name__ == '__main__':
    run()
