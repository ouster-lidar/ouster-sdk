#  type: ignore
"""Ouster command-line tool top-level module."""
import sys
import traceback
import collections
import click
from importlib_metadata import distributions, version
from more_itertools import always_iterable
import inspect

from typing import Optional, List, Mapping

from ouster.client import ClientError, init_logger

from .pcap import pcap_group
from .sensor import sensor_group
from .util import util_group

APP_NAME = 'ouster'
TRACEBACK = False


def print_version(ctx, param, value):
    if not value or ctx.resilient_parsing:
        return
    this_package_name = 'ouster-sdk'
    click.echo(f"ouster-cli, version {version(this_package_name)}")
    click.echo('\nOuster Python packages:')
    for dist_name in packages_distributions().get('ouster', []):
        if dist_name == this_package_name:
            continue
        click.echo(f"{dist_name}, {version(dist_name)}")
    click.echo('\nPlugins provided:')
    for plugin in find_plugins():
        click.echo(plugin.name)
    ctx.exit()


@click.group()
@click.option('-v', '--version', is_flag=True, callback=print_version,
              expose_value=False, is_eager=True)
@click.option("--traceback",
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


# pcap commands
cli.add_command(pcap_group)

# sensor commands
cli.add_command(sensor_group)

# util commands
cli.add_command(util_group)


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


def find_plugins():
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
        except Exception:
            click.echo(click.style(
                f"Failed to load plugin {module.name} due to an error.",
                fg="yellow"
            ))
            click.echo(click.style(
                traceback.format_exc(),
                fg="yellow"
            ))
    return submodules


def run(args=None) -> None:
    exit_code = None
    try:
        find_plugins()
        exit_code = cli.main(args=args, standalone_mode=False)
    except Exception as e:
        if isinstance(e, click.Abort):
            print('Aborted!')
            exit_code = 1
        elif isinstance(e, click.ClickException):
            e.show(file=sys.stderr)
            exit_code = e.exit_code
        elif isinstance(e, ClientError):
            print(f'Client error: {e}', file=sys.stderr)
            exit_code = 2
        else:
            if TRACEBACK:
                print("-" * 70)
                traceback.print_exc(file=sys.stderr)
                print("-" * 70)
            print(f'Internal error: {e}', file=sys.stderr)
            exit_code = 3

    sys.exit(exit_code)


if __name__ == '__main__':
    run()
