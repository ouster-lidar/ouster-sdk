"""Miscellaneous utilities.
"""

import hashlib
import click
import json

from ouster.sdk.core import parse_and_validate_metadata

click_ro_file = click.Path(exists=True, dir_okay=False, readable=True)

_rethrow_exceptions = False


@click.group(name='util')
@click.pass_context
def util_group(ctx) -> None:
    """Miscellaneous utilities."""
    global _rethrow_exceptions
    _rethrow_exceptions = ctx.obj.get('TRACEBACK', False)


def get_system_info() -> dict:
    """Collect system information."""
    import platform
    from ouster.sdk import __version__

    try:
        import cpuinfo
    except ModuleNotFoundError:
        click.echo(
            "This command requires the py-cpuinfo package. Try running "
            "`pip3 install py-cpuinfo` first.")
        exit(1)

    res = {}
    res['platform'] = {
        attr: getattr(platform, attr)()
        for attr in [
            'machine', 'platform', 'processor', 'python_version',
            'python_build', 'python_compiler', 'release', 'system'
        ]
    }

    # use a hash of the hostname to crudely identify systems
    res['platform']['node'] = hashlib.md5(
        platform.node().encode()).hexdigest()[:7]

    res['cpuinfo'] = cpuinfo.get_cpu_info()

    res['packages'] = {
        'ouster-sdk': __version__
    }

    return res


@util_group.command()
def system_info() -> None:
    """Print system information as a json blob."""
    click.echo(json.dumps(get_system_info(), indent=4))


@util_group.command()
@click.argument('file', required=True, type=click.Path(exists=True))
def validate_metadata(file: str) -> None:
    """Validate a metadata json file."""
    with open(file, 'r') as f:
        _, issues = parse_and_validate_metadata(f.read())
        have_issues = False
        if len(issues.critical) > 0:
            have_issues = True
            click.echo("CRITICAL ISSUES:")
            for item in issues.critical:
                click.echo(item)
        if len(issues.warning) > 0:
            have_issues = True
            click.echo("WARNING ISSUES:")
            for item in issues.warning:
                click.echo(item)
        if len(issues.information) > 0:
            have_issues = True
            click.echo("INFORMATION ISSUES:")
            for item in issues.information:
                click.echo(item)

        if not have_issues:
            click.echo("No issues found")
