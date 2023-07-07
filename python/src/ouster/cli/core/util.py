#  type: ignore
"""Miscellaneous utilites.

TODO:
- bench util:
  + record length of pcap using new info API
  + figure out how to avoid using 2x sizeof pcap memory
    * deallocating packets while batching didn't seem to work ...
  + option to run multiple times / report avg + best times
"""

from io import FileIO
import hashlib
import click
import requests  # type: ignore
from more_itertools import consume
import glob
from itertools import product
import json
import os
import re
import time
from typing import Optional, Tuple
import tempfile
import zipfile

from ouster import client
from ouster.sdk.util import resolve_metadata

DEFAULT_SAMPLE_URL = 'https://data.ouster.io/sdk-samples/OS2/OS2_128_bridge_sample.zip'

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
    import pkg_resources  # type: ignore
    import platform

    try:
        import cpuinfo  # type: ignore
    except ModuleNotFoundError:
        click.echo(
            "This command requires the py-cpuinfo packgage. Try running "
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

    # TODO: try importlib_metadata again?
    sdk = pkg_resources.get_distribution('ouster-sdk')

    res['packages'] = {
        sdk.project_name: sdk.version
    }

    try:
        deps = [pkg_resources.get_distribution(r) for r in sdk.requires()]
        res['packages'].update({d.project_name: d.version for d in deps})
    except Exception:
        if _rethrow_exceptions:
            raise

    return res


def download_sample_data(url: str,
                         dest_dir: str,
                         chunk_size=512 * 1024) -> Tuple[str, str]:
    """Download sample data file and extract into dest_dir.

    URL is expected to point to a .zip containing exactly one pcap file and
    one json file.
    """

    urlhash = hashlib.md5(url.encode()).hexdigest()
    url_dir = os.path.join(dest_dir, urlhash)
    try:

        if os.path.exists(url_dir):
            pcap_path = glob.glob(url_dir + "/**/*.pcap", recursive=True)[0]
            json_path = glob.glob(url_dir + "/**/*.json", recursive=True)[0]
            return pcap_path, json_path

        with tempfile.TemporaryFile(suffix='.zip') as tmpfile:
            click.echo(f"Downloading {url}...")
            res = requests.get(url, stream=True)
            res.raise_for_status()

            n_chunks = int(res.headers['Content-Length']) // chunk_size
            chunks = res.iter_content(chunk_size)
            with click.progressbar(chunks, length=n_chunks) as progress:
                for chunk in progress:
                    tmpfile.write(chunk)

            click.echo("Extracting contents...")
            zip = zipfile.ZipFile(tmpfile)
            contents = zip.namelist()
            pcap_mem = next(p for p in contents if re.match(r".*\.pcap", p))
            json_mem = next(p for p in contents if re.match(r".*\.json", p))
            pcap_path = zip.extract(pcap_mem, path=url_dir)
            json_path = zip.extract(json_mem, path=url_dir)
            return pcap_path, json_path
    except Exception as e:
        raise RuntimeError("Failed fetch sample data") from e


def md5file(path: str) -> str:
    """Calculate md5sum of a file."""
    bufsize = 256 * 1024
    md5 = hashlib.md5()
    buf = memoryview(bytearray(bufsize))
    with FileIO(path, 'rb') as f:
        for n in iter(lambda: f.readinto(buf), 0):
            md5.update(buf[:n])
    return md5.hexdigest()


ROS_MODULES_ERROR_MSG = """
Error: {err_msg}

Please verify that ROS Python modules are available.

The best option is to try to install unofficial rospy packages that work
with python3.7,3.8 on Ubuntu 18.04/20.04 and Debian 10 without ROS:

    pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag tf2_ros

NOTE: If during the attempt to run the above command you get an error:

    EnvironmentError: 404 Client Error: Not Found for url: https://pypi.org/simple/rospy/

Please check installed `pip` version (20.0+ works well with extra indexes), and
if needed upgrade `pip` with (in a sourced venv):

    pip install pip -U

Some users have even more packages missing so they may need to install additionally:

    pip install PyYAML pycryptodome pycryptodomex

"""


def import_rosbag_modules(raise_on_fail: bool = False):
    try:
        import rosbag  # noqa: F401
        import rospy   # noqa: F401
        import genpy   # noqa: F401
    except ImportError as err:
        if raise_on_fail:
            raise ModuleNotFoundError(
                ROS_MODULES_ERROR_MSG.format(err_msg=str(err)))
        return False
    return True


@util_group.command()
def system_info() -> None:
    """Print system information as a json blob."""
    click.echo(json.dumps(get_system_info(), indent=4))


@util_group.command()
@click.argument('file', required=False, type=click.Path(exists=True))
@click.option('-f',
              '--meta',
              required=False,
              type=click.Path(exists=True, dir_okay=False, readable=True))
@click.option('-u', '--url', required=False)
def benchmark(file: str, meta: Optional[str], url: Optional[str]) -> None:
    """Run a quick set of benchmarks and record results.

    Also gathers and records system information. Data and reports are written to
    $PWD/ouster-bench and re-used if possible. WARNING: will currently use
    memory >2x the size of the input data.
    """

    try:
        import ouster.pcap as pcap
    except ImportError:
        raise click.ClickException("Please verify that libpcap is installed")

    workdir = os.path.join(os.getcwd(), "ouster-bench")
    if not os.path.exists(workdir):
        os.mkdir(workdir)
    os.chdir(workdir)
    click.echo(f"Working directory: {workdir}")

    if file is None:
        file, meta = download_sample_data(url or DEFAULT_SAMPLE_URL, workdir)
    elif url is not None:
        raise click.ClickException("Cannot specify both a pcap and URL")

    meta = resolve_metadata(file, meta)
    if not os.path.exists(file) or not os.path.exists(meta):
        raise click.ClickException("pcap file doesn't exist")

    with open(meta) as f:
        raw_metadata = f.read()
        info = client.SensorInfo(raw_metadata)
    packets_per_frame = info.format.columns_per_frame / info.format.columns_per_packet

    report: dict = {'pcap': {}, 'sys_info': {}, 'times': {}}

    # gather system information
    click.echo("Gathering system info...")
    sys_info = get_system_info()
    click.echo(
        f"  cpu: {sys_info.get('cpuinfo',{}).get('brand_raw', 'UNKNOWN')}")
    click.echo(f"  platform: {sys_info['platform']['platform']}")
    click.echo(f"  python: {sys_info['platform']['python_version']}")
    click.echo(f"  ouster-sdk: {sys_info['packages']['ouster-sdk']}")

    report['sys_info'] = sys_info

    click.echo("Gathering pcap info...")
    hash = md5file(file)
    click.echo(f"  filename: {os.path.basename(file)}")
    click.echo(f"  md5: {hash}")
    click.echo(f"  size: {os.path.getsize(file) / 2**30:.3f} GB")
    click.echo(f"  mode: {info.mode}")
    click.echo(f"  prod line: {info.prod_line}")
    click.echo(f"  col window: {info.format.column_window}")

    report['pcap']['filename'] = os.path.basename(file)
    report['pcap']['metadata'] = json.loads(raw_metadata)
    report['pcap']['md5'] = hash

    click.echo(f'{"Timing reading packets...":<32}', nl=False)
    start = time.time()
    packets = list(pcap.Pcap(file, info))
    dur = time.time() - start

    # report both total and per-frame times
    n_packets = len(packets)
    n_frames = n_packets / packets_per_frame
    report['pcap']['n_packets'] = n_packets
    report['pcap']['n_frames'] = n_frames

    def report_dur(key: str, dur: float):
        click.echo(f"{dur:.2f}s ({dur / n_frames * 1e3:.2f}ms / frame)")
        report['times'][key] = {}
        report['times'][key]['total'] = dur
        report['times'][key]['per_frame'] = dur / n_frames

    report_dur('read', dur)

    click.echo(f'{"Timing writing packets...":<32}', nl=False)
    start = time.time()
    test_file = 'write-test.pcap'
    pcap.record(packets, test_file)
    dur = time.time() - start
    os.remove(test_file)

    report_dur('write', dur)

    click.echo(f'{"Timing batching scans...":<32}', nl=False)
    start = time.time()
    scans = list(client.Scans(client.Packets(packets, info)))
    dur = time.time() - start
    del packets

    report_dur('batch', dur)

    click.echo(f'{"Timing destaggering...":<32}', nl=False)
    start = time.time()
    fields = scans[0].fields
    consume(
        client.destagger(info, s.field(field))
        for field, s in product(fields, scans))
    dur = time.time() - start

    report_dur('destagger', dur)
    report['times']['destagger'] = dur

    click.echo(f'{"Timing cartesian conversion...":<32}', nl=False)
    xyzlut = client.XYZLut(info)
    start = time.time()
    consume(xyzlut(s) for s in scans)
    dur = time.time() - start

    report_dur('cartesian', dur)
    report['times']['cartesian'] = dur

    # name report based on file hash, hostname, python version, and sdk version
    report_file = (f"report-{hash[:7]}"
                   f"-{sys_info['platform']['node']}"
                   f"-py{sys_info['platform']['python_version']}"
                   f"-sdk{sys_info['packages']['ouster-sdk']}.json")

    click.echo(f"Total packets: {n_packets}")
    click.echo(f"Total frames: {int(n_frames)}")
    click.echo(f"Writing {report_file}... ")
    with open(report_file, 'w') as f:
        f.write(json.dumps(report, indent=4))


@util_group.command()
@click.option('-o', 'output_path', required=False, help='Name of output file')
@click.argument('meta', required=True, type=click.Path(exists=True))
def convert_metadata_to_legacy(meta: str, output_path: Optional[str]) -> None:
    """Convert nonlegacy metadata to legacy format

    The supplied metadata should be of nonlegacy form. Output will print to screen.
    Legacy metadata will error out by complaining it is an invalid non-legacy format.

    """

    with open(meta) as json:
        output = client.convert_to_legacy(json.read())
        if output_path is None:
            click.echo(output)
        else:
            click.echo(f"Reading metadata from: {meta} and outputting converted legacy metadata to: {output_path}")
            with open(output_path, "w") as outfile:
                outfile.write(output)
