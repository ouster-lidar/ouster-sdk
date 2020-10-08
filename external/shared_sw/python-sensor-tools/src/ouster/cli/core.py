"""Ouster command-line tool top-level module."""
from itertools import dropwhile, islice

import click

import ouster.client as client
from ouster.client import _bufstream
from ouster.client._digest import StreamDigest
from ouster.client._sensor import LidarScan


@click.group()
def run():
    pass


@run.group(hidden=True)
def testing():
    pass


@testing.command()
@click.option('--hostname', help="hostname to connect to")
@click.option('--lidar-port', default=7512, help="lidar port")
@click.option('--imu-port', default=7513, help="imu port")
def read_scans(hostname: str, lidar_port: int, imu_port: int) -> None:
    """Read packets from supplied ports and batch into scans.

    This is just a debug command to run the scan-batching code.
    """
    cli = client.init_client(hostname, lidar_port, imu_port)

    if cli is None:
        print("Failed to init")
        exit(1)

    print(f"Listening for packets on ports {lidar_port} and {imu_port}")

    nscans = 0

    for m in client.scans(cli):
        if isinstance(m, LidarScan):
            nscans += 1
            print(f"Scans received: {nscans}")

    print("Exiting...")


@testing.command()
@click.argument('digest_file')
def check_digest(digest_file: str) -> None:
    with open(digest_file, 'r') as f:
        digest = StreamDigest.from_json(f.read())
        digest.check()
    print("Ok!")


@testing.command()
@click.argument('digest_file')
def update_digest(digest_file: str) -> None:
    """Update the specified digest file with new hashes."""
    with open(digest_file, 'r') as f:
        digest = StreamDigest.from_json(f.read())

    with open(digest.file, 'rb') as fi:
        new_digest = StreamDigest.from_bufstream(digest.file, digest.meta, fi)

    with open(digest_file, 'w') as fj:
        fj.write(new_digest.to_json())


@testing.command()
@click.argument('hostname')
@click.option('--npackets', default=64, help="number of packets to capture")
def capture(hostname: str, npackets: int) -> None:
    """Capture packets and and compute a digest for use in testing.

    Query `hostname` for metadata and listen on port 7512 for incoming lidar
    packets. Collect `npackets` and write hashes of the parsed data out to a
    digest suitable for regression tests.

    """
    datafile = f"{hostname}_data.bin"
    dgstfile = f"{hostname}_digest.json"

    print(f"Connecting to {hostname}")

    cli = client.init_client(hostname, 7512, 7513)

    if cli is None:
        print("Failed to init")
        exit(1)

    print(f"Writing data to {datafile} ...")

    si = client.parse_metadata(client.get_metadata(cli))
    pf = client.get_format(si)

    lidar_packets = map(
        lambda p: p._data,
        dropwhile(lambda p: not isinstance(p, client.LidarPacket),
                  client.packets(pf, cli)))

    # read lidar packets starting at the beginning of the next frame
    frame_start = dropwhile(lambda b: pf.col_measurement_id(0, b) != 0,
                            lidar_packets)

    # write n packets to a bufstream on disk
    with open(datafile, 'wb') as fo:
        _bufstream.write(fo, islice(frame_start, npackets))

    print("Hashing results ...")

    with open(datafile, 'rb') as fi:
        digest = StreamDigest.from_bufstream(datafile, si, fi)

    print(f"Writing description to {dgstfile} ...")

    with open(dgstfile, 'w') as fj:
        fj.write(digest.to_json())

    print("Done!")
