"""Ouster command-line tool top-level module."""
from itertools import dropwhile, islice

import click

import ouster.client as oscli
from ouster.client import _bufstream
from ouster.client._digest import StreamDigest


@click.group()
def run():
    pass


@run.group(hidden=True)
def testing():
    pass


@testing.command()
@click.argument('digest_file')
def dump_mids(digest_file: str) -> None:
    """Print the measurement ids from a bufstream packet capture."""
    with open(digest_file, 'r') as f:
        digest = StreamDigest.from_json(f.read())

    pf = oscli.get_format(digest.meta)

    with open(digest.file, 'rb') as fb:
        ids = [pf.col_measurement_id(0, buf) for buf in _bufstream.read(fb)]

    print(ids)


@testing.command()
@click.argument('hostname')
@click.option('--npackets', default=64, help="number of packets to capture")
def capture(hostname: str, npackets: int) -> None:
    """Capture packets and and compute a digest for use in testing.

    Query `hostname` for metadata and listen on port 7512 for incoming lidar
    packets. Collect `npackets` and write hashes of the parsed data out to a
    digest suitable for regression tests.

    """
    tmpfile = f"{hostname}_data.bin"
    descrfile = f"{hostname}_digest.json"

    print(f"Connecting to {hostname}")

    cli = oscli.init_client(hostname, 7512, 7513)

    if cli is None:
        print("Failed to init")
        exit(1)

    print(f"Writing data to {tmpfile} ...")

    si = oscli.parse_metadata(oscli.get_metadata(cli))
    pf = oscli.get_format(si)

    # read lidar packets starting at the beginning of the next frame
    frame_start = dropwhile(lambda b: pf.col_measurement_id(0, b) != 0,
                            oscli.lidar_packets(pf, cli))

    # write n packets to a bufstream on disk
    with open(tmpfile, 'wb') as fo:
        _bufstream.write(fo, islice(frame_start, npackets))

    print("Hashing results ...")

    with open(tmpfile, 'rb') as fi:
        descr = StreamDigest.from_bufstream(tmpfile, si, fi)

    print(f"Writing description to {descrfile} ...")

    with open(descrfile, 'w') as fj:
        fj.write(descr.to_json())

    print("Done!")
