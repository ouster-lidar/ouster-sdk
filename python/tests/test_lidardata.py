"""Smoketest: check that packet parsing hasn't changed unexpectedly.

See the hidden testing commands in python-sensor-tools for tools to generate
and update digest files.

"""
from os import path

import pytest

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


def test_parse_and_batch_packets() -> None:
    """Test that parsing packets produces expected results."""
    digest_path = path.join(DATA_DIR, "os-992011000121_digest.json")
    bin_path = path.join(DATA_DIR, "os-992011000121_data.bin")

    with open(digest_path, 'r') as f:
        good = digest.StreamDigest.from_json(f.read())

    with open(bin_path, 'rb') as b:
        other = digest.StreamDigest.from_packets(
            digest.LidarBufStream(b, good.meta))

    good.check(other)
