"""Smoketest: check that packet parsing hasn't changed unexpectedly.

See the hidden testing commands in python-sensor-tools for tools to generate
and update digest files.

"""
from os import path

import pytest

pytest.register_assert_rewrite('ouster.client._digest')
from ouster.client._digest import StreamDigest  # noqa

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


def test_parse_packet() -> None:
    """Test that parsing packets produces expected results."""
    f = path.join(DATA_DIR, "os-992011000121_digest.json")
    with open(f, 'r') as o1:
        digest = StreamDigest.from_json(o1.read())
        digest.check()
