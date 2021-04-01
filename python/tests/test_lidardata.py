"""Tests for lidar data parsing.

Checks that the output of parsing hasn't changed unexpectedly.
"""
from os import path
import pickle

import numpy as np
import pytest

from ouster import client

pytest.register_assert_rewrite('ouster.client._digest')
import ouster.client._digest as digest  # noqa


DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


@pytest.fixture
def pf():
    return client.PacketFormat(
        client.SensorInfo.from_default(client.LidarMode.MODE_1024x10))


@pytest.fixture
def data():
    with open(path.join(DATA_DIR, "lidardata.pickle"), 'rb') as f:
        return pickle.load(f)


@pytest.fixture
def packet(pf, data):
    return client.LidarPacket(data['data'], pf)


def test_simple_px_range(packet, data):
    assert np.array_equal(packet.view(client.ChanField.RANGE),
                          np.array(data['px_range_data']))


def test_simple_px_reflectivity(packet, data):
    assert np.array_equal(packet.view(client.ChanField.REFLECTIVITY),
                          np.array(data['px_reflectivity_data']))


def test_simple_px_signal(packet, data):
    assert np.array_equal(packet.view(client.ChanField.INTENSITY),
                          np.array(data['px_signal_data']))


def test_simple_px_noise(packet, data):
    assert np.array_equal(packet.view(client.ChanField.AMBIENT),
                          np.array(data['px_noise_data']))


def test_simple_px_timestamp(packet, data):
    assert np.array_equal(packet.view(client.ColHeader.TIMESTAMP),
                          np.array(data['timestamp']))


def test_simple_px_measurement_id(packet, data):
    assert np.array_equal(packet.view(client.ColHeader.MEASUREMENT_ID),
                          np.array(data['measurement_id']))


def test_simple_px_frame_id(packet, data):
    assert np.array_equal(packet.view(client.ColHeader.FRAME_ID),
                          np.array(data['col_frame_id']))


def test_simple_px_valid(packet, pf):
    array = packet.view(client.ColHeader.VALID)
    assert len(array) == pf.columns_per_packet
    for item in array:
        assert item == 0xFFFFFFFF


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
