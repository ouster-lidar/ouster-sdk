from os import path
import numpy as np
from ouster.sdk.client import SensorInfo
from ouster.sdk.util import resolve_extrinsics
from ouster.sdk import open_source

PCAP_WITH_NO_EXT_DATA_DIR = path.join(path.dirname(
    path.abspath(__file__)), "../../tests/pcap_without_extrinsics")

PCAP_PATH_WITH_NO_EXT = path.join(
    PCAP_WITH_NO_EXT_DATA_DIR, "OS-0-128-U1_v2.3.0_10.pcap")

PCAP_WITH_EXT_DATA_DIR = path.join(path.dirname(
    path.abspath(__file__)), "../../tests/pcap_with_extrinsics")

PCAP_PATH_WITH_EXT = path.join(PCAP_WITH_EXT_DATA_DIR,
                               "OS-0-128-U1_v2.3.0_10.pcap")
EXT_PATH = path.join(PCAP_WITH_EXT_DATA_DIR, "extrinsic_parameters.json")


sensor_names = ["122150000150", "992313000353", "992225001114"]


def test_resolve_extrinscs_with_no_extrinscs():
    extrinsics = resolve_extrinsics(
        data_path=PCAP_PATH_WITH_NO_EXT, sensor_names=sensor_names)
    assert len(extrinsics) == 0


def test_resolve_extrinscs_with_sensor_names():
    extrinsics = resolve_extrinsics(data_path=PCAP_PATH_WITH_EXT)
    assert len(extrinsics) == 0

    extrinsics = resolve_extrinsics(
        data_path=PCAP_PATH_WITH_EXT, sensor_names=[sensor_names[0]])
    assert len(extrinsics) == 1
    assert extrinsics[0][0].shape == (4, 4)
    assert extrinsics[0][1] == EXT_PATH

    extrinsics = resolve_extrinsics(
        data_path=PCAP_PATH_WITH_EXT, sensor_names=sensor_names)
    assert len(extrinsics) == 3
    for ext, src in extrinsics:
        assert ext.shape == (4, 4)
        assert src == EXT_PATH


def test_resolve_extrinscs_with_sensor_infos():

    sensor_infos = [SensorInfo()] * len(sensor_names)
    for si, sn in zip(sensor_infos, sensor_names):
        si.sn = sn

    extrinsics = resolve_extrinsics(data_path=PCAP_PATH_WITH_EXT)
    assert len(extrinsics) == 0

    extrinsics = resolve_extrinsics(
        data_path=PCAP_PATH_WITH_EXT, infos=[sensor_infos[0]])
    assert len(extrinsics) == 1
    assert extrinsics[0][0].shape == (4, 4)
    assert extrinsics[0][1] == EXT_PATH

    extrinsics = resolve_extrinsics(
        data_path=PCAP_PATH_WITH_EXT, infos=sensor_infos)
    assert len(extrinsics) == 3
    for ext, src in extrinsics:
        assert ext.shape == (4, 4)
        assert src == EXT_PATH


def test_resolve_extrinscs_using_dir():

    extrinsics = resolve_extrinsics(data_path=PCAP_WITH_EXT_DATA_DIR)
    assert len(extrinsics) == 0

    extrinsics = resolve_extrinsics(
        data_path=PCAP_WITH_EXT_DATA_DIR, sensor_names=[sensor_names[0]])
    assert len(extrinsics) == 1
    assert extrinsics[0][0].shape == (4, 4)
    assert extrinsics[0][1] == EXT_PATH

    extrinsics = resolve_extrinsics(
        data_path=PCAP_WITH_EXT_DATA_DIR, sensor_names=sensor_names)
    assert len(extrinsics) == 3
    for ext, src in extrinsics:
        assert ext.shape == (4, 4)
        assert src == EXT_PATH


def test_open_source_with_file_that_has_no_valid_extrinscs():
    ss = open_source(source_url=PCAP_PATH_WITH_NO_EXT)
    np.testing.assert_array_equal(ss.metadata.extrinsic, np.eye(4))


def test_open_source_with_file_that_has_no_valid_extrinscs_but_supply_array():
    ss = open_source(source_url=PCAP_PATH_WITH_NO_EXT,
                     extrinsics=np.ones((4, 4)))
    np.testing.assert_array_equal(ss.metadata.extrinsic, np.ones((4, 4)))


def test_open_source_with_file_that_has_no_valid_extrinscs_but_supply_extrinscs_path():
    ss = open_source(source_url=PCAP_PATH_WITH_NO_EXT,
                     extrinsics=EXT_PATH)
    array_cmp = ss.metadata.extrinsic != np.eye(4)
    assert array_cmp.any()


def test_open_source_with_file_that_has_valid_extrinscs():
    ss = open_source(source_url=PCAP_PATH_WITH_EXT)
    array_cmp = ss.metadata.extrinsic != np.eye(4)
    assert array_cmp.any()
