from os import path
import numpy as np
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


def test_open_source_with_file_that_has_no_valid_extrinsics():
    ss = open_source(source_url=PCAP_PATH_WITH_NO_EXT)
    np.testing.assert_array_equal(ss.sensor_info[0].extrinsic, np.eye(4))


def test_open_source_with_file_that_has_no_valid_extrinsics_but_supply_array():
    ss = open_source(source_url=PCAP_PATH_WITH_NO_EXT,
                     extrinsics=[np.ones((4, 4))])
    np.testing.assert_array_equal(ss.sensor_info[0].extrinsic, np.ones((4, 4)))


def test_open_source_with_file_that_has_no_valid_extrinsics_but_supply_extrinsics_path():
    ss = open_source(source_url=PCAP_PATH_WITH_NO_EXT,
                     extrinsics_file=EXT_PATH)
    array_cmp = ss.sensor_info[0].extrinsic != np.eye(4)
    assert array_cmp.any()


def test_open_source_with_file_that_has_valid_extrinsics_no_automatic():
    ss = open_source(source_url=PCAP_PATH_WITH_EXT)
    array_cmp = ss.sensor_info[0].extrinsic == np.eye(4)
    assert array_cmp.any()
