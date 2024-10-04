from contextlib import closing
from copy import copy
import logging

from more_itertools import take
import pytest

from ouster.sdk import client
from ouster.sdk.client import SensorHttp, Version

import requests
import time

from packaging import version

logger = logging.getLogger("HIL")

# http takes a long time to start after reboot, esp on FW 2.1
reboot_time = 40
# reinit should be almost instantaneous
reinit_time = 1



def test_config_noop(hil_sensor_hostname, hil_sensor_firmware) -> None:
    """Test that setting the empty config does not change sensor params."""

    logger.debug("Grabbing initial config and metadata")
    cfg0 = client.get_config(hil_sensor_hostname)
    with closing(client.Sensor(hil_sensor_hostname, 7502, 7503)) as src:
        metadata0 = src.metadata

    logger.debug("Noop set_config")
    client.set_config(hil_sensor_hostname, client.SensorConfig(), force_reinit=True)

    logger.debug("Grabbing new config and metadata")
    cfg1 = client.get_config(hil_sensor_hostname)
    with closing(client.Sensor(hil_sensor_hostname, 7502, 7503)) as src:
        metadata1 = src.metadata

    logger.debug("Checking for changes")

    assert cfg0 == cfg1

    # After re-init we expect init_id always to be changed for FW >= v2.2.0
    # as previous versions had no init_id in metadata
    if hil_sensor_firmware >= Version.from_string("2.2.0"):
        assert metadata1.init_id != metadata0.init_id

    # but everything else should be the same in metadatas
    metadata1.init_id = metadata0.init_id
    assert metadata0 == metadata1
    assert metadata0.has_fields_equal(metadata1)  # but the decoded fields are still equal!


def test_config_basic(hil_sensor_hostname) -> None:
    """Test configuration of 'basic' values which have 1 stable representation in config since FW 2.0"""

    cfg0 = client.get_config(hil_sensor_hostname)
    cfg0.timestamp_mode = client.TimestampMode.TIME_FROM_PTP_1588
    cfg0.nmea_leap_seconds = 20

    client.set_config(hil_sensor_hostname, cfg0)

    cfg1 = client.get_config(hil_sensor_hostname)

    assert cfg1 == cfg0


def test_config_operating_mode(hil_sensor_hostname, hil_initial_config) -> None:
    """Test that operating mode specifically gets set since it has duplicated fields in the config settings which complicate its story."""

    # initial config sets it to OPERATING_NORMAL
    client.set_config(hil_sensor_hostname, hil_initial_config)

    cfg0 = client.SensorConfig()
    cfg0.operating_mode = client.OperatingMode.OPERATING_STANDBY
    client.set_config(hil_sensor_hostname, cfg0)

    cfg1 = client.get_config(hil_sensor_hostname)
    assert cfg1.operating_mode == client.OperatingMode.OPERATING_STANDBY


def test_config_udp_auto(hil_sensor_hostname, hil_initial_config) -> None:
    """Test that set_udp_dest_auto sends data to this host."""

    logger.debug("Turning off UDP output")
    no_udp_cfg = copy(hil_initial_config)
    no_udp_cfg.udp_dest = ""
    client.set_config(hil_sensor_hostname, no_udp_cfg)
    no_udp_cfg = client.get_config(hil_sensor_hostname)

    logger.debug(f"Sleeping for {reinit_time} seconds after reinit..")
    time.sleep(reinit_time)
    logger.debug("Checking for no incoming data")
    with pytest.raises(client.ClientTimeout):
        with closing(client.Sensor(hil_sensor_hostname, 7502, 7503)) as packets:
            take(10, packets)

    logger.debug("Setting UDP dest to auto")
    client.set_config(hil_sensor_hostname,
                      client.SensorConfig(),
                      udp_dest_auto=True)

    logger.debug(f"Sleeping for {reinit_time} seconds after reinit..")
    time.sleep(reinit_time)
    logger.debug("Attempting to read scans")
    with closing(client.Scans.stream(hil_sensor_hostname,
                                     complete=False)) as scans:
        take(10, scans)

    cfg1 = client.get_config(hil_sensor_hostname)
    logger.debug(f"Sanity check new config (New UDP dest: {cfg1.udp_dest})")
    assert no_udp_cfg.udp_dest != cfg1.udp_dest
    cfg1.udp_dest = ""
    assert no_udp_cfg == cfg1


def test_config_persist(hil_sensor_hostname, hil_initial_config) -> None:
    """Test that persist flag actually persists after reboot"""

    def reboot_sensor():
        version = SensorHttp.create(hil_sensor_hostname).firmware_version()
        reboot_endpoint = "http://" + hil_sensor_hostname + "/api/v1/system/cmd/reboot"
        if version.major >= 3 and version.minor >= 1:
            reboot_endpoint = "http://" + hil_sensor_hostname + "/api/v1/system/restart"

        response = requests.post(reboot_endpoint)
        if response.status_code not in [requests.codes.ok, requests.codes.no_content]:
            logger.warning("Failed to reboot sensor")
            raise RuntimeError("Failed to reboot sensor. NOTE! We expect this test to fail with 3.1-rc1 and 3.1-rc2!")
        logger.debug(f"Sleeping for {reboot_time} seconds after reboot..")
        time.sleep(reboot_time)  # Sleep to avoid issues - see comment on reboot_time

    # First let's make sure the saved config param is as we expect
    client.set_config(hil_sensor_hostname, hil_initial_config, persist=True)

    # Set one parameter differently
    cfg0 = client.SensorConfig()
    cfg0.azimuth_window = (583, 39402)
    client.set_config(hil_sensor_hostname, cfg0)

    # Test that it doesn't persist
    reboot_sensor()
    cfg1 = client.get_config(hil_sensor_hostname)
    assert cfg1.azimuth_window == hil_initial_config.azimuth_window

    # And then that it does
    client.set_config(hil_sensor_hostname, cfg0, persist=True)
    reboot_sensor()
    cfg2 = client.get_config(hil_sensor_hostname)
    assert cfg2.azimuth_window == cfg0.azimuth_window


# Need to have signal_multiplier due to autouse on skip_by_fw
# https://docs.pytest.org/en/6.2.x/fixture.html#override-a-fixture-with-direct-test-parametrization
@pytest.fixture()
def signal_multiplier():
    return 1


# Pytest injects fixtures so they are not imported at higher scope
# https://stackoverflow.com/questions/19169057/can-i-use-a-pytest-fixture-in-the-condition-of-my-skipif-logic
# https://stackoverflow.com/questions/28179026/how-to-skip-a-pytest-using-an-external-fixture
# Use extra fixture to work around this
@pytest.fixture(autouse=True)
def skip_by_fw(request, hil_sensor_firmware, signal_multiplier, lowest_signal_multiplier_fw,
               lowest_signal_multiplier_double_fw):
    if request.node.get_closest_marker('skip_under_fw'):
        if hil_sensor_firmware < lowest_signal_multiplier_fw:
            pytest.skip(f"Skip signal multiplier test as FW version {hil_sensor_firmware} lt {lowest_signal_multiplier_fw} which added signal multiplier")
        if hil_sensor_firmware < lowest_signal_multiplier_double_fw and int(
                signal_multiplier) != signal_multiplier:
            pytest.skip(f"Skipped signal_multiplier {signal_multiplier} as FW version {hil_sensor_firmware} lt {lowest_signal_multiplier_double_fw} which added support for non-intenger signal multiplier")


@pytest.mark.skip_under_fw()
@pytest.mark.parametrize('signal_multiplier', [0.25, 0.5, 1, 2, 3]) # all possible sig mult values
def test_good_signal_multiplier_values(hil_sensor_hostname,
                                  signal_multiplier, gen1_sensor) -> None:
    """Test that all valid values of signal multiplier can be get and set using SDK for FW 3.0+"""

    cfg0 = client.SensorConfig()
    # make sure azimuth window is small enough for everything
    cfg0.azimuth_window = (0, 10000)
    # make sure lidar mode is not "full res" in case it is a Rev 6 or under sensor
    cfg0.lidar_mode = client.LidarMode.MODE_1024x10
    cfg0.signal_multiplier = signal_multiplier

    if gen1_sensor and signal_multiplier != 1:
        with pytest.raises(RuntimeError):
            # expect RuntimeError because of Invalid Configuration Value
            client.set_config(hil_sensor_hostname, cfg0)
        # Return since the rest of the test is for non gen1
        return

    client.set_config(hil_sensor_hostname, cfg0)

    cfg1 = client.get_config(hil_sensor_hostname)
    assert cfg0.signal_multiplier == cfg1.signal_multiplier

@pytest.mark.parametrize('signal_multiplier', [0.3, 1.2, 5, 5.5]) # sig mult value < 1 that isn't .25 or .5, double sig mult between 1 and 3, int sig mult > 3, double sig int > 3
def test_bad_signal_multiplier_values(hil_sensor_hostname, signal_multiplier) -> None:
    cfg0 = client.SensorConfig()
    # even though these aren't valid signal multipliers, set lidar_mode and azimuth_window so they don't cause other errors
    # make sure azimuth window is small enough for everything
    cfg0.azimuth_window = (0, 10000)
    # make sure lidar mode is not "full res" in case it is a Rev 6 or under sensor
    cfg0.lidar_mode = client.LidarMode.MODE_1024x10

    cfg0.signal_multiplier = signal_multiplier

    with pytest.raises(RuntimeError):
        client.set_config(hil_sensor_hostname, cfg0)
