from contextlib import closing
import logging
from typing import Iterator, Optional

from more_itertools import partition
from copy import deepcopy
import pytest

from ouster.sdk import core, sensor
from ouster.sdk.core import LidarMode, UDPProfileLidar, Version
from ouster.sdk.sensor import SensorHttp

import requests
import json

logger = logging.getLogger("HIL")
logging.getLogger("requests").setLevel(logging.WARNING)
logging.getLogger("urllib3").setLevel(logging.WARNING)
GEN1_PROD_PNS = ["840-101855-02", "840-101396-03"]


def pytest_addoption(parser):
    parser.addoption("--sensor",
                     action="store",
                     required=True,
                     help="Sensor hostname")

    parser.addoption("--full",
                     action="store_true",
                     required=False,
                     default=False,
                     help="Run full test suite")

@pytest.fixture
def lowest_signal_multiplier_fw():
    return Version.from_string("2.1.0")


@pytest.fixture
def lowest_eudp_fw():
    return Version.from_string("2.2.0")


@pytest.fixture
def lowest_lb_single_returns_fw():
    return Version.from_string("2.3.0")

@pytest.fixture
def lowest_metadata_endpoint_fw():
    return Version.from_string("2.3.0")


@pytest.fixture
def lowest_4096_fw():
    return Version.from_string("2.4.0")


@pytest.fixture()
def lowest_signal_multiplier_double_fw():
    return Version.from_string("2.5.0")


@pytest.fixture
def lowest_full_speed_bloom_fw():
    return Version.from_string("3.0.0")


@pytest.fixture
def lowest_fusa_profile_fw():
    return Version.from_string("3.1.0")


@pytest.fixture(scope='session')
def hil_sensor_hostname(request) -> Iterator[str]:
    """Provide sensor hostname flag value as a fixture."""
    hil_sensor_hostname = request.config.getoption("--sensor")
    log_sensor_alerts(hil_sensor_hostname)
    # Store the fixture value in a global variable
    request.config.hil_sensor_hostname = hil_sensor_hostname

    yield hil_sensor_hostname
    return hil_sensor_hostname

def pytest_collection_modifyitems(items, config):
    """Deselect any items marked "full" unless the --full flag is set."""

    if config.option.full is True:
        return

    selected, deselected = partition(
        lambda item: item.get_closest_marker("full"), items)

    config.hook.pytest_deselected(items=deselected)
    items[:] = selected

def log_sensor_alerts(sensor_arg):
    alerts_endpoint = f"http://{sensor_arg}/api/v1/sensor/alerts"
    response = requests.get(alerts_endpoint)
    log = response.json().get('log', [])
    active = response.json().get('active', [])
    # ignore UDP transmission errors...
    # they happen all the time because we're not always listening to the sensor's UDP traffic
    whitelist = ['0x01000015', '0x01000018']
    alerts = [alert for alert in log if alert['id'] not in whitelist]
    previous_active_alerts = [alert for alert in log if alert['active'] == True]
    #not failing the tests because the sensor works as expected (no test failure)
    #even on some active alerts
    print(f"-------\nActive alerts on sensor {sensor_arg}: {active} -------\n")
    print(f"-------\nPrevious active alerts on sensor {sensor_arg}: {previous_active_alerts} -------\n")


def pytest_configure(config):
    """Register custom "full" marker to avoid warnings."""
    config.addinivalue_line("markers", "full: run only if --full is passed")


@pytest.fixture(autouse=True, scope="module")
def hil_initial_config(hil_sensor_hostname) -> Iterator[core.SensorConfig]:
    """Initialize the sensor under test.

    Wake up sensor, set default ports and auto udp dest.

    Note that the return for teardown means you will get an extra error at
    teardown if hil_initial_config fails.

    For ease of use, every parameter set here should be compatible with the
    lowest firmware currently supported by SDK.
    """
    logger.debug(f"Initializing {hil_sensor_hostname}...")

    try:
        # remember original configuration
        orig_cfg = sensor.get_config(hil_sensor_hostname)

        # wake up sensor and update udp settings
        cfg = core.SensorConfig()
        cfg.operating_mode = core.OperatingMode.OPERATING_NORMAL
        cfg.udp_dest = None
        cfg.udp_port_lidar = 7502
        cfg.udp_port_imu = 7503
        sensor.set_config(hil_sensor_hostname, cfg, udp_dest_auto=True)

        with closing(sensor.SensorPacketSource(hil_sensor_hostname,
                                   lidar_port=cfg.udp_port_lidar, imu_port=cfg.udp_port_imu)) as src:
            metadata = src.sensor_info

        logger.debug(f"Done {metadata}")

        config = sensor.get_config(hil_sensor_hostname)
        config.signal_multiplier = 1
        config.azimuth_window = (0, 360000)
        yield config

    except RuntimeError as e:
        logger.warning(f"Error with initial configuration of sensor: {e}")
        raise e


@pytest.fixture
def hil_sensor_config(hil_initial_config) -> core.SensorConfig:
    """Default to initial config. Usually overridden in each test module."""
    logger.warning("Using initial config")
    return hil_initial_config


@pytest.fixture
def hil_configured_sensor(hil_sensor_hostname, hil_sensor_config, request,
                          hil_sensor_firmware, gen1_sensor,
                          lowest_signal_multiplier_fw, lowest_eudp_fw,
                          lowest_lb_single_returns_fw, lowest_4096_fw,
                          lowest_signal_multiplier_double_fw,
                          lowest_full_speed_bloom_fw,
                          lowest_fusa_profile_fw) -> Optional[str]:
    """Configure the sensor using the config fixture.

    Checks if the desired configuration is already set to avoid waiting on
    reinit as an optimization.

    Returns:
        The sensor hostname or None if configuration failed. Tests may choose
        whether this is an error.
    """
    try:
        cfg = sensor.get_config(hil_sensor_hostname)
        if cfg == hil_sensor_config:
            logger.debug(f"Reusing current config for: {request.node.name}")
        else:
            logger.debug(f"Updating config for: {request.node.name}")

        if gen1_sensor and hil_sensor_config.udp_profile_lidar == UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL:
            pytest.skip(f"Dual returns is not supported on Gen 1 sensors")

        # Run through and check compatibility
        if hil_sensor_config.signal_multiplier and hil_sensor_firmware < lowest_signal_multiplier_fw:
            pytest.skip(
                f"Signal multiplier is not supported on FW {str(hil_sensor_firmware)}"
            )

        if (hil_sensor_config.udp_profile_lidar is not None and
            hil_sensor_config.udp_profile_lidar
            != UDPProfileLidar.PROFILE_LIDAR_LEGACY and hil_sensor_firmware
            < lowest_eudp_fw) or (hil_sensor_config.udp_profile_lidar in {
            UDPProfileLidar.PROFILE_LIDAR_RNG15_RFL8_NIR8,
            UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16
        } and hil_sensor_firmware < lowest_lb_single_returns_fw) or (
                hil_sensor_config.udp_profile_lidar
                == UDPProfileLidar.PROFILE_LIDAR_FUSA_RNG15_RFL8_NIR8_DUAL and
                hil_sensor_firmware < lowest_fusa_profile_fw):
            pytest.skip(
                f"Lidar profile {str(hil_sensor_config.udp_profile_lidar)} is not supported on FW {str(hil_sensor_firmware)}"
            )

        if hil_sensor_config.lidar_mode == LidarMode.MODE_4096x5 and hil_sensor_firmware < lowest_4096_fw:
            pytest.skip(
                f"Lidar Mode 4096x5 is not supported on FW {str(hil_sensor_firmware)}"
            )

        # rely on exact precision representation
        if (hil_sensor_config.signal_multiplier and int(hil_sensor_config.signal_multiplier) != hil_sensor_config.signal_multiplier and hil_sensor_firmware < lowest_signal_multiplier_double_fw):
            pytest.skip(
                f"Signal multiplier {hil_sensor_config.signal_multiplier} not supported on FW {str(hil_sensor_firmware)}"
            )

        if hil_sensor_firmware < lowest_full_speed_bloom_fw and hil_sensor_config.udp_profile_lidar == UDPProfileLidar.PROFILE_LIDAR_RNG19_RFL8_SIG16_NIR16_DUAL and hil_sensor_config.lidar_mode in {
            LidarMode.MODE_2048x10, LidarMode.MODE_1024x20,
            LidarMode.MODE_4096x5
        }:
            pytest.skip(
                f"Dual return profile is not supported with 2048x10 or 1024x20 lidar modes on FW {str(hil_sensor_firmware)}"
            )

        sensor.set_config(hil_sensor_hostname, hil_sensor_config)

    except ValueError as e:
        logger.warning(f"Config rejected: {str(e)}")
        return None

    except RuntimeError as e:
        logger.warning(f"Config rejected: {str(e)}")
        return None

    return hil_sensor_hostname


@pytest.fixture
def hil_sensor_firmware(hil_sensor_hostname):
    return SensorHttp.create(hil_sensor_hostname).firmware_version()


# TODO find a good place to store part numbers for these types of comparisons
@pytest.fixture
def gen1_sensor(hil_sensor_hostname):
    with closing(sensor.SensorPacketSource(hil_sensor_hostname, lidar_port=7502, imu_port=7503)) as src:
        prod_pn = src.metadata[0].prod_pn

    return prod_pn in GEN1_PROD_PNS


def deepcopy_iter(it):
    """Used in making copy of packets when reading from the Sensor source"""
    for x in it:
        yield deepcopy(x)

def pytest_sessionfinish(session, exitstatus):
    if exitstatus == pytest.ExitCode.TESTS_FAILED:
        log_sensor_alerts(session.config.hil_sensor_hostname)
