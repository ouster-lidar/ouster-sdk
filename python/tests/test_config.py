from copy import copy

from os import path

import pytest

from ouster import client

DATA_DIR = path.join(path.dirname(path.abspath(__file__)), "data")


@pytest.mark.parametrize("mode, string", [
    (client.OperatingMode.OPERATING_NORMAL, "NORMAL"),
    (client.OperatingMode.OPERATING_STANDBY, "STANDBY"),
])
def test_operating_mode(mode, string) -> None:
    """Check operating mode (un)parsing."""
    int(mode)  # make sure nothing is raised
    assert str(mode) == string
    assert client.OperatingMode.from_string(string) == mode


def test_operating_mode_misc() -> None:
    """Check some misc properties of operating modes."""
    assert len(
        client.OperatingMode.__members__) == 2, "Don't forget to update tests!"
    assert client.OperatingMode.from_string(
        "foo") is None
    assert client.OperatingMode(1) == client.OperatingMode.OPERATING_NORMAL


@pytest.mark.parametrize("mode, string", [
    (client.MultipurposeIOMode.MULTIPURPOSE_OFF, "OFF"),
    (client.MultipurposeIOMode.MULTIPURPOSE_INPUT_NMEA_UART,
     "INPUT_NMEA_UART"),
    (client.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC,
     "OUTPUT_FROM_INTERNAL_OSC"),
    (client.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN,
     "OUTPUT_FROM_SYNC_PULSE_IN"),
    (client.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_PTP_1588,
     "OUTPUT_FROM_PTP_1588"),
    (client.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE,
     "OUTPUT_FROM_ENCODER_ANGLE"),
])
def test_multipurpose_io_mode(mode, string) -> None:
    """Check multipurpose mode (un)parsing."""
    int(mode)  # make sure nothing is raised
    assert str(mode) == string
    assert client.MultipurposeIOMode.from_string(string) == mode


def test_multipurpose_io_mode_misc() -> None:
    """Check some misc properties of multipurpose mode."""
    assert len(client.MultipurposeIOMode.__members__
               ) == 6, "Don't forget to update tests!"
    assert client.MultipurposeIOMode.from_string(
        "foo") is None
    assert client.MultipurposeIOMode(
        1) == client.MultipurposeIOMode.MULTIPURPOSE_OFF


@pytest.mark.parametrize("polarity, string", [
    (client.Polarity.POLARITY_ACTIVE_HIGH, "ACTIVE_HIGH"),
    (client.Polarity.POLARITY_ACTIVE_LOW, "ACTIVE_LOW"),
])
def test_polarity(polarity, string) -> None:
    """Check polarity (un)parsing."""
    int(polarity)  # make sure nothing is raised
    assert str(polarity) == string
    assert client.Polarity.from_string(string) == polarity


def test_polarity_misc() -> None:
    """Check some misc properties of polarity."""
    assert len(
        client.Polarity.__members__) == 2, "Don't forget to update tests!"
    assert client.Polarity.from_string(
        "foo") is None
    assert client.Polarity(1) == client.Polarity.POLARITY_ACTIVE_LOW


@pytest.mark.parametrize("nmea_baud_rate, string", [
    (client.NMEABaudRate.BAUD_9600, "BAUD_9600"),
    (client.NMEABaudRate.BAUD_115200, "BAUD_115200"),
])
def test_nmea_baud_rate(nmea_baud_rate, string) -> None:
    """Check nmea baud rate (un)parsing."""
    int(nmea_baud_rate)  # make usre nothing is raised
    assert str(nmea_baud_rate) == string
    assert client.NMEABaudRate.from_string(string) == nmea_baud_rate


def test_nmea_baud_rate_misc() -> None:
    """Check some misc properties of nmea bad rate."""
    assert len(
        client.NMEABaudRate.__members__) == 2, "Don't forget to update tests!"
    assert client.NMEABaudRate.from_string(
        "foo") is None
    assert client.NMEABaudRate(1) == client.NMEABaudRate.BAUD_9600


def test_optional_config() -> None:
    """Check that all fields are optional."""
    config = client.SensorConfig()

    # make sure all the values are empty
    assert config.azimuth_window is None
    assert config.lidar_mode is None
    assert config.multipurpose_io_mode is None
    assert config.nmea_baud_rate is None
    assert config.nmea_in_polarity is None
    assert config.nmea_ignore_valid_char is None
    assert config.nmea_leap_seconds is None
    assert config.operating_mode is None
    assert config.phase_lock_enable is None
    assert config.phase_lock_offset is None
    assert config.signal_multiplier is None
    assert config.sync_pulse_out_pulse_width is None
    assert config.sync_pulse_out_frequency is None
    assert config.sync_pulse_in_polarity is None
    assert config.sync_pulse_out_angle is None
    assert config.sync_pulse_out_polarity is None
    assert config.timestamp_mode is None
    assert config.udp_dest is None
    assert config.udp_port_imu is None
    assert config.udp_port_lidar is None


def test_write_config() -> None:
    """Check modifying config."""
    config = client.SensorConfig()
    config.azimuth_window = (0, 0)
    config.lidar_mode = client.LidarMode.MODE_512x10
    config.multipurpose_io_mode = client.MultipurposeIOMode.MULTIPURPOSE_INPUT_NMEA_UART
    config.nmea_baud_rate = client.NMEABaudRate.BAUD_9600
    config.nmea_in_polarity = client.Polarity.POLARITY_ACTIVE_LOW
    config.nmea_ignore_valid_char = True
    config.nmea_leap_seconds = 20
    config.operating_mode = client.OperatingMode.OPERATING_STANDBY
    config.phase_lock_enable = True
    config.phase_lock_offset = 180000
    config.signal_multiplier = 2
    config.sync_pulse_out_pulse_width = 5
    config.sync_pulse_out_frequency = 2
    config.sync_pulse_in_polarity = client.Polarity.POLARITY_ACTIVE_HIGH
    config.sync_pulse_out_angle = 300
    config.sync_pulse_out_polarity = client.Polarity.POLARITY_ACTIVE_LOW
    config.timestamp_mode = client.TimestampMode.TIME_FROM_PTP_1588
    config.udp_dest = "udp-dest"
    config.udp_port_imu = 84
    config.udp_port_lidar = 3827

    with pytest.raises(TypeError):
        config.lidar_mode = 1  # type: ignore
    with pytest.raises(TypeError):
        config.sync_pulse_in_polarity = client.MultipurposeIOMode.MULTIPURPOSE_OFF  # type: ignore


@pytest.fixture()
def complete_config_string() -> str:
    complete_config_string = """
        {"azimuth_window": [0, 360000],
        "lidar_mode": "1024x10",
        "multipurpose_io_mode": "OFF",
        "nmea_baud_rate": "BAUD_9600",
        "nmea_ignore_valid_char": 0,
        "nmea_in_polarity": "ACTIVE_HIGH",
        "nmea_leap_seconds": 0,
        "operating_mode": "NORMAL",
        "phase_lock_enable": false,
        "phase_lock_offset": 0,
        "signal_multiplier": 2,
        "sync_pulse_in_polarity": "ACTIVE_HIGH",
        "sync_pulse_out_angle": 360,
        "sync_pulse_out_frequency": 1,
        "sync_pulse_out_polarity": "ACTIVE_HIGH",
        "sync_pulse_out_pulse_width": 10,
        "timestamp_mode": "TIME_FROM_INTERNAL_OSC",
        "udp_dest": "",
        "udp_port_imu": 7503,
        "udp_port_lidar": 7502}"""
    return complete_config_string


def test_read_config(complete_config_string: str) -> None:
    """Check reading from and writing to string."""
    config = client.SensorConfig(complete_config_string)  # read from string

    # make sure all the values are correct
    assert config.azimuth_window == (0, 360000)
    assert config.lidar_mode == client.LidarMode.MODE_1024x10
    assert config.multipurpose_io_mode == client.MultipurposeIOMode.MULTIPURPOSE_OFF
    assert config.nmea_baud_rate == client.NMEABaudRate.BAUD_9600
    assert config.nmea_in_polarity == client.Polarity.POLARITY_ACTIVE_HIGH
    assert config.nmea_ignore_valid_char is False
    assert config.nmea_leap_seconds == 0
    assert config.operating_mode == client.OperatingMode.OPERATING_NORMAL
    assert config.phase_lock_enable is False
    assert config.phase_lock_offset == 0
    assert config.signal_multiplier == 2
    assert config.sync_pulse_out_pulse_width == 10
    assert config.sync_pulse_out_frequency == 1
    assert config.sync_pulse_in_polarity == client.Polarity.POLARITY_ACTIVE_HIGH
    assert config.sync_pulse_out_angle == 360
    assert config.sync_pulse_out_polarity == client.Polarity.POLARITY_ACTIVE_HIGH
    assert config.timestamp_mode == client.TimestampMode.TIME_FROM_INTERNAL_OSC
    assert config.udp_dest == ""
    assert config.udp_port_imu == 7503
    assert config.udp_port_lidar == 7502

    # check output of string
    assert ''.join(str(config).split()) == ''.join(
        complete_config_string.split())


def test_equality_config(complete_config_string: str) -> None:
    """Check equality comparisons."""

    complete_config_1 = client.SensorConfig(complete_config_string)
    complete_config_2 = client.SensorConfig(complete_config_string)
    assert complete_config_1 == complete_config_2

    complete_config_2.multipurpose_io_mode = client.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_PTP_1588
    assert complete_config_1 != complete_config_2

    partial_config_1 = client.SensorConfig()
    partial_config_1.nmea_baud_rate = client.NMEABaudRate.BAUD_115200
    partial_config_1.operating_mode = client.OperatingMode.OPERATING_STANDBY

    partial_config_2 = client.SensorConfig()
    partial_config_2.nmea_baud_rate = client.NMEABaudRate.BAUD_115200
    partial_config_2.operating_mode = client.OperatingMode.OPERATING_STANDBY
    assert partial_config_1 == partial_config_2

    partial_config_2.operating_mode = client.OperatingMode.OPERATING_NORMAL
    assert partial_config_1 != partial_config_2

    empty_config_1 = client.SensorConfig()
    empty_config_2 = client.SensorConfig()
    assert empty_config_1 == empty_config_2

    assert complete_config_1 != empty_config_1
    assert complete_config_1 != partial_config_1
    assert complete_config_1 != partial_config_2
    assert partial_config_1 != empty_config_1
    assert partial_config_2 != empty_config_1


def test_copy_config(complete_config_string: str) -> None:
    """Check that copy() works."""
    config = client.SensorConfig(complete_config_string)
    config1 = copy(config)

    assert config1 is not config
    assert config1 == config


def test_parse_config() -> None:
    """Sanity check parsing from json string."""

    with pytest.raises(ValueError):
        client.SensorConfig('/')
    with pytest.raises(ValueError):
        client.SensorConfig('{ ')


@pytest.fixture()
def deprecated_params_config() -> client.SensorConfig:
    deprecated_params = """ { "udp_ip": "169.254.148.183", "auto_start_flag": 1 }"""
    return client.SensorConfig(deprecated_params)


def test_deprecated_config(deprecated_params_config) -> None:
    """Check that deprecated params are properly translated."""
    assert deprecated_params_config.udp_dest == "169.254.148.183"
    assert deprecated_params_config.operating_mode == client.OperatingMode.OPERATING_NORMAL
