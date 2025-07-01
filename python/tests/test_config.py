"""
Copyright (c) 2021, Ouster, Inc.
All rights reserved.
"""

from copy import copy, deepcopy

import pytest
import warnings
import inspect
import json

from ouster.sdk import core

# all valid values
valid_signal_multiplier_values = [0.25, 0.5, 1, 2, 3]
# make sure to hit different types of invalid values:
# 0, one double below 1, one double between 1 and 3,
# one int above 3, one double above 3
invalid_signal_multiplier_values = [0, 0.3, 1.3, 5, 5.5]


@pytest.mark.parametrize("mode, string", [
    (core.OperatingMode.OPERATING_NORMAL, "NORMAL"),
    (core.OperatingMode.OPERATING_STANDBY, "STANDBY"),
])
def test_operating_mode(mode, string) -> None:
    """Check operating mode (un)parsing."""
    int(mode)  # make sure nothing is raised
    assert str(mode) == string
    assert core.OperatingMode.from_string(string) == mode


def test_operating_mode_misc() -> None:
    """Check some misc properties of operating modes."""
    assert len(
        core.OperatingMode.__members__) == 2, "Don't forget to update tests!"
    assert core.OperatingMode.from_string("foo") is None
    assert core.OperatingMode(1) == core.OperatingMode.OPERATING_NORMAL


@pytest.mark.parametrize("mode, string", [
    (core.MultipurposeIOMode.MULTIPURPOSE_OFF, "OFF"),
    (core.MultipurposeIOMode.MULTIPURPOSE_INPUT_NMEA_UART,
     "INPUT_NMEA_UART"),
    (core.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_INTERNAL_OSC,
     "OUTPUT_FROM_INTERNAL_OSC"),
    (core.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_SYNC_PULSE_IN,
     "OUTPUT_FROM_SYNC_PULSE_IN"),
    (core.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_PTP_1588,
     "OUTPUT_FROM_PTP_1588"),
    (core.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_ENCODER_ANGLE,
     "OUTPUT_FROM_ENCODER_ANGLE"),
])
def test_multipurpose_io_mode(mode, string) -> None:
    """Check multipurpose mode (un)parsing."""
    int(mode)  # make sure nothing is raised
    assert str(mode) == string
    assert core.MultipurposeIOMode.from_string(string) == mode


def test_multipurpose_io_mode_misc() -> None:
    """Check some misc properties of multipurpose mode."""
    assert len(core.MultipurposeIOMode.__members__
               ) == 6, "Don't forget to update tests!"
    assert core.MultipurposeIOMode.from_string("foo") is None
    assert core.MultipurposeIOMode(
        1) == core.MultipurposeIOMode.MULTIPURPOSE_OFF


@pytest.mark.parametrize("polarity, string", [
    (core.Polarity.POLARITY_ACTIVE_HIGH, "ACTIVE_HIGH"),
    (core.Polarity.POLARITY_ACTIVE_LOW, "ACTIVE_LOW"),
])
def test_polarity(polarity, string) -> None:
    """Check polarity (un)parsing."""
    int(polarity)  # make sure nothing is raised
    assert str(polarity) == string
    assert core.Polarity.from_string(string) == polarity


def test_polarity_misc() -> None:
    """Check some misc properties of polarity."""
    assert len(
        core.Polarity.__members__) == 2, "Don't forget to update tests!"
    assert core.Polarity.from_string("foo") is None
    assert core.Polarity(1) == core.Polarity.POLARITY_ACTIVE_LOW


@pytest.mark.parametrize("nmea_baud_rate, string", [
    (core.NMEABaudRate.BAUD_9600, "BAUD_9600"),
    (core.NMEABaudRate.BAUD_115200, "BAUD_115200"),
])
def test_nmea_baud_rate(nmea_baud_rate, string) -> None:
    """Check nmea baud rate (un)parsing."""
    int(nmea_baud_rate)  # make usre nothing is raised
    assert str(nmea_baud_rate) == string
    assert core.NMEABaudRate.from_string(string) == nmea_baud_rate


def test_nmea_baud_rate_misc() -> None:
    """Check some misc properties of nmea bad rate."""
    assert len(
        core.NMEABaudRate.__members__) == 2, "Don't forget to update tests!"
    assert core.NMEABaudRate.from_string("foo") is None
    assert core.NMEABaudRate(1) == core.NMEABaudRate.BAUD_9600


def test_optional_config() -> None:
    """Check that all fields are optional."""
    config = core.SensorConfig()

    # make sure all the values are empty
    assert config.accel_fsr is None
    assert config.azimuth_window is None
    assert config.lidar_mode is None
    assert config.gyro_fsr is None
    assert config.min_range_threshold_cm is None
    assert config.multipurpose_io_mode is None
    assert config.nmea_baud_rate is None
    assert config.nmea_in_polarity is None
    assert config.nmea_ignore_valid_char is None
    assert config.nmea_leap_seconds is None
    assert config.operating_mode is None
    assert config.phase_lock_enable is None
    assert config.phase_lock_offset is None
    assert config.return_order is None
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
    assert config.udp_profile_lidar is None
    assert config.columns_per_packet is None
    assert len(config.extra_options) == 0


def test_write_config() -> None:
    """Check modifying config."""
    config = core.SensorConfig()
    config.azimuth_window = (0, 0)
    config.lidar_mode = core.LidarMode.MODE_512x10
    config.multipurpose_io_mode = core.MultipurposeIOMode.MULTIPURPOSE_INPUT_NMEA_UART
    config.nmea_baud_rate = core.NMEABaudRate.BAUD_9600
    config.nmea_in_polarity = core.Polarity.POLARITY_ACTIVE_LOW
    config.nmea_ignore_valid_char = True
    config.nmea_leap_seconds = 20
    config.operating_mode = core.OperatingMode.OPERATING_STANDBY
    config.phase_lock_enable = True
    config.phase_lock_offset = 180000
    config.signal_multiplier = 2.0
    config.sync_pulse_out_pulse_width = 5
    config.sync_pulse_out_frequency = 2
    config.sync_pulse_in_polarity = core.Polarity.POLARITY_ACTIVE_HIGH
    config.sync_pulse_out_angle = 300
    config.sync_pulse_out_polarity = core.Polarity.POLARITY_ACTIVE_LOW
    config.timestamp_mode = core.TimestampMode.TIME_FROM_PTP_1588
    config.udp_dest = "udp-dest"
    config.udp_port_imu = 84
    config.udp_port_lidar = 3827
    config.udp_profile_lidar = core.UDPProfileLidar.PROFILE_LIDAR_LEGACY
    config.udp_profile_imu = core.UDPProfileIMU.PROFILE_IMU_LEGACY
    config.columns_per_packet = 8
    config.return_order = core.ReturnOrder.ORDER_FARTHEST_TO_NEAREST
    config.gyro_fsr = core.FullScaleRange.FSR_NORMAL
    config.accel_fsr = core.FullScaleRange.FSR_EXTENDED
    config.min_range_threshold_cm = 30
    config.extra_options = {"hi": '"hello"'}

    with pytest.raises(TypeError):
        config.lidar_mode = 1  # type: ignore
    with pytest.raises(TypeError):
        config.sync_pulse_in_polarity = core.MultipurposeIOMode.MULTIPURPOSE_OFF  # type: ignore


@pytest.fixture()
def complete_config_string() -> str:
    complete_config_string = """
        {"accel_fsr": "EXTENDED",
        "azimuth_window": [0, 360000],
        "columns_per_packet": 8,
        "gyro_fsr": "EXTENDED",
        "lidar_mode": "1024x10",
        "min_range_threshold_cm": 30,
        "multipurpose_io_mode": "OFF",
        "nmea_baud_rate": "BAUD_9600",
        "nmea_ignore_valid_char": 0,
        "nmea_in_polarity": "ACTIVE_HIGH",
        "nmea_leap_seconds": 0,
        "operating_mode": "NORMAL",
        "phase_lock_enable": false,
        "phase_lock_offset": 0,
        "return_order": "STRONGEST_TO_WEAKEST",
        "signal_multiplier": 2,
        "sync_pulse_in_polarity": "ACTIVE_HIGH",
        "sync_pulse_out_angle": 360,
        "sync_pulse_out_frequency": 1,
        "sync_pulse_out_polarity": "ACTIVE_HIGH",
        "sync_pulse_out_pulse_width": 10,
        "timestamp_mode": "TIME_FROM_INTERNAL_OSC",
        "udp_dest": "",
        "udp_port_imu": 7503,
        "udp_port_lidar": 7502,
        "udp_profile_imu": "LEGACY",
        "udp_profile_lidar": "LEGACY",
        "unknown_parameter": "hello"}
    """
    return complete_config_string


@pytest.fixture()
def all_different_config_string() -> str:
    """All different from complete_config_string except for udp_profile_imu"""
    all_different_config_string = """
        {"accel_fsr": "NORMAL",
        "azimuth_window": [180000, 360000],
        "columns_per_packet": 16,
        "gyro_fsr": "NORMAL",
        "lidar_mode": "512x10",
        "min_range_threshold_cm": 0,
        "multipurpose_io_mode": "INPUT_NMEA_UART",
        "nmea_baud_rate": "BAUD_115200",
        "nmea_ignore_valid_char": 1,
        "nmea_in_polarity": "ACTIVE_LOW",
        "nmea_leap_seconds": 10,
        "operating_mode": "STANDBY",
        "phase_lock_enable": true,
        "phase_lock_offset": 180000,
        "return_order": "NEAREST_TO_FARTHEST",
        "signal_multiplier": 0.5,
        "sync_pulse_in_polarity": "ACTIVE_LOW",
        "sync_pulse_out_angle": 180,
        "sync_pulse_out_frequency": 10,
        "sync_pulse_out_polarity": "ACTIVE_LOW",
        "sync_pulse_out_pulse_width": 1,
        "timestamp_mode": "TIME_FROM_SYNC_PULSE_IN",
        "udp_dest": "1.1.1.1",
        "udp_port_imu": 8503,
        "udp_port_lidar": 8502,
        "udp_profile_imu": "LEGACY",
        "udp_profile_lidar": "RNG15_RFL8_NIR8",
        "unknown_parameter": "hi"}
    """
    return all_different_config_string


def test_read_config(complete_config_string: str) -> None:
    """Check reading from and writing to string."""
    config = core.SensorConfig(complete_config_string)  # read from string

    # make sure all the values are correct
    assert config.accel_fsr == core.FullScaleRange.FSR_EXTENDED
    assert config.azimuth_window == (0, 360000)
    assert config.gyro_fsr == core.FullScaleRange.FSR_EXTENDED
    assert config.lidar_mode == core.LidarMode.MODE_1024x10
    assert config.min_range_threshold_cm == 30
    assert config.multipurpose_io_mode == core.MultipurposeIOMode.MULTIPURPOSE_OFF
    assert config.nmea_baud_rate == core.NMEABaudRate.BAUD_9600
    assert config.nmea_in_polarity == core.Polarity.POLARITY_ACTIVE_HIGH
    assert config.nmea_ignore_valid_char is False
    assert config.nmea_leap_seconds == 0
    assert config.operating_mode == core.OperatingMode.OPERATING_NORMAL
    assert config.phase_lock_enable is False
    assert config.phase_lock_offset == 0
    assert config.return_order == core.ReturnOrder.ORDER_STRONGEST_TO_WEAKEST
    assert config.signal_multiplier == 2
    assert config.sync_pulse_out_pulse_width == 10
    assert config.sync_pulse_out_frequency == 1
    assert config.sync_pulse_in_polarity == core.Polarity.POLARITY_ACTIVE_HIGH
    assert config.sync_pulse_out_angle == 360
    assert config.sync_pulse_out_polarity == core.Polarity.POLARITY_ACTIVE_HIGH
    assert config.timestamp_mode == core.TimestampMode.TIME_FROM_INTERNAL_OSC
    assert config.udp_dest == ""
    assert config.udp_port_imu == 7503
    assert config.udp_port_lidar == 7502
    assert config.udp_profile_lidar == core.UDPProfileLidar.PROFILE_LIDAR_LEGACY
    assert config.udp_profile_imu == core.UDPProfileIMU.PROFILE_IMU_LEGACY

    assert config.columns_per_packet == 8

    assert config.extra_options == {"unknown_parameter": '"hello"'}

    # check output of string
    config_output = json.loads((str(config)))
    ground_truth = json.loads(complete_config_string)
    assert len(ground_truth) > 0
    assert len(config_output) > 0

    skip = ["udp_dest"]
    for key in ground_truth:
        if key not in skip:
            assert config_output[key] == ground_truth[key]


def test_equality_config(complete_config_string: str, all_different_config_string: str) -> None:
    """Check equality comparisons."""

    complete_config_1 = core.SensorConfig(complete_config_string)
    complete_config_2 = core.SensorConfig(complete_config_string)
    assert complete_config_1 == complete_config_2

    complete_config_2.multipurpose_io_mode = core.MultipurposeIOMode.MULTIPURPOSE_OUTPUT_FROM_PTP_1588
    assert complete_config_1 != complete_config_2

    partial_config_1 = core.SensorConfig()
    partial_config_1.nmea_baud_rate = core.NMEABaudRate.BAUD_115200
    partial_config_1.operating_mode = core.OperatingMode.OPERATING_STANDBY

    partial_config_2 = core.SensorConfig()
    partial_config_2.nmea_baud_rate = core.NMEABaudRate.BAUD_115200
    partial_config_2.operating_mode = core.OperatingMode.OPERATING_STANDBY
    assert partial_config_1 == partial_config_2

    partial_config_2.operating_mode = core.OperatingMode.OPERATING_NORMAL
    assert partial_config_1 != partial_config_2

    empty_config_1 = core.SensorConfig()
    empty_config_2 = core.SensorConfig()
    assert empty_config_1 == empty_config_2

    assert complete_config_1 != empty_config_1
    assert complete_config_1 != partial_config_1
    assert complete_config_1 != partial_config_2
    assert partial_config_1 != empty_config_1
    assert partial_config_2 != empty_config_1

    config_attributes = inspect.getmembers(core.SensorConfig, lambda a: not inspect.isroutine(a))
    config_properties = [a for a in config_attributes if not a[0].startswith('__')]

    base_config = core.SensorConfig(complete_config_string)
    different_config = core.SensorConfig(all_different_config_string)

    for config_property in config_properties:
        copy_config = deepcopy(base_config)  # reset to initial
        property_name = config_property[0]  # config_property is a tuple of (property_name as string, property)
        if property_name == "udp_profile_imu":
            warnings.warn(UserWarning("Skipping equality check on udp profile IMU while eUDP IMU is not implemented"))
        else:
            property_value = getattr(different_config, property_name)
            setattr(copy_config, property_name, property_value)
            assert copy_config != base_config

    assert len(config_properties) == 28, "Don't forget to update tests and the config == operator!"


def test_copy_config(complete_config_string: str) -> None:
    """Check that copy() works."""
    config = core.SensorConfig(complete_config_string)
    config1 = copy(config)

    assert config1 is not config
    assert config1 == config


def test_parse_config() -> None:
    """Sanity check parsing from json string."""

    with pytest.raises(RuntimeError):
        core.SensorConfig('/')
    with pytest.raises(RuntimeError):
        core.SensorConfig('{ ')


# Note that config.signal_multiplier will also accept values
# that aren't these (but will throw parsing from json or set_config)
# so we don't test the invalid_signal_multiplier_values
@pytest.mark.parametrize("signal_multiplier", valid_signal_multiplier_values)
def test_signal_multiplier(signal_multiplier) -> None:
    """Check that signal multiplier supports all FW values"""

    config = core.SensorConfig()
    config.signal_multiplier = signal_multiplier


def signal_multiplier_config_json(signal_multiplier):
    return '{"signal_multiplier": ' + str(signal_multiplier) + '}'


@pytest.mark.parametrize("signal_multiplier", valid_signal_multiplier_values)
def test_config_valid_signal_multiplier_parse(signal_multiplier) -> None:
    """Check parsing valid signal multiplier from json"""

    core.SensorConfig(signal_multiplier_config_json(signal_multiplier))


@pytest.mark.parametrize("signal_multiplier", invalid_signal_multiplier_values)
def test_config_invalid_signal_multiplier_parse(signal_multiplier) -> None:
    """Check parsing invalid signal multiplier from json"""

    with pytest.raises(RuntimeError):
        core.SensorConfig(signal_multiplier_config_json(signal_multiplier))


@pytest.mark.parametrize("signal_multiplier", valid_signal_multiplier_values)
def test_config_valid_signal_multiplier_to_str(signal_multiplier) -> None:
    """Check writing valid signal multiplier to string"""

    config = core.SensorConfig()
    config.signal_multiplier = signal_multiplier
    str(config)


@pytest.mark.parametrize("signal_multiplier", invalid_signal_multiplier_values)
def test_config_invalid_signal_multiplier_to_str(signal_multiplier) -> None:
    """Check writing invalid signal multiplier to string"""

    config = core.SensorConfig()
    # note: won't error out here bc it's just a double
    config.signal_multiplier = signal_multiplier
    with pytest.raises(RuntimeError):
        str(config)


def test_extra_options() -> None:
    """Test that extra_options is parsed and serialized correctly"""
    config = core.SensorConfig()
    config.signal_multiplier = 1
    config.extra_options = {"hi": '"hello"', "hi2": "2", "hi3": '{"hi":"hi"}'}

    assert config.extra_options == {"hi": '"hello"', "hi2": "2", "hi3": '{"hi":"hi"}'}
    assert str(config) == '{"hi":"hello","hi2":2,"hi3":{"hi":"hi"},"signal_multiplier":1}'

    copy = core.SensorConfig(str(config))

    assert copy == config
    assert str(copy) == str(config)

    # make sure that invalid json throws
    config.extra_options = {"hi": "hello"}
    with pytest.raises(RuntimeError):
        str(config)


@pytest.fixture()
def deprecated_params_config() -> core.SensorConfig:
    deprecated_params = """ { "udp_ip": "169.254.148.183", "auto_start_flag": 1 }"""
    return core.SensorConfig(deprecated_params)


def test_deprecated_config(deprecated_params_config) -> None:
    """Check that deprecated params are properly translated."""
    print(deprecated_params_config)
    assert deprecated_params_config.udp_dest == "169.254.148.183"
    assert deprecated_params_config.operating_mode == core.OperatingMode.OPERATING_NORMAL
