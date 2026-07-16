
import json
from ouster.sdk import core


def test_sensor_info_client_version_write():
    """It adds client_version to SensorInfo JSON."""

    # Create a default SensorInfo
    info = core.SensorInfo.from_default(core.LidarMode._1024x10)

    # Get current client version from core
    expected_version = core.client_version()

    # Serialize to JSON
    json_str = info.to_json_string()
    json_data = json.loads(json_str)

    # Check if client_version is set in JSON
    assert "ouster-sdk" in json_data
    assert "client_version" in json_data["ouster-sdk"]
    assert json_data["ouster-sdk"]["client_version"] == expected_version

    # Parse back to SensorInfo
    info2 = core.SensorInfo(json_str)

    # Ensure client_version was updated
    assert info2.client_version == expected_version


def test_sensor_info_client_version_read():
    """It reads a previously added client_version from SensorInfo JSON."""
    # Create a SensorInfo with a specific client_version, then parse it
    test_version = "ouster_core 0.0.0-test"
    json_data = {
        "ouster-sdk": {
            "client_version": test_version
        },
        "lidar_mode": "1024x10",
        "beam_altitude_angles": [0.0] * 64,
        "beam_azimuth_angles": [0.0] * 64,
    }

    info = core.SensorInfo(json.dumps(json_data))
    assert info.client_version == test_version


def test_sensor_info_client_version_legacy():
    # Test that it can parse legacy top-level client_version
    legacy_json = {
        "client_version": "ouster_core 0.0.0-test",
        "lidar_mode": "1024x10",
        "beam_altitude_angles": [0.0] * 64,
        "beam_azimuth_angles": [0.0] * 64,
        "lidar_to_sensor_transform": [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    }

    info = core.SensorInfo(json.dumps(legacy_json))
    assert info.client_version == "ouster_core 0.0.0-test"
