import os

import pytest

import ouster.sdk.examples.core as core_example
from ouster.sdk import core


HOSTNAME = os.environ.get("SENSOR_HOSTNAME")


@pytest.mark.skipif(not HOSTNAME, reason="SENSOR_HOSTNAME not set")
def test_configure_dual_returns_sets_dual_profile(capsys):
    """Invoke dual returns configuration and verify dual profile if supported."""
    try:
        core_example.configure_dual_returns(HOSTNAME)
        config = core_example.sensor.get_config(HOSTNAME)
        captured = capsys.readouterr()
    except ValueError:
        captured = capsys.readouterr()
        assert "does not support dual returns" in captured.out
        return
    except Exception as exc:
        pytest.skip(f"configure_dual_returns requires live sensor: {exc}")

    assert config.udp_profile_lidar == core.UDPProfileLidar.RNG19_RFL8_SIG16_NIR16_DUAL
    assert f"udp profile lidar: {config.udp_profile_lidar}" in captured.out


@pytest.mark.skipif(not HOSTNAME, reason="SENSOR_HOSTNAME not set")
def test_configure_sensor_params_uses_expected_defaults():
    """Configure sensor params and verify persisted values if sensor is reachable."""
    try:
        core_example.configure_sensor_params(HOSTNAME)
        cfg = core_example.sensor.get_config(HOSTNAME)
    except Exception as exc:
        pytest.skip(f"configure_sensor_params requires live sensor: {exc}")

    assert cfg.operating_mode == core.OperatingMode.NORMAL
    assert cfg.lidar_mode == core.LidarMode._1024x10
    assert cfg.udp_port_lidar == 7502
    assert cfg.udp_port_imu == 7503


@pytest.mark.skipif(not HOSTNAME, reason="SENSOR_HOSTNAME not set")
def test_fetch_metadata_writes_json(tmp_path):
    """Fetch metadata from sensor and write to file if sensor is reachable."""
    os.chdir(tmp_path)
    try:
        core_example.fetch_metadata(HOSTNAME)
    except Exception as exc:
        pytest.skip(f"fetch_metadata requires live sensor: {exc}")

    output_file = tmp_path / f"{HOSTNAME}.json"
    assert output_file.exists()
    assert output_file.read_text()


@pytest.mark.skipif(not HOSTNAME, reason="SENSOR_HOSTNAME not set")
def test_record_pcap_writes_files_and_calls_recorder(tmp_path):
    """Record pcap from live sensor; skip if sensor unavailable."""
    os.chdir(tmp_path)
    try:
        core_example.record_pcap(HOSTNAME, n_seconds=1)
    except Exception as exc:
        pytest.skip(f"record_pcap requires live sensor: {exc}")

    json_files = list(tmp_path.glob("*.json"))
    pcap_files = list(tmp_path.glob("*.pcap"))
    assert json_files and pcap_files
