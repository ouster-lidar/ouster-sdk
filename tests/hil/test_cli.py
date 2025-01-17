import logging
import pytest
import os

from click.testing import CliRunner

from ouster.sdk import client
from ouster.sdk.client import SensorHttp
from ouster.sdk.sensor.util import _auto_detected_udp_dest

from ouster.cli import core
from ouster.cli.core.cli_args import CliArgs
from ouster.cli.plugins import source, source_sensor  # noqa: F401

logger = logging.getLogger("HIL")

@pytest.fixture
def runner():
    return CliRunner()


@pytest.mark.parametrize("udp_dest", [None, "1.1.1.1"]) # blank value is a bit special
def test_udp_dest_finder_leaves_no_mark(hil_sensor_hostname, hil_configured_sensor, udp_dest):

        # set up with test udp value
        cfg = client.SensorConfig()
        cfg.udp_dest = udp_dest
        client.set_config(hil_sensor_hostname, cfg)

        # get post-set
        check_active_cfg = client.get_config(hil_sensor_hostname, active=True)
        check_passive_cfg = client.get_config(hil_sensor_hostname, active=False)

        http = SensorHttp.create(hil_sensor_hostname)
        _auto_detected_udp_dest(http)

        after_active_cfg = client.get_config(hil_sensor_hostname, active=True)
        after_passive_cfg = client.get_config(hil_sensor_hostname, active=False)

        assert check_active_cfg == after_active_cfg
        assert check_passive_cfg == after_passive_cfg


def test_diagnostics(hil_sensor_hostname, tmpdir, runner) -> None:
    """Test that we can record pcaps without dropping packets."""
    dump_path = os.path.join(tmpdir, "test.bin")
    result = runner.invoke(core.cli, ['source', str(hil_sensor_hostname), 'diagnostics', str(dump_path)]) #  type: ignore
    assert result.exit_code == 0
    assert os.path.isfile(str(dump_path))


def test_network(hil_sensor_hostname, runner) -> None:
    """Test that we can record pcaps without dropping packets."""
    result = runner.invoke(core.cli, ['source', str(hil_sensor_hostname), 'network']) #  type: ignore
    assert result.exit_code == 0
    assert "ipv4" in result.output
