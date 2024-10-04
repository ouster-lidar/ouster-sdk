import logging
import pytest

from ouster.sdk import client
from ouster.sdk.client import SensorHttp
from ouster.sdk.sensor.util import _auto_detected_udp_dest

logger = logging.getLogger("HIL")


@pytest.mark.parametrize("udp_dest", ["", "1.1.1.1"]) # blank value is a bit special
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
