import tempfile
import os
from ouster.sdk.core import SensorInfo
import ouster.sdk.examples.zone_monitor_example as zone_monitor_example
from ouster.sdk.zone_monitor import ZoneSet


def test_create_stl_zone_set(test_data_dir):
    """It should create a zone set from STL files without error."""
    for zone_set_create_method in [
        zone_monitor_example.create_stl_zone_set,
        zone_monitor_example.create_zrb_zone_set,
    ]:
        try:
            with tempfile.NamedTemporaryFile(delete=False) as zip_file:
                zone_monitor_dir = os.path.join(test_data_dir, "zone_monitor")
                sensor_info_path = os.path.join(zone_monitor_dir, "785.json")
                test_sensor_info = SensorInfo(open(sensor_info_path).read())
                zone_set_create_method(zone_monitor_dir, zip_file.name, test_sensor_info)
            zone_set = ZoneSet(zip_file.name)
            assert len(zone_set.zones) == 2
        finally:
            try:
                os.remove(zip_file.name)
            except OSError:
                pass
