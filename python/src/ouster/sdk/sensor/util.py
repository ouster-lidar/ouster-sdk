from typing import Optional
from copy import copy
import warnings
import ouster.sdk.core as core
from ouster.sdk.core import (SensorHttp, Version)
from ouster.sdk._bindings.client import in_multicast, SHORT_HTTP_REQUEST_TIMEOUT_SECONDS

MIN_AUTO_DEST_FW = Version.from_string("2.3.1")


def build_sensor_config(http_client: SensorHttp,
                        lidar_port: Optional[int] = None,
                        imu_port: Optional[int] = None,
                        do_not_reinitialize: bool = False,
                        no_auto_udp_dest: bool = False) -> core.SensorConfig:
    """
    Depending on the args do_not_reinitialize, and no_auto_udp_dest
    determine a configuration for the sensor
    """
    warnings.warn("build_sensor_config is deprecated: manually build the configuration or "
                  "use the SensorScanSource instead. "
                  "build_sensor_config will be removed in the upcoming release.",
                  DeprecationWarning, stacklevel=2)

    fw_version = http_client.firmware_version()

    auto_config_udp_dest = None
    use_set_config_auto = False

    orig_config = core.SensorConfig(http_client.get_config_params(True))

    if orig_config.udp_dest is None or not in_multicast(orig_config.udp_dest):
        # don't change the destination if it is multicast
        pass
    elif fw_version >= MIN_AUTO_DEST_FW:
        auto_config_udp_dest = http_client.auto_detected_udp_dest(SHORT_HTTP_REQUEST_TIMEOUT_SECONDS,
                                                                  orig_config.udp_dest)
        if orig_config.udp_dest != auto_config_udp_dest:
            if no_auto_udp_dest or do_not_reinitialize:
                print(f"WARNING: Your sensor's udp destination {orig_config.udp_dest} does "
                      f"not match the detected udp destination {auto_config_udp_dest}. "
                      f"If you get a Timeout error, drop -x and -y from your "
                      f"arguments to allow automatic udp_dest setting.")
    else:
        if no_auto_udp_dest or do_not_reinitialize:
            print("WARNING: You have opted not to allow us to reset your auto UDP dest "
                  "by using either -x or -y. If you get a Timeout error, drop -x and -y "
                  "from  your arguments to allow automatic udp_dest setting.")
        else:
            use_set_config_auto = True

    if do_not_reinitialize:
        if orig_config.operating_mode == core.OperatingMode.OPERATING_STANDBY:
            raise RuntimeError("Your sensor is in STANDBY mode but you have disallowed "
                               "reinitialization. Drop -x to allow reinitialization or "
                               "change your sensor's operating mode.")

        if lidar_port is not None and orig_config.udp_port_lidar != lidar_port:
            raise RuntimeError(
                f"Sensor's lidar port {orig_config.udp_port_lidar} does "
                f"not match provided lidar port but you have disallowed "
                f"reinitialization. Drop -x to allow reinitialization or "
                f"change your specified lidar_port {lidar_port}")
        return orig_config

    new_config = copy(orig_config)
    lidar_port_change = ((lidar_port is not None) and
                         orig_config.udp_port_lidar != lidar_port)
    imu_port_change = ((imu_port is not None) and orig_config.udp_port_imu != imu_port)
    port_changes = []
    if (lidar_port_change or imu_port_change):
        new_config.udp_port_lidar = lidar_port if lidar_port is not None else orig_config.udp_port_lidar
        new_config.udp_port_imu = imu_port if imu_port is not None else orig_config.udp_port_imu

        if lidar_port_change:
            new_port_name = "ephemeral" if new_config.udp_port_lidar == 0 else new_config.udp_port_lidar
            port_changes.append(f"lidar port from {orig_config.udp_port_lidar} "
                                f"to {new_port_name}")

        if imu_port_change:
            new_port_name = "ephemeral" if new_config.udp_port_imu == 0 else new_config.udp_port_imu
            port_changes.append(f"imu port from {orig_config.udp_port_imu} "
                                f"to {new_port_name}")

        port_changes_str = " and ".join(port_changes)
        print(f"Will change {port_changes_str} ...")

    if not no_auto_udp_dest and auto_config_udp_dest and orig_config.udp_dest != auto_config_udp_dest:
        print((f"Will change udp_dest from '{orig_config.udp_dest}' to automatically "
               f"detected '{auto_config_udp_dest}'..."))
        new_config.udp_dest = auto_config_udp_dest

    if use_set_config_auto:
        print(f"Will change udp_dest from '{orig_config.udp_dest}' to automatically "
              "detected UDP DEST")
        new_config.udp_dest = "@auto"

    new_config.operating_mode = core.OperatingMode.OPERATING_NORMAL
    if new_config.operating_mode != orig_config.operating_mode:
        print((f"Will change sensor's operating mode from {orig_config.operating_mode}"
               f" to {new_config.operating_mode}"))

    return new_config
