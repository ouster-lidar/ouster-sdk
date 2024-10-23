from typing import Optional
from copy import copy
from urllib import request
import json
import ouster.sdk.client as client
from ouster.sdk.client import (SensorHttp, Version)

MIN_AUTO_DEST_FW = Version.from_string("2.3.1")


def _auto_detected_udp_dest(http_client: SensorHttp,
                            current_config: Optional[client.SensorConfig] = None) -> Optional[str]:
    """
    Function which obtains the udp_dest the sensor would choose when automatically detecting
    without changing anything else about sensor state

    Args:
        hostname: sensor hostname
    Returns:
        udp_dest: the udp_dest the sensor detects automatically
    """
    hostname = http_client.hostname()
    orig_config = current_config or client.SensorConfig(http_client.get_config_params(True))

    # escape ipv6 addresses
    if hostname.count(':') >= 2:
        hostname = "[" + hostname + "]"

    # get what the possible auto udp_dest is
    config_endpoint = f"http://{hostname}/api/v1/sensor/config"
    req = request.Request(config_endpoint + "?reinit=False&persist=False", method="POST")
    req.add_header('Content-Type', 'application/json')
    data = json.dumps({'udp_dest': '@auto'})
    request.urlopen(req, data = data.encode()).read()

    # get staged config
    udp_auto_config = client.SensorConfig(http_client.get_config_params(False))

    req = request.Request(config_endpoint + "?reinit=False&persist=False", method="POST")
    req.add_header('Content-Type', 'application/json')
    data = json.dumps({'udp_dest': str(orig_config.udp_dest)})
    request.urlopen(req, data = data.encode()).read()

    return udp_auto_config.udp_dest


def build_sensor_config(http_client: SensorHttp,
                        lidar_port: Optional[int] = None,
                        imu_port: Optional[int] = None,
                        do_not_reinitialize: bool = False,
                        no_auto_udp_dest: bool = False) -> client.SensorConfig:
    """
    Depending on the args do_not_reinitialize, and no_auto_udp_dest
    determine a configuration for the sensor
    """

    fw_version = http_client.firmware_version()

    auto_config_udp_dest = None
    use_set_config_auto = False

    orig_config = client.SensorConfig(http_client.get_config_params(True))

    if fw_version >= MIN_AUTO_DEST_FW:
        auto_config_udp_dest = _auto_detected_udp_dest(http_client, orig_config)
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
        if orig_config.operating_mode == client.OperatingMode.OPERATING_STANDBY:
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

    new_config.operating_mode = client.OperatingMode.OPERATING_NORMAL
    if new_config.operating_mode != orig_config.operating_mode:
        print((f"Will change sensor's operating mode from {orig_config.operating_mode}"
               f" to {new_config.operating_mode}"))

    return new_config
