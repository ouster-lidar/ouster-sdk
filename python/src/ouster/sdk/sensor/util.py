from typing import List, Optional
from copy import copy
import requests
import ouster.sdk.client as client
from packaging import version
from ouster.sdk.util import firmware_version

MIN_AUTO_DEST_FW = version.Version("2.3.1")


def _auto_detected_udp_dest(hostname: str) -> Optional[str]:
    """
    Function which obtains the udp_dest the sensor would choose when automatically detecting
    without changing anything else about sensor state

    Args:
        hostname: sensor hostname
    Returns:
        udp_dest: the udp_dest the sensor detects automatically
    """
    orig_config = client.get_config(hostname, active=True)

    # get what the possible auto udp_dest is
    config_endpoint = f"http://{hostname}/api/v1/sensor/config"
    response = requests.post(config_endpoint, params={'reinit': False, 'persist': False},
                             json={'udp_dest': '@auto'})
    response.raise_for_status()

    # get staged config
    udp_auto_config = client.get_config(hostname, active=False)

    # set staged config back to original
    response = requests.post(config_endpoint, params={'reinit': False, 'persist': False},
                             json={'udp_dest': str(orig_config.udp_dest)})
    response.raise_for_status()

    return udp_auto_config.udp_dest


def configure_sensor(hostname: str,
                     lidar_port: Optional[int] = None,
                     imu_port: Optional[int] = None,
                     do_not_reinitialize: bool = False,
                     no_auto_udp_dest: bool = False) -> client.SensorConfig:
    """Depending on the args do_not_reinitialize, no_auto_udp_dest,
    possibly reconfigure the sensor. Then, return the configuration that is used."""

    print(f"Contacting sensor {hostname}...")

    fw_version = firmware_version(hostname)

    auto_config_udp_dest = None
    use_set_config_auto = False

    # original config
    orig_config = client.get_config(hostname, active=True)

    if fw_version >= MIN_AUTO_DEST_FW:
        auto_config_udp_dest = _auto_detected_udp_dest(hostname)
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

        if lidar_port and orig_config.udp_port_lidar != lidar_port:
            raise RuntimeError(
                f"Sensor's lidar port {orig_config.udp_port_lidar} does "
                f"not match provided lidar port but you have disallowed "
                f"reinitialization. Drop -x to allow reinitialization or "
                f"change your specified lidar_port {lidar_port}")
        return orig_config

    new_config = copy(orig_config)

    lidar_port_change = (lidar_port and
                         orig_config.udp_port_lidar != lidar_port)
    imu_port_change = (imu_port and orig_config.udp_port_imu != imu_port)
    port_changes = []
    if (lidar_port_change or imu_port_change):
        new_config.udp_port_lidar = lidar_port or orig_config.udp_port_lidar
        new_config.udp_port_imu = imu_port or orig_config.udp_port_imu

        if lidar_port_change:
            port_changes.append(f"lidar port from {orig_config.udp_port_lidar} "
                                f"to {new_config.udp_port_lidar}")

        if imu_port_change:
            port_changes.append(f"imu port from {orig_config.udp_port_imu} "
                                f"to {new_config.udp_port_imu}")

        port_changes_str = " and ".join(port_changes)
        print(f"Will change {port_changes_str} ...")

    if not no_auto_udp_dest and auto_config_udp_dest and orig_config.udp_dest != auto_config_udp_dest:
        print((f"Will change udp_dest from '{orig_config.udp_dest}' to automatically "
               f"detected '{auto_config_udp_dest}'..."))
        new_config.udp_dest = auto_config_udp_dest

    if use_set_config_auto:
        print(f"Will change udp_dest from '{orig_config.udp_dest}' to automatically "
              "detected UDP DEST")
        new_config.udp_dest = None

    new_config.operating_mode = client.OperatingMode.OPERATING_NORMAL
    if new_config.operating_mode != orig_config.operating_mode:
        print((f"Will change sensor's operating mode from {orig_config.operating_mode}"
               f" to {new_config.operating_mode}"))

    if orig_config != new_config or use_set_config_auto:
        print("Setting sensor config...")
        client.set_config(hostname, new_config, persist=False,
                          udp_dest_auto=use_set_config_auto)

        new_config = client.get_config(hostname)

    return new_config


def configure_sensor_multi(
        hostnames: List[str],
        first_lidar_port: Optional[int] = None,
        do_not_reinitialize: bool = False,
        no_auto_udp_dest: bool = False) -> List[client.SensorConfig]:
    """Configure multiple sensors by hostnames/ips"""

    first_lidar_port = first_lidar_port or 17502
    configs: List[client.SensorConfig] = []

    for idx, hn in enumerate(hostnames):
        lidar_port = first_lidar_port + idx * 2
        imu_port = lidar_port + 1
        configs.append(
            configure_sensor(hn,
                             lidar_port=lidar_port,
                             imu_port=imu_port,
                             do_not_reinitialize=do_not_reinitialize,
                             no_auto_udp_dest=no_auto_udp_dest))
        print(f"Initializing connection to sensor {hn} on "
              f"lidar port {configs[-1].udp_port_lidar} with udp dest "
              f"'{configs[-1].udp_dest}'...")
    return configs
