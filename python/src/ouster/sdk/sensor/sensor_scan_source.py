from typing import List, Optional, Union

import numpy as np
import ouster.sdk.client as client
from ouster.sdk.client import ScansMulti            # type: ignore
from ouster.sdk.util import default_scan_fields     # type: ignore
from .util import configure_sensor
# TODO: from .sensor_multi_packet_reader import SensorMultiPacketReader


class SensorScanSource(ScansMulti):
    """Implements MultiScanSource protocol for live sensors, multiple sensors isn't supported yet."""

    def __init__(
        self,
        hostnames: Union[str, List[str]],
        *,
        lidar_port: int = 7502,
        imu_port: int = 7503,
        complete: bool = False,
        soft_id_check: bool = False,
        do_not_reinitialize: bool = False,
        no_auto_udp_dest: bool = False,
        buf_size: int = 128,
        timeout: float = 1.0,
        extrinsics: Optional[List[float]] = None,
        flags: bool = True,
        **kwargs
    ) -> None:
        """
        Args:
            hostnames: sensor hostname urls or IPs.
            complete: set to True to only release complete scans.
            flags: when this option is set, the FLAGS field will be added to the list
                of fields of every scan, in case of dual returns FLAGS2 will also be
                appended (default is True).
        """

        self._source = None

        if isinstance(hostnames, str):
            hostnames = [hostnames]
        elif len(hostnames) > 1:
            raise NotImplementedError("multi sensor is not implemented")

        if 'meta' in kwargs and kwargs['meta']:
            raise TypeError(
                f"{SensorScanSource.__name__} does not support user-supplied metadata.")

        config = configure_sensor(hostnames[0],
                                  lidar_port,
                                  imu_port,
                                  do_not_reinitialize=do_not_reinitialize,
                                  no_auto_udp_dest=no_auto_udp_dest)

        print(f"Initializing connection to sensor {hostnames[0]} on "
              f"lidar port {config.udp_port_lidar} with udp dest '{config.udp_dest}'...")

        # make 0 timeout in the cli mean no timeout
        timeout_ = timeout if timeout > 0 else None

        lidar_port = config.udp_port_lidar if config.udp_port_lidar else 7502
        imu_port = config.udp_port_imu if config.udp_port_imu else 7503

        self._source = client.Sensor(hostnames[0],
                                     lidar_port,
                                     imu_port,
                                     buf_size=buf_size,
                                     timeout=timeout_,
                                     soft_id_check=soft_id_check)

        # enable parsing flags field
        # TODO: try to switch to using the resolve_field_types
        self._fields = default_scan_fields(self._source.metadata.format.udp_profile_lidar,
                                           flags=flags)

        self._scans = client.Scans(self._source,
                                   timeout=timeout_,
                                   complete=complete,
                                   fields=self._fields,
                                   _max_latency=2)

        if extrinsics:
            self._scans.metadata.extrinsic = np.array(
                extrinsics).reshape((4, 4))
            print(
                f"Using sensor extrinsics:\n{self._scans.metadata.extrinsic}")

    # NOTE: the following properties have been adapted to the multi sensor case
    # using the single client.Scans inteface.
    @property
    def sensors_count(self) -> int:
        return 1

    @property
    def metadata(self) -> List[client.SensorInfo]:
        return [self._source.metadata]  # type: ignore

    @property
    def is_live(self) -> bool:
        return True

    @property
    def is_seekable(self) -> bool:
        return False

    @property
    def is_indexed(self) -> bool:
        return False

    @property
    def fields(self) -> List[client.FieldTypes]:
        return [self._fields]

    def __iter__(self):

        def encompass(it):
            for x in it:
                yield [x]

        return encompass(self._scans)

    def close(self):
        if self._source:
            self._source.close()
