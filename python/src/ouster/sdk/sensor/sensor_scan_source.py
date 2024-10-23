from typing import List, Optional, Union, Iterator

import time
import numpy as np
import ouster.sdk.client as client
from ouster.sdk.client import (ScansMulti, SensorHttp, _SensorScanSource, _Sensor, MultiScanSource,
                               first_valid_packet_ts, last_valid_packet_ts, ClientTimeout, LidarScan)  # type: ignore
from ouster.sdk.util import (resolve_field_types)     # type: ignore
from .util import build_sensor_config
from ouster.sdk.client.multi import collate_scans


class SensorScanSource(ScansMulti):
    """Implements MultiScanSource protocol for multiple live sensors."""

    def __init__(
        self,
        hostnames: Union[str, List[str]],
        *,
        dt: int = 210000000,
        lidar_port: Optional[int] = None,
        imu_port: Optional[int] = None,
        complete: bool = False,
        raw_headers: bool = False,
        raw_fields: bool = False,
        soft_id_check: bool = False,
        do_not_reinitialize: bool = False,
        no_auto_udp_dest: bool = False,
        timeout: float = 1.0,
        extrinsics: Optional[List[Optional[List[float]]]] = None,
        field_names: Optional[List[str]] = None,
        **kwargs
    ) -> None:
        """
        Args:
            hostnames: sensor hostname urls or IPs.
            dt: max time difference between scans in the collated scan (i.e.
                max time period at which every new collated scan is released/cut),
                default is 0.21s
            complete: set to True to only release complete scans.
            raw_headers: if True, include raw headers in decoded LidarScans
            raw_fields: if True, include raw fields in decoded LidarScans
            extrinsics: list of extrinsincs to apply to each sensor
            field_names: list of fields to decode into a LidarScan, if not provided
                decodes all fields
        """

        if isinstance(hostnames, str):
            hostnames = [hostnames]

        if 'meta' in kwargs and kwargs['meta']:
            raise TypeError(
                f"{SensorScanSource.__name__} does not support user-supplied metadata.")

        # make 0 timeout in the cli mean no timeout
        self._timeout = int(timeout * 1e9) if timeout > 0 else None
        self._hostnames = hostnames
        self._complete = complete
        self._running = True

        s_list = []
        mode_metadata = []
        for hostname in hostnames:
            print(f"Contacting sensor {hostname}...")
            sensor_http = SensorHttp.create(hostname, client.LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
            config = build_sensor_config(sensor_http,
                                      lidar_port,
                                      imu_port,
                                      do_not_reinitialize=do_not_reinitialize,
                                      no_auto_udp_dest=no_auto_udp_dest)
            if config.udp_port_lidar == 0:
                print(f"Initializing connection to sensor {hostname} on an ephemeral "
                      f"lidar port with udp dest '{config.udp_dest}'...")
            else:
                print(f"Initializing connection to sensor {hostname} on "
                      f"lidar port {config.udp_port_lidar} with udp dest '{config.udp_dest}'...")

            sensor = _Sensor(hostname, config)
            mode_metadata.append(sensor.fetch_metadata())
            s_list.append(_Sensor(hostname, config))

        self._field_types = []
        self._fields = []
        raw_fields |= (field_names is not None and len(field_names) != 0)
        for m in mode_metadata:
            ft = resolve_field_types(m,
                                     raw_headers=raw_headers,
                                     raw_fields=raw_fields)
            real_ft = []
            fnames = []
            for f in ft:
                if field_names is not None and f.name not in field_names:
                    continue
                fnames.append(f.name)
                real_ft.append(f)
            if field_names is not None:
                for f in field_names:
                    if f not in fnames:
                        raise RuntimeError(f"Requested field '{f}' does not exist in packet format"
                                           f" {m.format.udp_profile_lidar}")
            self._fields.append(fnames)
            self._field_types.append(real_ft)

        self._cli = _SensorScanSource(
            s_list, [],
            config_timeout=client.LONG_HTTP_REQUEST_TIMEOUT_SECONDS, queue_size=2,
            soft_id_check=soft_id_check, fields=self._field_types
        )

        self._metadata = self._cli.get_sensor_info()
        self._dt = dt

        self._last_receive_times = [time.time() * 1e9] * len(self.fields)

        if extrinsics:
            for i, e in enumerate(extrinsics):
                if e is None:
                    continue

                self._metadata[i].extrinsic = np.array(e).reshape((4, 4))
                print(
                    f"Using sensor extrinsics:\n{self._metadata[i].extrinsic}")

    @property
    def sensors_count(self) -> int:
        return len(self._metadata)

    @property
    def metadata(self) -> List[client.SensorInfo]:
        return self._metadata

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
    def id_error_count(self) -> int:
        return self._cli.id_error_count()

    @property
    def dropped_scans(self) -> int:
        return self._cli.dropped_scans()

    @property
    def field_types(self) -> List[client.FieldTypes]:
        return self._field_types

    @property
    def fields(self) -> List[List[str]]:
        return self._fields

    def _scans_iter(self):
        while self._running:
            idx, scan = self._cli.get_scan()

            # check for timeouts if enabled
            if self._timeout is not None:
                now = int(last_valid_packet_ts(scan)) if scan is not None else int(time.time() * 1e9)
                if scan is not None:
                    self._last_receive_times[idx] = now
                for i, t in enumerate(self._last_receive_times):
                    age = now - t
                    if age > self._timeout:
                        metadata = self._metadata[i]
                        raise ClientTimeout(f"No valid scans received within {self._timeout/1e9}s from sensor "
                                            f"{self._hostnames[i]} using udp destination {metadata.config.udp_dest} "
                                            f"on port {metadata.config.udp_port_lidar}.")

            # the scan will be null if getting the next scan times out at 100ms
            if scan is not None:
                if not self._complete or scan.complete(self._metadata[idx].format.column_window):
                    yield (idx, scan)

    def __iter__(self):
        return collate_scans(self._scans_iter(), self.sensors_count,
                             first_valid_packet_ts, dt=self._dt)

    def close(self):
        self._running = False
        if hasattr(self, "_cli") and self._cli:
            self._cli.close()

    def _slice_iter(self, key: slice) -> Iterator[List[Optional[LidarScan]]]:
        raise RuntimeError("cannot invoke _slice_iter on a non-indexed source")

    def slice(self, key: slice) -> 'MultiScanSource':
        """Constructs a MultiScanSource matching the specificed slice"""
        raise RuntimeError("cannot invoke _slice_iter on a non-indexed source")
