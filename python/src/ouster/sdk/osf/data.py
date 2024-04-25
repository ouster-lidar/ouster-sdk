from typing import List, Optional, Union, cast, Iterator, Tuple

from ouster.sdk import client
from ouster.sdk.client.data import FieldTypes
from ouster.sdk.client import ScanSource
from ouster.sdk.client._client import LidarScan
import ouster.sdk.osf as osf


class Scans(ScanSource):
    """An iterable stream of ``LidarScan`` read from OSF file (for the first available sensor)."""

    def __init__(self,
                 osf_file: str,
                 *,
                 cycle: bool = False,
                 sensor_id: int = 0):
        """
        Args:
            osf_file: OSF filename as scans source
            cycle: repeat infinitely after iteration is finished is True
            sensor_id: id of the sensor which LidarScan stream data to read
            (i.e. id of the metadata entry with ``osf.LidarSensor`` type).
            0 (default) means that first LidarSensor from the OSF is used.
        """
        self._reader = osf.Reader(osf_file)
        self._cycle = cycle
        self._sensor_id = sensor_id

        if self._sensor_id:
            # sensor_id is passed so we can get the sensor metadata
            # entry directly by metadata entry id
            sensor_meta = self._reader.meta_store[self._sensor_id]
            if sensor_meta and sensor_meta.of(osf.LidarSensor):
                self._sensor = sensor_meta
            else:
                raise ValueError(f"Error: Sensor is not found by sensor_id: "
                                 f" {self._sensor_id}")
        else:
            # sensor_id is not provided, so we get the first
            # osf.LidarSensor metadata entry and use its stream
            sensor_meta = self._reader.meta_store.get(osf.LidarSensor)
            if not sensor_meta:
                raise ValueError("Error: No sensors found in OSF file")
            self._sensor = sensor_meta

        # check for Extrinsics
        extrinsics = self._reader.meta_store.find(osf.Extrinsics)
        for _, v in extrinsics.items():
            if v.ref_meta_id == self._sensor.id:
                print(f"Found extrinsics for sensor[{self._sensor.id}]:\n",
                      v.extrinsics)
                self._sensor.info.extrinsic = v.extrinsics

        # Find the corresponding stream_id for the sensor
        scan_streams = self._reader.meta_store.find(osf.LidarScanStream)
        self._sensor_stream_id = next((mid for mid, m in scan_streams.items()
                                       if m.sensor_meta_id == self._sensor.id),
                                      0)
        if not self._sensor_stream_id:
            raise ValueError(f"Error: No LidarScan stream found for sensor"
                             f" id:{self._sensor.id} in an OSF file")

    def __iter__(self) -> Iterator[client.LidarScan]:
        """Iterator that returns ``LidarScan`` objects."""
        for _, ls in self.withTs():
            yield ls

    def withTs(self) -> Iterator[Tuple[int, client.LidarScan]]:
        """Iterator that returns tuple of (``ts``, ``LidarScan``)

        Where ``ts`` - is a timestamp (ns) of a ``LidarScan`` (usually as a
        timestamp of a first packet in a ``LidarScan``)
        """
        while True:
            # TODO[pb]: Read only specified _sensor_stream_id stream
            for msg in self._reader.messages([self._sensor_stream_id],
                                             self._reader.start_ts,
                                             self._reader.end_ts):
                if msg.id == self._sensor_stream_id:
                    scan = msg.decode()
                    if scan:
                        yield msg.ts, cast(client.LidarScan, scan)
            if not self._cycle:
                break

    def close(self) -> None:
        # TODO[pb]: Do the close for Reader?
        pass

    @property
    def metadata(self) -> client.SensorInfo:
        """Return metadata of a Lidar Sensor used."""
        return self._sensor.info

    @property
    def is_live(self) -> bool:
        return False

    @property
    def is_seekable(self) -> bool:
        return False

    @property
    def is_indexed(self) -> bool:
        return False    # TODO: for now we just use False no matter what

    @property
    def fields(self) -> FieldTypes:
        return client.get_field_types(self.metadata)

    @property
    def scans_num(self) -> Optional[int]:
        raise NotImplementedError  # TODO: implement

    def __len__(self) -> int:
        raise NotImplementedError  # TODO: implement

    def _seek(self, key: int) -> None:
        pass

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[Optional[LidarScan], List[Optional[LidarScan]]]:
        raise NotImplementedError

    def __del__(self) -> None:
        pass
