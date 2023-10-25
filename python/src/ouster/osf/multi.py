from ouster import client
import ouster.osf as osf

from ouster.sdkx.multi import ScanMultiSource, ScanMultiItem  # type: ignore

from typing import cast, Iterator, Dict, Optional, List


class ScansMultiReader(ScanMultiSource):
    """Make ScanMultiSource from and OSF Reader with multiple sensors."""

    def __init__(
        self,
        reader: osf.Reader,
        *,
        cycle: bool = False,
        start_ts: int = 0,
        sensor_ids: Optional[List[int]] = None
    ) -> None:
        self._reader = reader
        self._cycle = cycle
        self._start_ts = start_ts

        self._sensors = [(sid, sm) for sid, sm in self._reader.meta_store.find(
            osf.LidarSensor).items()]

        # map stream_id to metadata entry
        self._sensor_idx: Dict[int, int]
        self._sensor_idx = {
            sid: sidx
            for sidx, (sid, _) in enumerate(self._sensors)
        }

        # check for Extrinsics
        extrinsics = self._reader.meta_store.find(osf.Extrinsics)
        for _, v in extrinsics.items():
            if v.ref_meta_id in self._sensor_idx:
                sidx = self._sensor_idx[v.ref_meta_id]
                print(f"Found extrinsics for sensor[{sidx}]:\n",
                    v.extrinsics)
                self._sensors[sidx][1].info.extrinsic = v.extrinsics

        self._metadatas = [sm.info for _, sm in self._sensors]
        self._metadatas_json = [sm.metadata for _, sm in self._sensors]

        # map stream_id to metadata entry
        self._stream_sensor_idx: Dict[int, int]
        self._stream_sensor_idx = {}
        for stream_type in [osf.LidarScanStream]:
            for stream_id, stream_meta in reader.meta_store.find(
                    stream_type).items():
                self._stream_sensor_idx[stream_id] = self._sensor_idx[
                    stream_meta.sensor_meta_id]

    def __iter__(self) -> Iterator[ScanMultiItem]:
        while True:
            for msg in self._reader.messages(self._start_ts,
                                             self._reader.end_ts):
                if msg.of(osf.LidarScanStream):
                    sidx = self._stream_sensor_idx[msg.id]
                    ls = msg.decode()
                    if ls:
                        yield sidx, cast(client.LidarScan, ls)
            if not self._cycle:
                break

    @property
    def metadata(self) -> List[client.SensorInfo]:
        return self._metadatas

    def close(self) -> None:
        """Close osf file."""
        # TODO[pb]: Need to add Reader.close() method, because now it's
        # all happens in dtor, which is not very clear by lifecycle.
        # del self._reader
        pass
