from typing import cast, Iterator, Dict, Optional, List, Tuple, Union

from more_itertools import ilen
from ouster.sdk import client
from ouster.sdk.client import LidarScan, SensorInfo, first_valid_packet_ts
from ouster.sdk.client import ScanSource, MultiScanSource

from ouster.sdk._bindings.osf import (Reader, Writer, MessageRef, LidarSensor,
                                 Extrinsics, LidarScanStream, StreamingInfo)

from ouster.sdk.client.multi import collate_scans       # type: ignore
from ouster.sdk.util import ForwardSlicer, progressbar    # type: ignore


class OsfScanSource(MultiScanSource):
    """Implements MultiScanSource protocol using OSF Reader with multiple sensors."""

    def __init__(
        self,
        file_path: str,
        *,
        dt: int = 210000000,
        complete: bool = False,
        index: bool = False,
        cycle: bool = False,
        field_names: Optional[List[str]] = None,
        **kwargs
    ) -> None:
        """
        Args:
            file_path: OSF file path to open as a scan source
            dt: max time difference between scans in the collated scan (i.e.
                max time period at which every new collated scan is released/cut),
                default is 0.21s
            complete: set to True to only release complete scans (not implemnted)
            index: if this flag is set to true an index will be built for the osf
                file enabling len, index and slice operations on the scan source, if
                the flag is set to False indexing is skipped (default is False).
            cycle: repeat infinitely after iteration is finished (default is False)
            field_names: list of fields to decode into a LidarScan, if not provided
                decodes all fields
        Remarks:
            In case the OSF file didn't have builtin-index and the index flag was
            was set to True the object will attempt to index the file in place.
        """

        self._complete = complete
        self._indexed = index

        if 'meta' in kwargs and kwargs['meta']:
            raise TypeError(
                f"{OsfScanSource.__name__} does not support user-supplied metadata.")

        self._reader = Reader(file_path)
        has_index = self._reader.has_message_idx and self._reader.has_timestamp_idx
        if not has_index:
            if index:
                print("OSF file not indexed! re-indexing file inplace...")
                try:
                    self._reindex_osf_inplace(self._reader, file_path)
                except RuntimeError as e:
                    print(f"Failed re-indexing OSF file!\n more details: {e}")
                    self._indexed = False
                self._reader = Reader(file_path)
                has_index = True

        self._cycle = cycle
        self._dt = dt

        self._desired_fields = field_names or []

        self._sensors = [(sid, sm) for sid, sm in self._reader.meta_store.find(
            LidarSensor).items()]

        # map stream_id to metadata entry
        self._sensor_idx: Dict[int, int]
        self._sensor_idx = {
            sid: sidx
            for sidx, (sid, _) in enumerate(self._sensors)
        }

        # load stored extrinsics (if any)
        self._metadatas = [sm.info for _, sm in self._sensors]
        extrinsics = self._reader.meta_store.find(Extrinsics)
        for _, v in extrinsics.items():
            if v.ref_meta_id in self._sensor_idx:
                sidx = self._sensor_idx[v.ref_meta_id]
                print(f"OSF: stored extrinsics for sensor[{sidx}]:\n",
                      v.extrinsics)
                self._metadatas[sidx].extrinsic = v.extrinsics

        # map stream_id to metadata entry
        self._stream_sensor_idx: Dict[int, int]
        self._stream_sensor_idx = {}
        self._stream_ids = []
        for stream_id, stream_meta in self._reader.meta_store.find(LidarScanStream).items():
            self._stream_sensor_idx[stream_id] = self._sensor_idx[
                stream_meta.sensor_meta_id]
            self._stream_ids.append(stream_id)

        # extract necessary values from the index/stats to calculate lengths
        self._scans_num: List[Optional[int]] = [None] * len(self._stream_ids)
        self._times = []  # a list of all scans and their times/indexes
        for stream_id, stream_meta in self._reader.meta_store.find(StreamingInfo).items():
            for id, stats in stream_meta.stream_stats:
                if id in self._stream_ids:
                    sensor_index = self._stream_ids.index(id)
                    self._scans_num[sensor_index] = stats.message_count
                    rts = stats.receive_timestamps
                    for t in rts:
                        self._times.append((sensor_index, t))

        # sort the list of scan times so that we can collate them below
        self._times.sort(key=lambda x: x[1])

        # get the first scan of each to get field types
        self._field_types = []
        self._fields = []
        for sid, mid in enumerate(self._stream_ids):
            ts_start = self._reader.ts_by_message_idx(mid, 0)
            if ts_start is None:
                # There are no messages in this stream,
                # so there are no fields for this stream.
                self._field_types.append([])
                self._fields.append([])
                continue
            for idx, scan in self._scans_iter(ts_start, ts_start, False):
                if idx == sid:
                    fts = scan.field_types
                    self._field_types.append(fts)
                    l = []
                    for ft in fts:
                        l.append(ft.name)
                    self._fields.append(l)
                    break

        if has_index:
            self._len = ilen(collate_scans(self._times, self.sensors_count, lambda msg: msg, dt=self._dt))
            self._indexed = True

    def _osf_convert(self, reader: Reader, output: str) -> None:
        # TODO: figure out how to get the current chunk_size
        chunk_size = 0
        progressbar(0, 1, "", "indexed")
        writer = Writer(output, chunk_size)
        writer.set_metadata_id(reader.metadata_id)
        for _, m in reader.meta_store.items():
            if m.of(StreamingInfo):
                # StreamingInfo is always generated by Writer automatically in
                # default STREAMING chunks layout, so we don't copy the original
                continue
            writer.add_metadata(m)
        # convert
        if not reader.has_stream_info:
            writer.close()
            raise Exception("Standard Message Layout No Longer Supported")
        msgs = reader.messages()
        msgs_count = ilen(msgs)
        msgs = reader.messages()
        for idx, msg in enumerate(msgs):
            # retrieve sensor_ts where possible (for lidar data)
            sensor_ts = 0
            if msg.of(LidarScanStream):
                sensor_ts = msg.decode().get_first_valid_column_timestamp()
            writer.save_message(msg.id, msg.ts, sensor_ts, msg.buffer)
            progressbar(idx, msgs_count, "", "indexed")
        print("\nfinished building index")
        writer.close()

    def _reindex_osf_inplace(self, reader, osf_file):
        import tempfile
        with tempfile.NamedTemporaryFile(delete=True) as f:
            self._osf_convert(reader, f.name)
            try:
                import shutil
                shutil.copy2(f.name, osf_file)
            except OSError as e:
                raise RuntimeError(f"Error overwriting osf file: {osf_file}"
                                   f"\nmore details: {e}")

    def _msgs_iter_stream(self, stream_id: int, start_ts: int, stop_ts: int
                          ) -> Iterator[MessageRef]:
        for _, msg in self._msgs_iter([stream_id], start_ts, stop_ts, False):
            yield msg

    def _msgs_iter(self, stream_ids: List[int], start_ts: int, stop_ts: int, cycle: bool
                   ) -> Iterator[Tuple[int, MessageRef]]:
        while True:
            had_message = False
            for msg in self._reader.messages(stream_ids, start_ts, stop_ts):
                if msg.of(LidarScanStream):
                    sidx = self._stream_sensor_idx[msg.id]
                    had_message = True
                    yield sidx, msg
            # exit if we had no messages to prevent an infinite loop
            if not cycle or not had_message:
                break

    def _scans_iter(self, start_ts: int, stop_ts: int, cycle: bool
                    ) -> Iterator[Tuple[int, LidarScan]]:
        for idx, msg in self._msgs_iter(self._stream_ids, start_ts, stop_ts, cycle):
            ls = msg.decode(self._desired_fields)
            if ls:
                window = self.metadata[idx].format.column_window
                scan = cast(LidarScan, ls)
                if not self._complete or scan.complete(window):
                    yield idx, scan

    @property
    def sensors_count(self) -> int:
        return len(self._stream_ids)

    @property
    def metadata(self) -> List[SensorInfo]:
        return self._metadatas

    @property
    def is_live(self) -> bool:
        return False

    @property
    def is_seekable(self) -> bool:
        return True

    @property
    def is_indexed(self) -> bool:
        return self._indexed

    @property
    def fields(self) -> List[List[str]]:
        return self._fields

    @property
    def field_types(self) -> List[client.FieldTypes]:
        return self._field_types

    @property
    def scans_num(self) -> List[Optional[int]]:
        return self._scans_num  # type: ignore

    def __len__(self) -> int:
        if not self.is_indexed:
            raise TypeError("len is not supported on non-indexed source")
        return self._len

    def __iter__(self) -> Iterator[List[Optional[LidarScan]]]:
        msgs_itr = self._scans_iter(
            self._reader.start_ts, self._reader.end_ts, self._cycle)
        return collate_scans(msgs_itr, self.sensors_count, first_valid_packet_ts, dt=self._dt)

    def _seek(self, key: int) -> None:
        """seek/jump to a specific item within the list of LidarScan objects that this particular scan
        source has access to"""
        ...

    def __getitem__(self, key: Union[int, slice]
                    ) -> Union[List[Optional[LidarScan]], MultiScanSource]:

        if not self.is_indexed:
            raise TypeError(
                "can not invoke __getitem__ on non-indexed source")

        scans_itr: Iterator[Tuple[int, LidarScan]]

        if isinstance(key, int):
            L = len(self)
            if key < 0:
                key += L
            if key < 0 or key >= L:
                raise IndexError("index is out of range")
            ts = [self._reader.ts_by_message_idx(
                mid, key) for mid in self._stream_ids]
            ts_start = min(ts)
            ts_stop = min(ts_start + self._dt, max(ts))
            scans_itr = self._scans_iter(ts_start, ts_stop, False)
            return next(collate_scans(scans_itr, self.sensors_count,
                                      first_valid_packet_ts, dt=self._dt))

        if isinstance(key, slice):
            return self.slice(key)

        raise TypeError(
            f"indices must be integer or slice, not {type(key).__name__}")

    def close(self) -> None:
        """Close osf file."""
        # TODO[pb]: Need to add Reader.close() method, because now it's
        # all happens in dtor, which is not very clear by lifecycle.
        if self._reader:
            del self._reader
            self._reader = None     # type: ignore

    def __del__(self) -> None:
        """Automatic release of any underlying resource."""
        # self.close() # TODO: currently this causes an exception, avoid
        pass

    def single_source(self, stream_idx: int) -> ScanSource:
        from ouster.sdk.client.scan_source_adapter import ScanSourceAdapter
        return ScanSourceAdapter(self, stream_idx)

    def _slice_iter(self, key: slice) -> Iterator[List[Optional[LidarScan]]]:
        # NOTE: In this method if key.step was negative, this won't be
        # result in the output being reversed, it is the responisbility of
        # the caller to accumulate the results into a vector then return them.
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        count = k.stop - k.start
        if count <= 0:
            return iter(())
        ts_start = min([self._reader.ts_by_message_idx(mid, k.start)
                        for mid in self._stream_ids])
        ts_stop = max([self._reader.ts_by_message_idx(mid, k.stop - 1)
                       for mid in self._stream_ids])
        scans_itr = collate_scans(self._scans_iter(ts_start, ts_stop, False),
                                  self.sensors_count, first_valid_packet_ts,
                                  dt=self._dt)
        return ForwardSlicer.slice_iter(scans_itr, k)

    def slice(self, key: slice) -> 'MultiScanSource':
        """Constructs a MultiScanSource matching the specificed slice"""
        from ouster.sdk.client.multi_sliced_scan_source import MultiSlicedScanSource
        L = len(self)
        k = ForwardSlicer.normalize(key, L)
        if k.step < 0:
            raise TypeError("slice() can't work with negative step")
        return MultiSlicedScanSource(self, k)
