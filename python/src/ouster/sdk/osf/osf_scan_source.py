from typing import cast, Iterator, Dict, Optional, List, Tuple, Union

from more_itertools import ilen
from ouster.sdk import client
from ouster.sdk.client import LidarScan, SensorInfo, first_valid_packet_ts
from ouster.sdk.client import ScanSource, MultiScanSource

from ouster.sdk.osf._osf import (Reader, Writer, MessageRef, LidarSensor,
                                 Extrinsics, LidarScanStream, StreamingInfo)

from ouster.sdk.client.multi import collate_scans       # type: ignore
from ouster.sdk.util import ForwardSlicer, progressbar    # type: ignore


class OsfScanSource(MultiScanSource):
    """Implements MultiScanSource protocol using OSF Reader with multiple sensors."""

    def __init__(
        self,
        file_path: str,
        *,
        dt: int = 10**8,
        complete: bool = False,
        index: bool = False,
        cycle: bool = False,
        flags: bool = True,
        **kwargs
    ) -> None:
        """
        Args:
            file_path: OSF file path to open as a scan source
            dt: max time difference between scans in the collated scan (i.e.
                time period at which every new collated scan is released/cut),
                default is 0.1s
            complete: set to True to only release complete scans (not implemnted)
            index: if this flag is set to true an index will be built for the osf
                file enabling len, index and slice operations on the scan source, if
                the flag is set to False indexing is skipped (default is False).
            cycle: repeat infinitely after iteration is finished (default is False)
            flags: when this option is set, the FLAGS field will be added to the list
                of fields of every scan, in case of dual returns FLAGS2 will also be
                 appended (default is True).

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

        if not self._reader.has_message_idx:
            if index:
                print("OSF file not indexed! re-indexing file inplace...")
                try:
                    self._reindex_osf_inplace(self._reader, file_path)
                except RuntimeError as e:
                    print(f"Failed re-indexing OSF file!\n more details: {e}")
                    self._indexed = False
                self._reader = Reader(file_path)
            else:
                print("OSF file not indexed, indexing not requested!")

        self._cycle = cycle
        self._dt = dt

        self._sensors = [(sid, sm) for sid, sm in self._reader.meta_store.find(
            LidarSensor).items()]

        # map stream_id to metadata entry
        self._sensor_idx: Dict[int, int]
        self._sensor_idx = {
            sid: sidx
            for sidx, (sid, _) in enumerate(self._sensors)
        }

        # load stored extrinsics (if any)
        extrinsics = self._reader.meta_store.find(Extrinsics)
        for _, v in extrinsics.items():
            if v.ref_meta_id in self._sensor_idx:
                sidx = self._sensor_idx[v.ref_meta_id]
                print(f"OSF: stored extrinsics for sensor[{sidx}]:\n",
                      v.extrinsics)
                self._sensors[sidx][1].info.extrinsic = v.extrinsics

        self._metadatas = [sm.info for _, sm in self._sensors]

        # map stream_id to metadata entry
        self._stream_sensor_idx: Dict[int, int]
        self._stream_sensor_idx = {}
        for stream_type in [LidarScanStream]:
            for stream_id, stream_meta in self._reader.meta_store.find(
                    stream_type).items():
                self._stream_sensor_idx[stream_id] = self._sensor_idx[
                    stream_meta.sensor_meta_id]

        def append_flags(ftypes: Dict, flags: bool) -> Dict:
            import numpy as np
            if flags:
                ftypes.update({client.ChanField.FLAGS: np.uint8})
                if client.ChanField.RANGE2 in ftypes:
                    ftypes.update({client.ChanField.FLAGS2: np.uint8})
            return ftypes

        scan_streams = self._reader.meta_store.find(LidarScanStream)
        self._stream_ids = [mid for mid, _ in scan_streams.items()]
        self._fields = [append_flags(lss.field_types, flags)
                        for _, lss in scan_streams.items()]
        # TODO: the following two properties (_scans_num, _len) are computed on
        # load but should rather be provided directly through OSF API. Obtain
        # these values directly from OSF API once implemented.
        if self._indexed:
            start_ts = self._reader.start_ts
            end_ts = self._reader.end_ts
            self._scans_num = [ilen(self._msgs_iter_stream(
                mid, start_ts, end_ts)) for mid in self._stream_ids]
            self._len = ilen(collate_scans(self._msgs_iter(
                self._stream_ids, start_ts, end_ts, False),
                self.sensors_count, lambda msg: cast(MessageRef, msg).ts, dt=self._dt))

    def _osf_convert(self, reader: Reader, output: str) -> None:
        # TODO: figure out how to get the current chunk_size
        chunk_size = 0
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
            writer.save_message(msg.id, msg.ts, msg.buffer)
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
                raise RuntimeError(f"Error overwriteing osf file: {osf_file}"
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
            ls = msg.decode()
            if ls:
                window = self.metadata[idx].format.column_window
                scan = cast(LidarScan, ls)
                if not self._complete or scan.complete(window):
                    if set(scan.fields) != set(self._fields[idx].keys()):
                        scan = client.LidarScan(scan, self._fields[idx])
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
    def fields(self) -> List[client.FieldTypes]:
        """Field types are present in the LidarScan objects on read from iterator"""
        return self._fields

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
                    ) -> Union[List[Optional[LidarScan]], List[List[Optional[LidarScan]]]]:

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
            L = len(self)
            k = ForwardSlicer.normalize(key, L)
            count = k.stop - k.start
            if count <= 0:
                return []
            ts_start = min([self._reader.ts_by_message_idx(mid, k.start)
                           for mid in self._stream_ids])
            ts_stop = max([self._reader.ts_by_message_idx(mid, k.stop - 1)
                          for mid in self._stream_ids])
            scans_itr = collate_scans(self._scans_iter(ts_start, ts_stop, False),
                                      self.sensors_count, first_valid_packet_ts,
                                      dt=self._dt)
            result = [scan for idx, scan in ForwardSlicer.slice(
                enumerate(scans_itr), k) if idx < count]
            return result if k.step > 0 else list(reversed(result))

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
