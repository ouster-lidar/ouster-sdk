"""Super initial osf typings, too rough yet ..."""
from typing import Any, ClassVar, List
from typing import overload, Iterator, Optional, Callable
import numpy
from numpy import ndarray
from ouster.sdk.core import Severity, Version
from ouster.sdk.core import BufferT, LidarScan, LidarScanSet, SensorInfo, FieldType, ScanSource

class LidarScanEncoder:
    ...

class OsfScanSource(ScanSource):

    def __init__(self, source: str, *, extrinsics: List[numpy.ndarray]=[], extrinsics_file: str='', field_names: List[str]=[], index: bool=False) -> None:
        """__init__(self: ouster.sdk._bindings.osf.OsfScanSource, file: str, **kwargs) -> None
"""
        ...

    def is_collated(self) -> bool:
        ...

class PngLidarScanEncoder(LidarScanEncoder):

    def __init__(self, compression_amount: int) -> None:
        """__init__(self: ouster.sdk._bindings.osf.PngLidarScanEncoder, compression_amount: int) -> None
"""
        ...

class ZPngLidarScanEncoder(LidarScanEncoder):

    def __init__(self, compression_amount: int) -> None:
        """__init__(self: ouster.sdk._bindings.osf.ZPngLidarScanEncoder, compression_amount: int) -> None
"""
        ...

class Encoder:

    def __init__(self, lidar_scan_encoder: LidarScanEncoder) -> None:
        """__init__(self: ouster.sdk._bindings.osf.Encoder, lidar_scan_encoder: ouster.sdk._bindings.osf.LidarScanEncoder) -> None
"""
        ...

class LidarScanStreamMeta:
    type_id: ClassVar[str] = ...

    @property
    def sensor_meta_id(self) -> int:
        ...

class LidarScanStream:
    type_id: ClassVar[str] = ...

    @property
    def meta(self) -> LidarScanStreamMeta:
        """`metadata entry` to store `LidarScanStream` metadata in an OSF file"""
        ...

class LidarSensor(MetadataEntry):
    type_id: ClassVar[str] = ...

    @overload
    def __init__(self, arg0: SensorInfo) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.osf.LidarSensor, arg0: ouster.sdk._bindings.client.SensorInfo) -> None

Create from ``SensorInfo`` object

2. __init__(self: ouster.sdk._bindings.osf.LidarSensor, metadata_json: str) -> None

Create from ``metadata_json`` string representation
"""
        ...

    @overload
    def __init__(self, metadata_json: str) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.osf.LidarSensor, arg0: ouster.sdk._bindings.client.SensorInfo) -> None

Create from ``SensorInfo`` object

2. __init__(self: ouster.sdk._bindings.osf.LidarSensor, metadata_json: str) -> None

Create from ``metadata_json`` string representation
"""
        ...

    @property
    def info(self) -> Any:
        """SensorInfo stored"""
        ...

    @property
    def metadata(self) -> str:
        """metadata_json string stored"""
        ...

class MetadataEntryRef(MetadataEntry):
    type_id: ClassVar[str] = ...

class Extrinsics(MetadataEntry):
    type_id: ClassVar[str] = ...

    def __init__(self, extrinsics: numpy.ndarray, ref_meta_id: int=..., name: str=...) -> None:
        """__init__(self: ouster.sdk._bindings.osf.Extrinsics, extrinsics: numpy.ndarray[numpy.float64[4, 4]], ref_meta_id: int = 0, name: str = '') -> None

Create Extrinsics object
"""
        ...

    @property
    def extrinsics(self) -> numpy.ndarray:
        """Extrisnics homogeneous 4x4 matrix"""
        ...

    @property
    def ref_meta_id(self) -> int:
        """reference to the metadata entry id of an object which extrisnics is it"""
        ...

    @property
    def name(self) -> str:
        """name of the Extrinsics object (optional)"""
        ...

class MessageRef:

    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
        ...

    @overload
    def decode(self) -> object:
        """decode(self: ouster.sdk._bindings.osf.MessageRef, fields: Optional[list[str]] = None) -> object


            Decodes the underlying object and returns it.

            Currently supports only LidarScans
        
"""
        ...

    @overload
    def decode(self, fields: Optional[List[str]]) -> object:
        """decode(self: ouster.sdk._bindings.osf.MessageRef, fields: Optional[list[str]] = None) -> object


            Decodes the underlying object and returns it.

            Currently supports only LidarScans
        
"""
        ...

    def of(self, arg0: object) -> bool:
        """of(self: ouster.sdk._bindings.osf.MessageRef, msg_stream: object) -> bool

Checks whether the message belongs to the ``msg_stream`` type
"""
        ...

    @property
    def id(self) -> int:
        """Message id which is a ``stream_id`` and point to the `metadata entry` that describes the stream"""
        ...

    @property
    def ts(self) -> int:
        """Message timestamp (ns)"""
        ...

    @property
    def buffer(self) -> BufferT:
        """Returns encoded message byte array"""
        ...

class MetadataEntry:

    def __init__(self) -> None:
        """__init__(self: ouster.sdk._bindings.osf.MetadataEntry) -> None
"""
        ...

    @staticmethod
    def from_buffer(arg0: List[int], arg1: str) -> MetadataEntry:
        """from_buffer(buf: list[int], type_str: str) -> ouster.sdk._bindings.osf.MetadataEntry

Decodes (deserialize) metadata entry buffer to a typed object
"""
        ...

    def of(self, arg0: object) -> bool:
        """of(self: ouster.sdk._bindings.osf.MetadataEntry, meta_obj_type: object) -> bool


                 Checks whether metadata entry is of particular type

                 It's just:
                 ``self.type == meta_obj_type.type_id``
            
"""
        ...

    @property
    def buffer(self) -> List[int]:
        """Encodes (serialize) metadata entry to a stored byte array"""
        ...

    @property
    def id(self) -> int:
        """Id of the metadata entry (unique for a file)"""
        ...

    @property
    def static_type(self) -> str:
        """Static type, C++ compile time (in Python use ``type_id`` of concrete type objects instead)"""
        ...

    @property
    def type(self) -> str:
        """Type of the metadata entry (use this)"""
        ...

class MetadataStore:

    def __init__(self) -> None:
        """__init__(self: ouster.sdk._bindings.osf.MetadataStore) -> None
"""
        ...

    def find(self, *args, **kwargs) -> Any:
        """find(self: ouster.sdk._bindings.osf.MetadataStore, meta_type: object) -> dict[int, ouster::sdk::osf::MetadataEntry]

Get all `metadata entries` of the specified ``meta_type``
"""
        ...

    def get(self, *args, **kwargs) -> Any:
        """get(self: ouster.sdk._bindings.osf.MetadataStore, meta_type: object) -> ouster::sdk::osf::MetadataEntry

Get the first `metadata entry` of the specified ``meta_type``
"""
        ...

    def items(self) -> Iterator:
        """items(self: ouster.sdk._bindings.osf.MetadataStore) -> Iterator[tuple[int, ouster::sdk::osf::MetadataEntry]]

Key/Value pairs of `metadata entries`
"""
        ...

    def __getitem__(self, index) -> Any:
        """__getitem__(self: ouster.sdk._bindings.osf.MetadataStore, meta_id: int) -> ouster::sdk::osf::MetadataEntry

Get `metadata entry` by id
"""
        ...

    def __iter__(self) -> Iterator:
        """__iter__(self: ouster.sdk._bindings.osf.MetadataStore) -> Iterator[int]

Creates an iterator to get metadata id's
"""
        ...

    def __len__(self) -> int:
        """__len__(self: ouster.sdk._bindings.osf.MetadataStore) -> int

Number of `metadata entries` in the file
"""
        ...

class Reader:

    def __init__(self, arg0: str, error_handler: Optional[Callable[[Severity, str], None]]=None) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.osf.Reader, file: str) -> None

2. __init__(self: ouster.sdk._bindings.osf.Reader, file: str, error_handler: Callable = None) -> None
"""
        ...

    def chunks(self) -> Iterator:
        """chunks(self: ouster.sdk._bindings.osf.Reader) -> Iterator[ouster::sdk::osf::ChunkRef]


                Creates an iterator to reads chunks as they appear in a file.
            
"""
        ...

    @overload
    def messages(self) -> Iterator:
        """messages(*args, **kwargs)
Overloaded function.

1. messages(self: ouster.sdk._bindings.osf.Reader) -> Iterator[ouster::sdk::osf::MessageRef]


                Creates an iterator to read messages in default ``STREAMING`` layout.
            

2. messages(self: ouster.sdk._bindings.osf.Reader, start_ts: int, end_ts: int) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` in ``[start_ts, end_ts]`` timestamp range
                    (inclusive)
                

3. messages(self: ouster.sdk._bindings.osf.Reader, stream_ids: list[int] = []) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` from only specified ``[<stream_ids>]`` list
                

4. messages(self: ouster.sdk._bindings.osf.Reader, stream_ids: list[int], start_ts: int, end_ts: int) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` in ``[start_ts, end_ts]`` timestamp range (inclusive) of a
                    specified ``<stream_ids>`` list
                
"""
        ...

    @overload
    def messages(self, start_ts: int, end_ts: int) -> Iterator:
        """messages(*args, **kwargs)
Overloaded function.

1. messages(self: ouster.sdk._bindings.osf.Reader) -> Iterator[ouster::sdk::osf::MessageRef]


                Creates an iterator to read messages in default ``STREAMING`` layout.
            

2. messages(self: ouster.sdk._bindings.osf.Reader, start_ts: int, end_ts: int) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` in ``[start_ts, end_ts]`` timestamp range
                    (inclusive)
                

3. messages(self: ouster.sdk._bindings.osf.Reader, stream_ids: list[int] = []) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` from only specified ``[<stream_ids>]`` list
                

4. messages(self: ouster.sdk._bindings.osf.Reader, stream_ids: list[int], start_ts: int, end_ts: int) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` in ``[start_ts, end_ts]`` timestamp range (inclusive) of a
                    specified ``<stream_ids>`` list
                
"""
        ...

    @overload
    def messages(self, stream_ids: List[int]) -> Iterator:
        """messages(*args, **kwargs)
Overloaded function.

1. messages(self: ouster.sdk._bindings.osf.Reader) -> Iterator[ouster::sdk::osf::MessageRef]


                Creates an iterator to read messages in default ``STREAMING`` layout.
            

2. messages(self: ouster.sdk._bindings.osf.Reader, start_ts: int, end_ts: int) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` in ``[start_ts, end_ts]`` timestamp range
                    (inclusive)
                

3. messages(self: ouster.sdk._bindings.osf.Reader, stream_ids: list[int] = []) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` from only specified ``[<stream_ids>]`` list
                

4. messages(self: ouster.sdk._bindings.osf.Reader, stream_ids: list[int], start_ts: int, end_ts: int) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` in ``[start_ts, end_ts]`` timestamp range (inclusive) of a
                    specified ``<stream_ids>`` list
                
"""
        ...

    @overload
    def messages(self, stream_ids: List[int], start_ts: int, end_ts: int) -> Iterator:
        """messages(*args, **kwargs)
Overloaded function.

1. messages(self: ouster.sdk._bindings.osf.Reader) -> Iterator[ouster::sdk::osf::MessageRef]


                Creates an iterator to read messages in default ``STREAMING`` layout.
            

2. messages(self: ouster.sdk._bindings.osf.Reader, start_ts: int, end_ts: int) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` in ``[start_ts, end_ts]`` timestamp range
                    (inclusive)
                

3. messages(self: ouster.sdk._bindings.osf.Reader, stream_ids: list[int] = []) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` from only specified ``[<stream_ids>]`` list
                

4. messages(self: ouster.sdk._bindings.osf.Reader, stream_ids: list[int], start_ts: int, end_ts: int) -> Iterator[ouster::sdk::osf::MessageRef]


                    Read `messages` in ``[start_ts, end_ts]`` timestamp range (inclusive) of a
                    specified ``<stream_ids>`` list
                
"""
        ...

    @property
    def end_ts(self) -> int:
        """
                End timestamp (ns) - the highest message timestamp present in the file
        """
        ...

    @property
    def metadata_id(self) -> str:
        """
            Data id string
        """
        ...

    @property
    def meta_store(self) -> Any:
        """
                Returns the metadata store that gives an access to all
                *metadata entries* in the file.
        """
        ...

    @property
    def start_ts(self) -> int:
        """
                Start timestamp (ns) - the lowest message timestamp present in the file
        """
        ...

    @property
    def has_stream_info(self) -> bool:
        """Whether ``StreamingInfo`` metadata is available (i.e. reading messages by timestamp and streams can be performed)"""
        ...

    @property
    def has_message_idx(self) -> bool:
        """Whether OSF contains the message counts that are needed for ``ts_by_message_idx()`` (message counts was added a bit later to the OSF core, so this function will be obsolete over time)"""
        ...

    @property
    def has_timestamp_idx(self) -> bool:
        """Whether OSF contains the message timestamp index in the metadata necessary to quickly collate and jump to a specific message time."""
        ...

    def ts_by_message_idx(self, stream_id: int, msg_idx: int) -> int:
        """ts_by_message_idx(self: ouster.sdk._bindings.osf.Reader, stream_id: int, message_idx: int) -> object


                    Find the timestamp of the message by its index and stream_id.

                    Requires the OSF with message_counts inside, i.e. has_message_idx()
                    is ``True``, otherwise return value is always None.
                
"""
        ...

    @property
    def version(self) -> Version:
        """Version of OSF file format in the file"""
        ...

class StreamStats:

    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
        ...

    @property
    def end_ts(self) -> int:
        """Highest timestamp (ns) of the stream messages"""
        ...

    @property
    def message_avg_size(self) -> int:
        """Average size (bytes) of a message in a stream"""
        ...

    @property
    def message_count(self) -> int:
        """Number of messages in a stream"""
        ...

    @property
    def start_ts(self) -> int:
        """Lowest timestamp (ns) of the stream messages"""
        ...

    @property
    def stream_id(self) -> int:
        """Id of a stream"""
        ...

    @property
    def receive_timestamps(self) -> ndarray:
        """Receive timestamps of each message in the stream."""
        ...

    @property
    def sensor_timestamps(self) -> ndarray:
        """Sensor timestamps of each message in the stream."""
        ...

class StreamingInfo(MetadataEntry):
    type_id: ClassVar[str] = ...

    def __init__(self, *args, **kwargs) -> None:
        """Initialize self.  See help(type(self)) for accurate signature."""
        ...

    @property
    def chunks_info(self) -> Iterator:
        """Maps `chunk` to `stream_id` by chunk offset"""
        ...

    @property
    def stream_stats(self) -> Iterator:
        """Statistics of messages in per stream"""
        ...

class Writer:
    @overload
    def __init__(self, file_name: str, chunk_size: int=...) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.osf.Writer, file_name: str, chunk_size: int = 0) -> None


        Creates a `Writer` with specified ``chunk_size``.

        Default ``chunk_size`` is ``2 MB``.
        

2. __init__(self: ouster.sdk._bindings.osf.Writer, filename: str, info: ouster.sdk._bindings.client.SensorInfo, fields_to_write: list[str] = [], chunk_size: int = 0, encoder: ouster::sdk::osf::Encoder = None) -> None


        Creates a `Writer` with deafault ``STREAMING`` layout chunks writer.

        Using default ``chunk_size`` of ``2MB``.

        Args:
            filename (str): The filename to output to.
            info (SensorInfo): The sensor info vector to use for a multi stream OSF
                file.
            chunk_size (int): The chunk size in bytes to use for the OSF file. This arg
                is optional, and if not provided the default value of 2MB
                is used. If the current chunk being written exceeds the
                chunk_size, a new chunk will be started on the next call to
                `save`. This allows an application to tune the number of
                messages (e.g. lidar scans) per chunk, which affects the
                granularity of the message index stored in the
                StreamingInfo in the file metadata. A smaller chunk_size
                means more messages are indexed and a larger number of
                index entries. A more granular index allows for more
                precise seeking at the slight expense of a larger file.
            fields_to_write (List[str]): The fields from scans to
                actually save into the OSF. If not provided uses the fields from
                the first saved lidar scan for each stream. This parameter is optional.

        

3. __init__(self: ouster.sdk._bindings.osf.Writer, filename: str, info: list[ouster.sdk._bindings.client.SensorInfo], fields_to_write: list[str] = [], chunk_size: int = 0, encoder: ouster::sdk::osf::Encoder = None) -> None


            Creates a `Writer` with specified ``chunk_size``.

            Default ``chunk_size`` is ``2MB``.

            Args:
            filename (str): The filename to output to.
            info (List[SensorInfo]): The sensor info vector to use for a
                multi stream OSF file.
            fields_to_write (List[str]): The fields from scans to
                actually save into the OSF. If not provided uses the fields from
                the first saved lidar scan for each stream. This parameter is optional.
            chunk_size (int): The chunksize to use for the OSF file, this arg
                is optional.

        
"""
        ...

    @overload
    def __init__(self, filename: str, info: SensorInfo, fields_to_write: List[str]=..., chunk_size: int=..., encoder: Encoder=...) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.osf.Writer, file_name: str, chunk_size: int = 0) -> None


        Creates a `Writer` with specified ``chunk_size``.

        Default ``chunk_size`` is ``2 MB``.
        

2. __init__(self: ouster.sdk._bindings.osf.Writer, filename: str, info: ouster.sdk._bindings.client.SensorInfo, fields_to_write: list[str] = [], chunk_size: int = 0, encoder: ouster::sdk::osf::Encoder = None) -> None


        Creates a `Writer` with deafault ``STREAMING`` layout chunks writer.

        Using default ``chunk_size`` of ``2MB``.

        Args:
            filename (str): The filename to output to.
            info (SensorInfo): The sensor info vector to use for a multi stream OSF
                file.
            chunk_size (int): The chunk size in bytes to use for the OSF file. This arg
                is optional, and if not provided the default value of 2MB
                is used. If the current chunk being written exceeds the
                chunk_size, a new chunk will be started on the next call to
                `save`. This allows an application to tune the number of
                messages (e.g. lidar scans) per chunk, which affects the
                granularity of the message index stored in the
                StreamingInfo in the file metadata. A smaller chunk_size
                means more messages are indexed and a larger number of
                index entries. A more granular index allows for more
                precise seeking at the slight expense of a larger file.
            fields_to_write (List[str]): The fields from scans to
                actually save into the OSF. If not provided uses the fields from
                the first saved lidar scan for each stream. This parameter is optional.

        

3. __init__(self: ouster.sdk._bindings.osf.Writer, filename: str, info: list[ouster.sdk._bindings.client.SensorInfo], fields_to_write: list[str] = [], chunk_size: int = 0, encoder: ouster::sdk::osf::Encoder = None) -> None


            Creates a `Writer` with specified ``chunk_size``.

            Default ``chunk_size`` is ``2MB``.

            Args:
            filename (str): The filename to output to.
            info (List[SensorInfo]): The sensor info vector to use for a
                multi stream OSF file.
            fields_to_write (List[str]): The fields from scans to
                actually save into the OSF. If not provided uses the fields from
                the first saved lidar scan for each stream. This parameter is optional.
            chunk_size (int): The chunksize to use for the OSF file, this arg
                is optional.

        
"""
        ...

    @overload
    def __init__(self, filename: str, info: List[SensorInfo], fields_to_write: List[str]=..., chunk_size: int=..., encoder: Encoder=...) -> None:
        """__init__(*args, **kwargs)
Overloaded function.

1. __init__(self: ouster.sdk._bindings.osf.Writer, file_name: str, chunk_size: int = 0) -> None


        Creates a `Writer` with specified ``chunk_size``.

        Default ``chunk_size`` is ``2 MB``.
        

2. __init__(self: ouster.sdk._bindings.osf.Writer, filename: str, info: ouster.sdk._bindings.client.SensorInfo, fields_to_write: list[str] = [], chunk_size: int = 0, encoder: ouster::sdk::osf::Encoder = None) -> None


        Creates a `Writer` with deafault ``STREAMING`` layout chunks writer.

        Using default ``chunk_size`` of ``2MB``.

        Args:
            filename (str): The filename to output to.
            info (SensorInfo): The sensor info vector to use for a multi stream OSF
                file.
            chunk_size (int): The chunk size in bytes to use for the OSF file. This arg
                is optional, and if not provided the default value of 2MB
                is used. If the current chunk being written exceeds the
                chunk_size, a new chunk will be started on the next call to
                `save`. This allows an application to tune the number of
                messages (e.g. lidar scans) per chunk, which affects the
                granularity of the message index stored in the
                StreamingInfo in the file metadata. A smaller chunk_size
                means more messages are indexed and a larger number of
                index entries. A more granular index allows for more
                precise seeking at the slight expense of a larger file.
            fields_to_write (List[str]): The fields from scans to
                actually save into the OSF. If not provided uses the fields from
                the first saved lidar scan for each stream. This parameter is optional.

        

3. __init__(self: ouster.sdk._bindings.osf.Writer, filename: str, info: list[ouster.sdk._bindings.client.SensorInfo], fields_to_write: list[str] = [], chunk_size: int = 0, encoder: ouster::sdk::osf::Encoder = None) -> None


            Creates a `Writer` with specified ``chunk_size``.

            Default ``chunk_size`` is ``2MB``.

            Args:
            filename (str): The filename to output to.
            info (List[SensorInfo]): The sensor info vector to use for a
                multi stream OSF file.
            fields_to_write (List[str]): The fields from scans to
                actually save into the OSF. If not provided uses the fields from
                the first saved lidar scan for each stream. This parameter is optional.
            chunk_size (int): The chunksize to use for the OSF file, this arg
                is optional.

        
"""
        ...

    @overload
    def save(self, stream_id: int, scan: LidarScan) -> None:
        """save(*args, **kwargs)
Overloaded function.

1. save(self: ouster.sdk._bindings.osf.Writer, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan) -> None


            Save a lidar scan to the OSF file.

            Args:
                stream_index (int): The index of the corresponding
                    sensor_info to use.
                scan (LidarScan): The scan to save.

            

2. save(self: ouster.sdk._bindings.osf.Writer, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan, ts: int) -> None


            Save a lidar scan to the OSF file.

            Args:
                stream_index (int): The index of the corresponding
                    sensor_info to use.
                scan (LidarScan): The scan to save.
                ts (int): The timestamp to index the scan with.
            

3. save(self: ouster.sdk._bindings.osf.Writer, scan: ouster.sdk._bindings.client.LidarScanSet) -> None


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (LidarScanSet): The collation to save.
            

4. save(self: ouster.sdk._bindings.osf.Writer, scans: list[ouster.sdk._bindings.client.LidarScan]) -> None


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (List[LidarScan]): The scans to save. This will correspond
                       to the list of SensorInfos.

            
"""
        ...

    @overload
    def save(self, stream_id: int, scan: LidarScan, ts: int) -> None:
        """save(*args, **kwargs)
Overloaded function.

1. save(self: ouster.sdk._bindings.osf.Writer, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan) -> None


            Save a lidar scan to the OSF file.

            Args:
                stream_index (int): The index of the corresponding
                    sensor_info to use.
                scan (LidarScan): The scan to save.

            

2. save(self: ouster.sdk._bindings.osf.Writer, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan, ts: int) -> None


            Save a lidar scan to the OSF file.

            Args:
                stream_index (int): The index of the corresponding
                    sensor_info to use.
                scan (LidarScan): The scan to save.
                ts (int): The timestamp to index the scan with.
            

3. save(self: ouster.sdk._bindings.osf.Writer, scan: ouster.sdk._bindings.client.LidarScanSet) -> None


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (LidarScanSet): The collation to save.
            

4. save(self: ouster.sdk._bindings.osf.Writer, scans: list[ouster.sdk._bindings.client.LidarScan]) -> None


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (List[LidarScan]): The scans to save. This will correspond
                       to the list of SensorInfos.

            
"""
        ...

    @overload
    def save(self, scan: List[Optional[LidarScan]]) -> None:
        """save(*args, **kwargs)
Overloaded function.

1. save(self: ouster.sdk._bindings.osf.Writer, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan) -> None


            Save a lidar scan to the OSF file.

            Args:
                stream_index (int): The index of the corresponding
                    sensor_info to use.
                scan (LidarScan): The scan to save.

            

2. save(self: ouster.sdk._bindings.osf.Writer, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan, ts: int) -> None


            Save a lidar scan to the OSF file.

            Args:
                stream_index (int): The index of the corresponding
                    sensor_info to use.
                scan (LidarScan): The scan to save.
                ts (int): The timestamp to index the scan with.
            

3. save(self: ouster.sdk._bindings.osf.Writer, scan: ouster.sdk._bindings.client.LidarScanSet) -> None


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (LidarScanSet): The collation to save.
            

4. save(self: ouster.sdk._bindings.osf.Writer, scans: list[ouster.sdk._bindings.client.LidarScan]) -> None


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (List[LidarScan]): The scans to save. This will correspond
                       to the list of SensorInfos.

            
"""
        ...

    @overload
    def save(self, collation: LidarScanSet) -> None:
        """save(*args, **kwargs)
Overloaded function.

1. save(self: ouster.sdk._bindings.osf.Writer, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan) -> None


            Save a lidar scan to the OSF file.

            Args:
                stream_index (int): The index of the corresponding
                    sensor_info to use.
                scan (LidarScan): The scan to save.

            

2. save(self: ouster.sdk._bindings.osf.Writer, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan, ts: int) -> None


            Save a lidar scan to the OSF file.

            Args:
                stream_index (int): The index of the corresponding
                    sensor_info to use.
                scan (LidarScan): The scan to save.
                ts (int): The timestamp to index the scan with.
            

3. save(self: ouster.sdk._bindings.osf.Writer, scan: ouster.sdk._bindings.client.LidarScanSet) -> None


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (LidarScanSet): The collation to save.
            

4. save(self: ouster.sdk._bindings.osf.Writer, scans: list[Optional[ouster.sdk._bindings.client.LidarScan]]) -> None


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (List[LidarScan]): The scans to save. This will correspond
                       to the list of SensorInfos.

            
"""
        ...

    def add_sensor(self, info: SensorInfo, fields_to_write: List[str]=...) -> int:
        """add_sensor(self: ouster.sdk._bindings.osf.Writer, info: ouster.sdk._bindings.client.SensorInfo, fields_to_write: list[str] = []) -> int


               Add a sensor to the OSF file.

               Args:
                   info (SensorInfo): Sensor to add.
                   fields_to_write (List[str]): The fields from scans to
                       actually save into the OSF. If not provided uses the fields from
                       the first saved lidar scan for each stream. This parameter is optional.

               Returns (int):
                   The stream index to use to write scans to this sensor.

            
"""
        ...

    def add_metadata(self, arg0: object) -> int:
        """add_metadata(self: ouster.sdk._bindings.osf.Writer, m: object) -> int

Add `metadata entry` to a file
"""
        ...

    def save_message(self, stream_id: int, receive_ts: int, sensor_ts: int, buffer: BufferT, type: str) -> int:
        """save_message(*args, **kwargs)
Overloaded function.

1. save_message(self: ouster.sdk._bindings.osf.Writer, stream_id: int, receive_ts: int, sensor_ts: int, buffer: numpy.ndarray[numpy.uint8], type: str) -> None


                 Low-level save message routine.

                 Directly saves the message `buffer` with `id` and `ts` (ns)
                 without any further checks.
            

2. save_message(self: ouster.sdk._bindings.osf.Writer, stream_id: int, receive_ts: int, sensor_ts: int, buffer: Buffer, type: str) -> None


                 Low-level save message routine.

                 Directly saves the message `buffer` with `id` and `ts` (ns)
                 without any further checks.
            
"""
        ...

    @overload
    def sensor_info(self) -> List[SensorInfo]:
        """sensor_info(*args, **kwargs)
Overloaded function.

1. sensor_info(self: ouster.sdk._bindings.osf.Writer) -> list[ouster.sdk._bindings.client.SensorInfo]


                 Return the sensor info list.

                 Returns (List[SensorInfo]):
                     The sensor info list.

            

2. sensor_info(self: ouster.sdk._bindings.osf.Writer, stream_index: int) -> ouster.sdk._bindings.client.SensorInfo


                 Return the sensor info of the specifed stream_index.

                 Args:
                     stream_index (in): The index of the sensor to return
                                        info about.

                 Returns (SensorInfo):
                     The correct sensor info

            
"""
        ...

    @overload
    def sensor_info(self, stream_id: int) -> SensorInfo:
        """sensor_info(*args, **kwargs)
Overloaded function.

1. sensor_info(self: ouster.sdk._bindings.osf.Writer) -> list[ouster.sdk._bindings.client.SensorInfo]


                 Return the sensor info list.

                 Returns (List[SensorInfo]):
                     The sensor info list.

            

2. sensor_info(self: ouster.sdk._bindings.osf.Writer, stream_index: int) -> ouster.sdk._bindings.client.SensorInfo


                 Return the sensor info of the specifed stream_index.

                 Args:
                     stream_index (in): The index of the sensor to return
                                        info about.

                 Returns (SensorInfo):
                     The correct sensor info

            
"""
        ...

    def sensor_info_count(self) -> int:
        """sensor_info_count(self: ouster.sdk._bindings.osf.Writer) -> int


                 Return the number of sensor_info objects.

                 Returns (int):
                     The number of sensor_info objects.

            
"""
        ...

    def filename(self) -> str:
        """filename(self: ouster.sdk._bindings.osf.Writer) -> str


                 Return the osf file name.

                 Returns (str):
                     The OSF filename.
            
"""
        ...

    def metadata_id(self) -> str:
        """metadata_id(self: ouster.sdk._bindings.osf.Writer) -> str


                 Return the metadata identifier string.

                 Returns (str):
                     The OSF metadata identifier string.
            
"""
        ...

    def set_metadata_id(self, id: str) -> None:
        """set_metadata_id(self: ouster.sdk._bindings.osf.Writer, arg0: str) -> None


                 Set the metadata identifier string.
            
"""
        ...

    @property
    def meta_store(self) -> MetadataStore:
        """
                Returns the metadata store that gives an access to all
                *metadata entries* in the file.
        """
        ...

    def close(self, fsync: bool=False) -> None:
        """close(self: ouster.sdk._bindings.osf.Writer, fsync: bool = False) -> None

Finish OSF file and flush everything to disk.
"""
        ...

    def is_closed(self) -> bool:
        """is_closed(self: ouster.sdk._bindings.osf.Writer) -> bool


                 Return the closed status of the writer.

                 Returns (bool):
                     The closed status of the writer.

            
"""
        ...

    def __enter__(self) -> Writer:
        """__enter__(self: ouster.sdk._bindings.osf.Writer) -> ouster.sdk._bindings.osf.Writer


                 Allow Writer to work within `with` blocks.
            
"""
        ...

    def __exit__(*args) -> None:
        """__exit__(self: ouster.sdk._bindings.osf.Writer, arg0: object, arg1: object, arg2: object) -> None


                 Allow Writer to work within `with` blocks.
            
"""
        ...

class FutureWrapper:

    def get(self) -> None:
        """get(self: ouster.sdk._bindings.osf.FutureWrapper) -> None
"""
        ...

    def valid(self) -> bool:
        """valid(self: ouster.sdk._bindings.osf.FutureWrapper) -> bool
"""
        ...

    def wait(self) -> None:
        """wait(self: ouster.sdk._bindings.osf.FutureWrapper) -> None
"""
        ...

class AsyncWriter:

    def __init__(self, filename: str, info: List[SensorInfo], fields_to_write: List[str]=..., chunk_size: int=..., encoder: Encoder=...) -> None:
        """__init__(self: ouster.sdk._bindings.osf.AsyncWriter, filename: str, info: list[ouster.sdk._bindings.client.SensorInfo], fields_to_write: list[str] = [], chunk_size: int = 0, encoder: ouster::sdk::osf::Encoder = None) -> None


             Creates an `AsyncWriter` with specified ``chunk_size``.

             Default ``chunk_size`` is ``2MB``.

             Args:
                filename (str): The filename to output to.
                info (List[SensorInfo]): The sensor info vector to use for a
                    multi stream OSF file.
                fields_to_write (List[str]): The fields from scans to
                    actually save into the OSF. If not provided uses the fields from
                    the first saved lidar scan for each stream. This parameter is optional.
                chunk_size (int): The chunksize to use for the OSF file, this arg
                    is optional.
                encoder (Encoder): an optional encoder instance,
                    used to configure how writer encodes the OSF.
        
"""
        ...

    @overload
    def save(self, stream_id: int, scan: LidarScan) -> FutureWrapper:
        """save(*args, **kwargs)
Overloaded function.

1. save(self: ouster.sdk._bindings.osf.AsyncWriter, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan) -> FutureWrapper


               Save a lidar scan to the OSF file.

               Args:
                   stream_index (int): The index of the corresponding
                       SensorInfo to use.
                   scan (LidarScan): The scan to save.

               Returns: a future.

            

2. save(self: ouster.sdk._bindings.osf.AsyncWriter, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan, ts: int) -> FutureWrapper


               Save a lidar scan to the OSF file.

               Args:
                   stream_index (int): The index of the corresponding
                       SensorInfo to use.
                   scan (LidarScan): The scan to save.
                   ts (int): The timestamp to index the scan with.

               Returns: a future.
            

3. save(self: ouster.sdk._bindings.osf.AsyncWriter, scan: list[ouster.sdk._bindings.client.LidarScan]) -> list[FutureWrapper]


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (List[LidarScan]): The scans to save. This will correspond
                       to the list of SensorInfos.

               Returns: a list of futures.
            
"""
        ...

    @overload
    def save(self, stream_id: int, scan: LidarScan, ts: int) -> FutureWrapper:
        """save(*args, **kwargs)
Overloaded function.

1. save(self: ouster.sdk._bindings.osf.AsyncWriter, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan) -> FutureWrapper


               Save a lidar scan to the OSF file.

               Args:
                   stream_index (int): The index of the corresponding
                       SensorInfo to use.
                   scan (LidarScan): The scan to save.

               Returns: a future.

            

2. save(self: ouster.sdk._bindings.osf.AsyncWriter, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan, ts: int) -> FutureWrapper


               Save a lidar scan to the OSF file.

               Args:
                   stream_index (int): The index of the corresponding
                       SensorInfo to use.
                   scan (LidarScan): The scan to save.
                   ts (int): The timestamp to index the scan with.

               Returns: a future.
            

3. save(self: ouster.sdk._bindings.osf.AsyncWriter, scan: list[ouster.sdk._bindings.client.LidarScan]) -> list[FutureWrapper]


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (List[LidarScan]): The scans to save. This will correspond
                       to the list of SensorInfos.

               Returns: a list of futures.
            
"""
        ...

    @overload
    def save(self, scan: List[LidarScan]) -> List[FutureWrapper]:
        """save(*args, **kwargs)
Overloaded function.

1. save(self: ouster.sdk._bindings.osf.AsyncWriter, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan) -> FutureWrapper


               Save a lidar scan to the OSF file.

               Args:
                   stream_index (int): The index of the corresponding
                       SensorInfo to use.
                   scan (LidarScan): The scan to save.

               Returns: a future.

            

2. save(self: ouster.sdk._bindings.osf.AsyncWriter, stream_index: int, scan: ouster.sdk._bindings.client.LidarScan, ts: int) -> FutureWrapper


               Save a lidar scan to the OSF file.

               Args:
                   stream_index (int): The index of the corresponding
                       SensorInfo to use.
                   scan (LidarScan): The scan to save.
                   ts (int): The timestamp to index the scan with.

               Returns: a future.
            

3. save(self: ouster.sdk._bindings.osf.AsyncWriter, scan: list[ouster.sdk._bindings.client.LidarScan]) -> list[FutureWrapper]


               Save a set of lidar scans to the OSF file.

               Args:
                   scans (List[LidarScan]): The scans to save. This will correspond
                       to the list of SensorInfos.

               Returns: a list of futures.
            
"""
        ...

    def close(self, fsync: bool=False) -> None:
        """close(self: ouster.sdk._bindings.osf.AsyncWriter, fsync: bool = False) -> None

Finish OSF file and flush everything to disk.
"""
        ...

    def __enter__(self) -> AsyncWriter:
        """__enter__(self: ouster.sdk._bindings.osf.AsyncWriter) -> ouster.sdk._bindings.osf.AsyncWriter


                 Allow AsyncWriter to work within `with` blocks.
            
"""
        ...

    def __exit__(*args) -> None:
        """__exit__(self: ouster.sdk._bindings.osf.AsyncWriter, arg0: object, arg1: object, arg2: object) -> None


                 Allow AsyncWriter to work within `with` blocks.
            
"""
        ...

def slice_and_cast(lidar_scan: LidarScan, field_types=...) -> LidarScan:
    """slice_and_cast(*args, **kwargs)
Overloaded function.

1. slice_and_cast(lidar_scan: ouster.sdk._bindings.client.LidarScan, field_types: list[ouster.sdk._bindings.client.FieldType]) -> ouster.sdk._bindings.client.LidarScan

Copies LidarScan with new field types

2. slice_and_cast(lidar_scan: ouster.sdk._bindings.client.LidarScan, field_types: dict[str, object]) -> ouster.sdk._bindings.client.LidarScan

Copies LidarScan with new field types
"""
    ...

def dump_metadata(file: str, full: bool=...) -> str:
    """dump_metadata(file: str, full: bool = True) -> str


        Dump OSF metadata/session info in JSON format. (aka osf-metadata)

        :file: OSF file path
        :returns: JSON formatted string of OSF metadata + header info
    
"""
    ...

def parse_and_print(file: str, with_decoding: bool=...) -> None:
    """parse_and_print(file: str, with_decoding: bool = False) -> None


        Parse OSF file and print messages types, timestamps and counts to
        stdout.

        :file: OSF file path (v1/v2)
    
"""
    ...

def backup_osf_file_metablob(file: str, backup_file_name: str) -> None:
    """backup_osf_file_metablob(file: str, backup_file_name: str) -> int


         Backup the metadata blob in an OSF file.

        :file: OSF file path (v1/v2)
        :backup_file_name: Backup path
    
"""
    ...

def restore_osf_file_metablob(file: str, backup_file_name: str) -> None:
    """restore_osf_file_metablob(file: str, backup_file_name: str) -> int


        Restore an OSF metadata blob from a backup file.

        :file: OSF file path (v1/v2)
        :backup_file_name: The backup to use
    
"""
    ...

def osf_file_modify_metadata(file: str, new_metadata: List[SensorInfo]) -> int:
    """osf_file_modify_metadata(file_name: str, new_metadata: list[ouster.sdk._bindings.client.SensorInfo]) -> int


        Modify an OSF files sensor_info metadata.

        :file_name: The OSF file to modify.
        :new_metadata: Array containing sensor infos to write to the file.
        :returns: The number of the bytes written to the OSF file.
    
"""
    ...
