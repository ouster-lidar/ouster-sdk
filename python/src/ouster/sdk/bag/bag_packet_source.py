# type: ignore
from typing import Iterator, List, Tuple, Union, Optional
import numpy as np

from threading import Lock
import logging
from ouster.sdk.core import SensorInfo, PacketFormat, PacketValidationFailure, LidarPacket, ImuPacket, Packet
from ouster.sdk.util import resolve_metadata_multi

from pathlib import Path

from ouster.sdk._bindings.client import populate_extrinsics
from ctypes import Structure, c_uint16, c_uint32, c_uint64

from ouster.sdk._bindings.client import PacketSource as _PacketSource

import sqlite3
from rosbags.highlevel import AnyReader  # type: ignore
from rosbags.typesys import Stores, get_typestore, get_types_from_msg  # type: ignore
logger = logging.getLogger("bag-logger")


# monkey patch sqlite connect to not enforce that it is called from only one thread
# if we don't do this we crash in the CLI when opening db3 bags
_old_connect = sqlite3.connect


def _new_connect(*args, **kwargs):
    kwargs['check_same_thread'] = False
    return _old_connect(*args, **kwargs)


sqlite3.connect = _new_connect


class MessageCount(Structure):
    _pack_ = 1
    _fields_ = [
        ('id', c_uint16),
        ('count', c_uint64),
    ]


class ChannelMetadata(Structure):
    _pack_ = 1
    _fields_ = [
        ('len', c_uint32),
    ]


def anybag_monkey(self, paths, default_typestore=None):
    if ".mcap" not in str(paths[0]):
        self.old_init(paths, default_typestore=default_typestore)
        return

    from rosbags.rosbag2 import Reader as Reader2
    from rosbags.typesys import Stores, get_typestore  # type: ignore
    self.is2 = True
    self.readers = [Reader2(paths, default_typestore=default_typestore)]
    self.isopen = False
    self.connections = []
    self.typestore = get_typestore(Stores.EMPTY) if default_typestore is None else default_typestore


def bag2_monkey(self, paths, default_typestore=None):
    path = paths[0] if type(paths) is list else paths
    if ".mcap" not in str(path):
        self.old_init(paths)
        return

    from rosbags.interfaces import Connection, ConnectionExtRosbag2
    from rosbags.rosbag2.storage_mcap import MCAPFile

    f = MCAPFile(path)
    f.open()
    schemas = f.get_schema_definitions()

    counts = {}
    # read message counts for each stream
    total_count = 0
    for i in range(0, len(f.statistics.channel_message_counts) // 10):
        item = f.statistics.channel_message_counts[i * 10:i * 10 + 10]
        c = MessageCount.from_buffer_copy(item)
        counts[c.id] = c.count
        total_count = total_count + c.count

    # read in metadata for each channel to get offered qos profiles
    qos = {}
    for i, c in f.channels.items():
        # each metadata is a Map<string, string>
        i = 0
        while i < len(c.metadata):
            key_len = ChannelMetadata.from_buffer_copy(c.metadata[i:i + 4]).len
            i = i + 4
            key = c.metadata[i:i + key_len].decode("utf-8")
            i = i + key_len
            value_len = ChannelMetadata.from_buffer_copy(c.metadata[i:i + 4]).len
            i = i + 4
            value = c.metadata[i:i + value_len].decode("utf-8")
            i = i + value_len
            if key == "offered_qos_profiles":
                qos[c.schema] = value

    conns = []
    for id, c in f.channels.items():
        con = Connection(id = id, topic=c.topic, msgtype=c.schema,
                         msgdef=schemas[c.schema][1],
                         msgcount = counts[id],
                         ext = ConnectionExtRosbag2(serialization_format='cdr',
                                                    offered_qos_profiles=qos.get(c.schema, '')),
                         owner = self, digest = None)
        conns.append(con)

    self.isopen = True
    self.is2 = True
    self.metadata = {}
    self.metadata["storage_identifier"] = "mcap"
    self.metadata["starting_time"] = {'nanoseconds_since_epoch': f.statistics.start_time}
    duration = f.statistics.end_time - f.statistics.start_time
    self.metadata["duration"] = {'nanoseconds': duration}
    self.metadata["message_count"] = total_count
    self.paths = paths
    self.storage = f
    self.connections = conns
    self.tmpdir = None


class BagPacketSource(_PacketSource):
    """Read a sensors packet streams out of a bag file as an iterator."""

    _metadata: List[SensorInfo]

    def __init__(self,
                 bag_path: Union[str, List[str]],
                 *,
                 extrinsics_file: Optional[str] = None,
                 meta: Optional[List[str]] = None,
                 soft_id_check: bool = False,
                 extrinsics: List[np.ndarray] = []):
        """Read sensor data streams from a single bag file.

        Args:
            bag_path: path to bag file or folder containing ROS2 db3 and yaml file
            meta: optional list of metadata files to load, if not provided metadata
                is loaded from the bag instead
            soft_id_check: if True, don't skip packets on init_id mismatch
        """
        _PacketSource.__init__(self)
        self._soft_id_check = soft_id_check
        self._id_error_count = 0
        self._size_error_count = 0
        self._lock = Lock()
        self._reset = False
        self._typestore = get_typestore(Stores.ROS2_FOXY)

        if type(bag_path) is list:
            if len(bag_path) > 1:
                raise ValueError("Only one bag file can be opened at a time per source.")
            self._bag_path = bag_path[0]
        else:
            self._bag_path = bag_path

        from rosbags.rosbag2 import Reader as Reader2

        # lets monkeypatch AnyReader
        if AnyReader.__init__ != anybag_monkey:
            AnyReader.old_init = AnyReader.__init__
            AnyReader.__init__ = anybag_monkey

        if Reader2.__init__ != bag2_monkey:
            Reader2.old_init = Reader2.__init__
            Reader2.__init__ = bag2_monkey

        msg_text = """
        uint8[] buf
        """
        self._typestore.register(get_types_from_msg(msg_text, "ouster_sensor_msgs/msg/PacketMsg"))
        self._typestore.register(get_types_from_msg(msg_text, "ouster_msgs/msg/PacketMsg"))

        self._reader = AnyReader([Path(self._bag_path)], default_typestore=self._typestore)
        self._reader.open()

        connections = self._reader.connections
        pkt_msg_types = ['ouster_ros/msg/PacketMsg', "ouster_sensor_msgs/msg/PacketMsg", "ouster_msgs/msg/PacketMsg"]
        pkt_connections = [x for x in connections if x.msgtype in pkt_msg_types]
        imu_connections = [x for x in pkt_connections if "imu_packets" in x.topic]
        lidar_connections = [x for x in pkt_connections if "lidar_packets" in x.topic]
        metadata_connections = [x for x in connections if x.msgtype == 'std_msgs/msg/String' and "metadata" in x.topic]

        metadata_paths = None

        # now try and map topics to lidars,
        self._id_map = {}
        self._metadata = [None] * len(lidar_connections)  # type: ignore
        self._msg_connections = []
        # to do this lets go through lidar topics and find matching imu and metadata
        for idx, conn in enumerate(lidar_connections):
            namespace = '/'.join(conn.topic.split('/')[0:-1]) + "/"
            self._msg_connections.append(conn)
            self._id_map[conn.topic] = (idx, 0)
            for conn2 in imu_connections:
                if namespace in conn2.topic:
                    self._id_map[conn2.topic] = (idx, 1)
                    self._msg_connections.append(conn2)
                    break
            if meta is None:
                metadata_found = False
                for conn2 in metadata_connections:
                    if namespace in conn2.topic:
                        self._id_map[conn2.topic] = (idx, 2)
                        metadata_found = True
                        break
                if not metadata_found:
                    # try and fallback to metadata resolving if we couldnt find metadata in the bag
                    if metadata_paths is None:
                        metadata_paths = resolve_metadata_multi(bag_path)
                    if idx < len(metadata_paths):
                        meta = metadata_paths
                    else:
                        raise RuntimeError(f"ERROR could not find metadata for topic {conn.topic}")

        if meta is None:
            for connection, timestamp, rawdata in self._reader.messages(connections=metadata_connections):
                if connection.topic in self._id_map:
                    msg = self._reader.deserialize(rawdata, connection.msgtype)
                    self._metadata[self._id_map[connection.topic][0]] = SensorInfo(msg.data)  # type: ignore
        else:
            if len(meta) != len(self._metadata):
                raise ValueError(f"Incorrect number of metadata files provided. Expected"
                                 f" {len(self._metadata)} got {len(meta)}.")
            for idx, m in enumerate(meta):
                with open(m, 'r') as file:
                    self._metadata[idx] = SensorInfo(file.read())

        self._pf = []
        for m in self._metadata:
            pf = PacketFormat(m)
            self._pf.append(pf)

        # populate extrinsics
        populate_extrinsics(extrinsics_file or "", extrinsics or [], self._metadata)

    def __iter__(self) -> Iterator[Tuple[int, Packet]]:
        with self._lock:
            if self._reader is None:
                raise ValueError("I/O operation on closed packet source")

        self._reset = True
        while self._reset:
            self._reset = False
            for connection, timestamp, rawdata in self._reader.messages(connections=self._msg_connections):
                if self._reset:
                    break
                msg = self._reader.deserialize(rawdata, connection.msgtype)
                idx = self._id_map[connection.topic][0]
                msg_type = self._id_map[connection.topic][1]
                msg_len = len(msg.buf)
                packet: Union[LidarPacket, ImuPacket]
                if msg_type == 0:
                    msg_len = len(msg.buf)
                    if msg_len != self._pf[idx].lidar_packet_size:
                        # are we off by one? (older ouster-ros bags are off by 1)
                        if msg_len == self._pf[idx].lidar_packet_size + 1:
                            msg_len -= 1
                        else:
                            print(f"got an unexpected lidar packet size {msg_len} != "
                                  f"{self._pf[idx].lidar_packet_size} for sensor {idx}")
                            continue
                    packet = LidarPacket(msg_len)  # type: ignore
                    packet.buf[:] = msg.buf[:msg_len]  # type: ignore
                elif msg_type == 1:
                    if msg_len != self._pf[idx].imu_packet_size:
                        # are we off by one? (older ouster-ros bags are off by 1)
                        if msg_len == self._pf[idx].imu_packet_size + 1:
                            msg_len -= 1
                        else:
                            print(f"got an unexpected lidar packet size {msg_len} != "
                                  f"{self._pf[idx].imu_packet_size} for sensor {idx}")
                            continue
                    packet = ImuPacket(msg_len)  # type: ignore
                    packet.buf[:] = msg.buf[:msg_len]  # type: ignore
                else:
                    continue
                packet.host_timestamp = timestamp
                packet.format = self._pf[idx]

                res = packet.validate(self._metadata[idx], self._pf[idx])
                if res == PacketValidationFailure.NONE:
                    yield (idx, packet)
                elif res == PacketValidationFailure.PACKET_SIZE:
                    self._size_error_count += 1
                elif res == PacketValidationFailure.ID:
                    self._id_error_count += 1
                    if self._soft_id_check:
                        yield (idx, packet)

    @property
    def sensor_info(self) -> List[SensorInfo]:
        """Metadata associated with the packet."""
        return self._metadata

    @property
    def is_live(self) -> bool:
        return False

    @property
    def is_indexed(self) -> bool:
        return False

    # diagnostics
    @property
    def id_error_count(self) -> int:
        return self._id_error_count

    @property
    def size_error_count(self) -> int:
        return self._size_error_count

    def restart(self) -> None:
        """Restart playback, only relevant to non-live sources"""
        with self._lock:
            self._reset = True

    def close(self) -> None:
        """Release Pcap resources. Thread-safe."""
        with self._lock:
            self._reader.close()
            self._reader = None  # type: ignore

    @property
    def closed(self) -> bool:
        """Check if source is closed. Thread-safe."""
        with self._lock:
            return self._reader is None
