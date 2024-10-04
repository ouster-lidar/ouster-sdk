# type: ignore
from typing import Iterator, List, Tuple, Union, Optional
from ouster.sdk.client import PacketMultiSource

from threading import Lock
import logging
from ouster.sdk.client import SensorInfo, PacketFormat, PacketValidationFailure, LidarPacket, ImuPacket, Packet

from pathlib import Path

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


class BagPacketSource(PacketMultiSource):
    """Read a sensors packet streams out of a bag file as an iterator."""

    _metadata: List[SensorInfo]

    def __init__(self,
                 bag_path: str,
                 *,
                 meta: Optional[List[str]] = None,
                 soft_id_check: bool = False):
        """Read sensor data streams from a single bag file.

        Args:
            bag_path: path to bag file or folder containing ROS2 db3 and yaml file
            meta: optional list of metadata files to load, if not provided metadata
                is loaded from the bag instead
            soft_id_check: if True, don't skip packets on init_id mismatch
        """

        self._soft_id_check = soft_id_check
        self._id_error_count = 0
        self._size_error_count = 0
        self._lock = Lock()
        self._reset = False
        self._typestore = get_typestore(Stores.ROS2_FOXY)
        self._bag_path = bag_path

        msg_text = """
        uint8[] buf
        """
        self._typestore.register(get_types_from_msg(msg_text, "ouster_sensor_msgs/msg/PacketMsg"))

        self._reader = AnyReader([Path(bag_path)], default_typestore=self._typestore)
        self._reader.open()

        connections = self._reader.connections
        pkt_msg_types = ['ouster_ros/msg/PacketMsg', "ouster_sensor_msgs/msg/PacketMsg"]
        pkt_connections = [x for x in connections if x.msgtype in pkt_msg_types]
        imu_connections = [x for x in pkt_connections if "imu_packets" in x.topic]
        lidar_connections = [x for x in pkt_connections if "lidar_packets" in x.topic]
        metadata_connections = [x for x in connections if x.msgtype == 'std_msgs/msg/String' and "metadata" in x.topic]

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
                type = self._id_map[connection.topic][1]

                packet: Union[LidarPacket, ImuPacket]
                if type == 0:
                    packet = LidarPacket(len(msg.buf))  # type: ignore
                elif type == 1:
                    packet = ImuPacket(len(msg.buf))  # type: ignore
                else:
                    continue
                packet.buf[:] = msg.buf  # type: ignore
                packet.host_timestamp = timestamp

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
    def metadata(self) -> List[SensorInfo]:
        """Metadata associated with the packet."""
        return self._metadata

    @property
    def is_live(self) -> bool:
        return False

    @property
    def is_seekable(self) -> bool:
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
