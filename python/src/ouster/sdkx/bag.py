#  type: ignore
"""ROS bag utilities (requires ROS environment to import)"""

import rosbag
import rospy   # noqa: F401
import genpy
import collections

from ouster import client

import time

from typing import List, Iterator

# MD5 sum of PacketMsg
PACKETMSG_MD5 = '4f7b5949e76f86d01e96b0e33ba9b5e3'

# Define ouster_ros/PacketMsg
MsgInfo = collections.namedtuple('MsgInfo', 'datatype msg_def md5sum')
PacketMsgInfo = MsgInfo(datatype="ouster_ros/PacketMsg",
                        msg_def="uint8[] buf",
                        md5sum=PACKETMSG_MD5)
PacketMsg = genpy.dynamic.generate_dynamic(
    PacketMsgInfo.datatype, PacketMsgInfo.msg_def)[PacketMsgInfo.datatype]
assert PacketMsgInfo.md5sum == PacketMsg._md5sum


def get_lidar_imu_topics(bag_path: str) -> List[str]:
    bag = rosbag.Bag(bag_path, 'r')

    type_and_topic_info = bag.get_type_and_topic_info()
    topics = type_and_topic_info.topics

    ouster_msg_topics = ['{}_packets'.format(pkt) for pkt in ['lidar', 'imu']]

    selected_topics = list()

    for topic in topics:
        suffix = topic.split('/')[-1]
        if suffix in ouster_msg_topics:
            msg_type = topics[topic].msg_type
            if type_and_topic_info.msg_types[msg_type] == PACKETMSG_MD5:
                selected_topics.append(topic)

    return sorted(selected_topics)


class BagSource(client.PacketSource):
    """Read a sensor packet stream out of a ROS bag file as an iterator."""
    _topics: List[str]
    _metadata: client.SensorInfo
    _rate: float
    _bag: rosbag.Bag

    def __init__(self,
                bag_path: str,
                info: client.SensorInfo,
                *,
                rate: float = 0.0,
                lidar_topic: str = "",
                imu_topic: str = "") -> None:

        if not lidar_topic and not imu_topic:
            # Use first lidar/imu_packets topics if not set anything in ctor
            topics = get_lidar_imu_topics(bag_path)
            self._topics = []
            self._topics.extend(next(([t] for t in topics if t.endswith("lidar_packets")), []))
            self._topics.extend(next(([t] for t in topics if t.endswith("imu_packets")), []))
        else:
            self._topics = [t for t in [lidar_topic, imu_topic] if t]

        self._bag = rosbag.Bag(bag_path, 'r')
        self._metadata = info
        self._rate = rate

    def __iter__(self) -> Iterator[client.Packet]:
        real_start_ts = time.monotonic()
        bag_start_ts = None
        for m in self._bag.read_messages(topics = self._topics):
            msg_ts_sec = m.timestamp.to_sec()

            if self._rate:
                if not bag_start_ts:
                    bag_start_ts = msg_ts_sec
                real_delta = time.monotonic() - real_start_ts
                bag_delta = (msg_ts_sec -
                             bag_start_ts) / self._rate
                delta = max(0, bag_delta - real_delta)
                time.sleep(delta)

            if m.topic.endswith("lidar_packets"):
                msg_buf = m.message.buf
                yield client.LidarPacket(msg_buf, self._metadata, msg_ts_sec)

            elif m.topic.endswith("imu_packets"):
                msg_buf = m.message.buf
                yield client.ImuPacket(msg_buf, self._metadata, msg_ts_sec)

    @property
    def topics(self) -> List[str]:
        return self._topics

    @property
    def metadata(self) -> client.SensorInfo:
        return self._metadata

    def close(self) -> None:
        self._bag.close()
