import click
import os
import shutil
from pathlib import Path
from typing import (cast, Union, List, Iterable, Optional)
from ouster.sdk.core import (LidarScan,
                             SensorInfo,
                             LidarPacket, ImuPacket,
                             PacketSource)
from ouster.sdk.util import scan_to_packets  # type: ignore
from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand,
                          _nanos_to_string)

from ouster.sdk.bag.bag_packet_source import bag2_monkey, anybag_monkey  # type: ignore


@click.command
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def bag_info(ctx: SourceCommandContext, click_ctx: click.core.Context) -> None:
    """Print information about a BAG file to stdout.
    """

    from rosbags.highlevel import AnyReader
    from rosbags.rosbag2 import Reader as Reader2

    file = ctx.source_uri or ""

    # monkeypatch AnyReader and Reader2 to support mcap directly
    if AnyReader.__init__ != anybag_monkey:
        AnyReader.old_init = AnyReader.__init__  # type: ignore
        AnyReader.__init__ = anybag_monkey  # type: ignore

    if Reader2.__init__ != bag2_monkey:
        Reader2.old_init = Reader2.__init__  # type: ignore
        Reader2.__init__ = bag2_monkey  # type: ignore

    with AnyReader([Path(file)]) as reader:
        print(f"Filename: {file}")
        print(f"Start: {reader.start_time / 1e9} ({_nanos_to_string(reader.start_time)})")
        print(f"End: {reader.end_time / 1e9} ({_nanos_to_string(reader.end_time)})")
        print(f"Duration: {reader.duration / 1e9}")
        print(f"Message Count: {reader.message_count}")
        print("\nTopics:")
        for c in reader.connections:
            print(f"  {c.topic} ({c.msgtype}): {c.msgcount}")


def _source_to_bag_iter(source: Union[Iterable[List[Optional[LidarScan]]], PacketSource],
                       prefix: str = "", raw: bool = False,
                       dir: str = "", filename: str = "", overwrite: bool = False,
                       metadata: List[SensorInfo] = [],
                       duration = None, ros2 = False) -> Iterable[List[Optional[LidarScan]]]:
    """Create a ROSBAG saving iterator from a LidarScan iterator

    Requires a packet source otherwise, each LidarScan in scans is deparsed into UDP packets and saved.
    """

    from .source_save import create_directories_if_missing, determine_filename, _file_exists_error

    # Build filename
    filename = determine_filename(filename=filename, info=metadata[0], extension=".bag", prefix=prefix, dir=dir)

    if os.path.isfile(filename) and not overwrite:
        print(_file_exists_error(filename))
        exit(1)
    elif ros2 and os.path.isdir(filename) and not overwrite:
        print(_file_exists_error(filename))
        exit(1)
    elif overwrite:
        shutil.rmtree(filename, ignore_errors=True)
        try:
            os.remove(filename)
        except (FileNotFoundError, IsADirectoryError):
            pass

    create_directories_if_missing(filename)

    click.echo(f"Saving ROSBAG file at {filename}")

    from rosbags.rosbag2 import Writer as Writer2  # type: ignore
    from rosbags.rosbag1 import Writer as Writer  # type: ignore
    from rosbags.typesys import get_types_from_msg  # type: ignore
    from rosbags.typesys import Stores, get_typestore  # type: ignore

    if ros2:
        packet_type = "ouster_sensor_msgs/msg/PacketMsg"
        my_typestore = get_typestore(Stores.ROS2_FOXY)
        fn_serialize = my_typestore.serialize_cdr
        MyWriter = Writer2
        writer_params = {}  # type: ignore
    else:
        packet_type = "ouster_ros/msg/PacketMsg"
        my_typestore = get_typestore(Stores.ROS1_NOETIC)
        fn_serialize = my_typestore.serialize_ros1  # type: ignore
        MyWriter = Writer  # type: ignore
        writer_params = {}

    msg_text = """
    uint8[] buf
    """
    my_typestore.register(get_types_from_msg(msg_text, packet_type))

    StringMsg = my_typestore.types['std_msgs/msg/String']
    PacketMsg = my_typestore.types[packet_type]

    def bag_save_metadata(writer, metadata, timestamp):
        for i, m in enumerate(metadata):
            topic = f"/ouster{i}/metadata" if len(metadata) > 1 else "/ouster/metadata"
            conn = writer.add_connection(topic, "std_msgs/msg/String", typestore=my_typestore)
            msg = StringMsg(m.to_json_string())
            serialized = fn_serialize(msg, "std_msgs/msg/String")
            writer.write(conn, timestamp, serialized)

    def create_connections(writer, metadata):
        l = []
        for i, m in enumerate(metadata):
            imu_topic = f"/ouster{i}/imu_packets" if len(metadata) > 1 else "/ouster/imu_packets"
            lidar_topic = f"/ouster{i}/lidar_packets" if len(metadata) > 1 else "/ouster/lidar_packets"
            imu_conn = writer.add_connection(imu_topic, packet_type, typestore=my_typestore)
            lidar_conn = writer.add_connection(lidar_topic, packet_type, typestore=my_typestore)
            l.append((lidar_conn, imu_conn))
        return l

    def bag_save_packet(writer, conns, packet):
        msg = PacketMsg(packet.buf)
        data = fn_serialize(msg, packet_type)
        if isinstance(packet, LidarPacket):
            writer.write(conns[0], packet.host_timestamp, data)
        elif isinstance(packet, ImuPacket):
            writer.write(conns[1], packet.host_timestamp, data)

    if not raw:
        click.echo("Warning: Saving bag without using save_raw will not save LEGACY IMU packets.")

        def save_iter():
            try:
                first = True
                with MyWriter(filename, **writer_params) as writer:
                    conns = create_connections(writer, metadata)
                    for l in source():
                        for idx, scan in enumerate(l):
                            if scan:
                                packets = scan_to_packets(scan, metadata[idx])
                                for packet in packets:
                                    bag_save_packet(writer, conns[idx], packet)

                                    if first:
                                        first = False
                                        bag_save_metadata(writer, metadata, packet.host_timestamp)
                        yield l
            except (KeyboardInterrupt, StopIteration):
                writer.close()

        # type ignored because generators are tricky to mypy
        return save_iter  # type: ignore
    else:
        end_time = None
        try:
            first = True
            with MyWriter(filename, **writer_params) as writer:  # type: ignore
                conns = create_connections(writer, metadata)
                source = cast(PacketSource, source)
                for idx, packet in source:
                    bag_save_packet(writer, conns[idx], packet)
                    if first:
                        first = False
                        bag_save_metadata(writer, metadata, packet.host_timestamp)

                    if duration is not None:
                        if end_time is None:
                            end_time = packet.host_timestamp + duration * 1e9
                        if packet.host_timestamp > end_time:
                            break
        except (KeyboardInterrupt, StopIteration):
            writer.close()
        return None  # type: ignore
