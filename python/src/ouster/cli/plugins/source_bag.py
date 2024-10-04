# type: ignore
import click
import os
import shutil
from pathlib import Path
from typing import (cast, Union, List, Iterator, Optional)
from ouster.sdk.client import (LidarScan,
                               SensorInfo,
                               LidarPacket, ImuPacket,
                               PacketMultiSource)
from ouster.sdk.util import scan_to_packets  # type: ignore
from .source_util import (SourceCommandContext,
                          SourceCommandType,
                          source_multicommand,
                          _nanos_to_string)

from ouster.sdk.bag import BagPacketSource


@click.command
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def bag_info(ctx: SourceCommandContext, click_ctx: click.core.Context) -> None:
    """Print information about a BAG file to stdout.
    """

    from rosbags.highlevel import AnyReader

    file = ctx.source_uri or ""

    with AnyReader([Path(file)]) as reader:
        print(f"Filename: {file}")
        print(f"Start: {reader.start_time / 1e9} ({_nanos_to_string(reader.start_time)})")
        print(f"End: {reader.end_time / 1e9} ({_nanos_to_string(reader.end_time)})")
        print(f"Duration: {reader.duration / 1e9}")
        print(f"Message Count: {reader.message_count}")
        print("\nTopics:")
        for c in reader.connections:
            print(f"  {c.topic} ({c.msgtype}): {c.msgcount}")


@click.command
@click.option('-n',
              type=int,
              default=0,
              help="Index of lidar",
              show_default=True)
@click.pass_context
@source_multicommand(type=SourceCommandType.MULTICOMMAND_UNSUPPORTED,
                     retrieve_click_context=True)
def bag_metadata(ctx: SourceCommandContext, click_ctx: click.core.Context, n: int) -> None:
    """
    Display sensor metadata about the SOURCE.
    """
    file = ctx.source_uri or ""
    md = BagPacketSource(file).metadata
    if n < 0 or n >= len(md):
        raise click.ClickException(f"Sensor Index {n} Not Found")
    print(md[n].to_json_string())


def _source_to_bag_iter(source: Union[Iterator[List[Optional[LidarScan]]], PacketMultiSource],
                       prefix: str = "", raw: bool = False,
                       dir: str = "", filename: str = "", overwrite: bool = False,
                       metadata: List[SensorInfo] = [],
                       duration = None, ros2 = False) -> Iterator[List[Optional[LidarScan]]]:
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
        writer_params = {}
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
            topic = f"/os_node{i}/metadata" if len(metadata) > 1 else "/os_node/metadata"
            conn = writer.add_connection(topic, "std_msgs/msg/String", typestore=my_typestore)
            msg = StringMsg(m.to_json_string())
            serialized = fn_serialize(msg, "std_msgs/msg/String")
            writer.write(conn, timestamp, serialized)

    def create_connections(writer, metadata):
        l = []
        for i, m in enumerate(metadata):
            imu_topic = f"/os_node{i}/imu_packets" if len(metadata) > 1 else "/os_node/imu_packets"
            lidar_topic = f"/os_node{i}/lidar_packets" if len(metadata) > 1 else "/os_node/lidar_packets"
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
                    for l in source:
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
                pass

        return save_iter()
    else:
        end_time = None
        try:
            first = True
            with MyWriter(filename, **writer_params) as writer:  # type: ignore
                conns = create_connections(writer, metadata)
                source = cast(PacketMultiSource, source)
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
            pass
        return None  # type: ignore
