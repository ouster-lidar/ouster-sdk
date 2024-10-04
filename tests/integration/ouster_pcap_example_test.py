# Please note that the tests in `tests/integration` folder cannot be run on your system
# as they use external data (not included in the repository).

import os
from random import getrandbits, random
from itertools import chain

from ouster.sdk import client, pcap
from ouster.sdk.client import PacketFormat, _digest

TEST_DATA_DIR = os.getenv("TEST_DATA_DIR", default="./")

def test_simple_packet_record(tmpdir):
    """Simple example code snippet to record data using packet objects
       The buffers can be of variable size, you dont have to use
       pf.lidar_packet_size
       Args:
            tmpdir: Temporary directory to use
    """
    # ========= METADATA SECTION =========
    sensor_info_path = os.path.join(TEST_DATA_DIR,
                                    "lib-osf/OS1_128_sample_legacy.json")
    with open(sensor_info_path, 'r') as f:
        # SensorInfo accepts the string contents of the sensor info
        # metadata rather than a path
        metadata = client.SensorInfo(f.read())

    # Set up our packet sizes
    pf = PacketFormat.from_info(metadata)
    lidar_packet_size = pf.lidar_packet_size
    imu_packet_size = pf.imu_packet_size

    timestamp = 0

    # ========= BUFFER CREATION SECTION =========
    # Replace this with actual real data
    lidar_buffers = []
    for i in range(1, 10):
        # set random incremented timestamp to use (you can use actual value here)
        timestamp += random() * 4.0
        # generate random byte buffer for the packet
        buf = bytearray(getrandbits(8) for _ in range(lidar_packet_size))
        # create and append the lidar packet to our list
        lidar_buffers.append((buf, timestamp))

    imu_buffers = []
    for i in range(1, 10):
        # set random incremented timestamp to use (you can use actual value here)
        timestamp += random() * 4.0
        # generate random byte buffer for the packet
        buf = bytearray(getrandbits(8) for _ in range(imu_packet_size))
        # create and append the lidar packet to our list
        imu_buffers.append((buf, timestamp))

    # ========= PACKET CREATION SECTION =========
    lidar_packets = []
    for buf, timestamp in lidar_buffers:
        # Create the lidar packet
        packet = client.LidarPacket(lidar_packet_size)
        packet.buf[:] = buf[:]
        packet.host_timestamp = int(timestamp * 1e9)
        lidar_packets.append(packet)

    imu_packets = []
    for buf, timestamp in imu_buffers:
        # Create the imu packet
        packet = client.ImuPacket(imu_packet_size)
        packet.buf[:] = buf[:]
        packet.host_timestamp = int(timestamp * 1e9)
        imu_packets.append(packet)

    # ========= RECORD SECTION =========
    # concat the imu and lidar packets together
    packets_in_memory = list(chain(lidar_packets, imu_packets))

    # get our temp file
    file_path = os.path.join(tmpdir, "pcap_test.pcap")

    # write the packets to file, this takes in an iterator of
    # Packet objects and a file path
    pcap.record(packets_in_memory, file_path)

    # ========= TESTING SECTION =========
    # read the packets from our test file
    packets_from_file = list(pcap.Pcap(file_path, metadata))

    # grab the data
    bufs_in_memory = [bytes(p.buf) for p in packets_in_memory]
    bufs_from_file = [bytes(p.buf) for p in packets_from_file]

    # test that random packets read back from pcap are identical.
    assert bufs_in_memory == bufs_from_file
