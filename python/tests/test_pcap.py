import os
import tempfile

import pytest

import ouster.pcap._pcap as _pcap


def write_test_pcap(test_file,
                    total_complete_packets,
                    total_size,
                    frag_size,
                    port,
                    dst_port,
                    record=None):
    '''Write bitpatterns of the specified size to a pcap file.

    Args:
        test_file: The output file to record pcap to
        total_complete_packets: The total number of completed packets to record
        total_size: The number of bytes to record
        frag_size: The fragmentation size in bytes
        port: The port to record to
        dst_port: The target port to record to
        record: If we have a previous record object, dont recreate it
    '''
    if record is None:
        record = _pcap.record_initialize(test_file, "127.0.0.1", "127.0.0.1",
                                         frag_size)
    frag_size = min(frag_size, total_size)
    frags = max(int(total_size / frag_size), 1)
    result = []
    for i in range(0, total_complete_packets):
        data_out = []
        for j in range(0, frags):
            data_out.append(i)
            data_out.append(j)
            for k in range(0, (frag_size - 2)):
                data_out.append(k % 255)
        _pcap.record_packet(record, port, dst_port, bytearray(data_out))

        result.extend(data_out)
        result.extend([0 for x in range(len(result), (i + 1) * total_size)])

    return (bytearray(result), record)


@pytest.mark.parametrize("lidar_packets, imu_packets", [
    pytest.param(1, 0, id="one lidar"),
    pytest.param(20, 0, id="multi lidar"),
    pytest.param(0, 1, id="one imu"),
    pytest.param(0, 20, id="multi imu"),
    pytest.param(1, 1, id="one each"),
    pytest.param(20, 20, id="multi each"),
])
def test_read_write_lidar_imu(lidar_packets, imu_packets):
    lidar_size = 12608
    imu_size = 48
    frag_size = 1480
    lidar_src_port = 7000
    lidar_dst_port = 7001
    imu_src_port = 8000
    imu_dst_port = 8001
    src_address = "127.0.0.1"
    dst_address = src_address
    lidar_buf = bytearray(lidar_size)
    imu_buf = bytearray(imu_size)

    tmp_dir = tempfile.mkdtemp()
    file_path = os.path.join(tmp_dir, "pcap_test.pcap")

    try:
        lidar_data, record = write_test_pcap(file_path, lidar_packets,
                                             lidar_size, frag_size,
                                             lidar_src_port, lidar_dst_port)
        imu_data, record = write_test_pcap(file_path,
                                           imu_packets,
                                           imu_size,
                                           frag_size,
                                           imu_src_port,
                                           imu_dst_port,
                                           record=record)
        _pcap.record_uninitialize(record)
        lidar_result = bytearray()
        imu_result = bytearray()
        pcap_read = _pcap.replay_initialize(file_path, src_address,
                                            dst_address, {})

        info = _pcap.packet_info()
        while _pcap.next_packet_info(pcap_read, info):
            if info.dst_port == imu_dst_port:
                _pcap.read_packet(pcap_read, imu_buf)
                imu_result += imu_buf
            if info.dst_port == lidar_dst_port:
                _pcap.read_packet(pcap_read, lidar_buf)
                lidar_result += lidar_buf

        assert lidar_data == lidar_result
        assert imu_data == imu_result

        _pcap.replay_uninitialize(pcap_read)
    finally:
        os.remove(file_path)
        os.rmdir(tmp_dir)
