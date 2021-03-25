import os
import tempfile
import unittest
import ouster.client._pcap as ouster_pcap


def run_pcap(test_file,
             total_complete_packets,
             total_size,
             frag_size,
             port,
             dst_port,
             record=None):
    '''
    This function runs the pcap tests given various options
    Parameters:
        test_file: The output file to record pcap to
        total_complete_packets: The total number of completed packets to record
        total_size: The number of bytes to record
        frag_size: The fragmentation size in bytes
        port: The port to record to
        dst_port: The target port to record to
        record: If we have a previous record object, dont recreate it
    '''
    if record is None:
        record = ouster_pcap.record_initialize(test_file, "127.0.0.1", "127.0.0.1", frag_size)
    frag_size = min(frag_size, total_size)
    frags = max(int((total_size) / frag_size), 1)
    result = []
    for i in range(0, total_complete_packets):
        data_out = []
        for j in range(0, frags):
            data_out.append(i)
            data_out.append(j)
            for k in range(0, (frag_size - 2)):
                data_out.append(k % 255)
        ouster_pcap.record_packet(record, port, dst_port, bytearray(data_out))

        result.extend(data_out)
        result.extend([0 for x in range(len(result), (i + 1) * total_size)])

    return (bytearray(result), record)


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
ports = {}


def initialize_test():
    tmp_dir = tempfile.mkdtemp()
    file_path = os.path.join(tmp_dir, "pcap_test.pcap")

    return tmp_dir, file_path


def uninitialize_test(tmp_dir, file_path):
    os.remove(file_path)
    os.rmdir(tmp_dir)


def read_write_lidar(cls, packets):
    tmp_dir, file_path = initialize_test()

    data, record = run_pcap(file_path, packets, lidar_size, frag_size, lidar_src_port, lidar_dst_port)
    del record  # Delete the record object to write it to file

    result = bytearray()
    pcap_read = ouster_pcap.replay_initialize(file_path, src_address,
                                              dst_address, ports)
    info = ouster_pcap.packet_info()
    while ouster_pcap.next_packet_info(pcap_read, info):
        ouster_pcap.read_packet(pcap_read, lidar_buf)
        result += lidar_buf

    cls.assertEqual(data, result)
    del pcap_read
    uninitialize_test(tmp_dir, file_path)


def read_write_imu(cls, packets):
    tmp_dir, file_path = initialize_test()
    data, record = run_pcap(file_path, packets, imu_size, frag_size,
                            imu_src_port, imu_dst_port)
    del record  # Delete the record object to write it to file

    result = bytearray()
    pcap_read = ouster_pcap.replay_initialize(file_path, src_address,
                                              dst_address, ports)
    info = ouster_pcap.packet_info()
    while ouster_pcap.next_packet_info(pcap_read, info):
        ouster_pcap.read_packet(pcap_read, imu_buf)
        result += imu_buf

    cls.assertEqual(data, result)
    del pcap_read
    uninitialize_test(tmp_dir, file_path)


def read_write_lidar_imu(cls, packets):
    tmp_dir, file_path = initialize_test()
    lidar_data, record = run_pcap(file_path, packets, lidar_size, frag_size, lidar_src_port, lidar_dst_port)
    imu_data, record = run_pcap(file_path, packets, imu_size, frag_size, imu_src_port, imu_dst_port, record=record)
    del record  # Delete the record object to write it to file
    lidar_result = bytearray()
    imu_result = bytearray()
    pcap_read = ouster_pcap.replay_initialize(file_path, src_address,
                                              dst_address, ports)

    info = ouster_pcap.packet_info()
    while ouster_pcap.next_packet_info(pcap_read, info):
        if info.dst_port == imu_dst_port:
            ouster_pcap.read_packet(pcap_read, imu_buf)
            imu_result += imu_buf
        if info.dst_port == lidar_dst_port:
            ouster_pcap.read_packet(pcap_read, lidar_buf)
            lidar_result += lidar_buf

    cls.assertEqual(lidar_data, lidar_result)
    cls.assertEqual(imu_data, imu_result)

    del pcap_read
    uninitialize_test(tmp_dir, file_path)


# NOTE: Until we move away from winpcap this test uses a terrible hack located in the pcap code
class TestPcap(unittest.TestCase):
    def test_pcap_read_write_lidar(self):
        read_write_lidar(self, 1)

    def test_pcap_read_write_multi_packets_lidar(self):
        read_write_lidar(self, 20)

    def test_pcap_read_write_imu(self):
        read_write_imu(self, 1)

    def test_pcap_read_write_multi_packets_imu(self):
        read_write_imu(self, 20)

    def test_pcap_read_write_lidar_imu(self):
        read_write_lidar_imu(self, 1)

    def test_pcap_read_write_multi_packets_lidar_imu(self):
        read_write_lidar_imu(self, 20)


if __name__ == '__main__':
    unittest.main()
