#include <ouster/client.h>
#include <ouster/lidar_scan.h>
#include <ouster/os_pcap.h>

#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "\n\nUsage: pcap_test <pcap_file>" << std::endl;

        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    const std::string pcap_file = argv[1];
    auto stream_info = ouster::sensor_utils::get_stream_info(pcap_file);
    std::cout << *stream_info << std::endl;
}
