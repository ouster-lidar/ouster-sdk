#include <iostream>

#include "ouster/osf/osf.h"
#include "ouster/osf/util.h"

using MessageType = ouster::OSF::MessageType;

void parse(std::string file) {
    // Open the file
    auto osf_file = ouster::OSF::OsfFile(file);

    ouster::OSF::Reader reader(osf_file);
    ouster::OSF::FileInfo file_info = reader.file_info();  // Get file info

    // All exposed timestamps typed as nanoseconds
    std::cout << "file_info.start_ts = " << file_info.start_ts().count()
              << std::endl;

    int imu_c = 0;
    int gps_c = 0;
    int traj_c = 0;
    int ls_c = 0;
    int ext_c = 0;

    // lightweight interface to the message content
    using MessageRef = ouster::OSF::MessageRef;

    const auto frame_keys = reader.file_info().frames_keys();

    // Loop over frames/chunks
    for (const auto frame_key : frame_keys) {
        ouster::OSF::ts_t frame_start_ts = reader.start_ts(frame_key);
        ouster::OSF::ts_t frame_end_ts = reader.end_ts(frame_key);

        std::cout << "Chunk id: " << frame_key
                  << ", start_ts = " << frame_start_ts.count()
                  << ", end_ts = " << frame_end_ts.count() << std::endl;

        // Loop over message in frame/chunk
        for (const MessageRef& m : reader.messages(frame_key)) {
            if (m.type() == MessageType::LIDAR_SCAN) {
                std::cout << "  Ls     ts: " << m.ts().count();
                ls_c++;
            }
            if (m.type() == MessageType::TRAJECTORY) {
                std::cout << "  Tr     ts: " << m.ts().count();
                traj_c++;
            }
            if (m.type() == MessageType::IMU) {
                std::cout << "  Im     ts: " << m.ts().count();
                imu_c++;
            }
            if (m.type() == MessageType::GPS_WAYPOINT) {
                std::cout << "  Gp     ts: " << m.ts().count();
                gps_c++;
            }
            if (m.type() == MessageType::MESSAGE_EXTENSION) {
                std::cout << "  MsgExt ts: " << m.ts().count();
                ext_c++;
            }
            std::cout << ", id = " << (int)m.id() << std::endl;
        }
    }

    std::cout << "\nSUMMARY: \n";
    std::cout << "  lidar_scan     (Ls)     count = " << ls_c << std::endl;
    std::cout << "  trajectory     (Tr)     count = " << traj_c << std::endl;
    std::cout << "  imu            (Im)     count = " << imu_c << std::endl;
    std::cout << "  gps_waypoint   (Gp)     count = " << gps_c << std::endl;
    std::cout << "  msg extensions (MsgExt) count = " << ext_c << std::endl;

}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Expected OSF file as input." << std::endl;
        return EXIT_FAILURE;
    }
    parse(argv[1]);
}
