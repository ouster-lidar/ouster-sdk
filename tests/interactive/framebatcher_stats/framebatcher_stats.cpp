#include <iostream>

#include "ouster/sensor/sensor_frame_set_source.h"

using namespace ouster::sdk;

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <packet_source> <iterations> <cache size, packets>"
                  << std::endl;
        return 1;
    }
    auto source = open_packet_source(argv[1]);
    auto iterations = std::atoi(argv[2]);
    if (iterations <= 0) {
        std::cerr << "Iterations must be a positive integer." << std::endl;
        return 1;
    }
    auto cache_size = std::atoi(argv[3]);
    auto sensor_info = source.sensor_info()[0];
    core::FrameBatcher batcher(sensor_info);
    batcher.set_max_cache_size(static_cast<size_t>(cache_size));
    core::LidarFrame frame(sensor_info);
    size_t total_frames = 0;
    size_t complete_frames = 0;
    size_t partial_frames = 0;
    bool first_frame = true;
    size_t last_imu_ts_zero = 0;
    size_t last_zm_ts_zero = 0;

    for (const auto& idx_and_packet : source) {
        const auto& packet = idx_and_packet.second;
        bool frame_finalized = batcher(*packet, frame);
        if (frame_finalized && first_frame) {
            first_frame = false;
            frame = core::LidarFrame(sensor_info);
            continue;
        }

        if (frame_finalized) {
            total_frames++;
            Eigen::Ref<Eigen::Array<uint64_t, -1, 1> > lidar_packet_timestamps =
                frame.packet_timestamp();
            Eigen::Ref<Eigen::Array<uint64_t, -1, 1> > imu_packet_timestamps =
                frame.field("IMU_PACKET_TIMESTAMP");
            Eigen::Ref<Eigen::Array<uint64_t, -1, 1> > zm_packet_timestamps =
                frame.field("ZONE_PACKET_TIMESTAMP");
            auto zero_lidar_ts = lidar_packet_timestamps.rows() - lidar_packet_timestamps.count();
            auto zero_imu_ts = imu_packet_timestamps.rows() - imu_packet_timestamps.count();
            auto zero_zm_ts = zm_packet_timestamps.rows() - zm_packet_timestamps.count();
            if (zero_lidar_ts + zero_imu_ts + zero_zm_ts > 0) {
                std::cerr << "Frame " << total_frames
                          << ": Zero timestamps - Lidar: " << zero_lidar_ts
                          << ", IMU: " << zero_imu_ts << ", ZM: " << zero_zm_ts << std::endl;
                partial_frames++;
            } else {
                std::cerr << "Frame " << total_frames << std::endl;
                complete_frames++;
            }
            if (imu_packet_timestamps(imu_packet_timestamps.size() - 1) == 0) {
                last_imu_ts_zero++;
            }
            if (zm_packet_timestamps(zm_packet_timestamps.size() - 1) == 0) {
                last_zm_ts_zero++;
            }
            if (total_frames >= static_cast<size_t>(iterations)) {
                break;
            }
            // reset the frame to account for a bug
            frame = core::LidarFrame(sensor_info);
        }
    }
    std::cerr << "Total frames processed: " << total_frames << std::endl;
    std::cerr << "Complete frames: " << complete_frames << std::endl;
    std::cerr << "Partial frames: " << partial_frames << std::endl;
    std::cerr << "Frames with last IMU timestamp zero: " << last_imu_ts_zero << std::endl;
    std::cerr << "Frames with last ZM timestamp zero: " << last_zm_ts_zero << std::endl;
    std::cerr << "" << std::endl;
    std::cerr << "Dropped packets: " << batcher.dropped_packets() << std::endl;
}
