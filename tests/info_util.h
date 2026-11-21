#include <ouster/core/lidar_info.h>

std::shared_ptr<ouster::sdk::core::LidarInfo> fake_sensor_info(
    int h, int w, std::string profile = ouster::sdk::core::UDPProfileLidar::LEGACY,
    int columns_per_packet = 16) {
    ouster::sdk::core::LidarInfo info;
    info.format.columns_per_packet = columns_per_packet;
    info.format.pixels_per_column = h;
    info.format.columns_per_frame = w;
    info.format.udp_profile_lidar = profile;
    info.format.udp_profile_imu = ouster::sdk::core::UDPProfileIMU::LEGACY;
    info.fw_rev = "3.2.1";
    info.image_rev = "3.2.1";
    return std::make_shared<ouster::sdk::core::LidarInfo>(std::move(info));
}
