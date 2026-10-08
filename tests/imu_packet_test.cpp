/**
 * Copyright (c) 2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "ouster/impl/packet_writer.h"
#include "ouster/types.h"

#ifndef _WIN32
#include <sys/mman.h>
#include <unistd.h>
#endif

using namespace ouster::sdk::core;

namespace {

PacketFormat legacy_imu_format(UDPProfileLidar lidar_profile) {
    auto info = default_sensor_info(LidarMode::_1024x10);
    info.format.udp_profile_lidar = lidar_profile;
    info.format.udp_profile_imu = UDPProfileIMU::LEGACY;
    return PacketFormat(info);
}

class LegacyImuTest : public ::testing::TestWithParam<UDPProfileLidar> {};

TEST_P(LegacyImuTest, ReadsAndWritesPreserveAdjacentFields) {
    const auto format = legacy_imu_format(GetParam());
    const impl::PacketWriter writer(format);
    // Padding makes this portable value test independent of allocation layout.
    std::array<uint8_t, 56> packet{};
    const std::array<uint64_t, 3> timestamps{{123456789, 234567890, 345678901}};
    std::memcpy(packet.data(), timestamps.data(), sizeof(timestamps));
    std::array<uint8_t, 56> expected = packet;
    const std::array<float, 6> values{{1.25f, -2.5f, 3.75f, -4.5f, 5.25f, -6.75f}};
    std::memcpy(expected.data() + 24, values.data(), sizeof(values));
    writer.set_imu_la_x(packet.data(), values[0]);
    writer.set_imu_la_y(packet.data(), values[1]);
    writer.set_imu_la_z(packet.data(), values[2]);
    writer.set_imu_av_x(packet.data(), values[3]);
    writer.set_imu_av_y(packet.data(), values[4]);
    writer.set_imu_av_z(packet.data(), values[5]);

    EXPECT_EQ(format.imu_packet_size, 48u);
    EXPECT_EQ(packet, expected);
    EXPECT_EQ(format.imu_sys_ts(packet.data()), timestamps[0]);
    EXPECT_EQ(format.imu_accel_ts(packet.data()), timestamps[1]);
    EXPECT_EQ(format.imu_gyro_ts(packet.data()), timestamps[2]);
    EXPECT_FLOAT_EQ(format.imu_la_x(packet.data()), values[0]);
    EXPECT_FLOAT_EQ(format.imu_la_y(packet.data()), values[1]);
    EXPECT_FLOAT_EQ(format.imu_la_z(packet.data()), values[2]);
    EXPECT_FLOAT_EQ(format.imu_av_x(packet.data()), values[3]);
    EXPECT_FLOAT_EQ(format.imu_av_y(packet.data()), values[4]);
    EXPECT_FLOAT_EQ(format.imu_av_z(packet.data()), values[5]);
}

INSTANTIATE_TEST_CASE_P(LidarProfiles, LegacyImuTest,
                       ::testing::Values(UDPProfileLidar::LEGACY,
                                         UDPProfileLidar::RNG19_RFL8_SIG16_NIR16));

#ifndef _WIN32
// A guard page catches the unaligned eight-byte access at byte 44 even when
// allocator padding or sanitizer instrumentation would hide the overrun.
class GuardedPacket {
   public:
    explicit GuardedPacket(size_t size) {
        const long page_size = sysconf(_SC_PAGESIZE);
        if (page_size <= 0 || size > static_cast<size_t>(page_size)) {
            throw std::runtime_error("invalid guard-page size");
        }
        length_ = static_cast<size_t>(page_size) * 2;
        mapping_ = mmap(nullptr, length_, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        if (mapping_ == MAP_FAILED) {
            throw std::runtime_error("cannot allocate guard page");
        }
        auto* end = static_cast<uint8_t*>(mapping_) + page_size;
        if (mprotect(end, page_size, PROT_NONE) != 0) {
            munmap(mapping_, length_);
            throw std::runtime_error("cannot protect guard page");
        }
        data = end - size;
    }

    ~GuardedPacket() { munmap(mapping_, length_); }
    GuardedPacket(const GuardedPacket&) = delete;
    GuardedPacket& operator=(const GuardedPacket&) = delete;

    uint8_t* data;

   private:
    void* mapping_;
    size_t length_;
};

class LegacyImuBoundsDeathTest
    : public ::testing::TestWithParam<UDPProfileLidar> {};

TEST_P(LegacyImuBoundsDeathTest, ReadStaysWithinPacket) {
    const auto format = legacy_imu_format(GetParam());
    ASSERT_EXIT(
        {
            GuardedPacket packet(format.imu_packet_size);
            const float value = -6.75f;
            std::memcpy(packet.data + 44, &value, sizeof(value));
            const float decoded = format.imu_av_z(packet.data);
            _exit(decoded == value ? 0 : 1);
        },
        ::testing::ExitedWithCode(0), "");
}

TEST_P(LegacyImuBoundsDeathTest, WriteStaysWithinPacket) {
    const impl::PacketWriter writer(legacy_imu_format(GetParam()));
    ASSERT_EXIT(
        {
            GuardedPacket packet(writer.imu_packet_size);
            writer.set_imu_av_z(packet.data, -6.75f);
            float decoded;
            std::memcpy(&decoded, packet.data + 44, sizeof(decoded));
            _exit(decoded == -6.75f ? 0 : 1);
        },
        ::testing::ExitedWithCode(0), "");
}

INSTANTIATE_TEST_CASE_P(LidarProfiles, LegacyImuBoundsDeathTest,
                       ::testing::Values(UDPProfileLidar::LEGACY,
                                         UDPProfileLidar::RNG19_RFL8_SIG16_NIR16));
#endif

}  // namespace
