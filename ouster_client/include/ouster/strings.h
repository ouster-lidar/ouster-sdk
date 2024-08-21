/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <array>
#include <string>

namespace ouster {
namespace strings {

/**
 * Channel strings, used as keys for LidarScan
 */
namespace channel {

const std::string RANGE = "RANGE";
const std::string RANGE2 = "RANGE2";
const std::string INTENSITY = "INTENSITY";
const std::string SIGNAL = "SIGNAL";
const std::string SIGNAL2 = "SIGNAL2";
const std::string REFLECTIVITY = "REFLECTIVITY";
const std::string REFLECTIVITY2 = "REFLECTIVITY2";
const std::string AMBIENT = "AMBIENT";
const std::string NEAR_IR = "NEAR_IR";
const std::string FLAGS = "FLAGS";
const std::string FLAGS2 = "FLAGS2";
const std::string RAW_HEADERS = "RAW_HEADERS";
const std::string RAW32_WORD1 = "RAW32_WORD1";
const std::string RAW32_WORD2 = "RAW32_WORD2";
const std::string RAW32_WORD3 = "RAW32_WORD3";
const std::string RAW32_WORD4 = "RAW32_WORD4";
const std::string RAW32_WORD5 = "RAW32_WORD5";
const std::string RAW32_WORD6 = "RAW32_WORD6";
const std::string RAW32_WORD7 = "RAW32_WORD7";
const std::string RAW32_WORD8 = "RAW32_WORD8";
const std::string RAW32_WORD9 = "RAW32_WORD9";

}  // namespace channel

/**
 * Header strings, used as keys for LidarScan
 */
namespace header {

const std::string timestamp = "timestamp";
const std::string measurement_id = "measurement_id";
const std::string status = "status";
const std::string packet_timestamp = "packet_timestamp";
const std::string pose = "pose";

const std::array<std::string, 5> all = {header::timestamp,
                                        header::measurement_id, header::status,
                                        header::packet_timestamp, header::pose};

}  // namespace header

// clang-format off
const std::array<std::string, 26> all = {
    /* channels */
    channel::RANGE,
    channel::RANGE2,
    channel::INTENSITY,
    channel::SIGNAL,
    channel::SIGNAL2,
    channel::REFLECTIVITY,
    channel::REFLECTIVITY2,
    channel::AMBIENT,
    channel::NEAR_IR,
    channel::FLAGS,
    channel::FLAGS2,
    channel::RAW_HEADERS,
    channel::RAW32_WORD1,
    channel::RAW32_WORD2,
    channel::RAW32_WORD3,
    channel::RAW32_WORD4,
    channel::RAW32_WORD5,
    channel::RAW32_WORD6,
    channel::RAW32_WORD7,
    channel::RAW32_WORD8,
    channel::RAW32_WORD9,
    /* headers */
    header::timestamp,
    header::measurement_id,
    header::status,
    header::packet_timestamp,
    header::pose
};
// clang-format on

}  // namespace strings
}  // namespace ouster
