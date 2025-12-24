#pragma once

#include <string>

// clang-format off
#include "chunk_generated.h"
#include "header_generated.h"
#include "metadata_generated.h"
#include "os_sensor/common_generated.h"
#include "os_sensor/collation_stream_generated.h"
#include "os_sensor/lidar_scan_stream_generated.h"
#include "os_sensor/lidar_sensor_generated.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"
// clang-format on

namespace ouster {
namespace sdk {

/**
 * %OSF v2 space
 */
namespace osf {
namespace impl {
namespace gen {
using namespace v2;
}  // namespace gen

// stable common types mapped to ouster::sdk::osf
using v2::HEADER_STATUS;

/**
 * To String Functionality For HEADER_STATUS
 *
 * @param[in] status The data to get the string representation format
 * @return The string representation
 */
std::string to_string(const HEADER_STATUS status);

}  // namespace impl
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
