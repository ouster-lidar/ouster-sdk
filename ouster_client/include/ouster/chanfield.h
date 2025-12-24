/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "ouster/deprecation.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Channel field identifier type.
 */
using cf_type = const char*;
/**
 * @namespace ChanField
 * Tag to identitify a paricular value reported in the sensor channel data
 * block. */
namespace ChanField {
static constexpr cf_type RANGE = "RANGE";      ///< 1st return range in mm
static constexpr cf_type RANGE2 = "RANGE2";    ///< 2nd return range in mm
static constexpr cf_type SIGNAL = "SIGNAL";    ///< 1st return signal in photons
static constexpr cf_type SIGNAL2 = "SIGNAL2";  ///< 2nd return signal in photons
static constexpr cf_type REFLECTIVITY =
    "REFLECTIVITY";  ///< 1st return reflectivity, calibrated by range and
                     ///< sensor sensitivity in FW 2.1+. See sensor docs for
                     ///< more details
static constexpr cf_type REFLECTIVITY2 =
    "REFLECTIVITY2";  ///< 2nd return reflectivity, calibrated by range and
                      ///< sensor sensitivity in FW 2.1+. See sensor docs for
                      ///< more details

static constexpr cf_type NEAR_IR = "NEAR_IR";    ///< near_ir in photons
static constexpr cf_type FLAGS = "FLAGS";        ///< 1st return flags
static constexpr cf_type FLAGS2 = "FLAGS2";      ///< 2nd return flags
static constexpr cf_type NORMALS = "NORMALS";    ///< 1st return normal values
static constexpr cf_type NORMALS2 = "NORMALS2";  ///< 2nd return normal values
static constexpr cf_type WINDOW = "WINDOW";      ///< window blockage
static constexpr cf_type ZONE_MASK =
    "ZONE_MASK";  ///< per pixel mask describing zone occupancy
static constexpr cf_type RAW_HEADERS =
    "RAW_HEADERS";  ///< raw headers for packet/footer/column for dev use
static constexpr cf_type RAW32_WORD5 =
    "RAW32_WORD5";  ///< raw word access to packet for dev use
static constexpr cf_type RAW32_WORD6 =
    "RAW32_WORD6";  ///< raw word access to packet for dev use
static constexpr cf_type RAW32_WORD7 =
    "RAW32_WORD7";  ///< raw word access to packet for dev use
static constexpr cf_type RAW32_WORD8 =
    "RAW32_WORD8";  ///< raw word access to packet for dev use
static constexpr cf_type RAW32_WORD9 =
    "RAW32_WORD9";  ///< raw word access to packet for dev use
static constexpr cf_type RAW32_WORD1 =
    "RAW32_WORD1";  ///< raw word access to packet for dev use
static constexpr cf_type RAW32_WORD2 =
    "RAW32_WORD2";  ///< raw word access to packet for dev use
static constexpr cf_type RAW32_WORD3 =
    "RAW32_WORD3";  ///< raw word access to packet for dev use
static constexpr cf_type RAW32_WORD4 =
    "RAW32_WORD4";  ///< raw word access to packet for dev use

static constexpr cf_type IMU_ACC = "IMU_ACC";  ///< IMU accelerometer read in g
static constexpr cf_type IMU_GYRO =
    "IMU_GYRO";  ///< IMU gyroscope read in degrees/second
static constexpr cf_type IMU_TIMESTAMP =
    "IMU_TIMESTAMP";  ///< timestamps corresponding to IMU measurements
static constexpr cf_type IMU_MEASUREMENT_ID =
    "IMU_MEASUREMENT_ID";  ///< measurement IDs corresponding to IMU reads
static constexpr cf_type IMU_STATUS =
    "IMU_STATUS";  ///< status of the imu measurement
static constexpr cf_type IMU_PACKET_TIMESTAMP =
    "IMU_PACKET_TIMESTAMP";  ///< imu packet timestamps
static constexpr cf_type IMU_ALERT_FLAGS =
    "IMU_ALERT_FLAGS";  ///< imu alert flags
static constexpr cf_type POSITION_STRING = "POSITION_STRING";  ///< nmea string
static constexpr cf_type POSITION_LAT_LONG =
    "POSITION_LAT_LONG";  ///< parsed nmea coordinates timestamps
static constexpr cf_type POSITION_TIMESTAMP =
    "POSITION_TIMESTAMP";  ///< nmea timestamps
// TODO: check questionable naming -- Tim T.
static constexpr cf_type LIVE_ZONESET_HASH =
    "LIVE_ZONESET_HASH";  ///< cryptographic hash of all zone configs and output
                          ///< product hashes. used to simplifiy client
                          ///< verification that config data has not changed on
                          ///< sensor
static constexpr cf_type ZONE_TIMESTAMP =
    "ZONE_TIMESTAMP";  ///< zone monitoring timestamp
static constexpr cf_type ZONE_PACKET_TIMESTAMP =
    "ZONE_PACKET_TIMESTAMP";  ///< zone monitoring packet timestamp
static constexpr cf_type ZONE_STATES = "ZONE_STATES";  ///< zone states
static constexpr cf_type ZONE_ALERT_FLAGS =
    "ZONE_ALERT_FLAGS";  ///< zone alert flags
}  // namespace ChanField

#if defined(VOID)
#define OUSTER_REMOVED_VOID
#pragma push_macro("VOID")
#undef VOID
#endif

enum class ChanFieldType {
    VOID = 0,
    UINT8 = 1,
    UINT16 = 2,
    UINT32 = 3,
    UINT64 = 4,
    INT8 = 5,
    INT16 = 6,
    INT32 = 7,
    INT64 = 8,
    FLOAT32 = 9,
    FLOAT64 = 10,
    CHAR = 11,
    /* offsetting 30 for structs to pad their numbers for versioning */
    ZONE_STATE = 30,
    UNREGISTERED = 100
};

#if defined(OUSTER_REMOVED_VOID)
#pragma pop_macro("VOID")
#undef OUSTER_REMOVED_VOID
#endif

/**
 * Get the size of the ChanFieldType in bytes.
 *
 * @param[in] ft the field type
 *
 * @return size of the field type in bytes
 */
OUSTER_API_FUNCTION
size_t field_type_size(ChanFieldType ft);

/**
 * Get the bit mask of the ChanFieldType.
 *
 * @param[in] ft the field type
 *
 * @return 64 bit mask
 */
OUSTER_API_FUNCTION
uint64_t field_type_mask(ChanFieldType ft);

/**
 * Get string representation of a channel field.
 *
 * @param[in] ft The field type to get the string representation of.
 *
 * @return string representation of the channel field type.
 */
OUSTER_API_FUNCTION
std::string to_string(ChanFieldType ft);

}  // namespace core
}  // namespace sdk
}  // namespace ouster

#include "impl/deprecated/chanfieldtype_enums.h"
