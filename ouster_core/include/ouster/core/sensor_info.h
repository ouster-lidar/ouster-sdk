/**
 * Copyright (c) 2018, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief Ouster sensor information and metadata
 */

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/core/data_format.h"
#include "ouster/core/impl/cache_ptr.h"
#include "ouster/core/sensor_config.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/version.h"
#include "ouster/core/visibility.h"
#include "ouster/core/zone_monitor.h"

namespace ouster {
namespace sdk {
namespace core {

/** Forward declaration for XYZLut */
template <typename T>
class XYZLutT;

/** Thermal Shutdown status. */
enum class ThermalShutdownStatus : uint8_t {
    NORMAL = 0x00,    ///< Normal operation
    IMMINENT = 0x01,  ///< Thermal Shutdown imminent
};

/** Shot Limiting status. */
enum class ShotLimitingStatus : uint8_t {
    NORMAL = 0x00,           ///< Normal operation
    IMMINENT = 0x01,         ///< Shot limiting imminent
    REDUCTION_0_10 = 0x02,   ///< Shot limiting reduction by 0 to 10%
    REDUCTION_10_20 = 0x03,  ///< Shot limiting reduction by 10 to 20%
    REDUCTION_20_30 = 0x04,  ///< Shot limiting reduction by 20 to 30%
    REDUCTION_30_40 = 0x05,  ///< Shot limiting reduction by 30 to 40%
    REDUCTION_40_50 = 0x06,  ///< Shot limiting reduction by 40 to 50%
    REDUCTION_50_60 = 0x07,  ///< Shot limiting reduction by 50 to 60%
    REDUCTION_60_70 = 0x08,  ///< Shot limiting reduction by 60 to 70%
    REDUCTION_70_75 = 0x09,  ///< Shot limiting reduction by 70 to 80%
};

/** Stores from-sensor calibration information */
struct OUSTER_API_CLASS CalibrationStatus {
    optional<bool> reflectivity_status;            ///< Whether reflectivity calibration is present.
    optional<std::string> reflectivity_timestamp;  ///< Timestamp of reflectivity calibration.
};

/** Stores parsed information about the prod line */
class OUSTER_API_CLASS ProductInfo {
   public:
    /**
     * The original full product line string.
     */
    const std::string full_product_info;

    /**
     * The form factor of the sensor. This will
     * look like: "OS1", "OS1MAX" or "OS2" and such.
     */
    const std::string form_factor;

    /**
     * Is the sensor a short range build or not.
     */
    const bool short_range;

    /**
     * The beam configuration of the sensor.
     */
    const std::string beam_config;

    /**
     * The number of beams on the sensor.
     */
    const int beam_count;

    /**
     * If is a RGB sensor or not.
     */
    const bool rgb;

    /**
     * Static method used to create ProductInfo classes.
     *
     * @throws std::runtime_error on a bad product info line.
     *
     * @param[in] product_info_string The product info string to create
     *                                the ProductInfo class from.
     * @return The new ProductInfo class.
     */
    OUSTER_API_FUNCTION
    static ProductInfo create_product_info(const std::string& product_info_string);

    /**
     * Default constructor for ProductInfo that
     * sets everything to blank.
     */
    OUSTER_API_FUNCTION
    ProductInfo();

   protected:
    /**
     * Constructor to initialize each of the members off of.
     * @brief Constructor for ProductInfo that takes params (internal only)
     *
     * @param[in] product_info_string The full product line string.
     * @param[in] form_factor The sensor form factor.
     * @param[in] short_range If the sensor is short range or not.
     * @param[in] beam_config The beam configuration for the sensor.
     * @param[in] beam_count The number of beams for a sensor.
     * @param[in] rgb If the sensor is RGB or not.
     *
     * @internal
     */
    OUSTER_API_FUNCTION
    ProductInfo(std::string product_info_string, std::string form_factor, bool short_range,
                std::string beam_config, int beam_count, bool rgb = false);
};

/**
 * Equality for ProductInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const ProductInfo& lhs, const ProductInfo& rhs);

/**
 * Inequality for ProductInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const ProductInfo& lhs, const ProductInfo& rhs);

/**
 * Get string representation of a product info.
 *
 * @param[in] info Product Info to get the string representation from.
 *
 * @return string representation of the product info.
 */
OUSTER_API_FUNCTION
std::string to_string(const ProductInfo& info);

namespace impl {
struct SensorInfoCache;
}

/**
 * Stores parsed information from metadata.
 */
class OUSTER_API_CLASS SensorInfo {
    mutable impl::CachePtr<ouster::sdk::core::impl::SensorInfoCache> cache_;

   public:
    /// Copy constructor for sensor_info.
    OUSTER_API_FUNCTION
    SensorInfo(const SensorInfo&);
    /// Move constructor for SensorInfo.
    OUSTER_API_FUNCTION
    SensorInfo(SensorInfo&&);
    OUSTER_API_FUNCTION
    ~SensorInfo();
    OUSTER_API_FUNCTION
    SensorInfo& operator=(const SensorInfo&);

    // clang-format off
    uint64_t sn{};              ///< sensor serial number corresponding to prod_sn in
                                ///< metadata.json
    std::string
        fw_rev{};               ///< fw revision corresponding to build_rev in metadata.json
    std::string prod_line{};    ///< prod line

    DataFormat format{};       ///< data format of sensor
    std::vector<double>
        beam_azimuth_angles{};  ///< beam azimuth angles for 3D projection
    std::vector<double>
        beam_altitude_angles{}; ///< beam altitude angles for 3D projection
    double lidar_origin_to_beam_origin_mm{};  ///< distance between lidar origin
                                              ///< and beam origin in mm
    mat4d beam_to_lidar_transform =
        mat4d::Zero();          ///< transform between beam and lidar frame
    mat4d imu_to_sensor_transform =
        mat4d::Zero();          ///< transform between sensor coordinate
                                ///< frame and imu
    mat4d lidar_to_sensor_transform =
        mat4d::Zero();          ///< transform between lidar and sensor
                                ///< coordinate frames
    mat4d sensor_to_body =
        mat4d::Zero();          ///< homogeneous transform from sensor frame to
                                ///< body frame
    uint32_t init_id{};         ///< initialization ID updated every reinit

    std::string build_date{};   ///< build date from FW SensorInfo
    std::string image_rev{};    ///< image rev from FW SensorInfo
    std::string prod_pn{};      ///< prod pn
    std::string status{};       ///< sensor status at time of pulling metadata

    CalibrationStatus cal{};   ///< sensor calibration
    SensorConfig config{};     ///< parsed sensor config if available from metadata
    std::string user_data{};    ///< userdata from sensor if available
    std::string client_version{};  ///< version of library that wrote metadata
    nonstd::optional<ZoneSet> zone_set{};  ///< zone monitor configuration, if present

    /**
     * Constructs a SensorInfo object by parsing a metadata.
     *
     * @param[in] metadata JSON-formatted string representing sensor metadata.
     * @throws std::runtime_error If the metadata string is empty.
     */
    OUSTER_API_FUNCTION
    explicit SensorInfo(const std::string& metadata);

    /* Empty constructor -- keep for  */
    OUSTER_API_FUNCTION
    SensorInfo();

    /** Return an updated version of the metadata string reflecting any
     * changes to the SensorInfo.
     * Errors out if changes are incompatible but does not check for validity
     *
     * @return json serialized version of this object
     */
    OUSTER_API_FUNCTION
    std::string to_json_string() const;

    /**
     * Parse and return version info about this sensor.
     *
     * @return sensor version info
     */
    OUSTER_API_FUNCTION
    Version get_version() const;

    /**
     * Extracts and returns parsed product information for the sensor.
     *
     * @return A ProductInfo object containing the form factor, beam count, and other
     *         relevant product metadata parsed from the SensorInfo.
     */
    OUSTER_API_FUNCTION
    ProductInfo get_product_info() const;

    /**
     * Compares the current SensorInfo object to another and checks if the
     * fields (excluding user metadata) are equal.
     *
     * @param[in] other Another SensorInfo object to compare with.
     * @return True if key fields are equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool has_fields_equal(const SensorInfo& other) const;

    /**
     * Get the number of returns output by the sensor in its current setup.
     *
     * @return number of returns
     */
    OUSTER_API_FUNCTION
    int num_returns() const;

    /**
     * Retrieves the width of a frame
     *
     * @return width of a frame.
     */
    OUSTER_API_FUNCTION
    auto w() const -> decltype(format.columns_per_frame);  ///< returns the width of a frame (equivalent to format.columns_per_frame)

    /**
     * Retrieves the height of a frame
     *
     * @return height of a frame.
     */
    OUSTER_API_FUNCTION
    auto h() const -> decltype(format.pixels_per_column);  ///< returns the height of a frame (equivalent to format.pixels_per_column)
    // clang-format on

    /**
     * Gets a XYZLut for this sensor, caching between calls.
     *
     * @tparam T floating point type to use for the XYZLut: float or double
     * @return cached xyzlut
     *
     * @see clear_xyzlut_cache()
     */
    template <typename T>
    const XYZLutT<T>& xyzlut() const {
        // this check is redundant, but need one that depends on T and always
        // fails or else we get a compiler error even if this function isnt used
        static_assert(std::is_same<T, float>() || std::is_same<T, double>(),
                      "Unsupported type. Must use float or double.");
    }

    /**
     * Clears cached data such as xyzluts calculated from other members.
     */
    OUSTER_API_FUNCTION
    void clear_cache();

    /**
     * Get a default SensorInfo for the given lidar mode.
     *
     * @param[in] mode lidar mode to generate default SensorInfo for.
     *
     * @return default SensorInfo for the OS1-64.
     */
    OUSTER_API_FUNCTION
    static std::shared_ptr<SensorInfo> from_default(LidarMode mode);
};

/// \cond
template <>
OUSTER_API_FUNCTION const XYZLutT<float>& SensorInfo::xyzlut() const;
template <>
OUSTER_API_FUNCTION const XYZLutT<double>& SensorInfo::xyzlut() const;
/// \endcond

/**
 * Equality for SensorInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs == rhs
 */
OUSTER_API_FUNCTION
bool operator==(const SensorInfo& lhs, const SensorInfo& rhs);

/**
 * Inequality for SensorInfo.
 *
 * @param[in] lhs The first object to compare.
 * @param[in] rhs The second object to compare.
 *
 * @return lhs != rhs
 */
OUSTER_API_FUNCTION
bool operator!=(const SensorInfo& lhs, const SensorInfo& rhs);

/**
 * Equality of sensor calibration.
 *
 * @param[in] lhs The first object to compare.
 * @param[out] rhs The second object to compare.
 */
OUSTER_API_FUNCTION
bool operator==(const CalibrationStatus& lhs, const CalibrationStatus& rhs);

/**
 * Inequality of sensor calibration.
 *
 * @param[in] lhs The first object to compare.
 * @param[out] rhs The second object to compare.
 */
OUSTER_API_FUNCTION
bool operator!=(const CalibrationStatus& lhs, const CalibrationStatus& rhs);

/**
 * Get string representation of a Shot Limiting Status.
 *
 * @param[in] shot_limiting_status The shot limiting status to get the string
 *                                 representation of.
 *
 * @return string representation of the shot limiting status.
 */
OUSTER_API_FUNCTION
std::string to_string(ShotLimitingStatus shot_limiting_status);

/**
 * Get string representation of Thermal Shutdown Status.
 *
 * @param[in] thermal_shutdown_status The thermal shutdown status to get the
 *                                    string representation of.
 *
 * @return string representation of thermal shutdown status.
 */
OUSTER_API_FUNCTION
std::string to_string(ThermalShutdownStatus thermal_shutdown_status);

/**
 * Parse metadata given path to a json file.
 *
 * @throw runtime_error if json file does not exist or is malformed.
 *
 * @param[in] json_file path to a json file containing sensor metadata.
 * @param[in] skip_beam_validation whether to skip validation on metadata - not
 *                                 for use on recorded data or metadata
 *                                 from sensors
 *
 * @return a SensorInfo struct populated with a subset of the metadata.
 */
OUSTER_API_FUNCTION
SensorInfo metadata_from_json(const std::string& json_file, bool skip_beam_validation = false);

/**
 * Get a string representation of sensor calibration. Only set fields will be
 * represented.
 *
 * @param[in] cal a struct of calibration.
 *
 * @return string representation of sensor calibration.
 */
OUSTER_API_FUNCTION
std::string to_string(const CalibrationStatus& cal);

/**
 * Get client version.
 *
 * @return client version string
 */
OUSTER_API_FUNCTION
std::string client_version();

}  // namespace core
}  // namespace sdk
}  // namespace ouster
