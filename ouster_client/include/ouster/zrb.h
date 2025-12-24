#pragma once

#include <bitset>
#include <cstdint>
#include <vector>

#include "nonstd/optional.hpp"
#include "ouster/sha256.h"
#include "ouster/typedefs.h"
#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

using zrb_pixel_t = uint16_t;  ///< the parsed image pixel type

/// Default scale factor for Zrb encoding
static constexpr float DEFAULT_M_PER_ZMBIN = 0.0074927621875f;

/**
 * This class represents a zone as a pair of range images (with mm units)
 * and associated metadata, which it encodes in the Zone Rendered Binary (ZRB)
 * file format.
 *
 * The ZRB file format is a compact representation of the near and far range
 * images for a zone, along with metadata about the rendering process. The
 * near and far range images are stored as 16-bit unsigned integers, with a
 * scale factor (`m_per_zmbin`) used to convert the encoded values to actual
 * ranges in meters. The ZRB file also includes information about the sensor
 * that rendered the zone, including its serial number and an optional hash of
 * the STL used during rendering.
 *
 * The ZRB file format is designed to be efficient for storage and transmission,
 * while still providing enough information to accurately reconstruct the zone's
 * geometry when needed. Note that the encoding of ranges into 16-bit values
 * introduces a minor loss of range precision.
 */
class OUSTER_API_CLASS Zrb {
   public:
    /**
     * Default constructor for Zrb.
     * Initializes an empty Zrb object.
     */
    OUSTER_API_FUNCTION
    Zrb();

    /**
     * Constructs a Zrb object from a file path.
     * @param[in] path The path to the ZRB file.
     * @throws std::runtime_error if the file cannot be read or is not a valid
     * ZRB file.
     */
    OUSTER_API_FUNCTION
    explicit Zrb(const std::string& path);

    /**
     * Constructs a Zrb object from an input stream.
     * @param[in] input_stream The input stream containing the ZRB file data.
     * @throws std::runtime_error if the stream cannot be read or is not a valid
     * ZRB file.
     */
    OUSTER_API_FUNCTION
    explicit Zrb(std::istream& input_stream);

    /**
     * Constructs a Zrb object from a binary blob.
     * @param[in] blob The binary blob containing the ZRB file data.
     * @throws std::runtime_error if the blob is not a valid ZRB file.
     */
    OUSTER_API_FUNCTION
    explicit Zrb(const std::vector<uint8_t>& blob);

    /**
     * Constructs a Zrb object with the specified parameters.
     * @param[in] n_rows The number of rows for near and far images.
     * @param[in] n_cols The number of columns for near and far images.
     * 16-bit unsigned values.
     * @param[in] m_per_zmbin The scale factor for converting encoded ranges to
     * Zrb.
     * @param[in] serial_number The serial number of the sensor deploying the
     * Zrb.
     * @param[in] stl_hash A hash of the STL used during rendering.
     * @param[in] beam_to_lidar The beam to lidar transform used during
     * rendering.
     * @param[in] lidar_to_sensor The lidar to sensor transform used during
     * rendering.
     * @param[in] sensor_to_body The sensor to body transform used during
     * rendering.
     */
    OUSTER_API_FUNCTION
    Zrb(uint32_t n_rows, uint32_t n_cols, float m_per_zmbin,
        uint64_t serial_number, Sha256 stl_hash, const mat4d& beam_to_lidar,
        const mat4d& lidar_to_sensor, const mat4d& sensor_to_body);

    /**
     * Encodes the ZRB data into a binary blob and returns it.
     * @note This operation is lossy due to the 16-bit encoding of ranges.
     * @return A vector of bytes representing the ZRB file data.
     */
    OUSTER_API_FUNCTION
    std::vector<uint8_t> blob() const;

    /**
     * Writes the ZRB data to an output stream.
     * @note This operation is lossy due to the 16-bit encoding of ranges.
     * @param[in] out The output stream to write the ZRB data to.
     */
    OUSTER_API_FUNCTION
    void save(std::ostream& out) const;

    /**
     * Writes the ZRB data to a file.
     * @note This operation is lossy due to the 16-bit encoding of ranges.
     * @param[in] file_path The path to the file where the ZRB data will be
     * written.
     */
    OUSTER_API_FUNCTION
    void save(const std::string& file_path) const;

    /**
     * Computes the SHA-256 hash of the ZRB file data.
     * @note This operation computes the hash on-the-fly and does not cache the
     * result.
     * @return The SHA-256 hash of the ZRB file data.
     */
    OUSTER_API_FUNCTION
    Sha256 hash() const;

    /// The near range image (units in mm)
    img_t<uint32_t> near_range_mm;

    /// The far range image (units in mm)
    img_t<uint32_t> far_range_mm;

    nonstd::optional<Sha256> stl_hash;  ///< Optional hash of the STL used
                                        ///< during rendering

    mat4d beam_to_lidar_transform{mat4d::Zero()};  ///< Beam to lidar transform
                                                   ///< used during rendering
    mat4d lidar_to_sensor_transform{
        mat4d::Zero()};  ///< Lidar to sensor transform used
                         ///< during rendering
    mat4d sensor_to_body_transform{
        mat4d::Zero()};         ///< Sensor to body transform used during
                                ///< rendering
    uint64_t serial_number{0};  ///< Serial number of the sensor that
    std::bitset<2048>
        valid_col_mask;  ///< Mask indicating which columns are valid

    /**
     * Compares two Zrb objects for equality.
     * @param[in] lhs The first Zrb object.
     * @param[in] rhs The second Zrb object.
     * @return true if the two Zrb objects are equal, false otherwise.
     */
    friend bool operator==(const Zrb& lhs, const Zrb& rhs);

    /**
     * Compares two Zrb objects for inequality.
     * @param[in] lhs The first Zrb object.
     * @param[in] rhs The second Zrb object.
     * @return true if the two Zrb objects are not equal, false otherwise.
     */
    friend bool operator!=(const Zrb& lhs, const Zrb& rhs);

   private:
    friend struct CacheRenderMetadata;
    float m_per_zmbin_{DEFAULT_M_PER_ZMBIN};
    bool read_zone_cache(std::istream& file, bool load_bounds,
                         std::vector<std::string>& mismatches);
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
