#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
namespace frame_ops {

/**
 * @brief Clip field values to a range, setting out-of-range values to invalid.
 *
 * For each resolved pixel field, values outside [lower, upper] are replaced
 * with the invalid value. If fields is empty, all pixel fields are clipped.
 *
 * @param[in,out] frame The lidar frame whose fields will be modified in place.
 * @param[in] fields List of field names to clip. If empty, all pixel fields
 *            are used.
 * @param[in] lower Lower bound of the valid range (inclusive).
 * @param[in] upper Upper bound of the valid range (inclusive).
 * @param[in] invalid Value to assign to out-of-range elements. Defaults to 0.
 */
OUSTER_API_FUNCTION
void clip(LidarFrame& frame, const std::vector<std::string>& fields, double lower, double upper,
          double invalid = 0);

/**
 * @brief Filter frame fields using a value range on a specific field as a mask.
 *
 * Builds a binary mask from the given field where values in [lower, upper]
 * pass, then applies that mask to all resolved pixel fields. Pixels that
 * fail the mask are set to the invalid value.
 *
 * @param[in,out] frame The lidar frame to filter in place.
 * @param[in] field Name of the field used to build the filter mask. Must be a
 *            pixel field with shape (h, w).
 * @param[in] lower Lower bound of the filter range (inclusive).
 * @param[in] upper Upper bound of the filter range (inclusive).
 * @param[in] invalid Value assigned to filtered-out pixels. Defaults to 0.
 * @param[in] filtered_fields Optional list of field names to apply the mask to.
 *            If null, all pixel fields are filtered.
 * @throws std::invalid_argument If the filter field does not have shape (h, w).
 */
OUSTER_API_FUNCTION
void filter_field(LidarFrame& frame, const std::string& field, double lower, double upper,
                  double invalid = 0, const std::vector<std::string>* filtered_fields = nullptr);

/**
 * @brief Filter frame fields by zeroing out rows or columns in UV coordinates.
 *
 * When coord_2d is "u", rows in [lower, upper) are invalidated directly.
 * When coord_2d is "v", columns in [lower, upper) are invalidated in the
 * destaggered domain and then re-staggered.
 *
 * @param[in,out] frame The lidar frame to filter in place.
 * @param[in] coord_2d Coordinate axis to filter along: "u" for rows or "v"
 *            for columns.
 * @param[in] lower Start index of the range to invalidate (inclusive).
 * @param[in] upper End index of the range to invalidate (exclusive).
 * @param[in] invalid Value assigned to filtered-out pixels. Defaults to 0.
 * @param[in] filtered_fields Optional list of field names to apply the filter
 *            to. If null, all pixel fields are filtered.
 * @throws std::invalid_argument If coord_2d is not "u" or "v", or if bounds
 *         are out of range.
 */
OUSTER_API_FUNCTION
void filter_uv(LidarFrame& frame, const std::string& coord_2d, size_t lower, size_t upper,
               double invalid = 0, const std::vector<std::string>* filtered_fields = nullptr);

/**
 * @brief Apply a binary mask to frame fields.
 *
 * Pixels where the mask is 0 are set to 0 in the specified fields. The mask
 * dimensions must match the frame dimensions.
 *
 * @param[in,out] frame The lidar frame to mask in place.
 * @param[in] fields List of field names to apply the mask to. If empty, all
 *            pixel fields are used.
 * @param[in] mask Binary mask data as a 2D Eigen array (row-major).
 *            Non-zero values keep the pixel, zero values invalidate it.
 * @throws std::invalid_argument If mask dimensions don't match frame
 * dimensions.
 */
OUSTER_API_FUNCTION
void mask(LidarFrame& frame, const std::vector<std::string>& fields,
          Eigen::Ref<const ouster::sdk::core::img_t<uint8_t>> mask);

/**
 * @brief Convert a downsampling factor to a list of beam indices.
 *
 * Generates indices by selecting every Nth row. If factor equals height,
 * returns only the middle row index.
 *
 * @param[in] factor Downsampling factor. Must be a non-zero divisor of height.
 * @param[in] height Total number of beam rows.
 * @return Vector of selected row indices.
 * @throws std::invalid_argument If factor is 0 or does not evenly divide
 *         height.
 */
OUSTER_API_FUNCTION
std::vector<size_t> reduce_factor_to_indices(size_t factor, size_t height);

/**
 * @brief Create updated SensorInfo for a subset of beam indices.
 *
 * Produces a new SensorInfo with beam angles, pixel shifts, and zone
 * boundaries adjusted to match the selected beam indices.
 *
 * @param[in] metadata Original sensor metadata.
 * @param[in] indices Beam row indices to select. Must be unique and within
 *            [0, metadata height).
 * @return New SensorInfo reflecting the selected beams.
 * @throws std::invalid_argument If indices are empty, contain duplicates, or
 *         are out of range.
 */
OUSTER_API_FUNCTION
SensorInfo select_by_index_metadata(const SensorInfo& metadata, const std::vector<size_t>& indices);

/**
 * @brief Create a new LidarFrame containing only selected beam rows.
 *
 * Copies header fields and selected pixel field rows into a new frame.
 * Non-pixel fields are copied as-is.
 *
 * @param[in] frame Source lidar frame. Must have sensor_info set.
 * @param[in] indices Beam row indices to select. Must be unique and within
 *            [0, frame height).
 * @param[in] update_metadata If true, the output frame's sensor_info is updated
 *            to reflect the selected beams. Defaults to false.
 * @return New LidarFrame with only the selected beam rows.
 * @throws std::invalid_argument If frame has no sensor_info, or indices are
 *         invalid.
 */
OUSTER_API_FUNCTION
LidarFrame select_by_index(const LidarFrame& frame, const std::vector<size_t>& indices,
                           bool update_metadata = false);

/**
 * @brief Create updated SensorInfo for downsampling by a given factor.
 *
 * Equivalent to calling select_by_index_metadata with indices generated by
 * reduce_factor_to_indices.
 *
 * @param[in] metadata Original sensor metadata.
 * @param[in] factor Downsampling factor. Must be a non-zero divisor of the
 *            metadata height.
 * @return New SensorInfo reflecting the downsampled beams.
 * @throws std::invalid_argument If factor is 0 or does not evenly divide the
 *         metadata height.
 */
OUSTER_API_FUNCTION
SensorInfo reduce_by_factor_metadata(const SensorInfo& metadata, size_t factor);

/**
 * Downsample a LidarFrame by selecting every Nth beam row.
 *
 * Equivalent to calling select_by_index with indices generated by
 * reduce_factor_to_indices.
 *
 * @param[in] frame Source lidar frame. Must have sensor_info set.
 * @param[in] factor Downsampling factor. Must be a non-zero divisor of the
 *            frame height.
 * @param[in] update_metadata If true, the output frame's sensor_info is updated
 *            to reflect the downsampled beams. Defaults to false.
 * @return New downsampled LidarFrame.
 * @throws std::invalid_argument If factor is invalid or frame has no
 *         sensor_info.
 */
OUSTER_API_FUNCTION
LidarFrame reduce_by_factor(const LidarFrame& frame, size_t factor, bool update_metadata = false);

}  // namespace frame_ops
}  // namespace core
}  // namespace sdk
}  // namespace ouster
