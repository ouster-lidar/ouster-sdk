#pragma once

#include <cstddef>
#include <vector>

#include "ouster/core/deprecation.h"
#include "ouster/core/impl/cartesian.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
// Forward declaration
class SensorInfo;

template <typename T>
class XYZLutT;
/**
 * Alias for XYZLutT using `double` precision.
 */
using XYZLut = XYZLutT<double>;

namespace impl {
/**
 * Generate a set of lookup tables useful for computing Cartesian coordinates
 * from ranges.
 *
 * The lookup tables are:
 * - direction: a matrix of unit vectors pointing radially outwards.
 * - offset: a matrix of offsets dependent on beam origin distance from lidar
 *           origin.
 *
 * Each table is an n x 3 array of doubles stored in column-major order where
 * each row corresponds to the nth point in a lidar frame, with 0 <= n < h*w.
 *
 * Projections to XYZ made with this XYZLut will be in the coordinate frame
 * defined by transform*beam_to_lidar_transform.
 *
 * @param[in] w number of columns in the lidar frame. e.g. 512, 1024, or 2048.
 * @param[in] h number of rows in the lidar frame.
 * @param[in] range_unit the unit, in meters, of the range,  e.g.
 * ouster::sdk::core::range_unit.
 * @param[in] beam_to_lidar_transform transform between beams and
 * lidar origin. Translation portion is in millimeters.
 * @param[in] transform additional transformation to apply to resulting points.
 * @param[in] azimuth_angles_deg azimuth offsets in degrees for each of h beams.
 * @param[in] altitude_angles_deg altitude in degrees for each of h beams.
 *
 * @return xyz direction and offset vectors for each point in the lidar frame.
 */
OUSTER_API_FUNCTION
XYZLut make_xyz_lut(size_t w, size_t h, double range_unit, const mat4d& beam_to_lidar_transform,
                    const mat4d& transform, const std::vector<double>& azimuth_angles_deg,
                    const std::vector<double>& altitude_angles_deg);

/**
 * Convenient overload that uses parameters from the supplied SensorInfo.
 * Projections to XYZ made with this XYZLut will be in the sensor coordinate
 * frame defined in the sensor documentation unless use_extrinics is true.
 * Then the projections will be in the coordinate frame defined by the provided
 * extrinsics in the metadata.
 *
 * @param[in] sensor metadata returned from the client.
 * @param[in] use_extrinsics if true, applies the ``sensor.sensor_to_body``
 * transform to the resulting "sensor frame" coordinates
 *
 * @return xyz direction and offset vectors for each point in the lidar frame.
 */
OUSTER_API_FUNCTION
XYZLut make_xyz_lut(const SensorInfo& sensor, bool use_extrinsics);
}  // namespace impl

/** Lookup table of beam directions and offsets. */
template <typename T>
class XYZLutT {
   public:
    /**
     * Unit direction vectors from the sensor for each pixel.
     */
    const ArrayX3R<T> direction;
    /**
     * Offset vectors for each pixel, typically representing per-pixel
     * translations.
     *
     * Combined with direction and range to compute 3D coordinates.
     */
    const ArrayX3R<T> offset;

    /**
     * Height of the XYZLut data.
     */
    const size_t h = 0;

    /**
     * Width of the XYZLut data.
     */
    const size_t w = 0;

    // Add this to make XYZLut<double> and XYZLut<float> friends to cast
    template <typename>
    friend class XYZLutT;

    /**
     * Construct an XYZ lookup table using the provided sensor information.
     *
     * @param[in] sensor Sensor metadata including intrinsics and resolution.
     * @param[in] use_extrinsics Whether to apply extrinsic calibration.
     */
    XYZLutT(const SensorInfo& sensor, bool use_extrinsics = true)
        : XYZLutT(impl::make_xyz_lut(sensor, use_extrinsics)) {}

    /**
     * Construct an XYZ lookup table from an existing one of a different type.
     *
     * @param[in] xyzlut XYZLut to duplicate from.
     */
    template <typename OldT>
    XYZLutT(const XYZLutT<OldT>& xyzlut)
        : direction(xyzlut.direction.template cast<T>()),
          offset(xyzlut.offset.template cast<T>()),
          h(xyzlut.h),
          w(xyzlut.w) {}

    /**
     * Construct an XYZ lookup table from its elements.
     *
     * @param[in] direction Direction vectors from the sensor for each pixel.
     * @param[in] offset Offset vectors for each pixel.
     * @param[in] h Height of sensor array.
     * @param[in] w Width of sensor array.
     */
    XYZLutT(ArrayX3R<T> direction, ArrayX3R<T> offset, size_t h, size_t w)
        : direction(direction), offset(offset), h(h), w(w) {}

    XYZLutT() = default;

    PointCloudXYZ<T> operator()(const Eigen::Ref<const img_t<uint32_t>>& range) const {
        PointCloudXYZ<T> points(range.rows() * range.cols(), 3);
        impl::cartesianT<T>(points, range, direction, offset);
        return points;
    }

    PointCloudXYZ<T> operator()(const LidarFrame& frame) const {
        Eigen::Ref<const img_t<uint32_t>> range = frame.field<uint32_t>(ChanField::RANGE);
        PointCloudXYZ<T> points(range.rows() * range.cols(), 3);
        impl::cartesianT<T>(points, range, direction, offset);
        return points;
    }

   private:
    template <typename NewT>
    XYZLutT<NewT> cast() const {
        XYZLutT<NewT> result(direction.template cast<NewT>(), offset.template cast<NewT>(), h, w);
        return result;
    }
};

/** \defgroup ouster_core_lidar_frame_cartesian Ouster Core lidar_scan.h
 * XYZLut related items.
 * @{
 */
/**
 * Convert LidarFrame to Cartesian points.
 *
 * @param[in] frame a LidarFrame to generate a point cloud from.
 * @param[in] lut lookup tables generated by make_xyz_lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarFrame where i = row * w + col.
 */
OUSTER_DEPRECATED_MSG(XYZLut operator()(LidarFrame), OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_API_FUNCTION
PointCloudXYZd cartesian(const LidarFrame& frame, const XYZLut& lut);

/**
 * Convert a staggered range image to Cartesian points.
 *
 * @param[in] range a range image in the same format as the RANGE field of a
 * LidarFrame.
 * @param[in] lut lookup tables generated by make_xyz_lut.
 *
 * @return Cartesian points where ith row is a 3D point which corresponds
 *         to ith pixel in LidarFrame where i = row * w + col.
 */
OUSTER_DEPRECATED_MSG(XYZLut operator()(img_t<uint32_t>), OUSTER_DEPRECATED_LAST_SUPPORTED_1_0)
OUSTER_API_FUNCTION
PointCloudXYZd cartesian(const Eigen::Ref<const img_t<uint32_t>>& range, const XYZLut& lut);
/** @}*/

}  // namespace core
}  // namespace sdk
}  // namespace ouster
