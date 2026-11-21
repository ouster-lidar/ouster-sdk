#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <vector>

#include "ouster/core/deprecation.h"
#include "ouster/core/frame.h"
#include "ouster/core/impl/cartesian.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/typedefs.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

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
 * Dispatches on the concrete sensor kind: a LidarInfo uses its per-row beam
 * angles and beam/lidar transforms, while a CameraInfo computes per-pixel
 * direction/offset vectors directly from its CameraModel and frame
 * dimensions (and returns an empty 0x0 LUT if it carries no CameraModel or has
 * a zero width or height).
 * Throws std::invalid_argument for any other SensorInfo.
 * The sensor's sensor_to_body transform is always applied, so projections to
 * XYZ made with this XYZLut are in the body frame defined by the extrinsics in
 * the metadata.
 *
 * @param[in] sensor metadata returned from the client.
 *
 * @return xyz direction and offset vectors for each point in the frame.
 */
OUSTER_API_FUNCTION
XYZLut make_xyz_lut(const SensorInfo& sensor);
}  // namespace impl

/**
 * Lookup table of beam directions and offsets.
 *
 * Derives from enable_shared_from_this so that a LUT owned by a shared_ptr
 * keeps the same ownership when it round-trips through the Python bindings,
 * which lets APIs that key on LUT identity, like viz::Cloud, recognize it.
 */
template <typename T>
class XYZLutT : public std::enable_shared_from_this<XYZLutT<T>> {
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
     * Accepts any SensorInfo subclass (LidarInfo or CameraInfo) that provides
     * beam intrinsics and frame dimensions. A CameraInfo without a CameraModel,
     * or with a zero width or height, yields an empty (0x0) LUT that cannot
     * project frames. The sensor's sensor_to_body
     * transform is applied, so the LUT produces body-frame points. Prefer the
     * shared LUT from SensorInfo::xyzlut() when projecting frames from one
     * sensor repeatedly.
     *
     * @param[in] sensor Sensor metadata including intrinsics and resolution.
     */
    explicit XYZLutT(const SensorInfo& sensor) : XYZLutT(impl::make_xyz_lut(sensor)) {}

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

    /**
     * Project a range image into Cartesian points.
     *
     * @param[in] range Range values in millimeters, shaped (h, w).
     *
     * @return point cloud with one row per pixel.
     *
     * @throws std::invalid_argument if the range image pixel count does not
     * match the LUT.
     */
    PointCloudXYZ<T> operator()(const Eigen::Ref<const img_t<uint32_t>>& range) const {
        check_dimensions(range);
        PointCloudXYZ<T> points(range.rows() * range.cols(), 3);
        impl::cartesianT<T>(points, range, direction, offset);
        return points;
    }

    /**
     * Project a frame's RANGE field into Cartesian points.
     *
     * Works for any Frame (lidar or camera) that carries a uint32 RANGE field
     * (millimeters), so lidar and camera point clouds are handled identically.
     *
     * @throws std::invalid_argument if the frame's pixel count does not match
     * the LUT.
     */
    PointCloudXYZ<T> operator()(const Frame& frame) const {
        Eigen::Ref<const img_t<uint32_t>> range = frame.field<uint32_t>(ChanField::RANGE);
        check_dimensions(range);
        PointCloudXYZ<T> points(range.rows() * range.cols(), 3);
        impl::cartesianT<T>(points, range, direction, offset);
        return points;
    }

   private:
    void check_dimensions(const Eigen::Ref<const img_t<uint32_t>>& range) const {
        if (range.size() != direction.rows() || range.size() != offset.rows()) {
            throw std::invalid_argument("range image size does not match the XYZLut");
        }
    }

    template <typename NewT>
    XYZLutT<NewT> cast() const {
        XYZLutT<NewT> result(direction.template cast<NewT>(), offset.template cast<NewT>(), h, w);
        return result;
    }
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
