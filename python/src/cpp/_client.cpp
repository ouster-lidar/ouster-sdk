/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief ouster_pyclient python module
 *
 * Note: the type annotations in `client.pyi` need to be updated whenever this
 * file changes. See the mypy documentation for details.
 */

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#include "common.h"
#include "ouster/client.h"
#include "ouster/cloud_io.h"
#include "ouster/downsample.h"
#include "ouster/image_processing.h"
#include "ouster/impl/build.h"
#include "ouster/impl/logging.h"
#include "ouster/impl/packet_writer.h"
#include "ouster/impl/profile_extension.h"
#include "ouster/lidar_scan.h"
#include "ouster/metadata.h"
#include "ouster/osf/osf_scan_source.h"
#include "ouster/packet_source.h"
#include "ouster/pcap_scan_source.h"
#include "ouster/pose_conversion.h"
#include "ouster/pose_util.h"
#include "ouster/sensor_http.h"
#include "ouster/sensor_packet_source.h"
#include "ouster/sensor_scan_source.h"

namespace py = pybind11;
namespace chrono = std::chrono;

using ouster::sensor::calibration_status;
using ouster::sensor::data_format;
using ouster::sensor::ImuPacket;
using ouster::sensor::LidarPacket;
using ouster::sensor::Packet;
using ouster::sensor::packet_format;
using ouster::sensor::product_info;
using ouster::sensor::sensor_config;
using ouster::sensor::sensor_info;
using ouster::sensor::impl::packet_writer;
using ouster::sensor::util::SensorHttp;
using namespace ouster;

using client_shared_ptr = std::shared_ptr<sensor::client>;
PYBIND11_MAKE_OPAQUE(client_shared_ptr);

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

namespace ouster {
namespace sensor {

namespace impl {

extern const Table<lidar_mode, const char*, 7> lidar_mode_strings;
extern const Table<timestamp_mode, const char*, 4> timestamp_mode_strings;
extern const Table<OperatingMode, const char*, 2> operating_mode_strings;
extern const Table<MultipurposeIOMode, const char*, 6>
    multipurpose_io_mode_strings;
extern const Table<Polarity, const char*, 2> polarity_strings;
extern const Table<NMEABaudRate, const char*, 2> nmea_baud_rate_strings;
extern Table<UDPProfileLidar, const char*, MAX_NUM_PROFILES>
    udp_profile_lidar_strings;
extern Table<UDPProfileIMU, const char*, 1> udp_profile_imu_strings;
extern Table<ShotLimitingStatus, const char*, 10> shot_limiting_status_strings;
extern Table<ThermalShutdownStatus, const char*, 2>
    thermal_shutdown_status_strings;
extern Table<FullScaleRange, const char*, 2> full_scale_range_strings;
extern Table<ReturnOrder, const char*, 5> return_order_strings;
extern const Table<FieldClass, const char*, 4> field_class_strings;

}  // namespace impl
}  // namespace sensor
}  // namespace ouster

// alias for non-casting row-major array arguments
template <typename T>
using pyimg_t = py::array_t<T, py::array::c_style>;

// factor out overloaded call operator for ae/buc
template <typename T, typename U>
void image_proc_call(T& self, pyimg_t<U> image, bool update_state) {
    if (image.ndim() != 2) throw std::invalid_argument("Expected a 2d array");
    self(Eigen::Map<img_t<U>>(image.mutable_data(), image.shape(0),
                              image.shape(1)),
         update_state);
}

std::pair<Eigen::Matrix<double, Eigen::Dynamic, 3>,
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>
downsample_point_cloud(const py::array_t<double>& voxel_size,
                       const py::array_t<double>& pts,
                       const py::array_t<double>& attributes,
                       int min_points_per_voxel) {
    auto size_buf = voxel_size.request();
    auto size_ptr = static_cast<double*>(size_buf.ptr);
    Eigen::Matrix<double, 3, 1> eigen_voxel_size;
    if (voxel_size.size() == 1) {
        eigen_voxel_size(0, 0) = size_ptr[0];
        eigen_voxel_size(1, 0) = size_ptr[0];
        eigen_voxel_size(2, 0) = size_ptr[0];
    } else if (voxel_size.size() == 3) {
        eigen_voxel_size(0, 0) = size_ptr[0];
        eigen_voxel_size(1, 0) = size_ptr[1];
        eigen_voxel_size(2, 0) = size_ptr[2];
    } else {
        throw std::invalid_argument(
            "Expected a float/double or 3x1 array for voxel size.");
    }

    if (pts.ndim() != 2 || pts.shape(1) != 3) {
        throw std::invalid_argument("Points array must have a shape of Nx3");
    }

    py::array_t<double> c_style_points;
    py::array_t<double> c_style_attrs;

    // Create a C-style copy of arrays if it's neither C-style nor F-style
    const py::array_t<double>* points_ptr = &pts;
    if (!(pts.flags() & py::array::c_style)) {
        c_style_points = py::array_t<double, py::array::c_style>(pts);
        points_ptr = &c_style_points;  // Use the C-style array for processing
    }
    const py::array_t<double>* attr_ptr = &attributes;
    if (!(attributes.flags() & py::array::c_style)) {
        c_style_attrs = py::array_t<double, py::array::c_style>(attributes);
        attr_ptr = &c_style_attrs;  // Use the C-style array for processing
    }

    auto pts_buf = points_ptr->request();
    auto attr_buf = attr_ptr->request();

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>>
        points_mat(static_cast<double*>(pts_buf.ptr), pts_buf.shape[0],
                   pts_buf.shape[1]);

    Eigen::Map<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
        attr_mat(static_cast<double*>(attr_buf.ptr), attr_buf.shape[0],
                 attr_buf.shape[1]);

    Eigen::Matrix<double, Eigen::Dynamic, 3> eigen_out_pts;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> eigen_out_attr;
    ouster::core::voxel_downsample(eigen_voxel_size, points_mat, attr_mat,
                                   eigen_out_pts, eigen_out_attr,
                                   min_points_per_voxel);

    return {eigen_out_pts, eigen_out_attr};
}

/*
 * Define an enum from a table of strings, along with some properties to make
 * the class behave more like a Python enum
 */
template <typename C, typename E, size_t N>
void def_enum(C& Enum, const Table<E, const char*, N>& strings_table,
              const std::string& enum_prefix = "") {
    // weed out empty profiles
    auto end = std::find_if(
        strings_table.begin(), strings_table.end(),
        [](const std::pair<E, const char*>& p) { return p.second == nullptr; });

    // in pybind11 2.0, calling enum.value(const char* name, val) doesn't make a
    // copy of the name argument. When value names aren't statically allocated,
    // we have to keep them alive. Use deque for stability of c_str() pointers
    static std::deque<std::string> enumerator_names;

    // module imports
    py::object MappingProxy =
        py::module::import("types").attr("MappingProxyType");

    // declare enumerators
    for (auto it = strings_table.begin(); it != end; ++it) {
        enumerator_names.push_back(enum_prefix + it->second);
        Enum.value(enumerator_names.back().c_str(), it->first);
    }

    // use immutable MappingProxy to return members dict
    std::map<std::string, E> members;
    for (auto it = strings_table.begin(); it != end; ++it)
        members[it->second] = it->first;
    py::object py_members = MappingProxy(members);
    Enum.def_property_readonly_static(
        "__members__", [=](py::object) { return py_members; },
        "Returns a mapping of member name->value.");

    // can't make the class iterable itself easily
    Enum.def_property_readonly_static(
        "values",
        [&, end](py::object) {
            return py::make_key_iterator(strings_table.begin(), end);
        },
        "Returns an iterator of all enum members.");

    // support name / value properties like regular enums
    Enum.def_property_readonly(
        "value", [](const E& self) { return static_cast<int>(self); },
        "The value of the Enum member.");
    Enum.def_property_readonly(
        "name", [](const E& self) { return to_string(self); },
        "The name of the Enum member.");
    Enum.attr("__str__") =
        py::cpp_function([](const E& u) { return to_string(u); },
                         py::name("__str__"), py::is_method(Enum));

    Enum.def_static(
        "from_string",
        [=](const std::string& s) {
            return members.count(s) > 0 ? py::cast(members.at(s)) : py::none();
        },
        "Create enum value from string.");
}

/*
 * Check that buffer is a 1-d byte array of size > bound and return an internal
 * pointer to the data for writing. Check is strictly greater to account for the
 * extra byte required to determine if a datagram is bigger than expected.
 */
inline uint8_t* getptr(size_t bound, py::buffer& buf) {
    auto info = buf.request();
    if (info.format != py::format_descriptor<uint8_t>::format() ||
        // type of info.size has changed from size_t to ssize_t
        info.ndim != 1 || info.size < static_cast<decltype(info.size)>(bound)) {
        throw std::invalid_argument(
            "Incompatible argument: expected a bytearray of size >= " +
            std::to_string(bound));
    }
    return (uint8_t*)info.ptr;
}

/*
 * Map a dtype to a channel field type
 */
static sensor::ChanFieldType field_type_of_dtype(const py::dtype& dt) {
    if (dt.is(py::dtype::of<uint8_t>()))
        return sensor::ChanFieldType::UINT8;
    else if (dt.is(py::dtype::of<uint16_t>()))
        return sensor::ChanFieldType::UINT16;
    else if (dt.is(py::dtype::of<uint32_t>()))
        return sensor::ChanFieldType::UINT32;
    else if (dt.is(py::dtype::of<uint64_t>()))
        return sensor::ChanFieldType::UINT64;
    else if (dt.is(py::dtype::of<int8_t>()))
        return sensor::ChanFieldType::INT8;
    else if (dt.is(py::dtype::of<int16_t>()))
        return sensor::ChanFieldType::INT16;
    else if (dt.is(py::dtype::of<int32_t>()))
        return sensor::ChanFieldType::INT32;
    else if (dt.is(py::dtype::of<int64_t>()))
        return sensor::ChanFieldType::INT64;
    else if (dt.is(py::dtype::of<float>()))
        return sensor::ChanFieldType::FLOAT32;
    else if (dt.is(py::dtype::of<double>()))
        return sensor::ChanFieldType::FLOAT64;
    else
        throw std::invalid_argument("Invalid dtype for a channel field");
}

/*
 * Map a channel field type to a dtype
 */
static py::dtype dtype_of_field_type(const sensor::ChanFieldType& ftype) {
    switch (ftype) {
        case sensor::ChanFieldType::UINT8:
            return py::dtype::of<uint8_t>();
        case sensor::ChanFieldType::UINT16:
            return py::dtype::of<uint16_t>();
        case sensor::ChanFieldType::UINT32:
            return py::dtype::of<uint32_t>();
        case sensor::ChanFieldType::UINT64:
            return py::dtype::of<uint64_t>();
        case sensor::ChanFieldType::INT8:
            return py::dtype::of<int8_t>();
        case sensor::ChanFieldType::INT16:
            return py::dtype::of<int16_t>();
        case sensor::ChanFieldType::INT32:
            return py::dtype::of<int32_t>();
        case sensor::ChanFieldType::INT64:
            return py::dtype::of<int64_t>();
        case sensor::ChanFieldType::FLOAT32:
            return py::dtype::of<float>();
        case sensor::ChanFieldType::FLOAT64:
            return py::dtype::of<double>();
        default:
            throw std::invalid_argument(
                "Invalid field_type for conversion to dtype");
    }
    return py::dtype();  // unreachable ...
}

/**
 * Retrieves numpy shape from field descriptor
 */
py::array::ShapeContainer shape_of_descriptor(const FieldDescriptor& rd) {
    /**
     * Throws if shape is absent i.e. field is not declared as array; we are
     * currently not handling those cases, change if/when the usecase arises
     */
    if (rd.shape.size() == 0) {
        throw std::invalid_argument("Field is not an array");
    }
    return py::array::ShapeContainer(rd.shape.begin(), rd.shape.end());
}

template <typename Fn>
struct lambda_iter {
    Fn lambda;

    // generative output iterators return themselves
    lambda_iter& operator*() { return *this; }
    // prefix
    lambda_iter& operator++() { return *this; }
    // postfix
    lambda_iter& operator++(int) { return *this; }

    template <typename T>
    lambda_iter& operator=(T&& item) {
        lambda(item);
        return *this;
    }
};

template <typename Fn>
lambda_iter<Fn> make_lambda_iter(Fn&& f) {
    return lambda_iter<Fn>{f};
}

template <typename T>
struct set_field {
    using Field = py::array_t<T, py::array::c_style | py::array::forcecast>;

    void operator()(const packet_writer& self, LidarPacket& p,
                    const std::string& i, Field field) {
        if (field.ndim() != 2 || field.shape(0) != self.pixels_per_column ||
            field.shape(1) != self.columns_per_packet)
            throw std::invalid_argument("field dimension mismatch");

        /**
         * It is a bit weird to be setting these back and forth to work around
         * packet_writer::set_block logic that is intended for lidarscan usage
         * but I do think it is better than keeping two versions of the same.
         *
         * This is intended for python users, which will expect it to work out
         * of the box without any extra fiddling.
         */
        std::vector<uint16_t> m_ids(self.columns_per_packet);
        std::vector<uint32_t> statuses(self.columns_per_packet);

        for (int icol = 0; icol < self.columns_per_packet; ++icol) {
            uint8_t* col_buf = self.nth_col(icol, p.buf.data());
            // store for later reassignment
            m_ids[icol] = self.col_measurement_id(col_buf);
            statuses[icol] = self.col_status(col_buf);
            // overwrite with 0..columns_per_packet
            self.set_col_measurement_id(col_buf, icol);
            self.set_col_status(col_buf, 0x1);
        }

        Eigen::Map<const img_t<T>> field_map(field.data(), field.shape(0),
                                             field.shape(1));
        // eigen is trash; this extra step is extra annoying
        Eigen::Ref<const img_t<T>> ref = field_map;
        self.set_block(ref, i, p.buf.data());

        // restore m_ids and statuses
        for (int icol = 0; icol < self.columns_per_packet; ++icol) {
            uint8_t* col_buf = self.nth_col(icol, p.buf.data());
            self.set_col_measurement_id(col_buf, m_ids[icol]);
            self.set_col_status(col_buf, statuses[icol]);
        }
    }
};

class PyPacketSource : public ouster::core::PacketSource {
    mutable std::vector<std::shared_ptr<ouster::sensor::sensor_info>>
        sensor_info_;

   public:
    PyPacketSource() {}

    // dummy
    ouster::core::PacketIterator begin() const override {
        return ouster::core::PacketIterator(this);
    }

    void close() override {
        PYBIND11_OVERRIDE_PURE(
            void,         /* Return type */
            PacketSource, /* Parent class */
            close         /* Name of function in C++ (must match Python name) */
        );
    }

    bool is_live() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("is_live");
        return py::cast<bool>(res);
    }

    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("sensor_info");
        sensor_info_ =
            py::cast<std::vector<std::shared_ptr<ouster::sensor::sensor_info>>>(
                res);
        return sensor_info_;
    }
};

class PythonScanIteratorImpl : public ouster::core::ScanIteratorImpl {
    py::iterator iterator_;

    std::vector<std::shared_ptr<LidarScan>> scans_;
    uint64_t desired_sn_;

   public:
    PythonScanIteratorImpl(py::iterator iter, uint64_t desired_sn = 0)
        : iterator_(iter), desired_sn_(desired_sn) {}

    ~PythonScanIteratorImpl() {
        py::gil_scoped_acquire acquire;
        iterator_ = {};
    }

    bool advance(size_t offset) override {
        py::gil_scoped_acquire acquire;
        for (size_t i = 0; i < offset; i++) {
            if (iterator_ == py::iterator::sentinel()) {
                return true;
            }
            auto value = *iterator_;
            scans_ = value.cast<std::vector<std::shared_ptr<LidarScan>>>();
            ++iterator_;

            if (scans_.size() != 1) {
                throw std::runtime_error("Must only yield arrays of 1 scan.");
            }

            // skip data from undesired sensors
            if (desired_sn_ != 0 && scans_[0]->sensor_info->sn != desired_sn_) {
                offset++;
            }
        }
        return false;
    }

    std::vector<std::shared_ptr<LidarScan>>& value() override { return scans_; }
};

class PyScanSource : public ouster::core::ScanSource {
    mutable std::vector<std::shared_ptr<ouster::sensor::sensor_info>>
        sensor_info_;

   public:
    PyScanSource() {}

    ouster::core::ScanIterator begin() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto it = self.attr("__iter__")();
        return ouster::core::ScanIterator(this, new PythonScanIteratorImpl(it));
    }

    ouster::core::ScanIterator begin(int sensor_idx) const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto it = self.attr("__iter__")();
        auto sn = sensor_info()[sensor_idx]->sn;
        return ouster::core::ScanIterator(this,
                                          new PythonScanIteratorImpl(it, sn));
    }

    bool is_live() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("is_live");
        return py::cast<bool>(res);
    }

    const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
    sensor_info() const override {
        py::gil_scoped_acquire acquire;
        py::object self = py::cast(this);
        auto res = self.attr("sensor_info");
        sensor_info_ =
            py::cast<std::vector<std::shared_ptr<ouster::sensor::sensor_info>>>(
                res);
        return sensor_info_;
    }

    // dummy
    ouster::core::ScanSource* move() override {
        throw std::runtime_error("Moving not supported with this type.");
    }

   protected:
    void close() override {
        PYBIND11_OVERRIDE_PURE(
            void,       /* Return type */
            ScanSource, /* Parent class */
            close       /* Name of function in C++ (must match Python name) */
        );
    }
};

/**
 * Applies a set of 4x4 pose transformations to a collection of 3D points,
 * reshapes the input into appropriate Eigen matrices, and invokes the C++
 * version of the `dewarp` function for performing the transformations.
 *
 * This function is designed to convert NumPy input arrays from Python into
 * Eigen matrices, which are suitable for efficient matrix operations in C++.
 * The 3D points are passed in the shape of (H, W, 3), and the 4x4 pose matrices
 * are passed in the shape of (W, 4, 4).
 *
 * @param[in] points A NumPy array of shape (H, W, 3) representing the 3D
 * points.
 *               - H: Number of columns (groups of points)
 *               - W: Number of points per column
 *               - 3: 3D coordinates (x, y, z)
 *
 * @param[in] poses A NumPy array of shape (W, 4, 4) representing the 4x4 pose
 * matrices.
 *              - W: Number of pose matrices
 *              - 4x4: The transformation matrices
 *
 * @return A NumPy array of shape (H, W, 3) containing the dewarped 3D points
 * after applying the corresponding 4x4 transformation matrices to the points.
 *
 */

template <typename T>
py::array_t<T> dewarp(const py::array_t<T>& points,
                      const py::array_t<T>& poses) {
    py::array_t<T> c_style_points;
    py::array_t<T> c_style_poses;

    // Ensure poses is in C-style format
    const py::array_t<T>* poses_ptr = &poses;
    if (!(poses.flags() & py::array::c_style)) {
        c_style_poses = py::array_t<T, py::array::c_style>(poses);
        poses_ptr = &c_style_poses;
    }

    // Ensure points is in C-style format
    const py::array_t<T>* points_ptr = &points;
    if (!(points.flags() & py::array::c_style)) {
        c_style_points = py::array_t<T, py::array::c_style>(points);
        points_ptr = &c_style_points;
    }

    auto poses_buf = poses_ptr->request();
    auto points_buf = points_ptr->request();

    // Validate poses dims: (W, 4, 4)
    if (poses_buf.ndim != 3 || poses_buf.shape[1] != 4 ||
        poses_buf.shape[2] != 4) {
        throw std::runtime_error("Invalid shape for poses, expected (W, 4, 4)");
    }

    // Validate points dims: (H, W, 3)
    if (points_buf.ndim != 3 || points_buf.shape[2] != 3) {
        throw std::runtime_error(
            "Invalid shape for points, expected (H, W, 3)");
    }

    const int num_poses = poses_buf.shape[0];            // W
    const int num_rows = points_buf.shape[0];            // H
    const int num_points_per_col = points_buf.shape[1];  // W
    const int point_dim = 3;

    if (num_points_per_col != num_poses) {
        throw std::runtime_error(
            "Number of points per set must match number of poses");
    }

    // Map poses as a (W×16) row-major matrix of T
    Eigen::Map<core::PosesT<T>> poses_mat(static_cast<T*>(poses_buf.ptr),
                                          num_poses,  // rows
                                          16          // cols (flattened 4×4)
    );

    // Allocate output (H, W, 3) and map as (H×W)×3
    auto result = py::array_t<T>({num_rows, num_poses, point_dim});
    auto result_buf = result.request();
    Eigen::Map<core::PointsT<T>> dewarped_points(
        static_cast<T*>(result_buf.ptr), num_rows * num_poses, point_dim);

    Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>> points_mat(
        static_cast<T*>(points_buf.ptr), num_rows * num_poses, point_dim);

    Eigen::Ref<core::PointsT<T>> dewarped_ref(dewarped_points);
    Eigen::Ref<const core::PointsT<T>> points_ref(points_mat);
    Eigen::Ref<const core::PosesT<T>> poses_ref(poses_mat);

    // Call your templated, in-place dewarp
    core::dewarp<T>(dewarped_ref, points_ref, poses_ref);

    return result;
}

/**
 * Applies a single of 4x4 pose transformations to a collection of 3D points,
 * reshapes the input into appropriate Eigen matrices, and invokes the C++
 * version of the `transfrom` function for performing the transformations.
 *
 * This function is designed to convert NumPy input arrays from Python into
 * Eigen matrices, which are suitable for efficient matrix operations in C++.
 * The 3D points are passed in the shape of (H, W, 3), and the single 4x4 pose
 * matrics in the shape of (4, 4).
 *
 * @param[in] points A NumPy array of shape (H, W, 3), or (N, 3)
 * representing the 3D points.
 *               - H: Number of columns (groups of points)
 *               - W: Number of points per column
 *               - 3: 3D coordinates (x, y, z)
 *
 * @param[in] poses A NumPy array of shape (4, 4) representing the 4x4 pose
 * matrices.
 *              - 4x4: The transformation matrices
 *
 * @return A NumPy array of shape (H, W, 3) or (N, 3) containing the transformed
 * 3D points after applying the corresponding 4x4 transformation matrices to the
 * points.
 *
 */
template <typename T>
py::array_t<T> transform(const py::array_t<T>& points,
                         const py::array_t<T>& pose) {
    // Ensure the pose is a 4x4 matrix
    if (pose.ndim() != 2 || pose.shape(0) != 4 || pose.shape(1) != 4) {
        throw std::runtime_error("pose array must have shape (4, 4)");
    }

    py::array_t<T> c_style_points;
    py::array_t<T> c_style_pose;

    // Create a C-style copy of points if it's neither C-style nor F-style
    const py::array_t<T>* points_ptr = &points;
    if (!(points.flags() & py::array::c_style)) {
        c_style_points = py::array_t<T, py::array::c_style>(points);
        points_ptr = &c_style_points;  // Use the C-style array for processing
    }
    const py::array_t<T>* pose_ptr = &pose;
    if (!(pose.flags() & py::array::c_style)) {
        c_style_pose = py::array_t<T, py::array::c_style>(pose);
        pose_ptr = &c_style_pose;  // Use the C-style array for processing
    }

    // Convert pose to Eigen format
    Eigen::Map<const Eigen::Matrix<T, 1, 16, Eigen::RowMajor>> pose_eigen(
        pose_ptr->data());

    // Handle case where points is a 2D array: (N, 3)
    if (points_ptr->ndim() == 2 && points_ptr->shape(1) == 3) {
        const int n = points_ptr->shape(0);

        // Define a matrix type for points using the template parameter
        using PointsMatrix =
            Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;
        Eigen::Map<const PointsMatrix> points_eigen(points_ptr->data(), n, 3);

        auto result = py::array_t<T>({n, 3});
        auto result_buf = result.request();
        Eigen::Map<PointsMatrix> transformed(static_cast<T*>(result_buf.ptr), n,
                                             3);

        core::transform<T>(transformed, points_eigen, pose_eigen);
        return result;
    }

    // Handle case where points is a 3D array: (H, W, 3)
    else if (points_ptr->ndim() == 3 && points_ptr->shape(2) == 3) {
        const int h = points_ptr->shape(0);
        const int w = points_ptr->shape(1);

        // Define a matrix type for points using the template parameter
        using PointsMatrix =
            Eigen::Matrix<T, Eigen::Dynamic, 3, Eigen::RowMajor>;
        Eigen::Map<const PointsMatrix> points_eigen(points_ptr->data(), h * w,
                                                    3);

        auto result = py::array_t<T>({h, w, 3});
        auto result_buf = result.request();
        Eigen::Map<PointsMatrix> transformed(static_cast<T*>(result_buf.ptr),
                                             h * w, 3);

        core::transform<T>(transformed, points_eigen, pose_eigen);
        return result;
    } else {
        throw std::invalid_argument(
            "points array must have shape (n, 3) or (h, w, 3)");
    }
}

/**
 * Interpolates 4x4 pose matrices at given x-coordinate values.
 *
 * @param[in] x_interp NumPy array of shape (N, 1) or (N,) representing
 * interpolation x-coordinate values.
 * @param[in] x_known NumPy array of shape (M, 1) or (M,) representing known
 * x-coordinate values.
 * @param[in] poses_known NumPy array of shape (M, 4, 4) representing known pose
 * matrices.
 *
 * @return NumPy array of shape (N, 4, 4) containing the interpolated 4x4 pose
 * matrices.
 */
py::array_t<double> interp_pose(const py::array_t<double>& x_interp,
                                const py::array_t<double>& x_known,
                                const py::array_t<double>& poses_known) {
    const py::array_t<double>* poses_known_ptr = &poses_known;
    if ((poses_known.flags() & py::array::c_style) == 0) {
        py::array_t<double> c_style_poses =
            py::array_t<double, py::array::c_style>(poses_known);
        poses_known_ptr = &c_style_poses;
    }

    py::buffer_info x_interp_buf = x_interp.request();
    py::buffer_info x_known_buf = x_known.request();
    py::buffer_info poses_known_buf = poses_known_ptr->request();

    // Check if x_interp or x_known is either a one-dimensional array (N,)
    // or a two-dimensional array with one column (N,1).
    if (!(x_interp_buf.ndim == 1 ||
          (x_interp_buf.ndim == 2 && x_interp_buf.shape[1] == 1))) {
        throw std::runtime_error("x_interp must have shape (N,) or (N,1)");
    }
    if (!(x_known_buf.ndim == 1 ||
          (x_known_buf.ndim == 2 && x_known_buf.shape[1] == 1))) {
        throw std::runtime_error("x_known must have shape (N,) or (N,1)");
    }
    // Check that poses_known is a 3D array with shape (M, 4, 4)
    if (poses_known_buf.ndim != 3 || poses_known_buf.shape[1] != 4 ||
        poses_known_buf.shape[2] != 4) {
        throw std::runtime_error(
            "poses_known must be a 3D array with shape (N, 4, 4)");
    }

    size_t N = x_interp_buf.shape[0];
    size_t M = x_known_buf.shape[0];
    // Ensure that poses_known has the same number of poses as M
    size_t poses_known_size = poses_known_buf.shape[0];
    if (M != poses_known_size) {
        throw std::runtime_error(
            "The number of poses in poses_known must match the number of "
            "values in x_known");
    }

    double* x_interp_ptr = static_cast<double*>(x_interp_buf.ptr);
    std::vector<double> x_interp_vec(x_interp_ptr, x_interp_ptr + N);

    double* x_known_ptr = static_cast<double*>(x_known_buf.ptr);
    std::vector<double> x_known_vec(x_known_ptr, x_known_ptr + M);

    Eigen::Map<core::Poses> poses_mat(static_cast<double*>(poses_known_buf.ptr),
                                      M, 16);

    core::Poses result =
        core::interp_pose(x_interp_vec, x_known_vec, poses_mat);

    std::vector<ssize_t> shape = {result.rows(), 4, 4};

    std::vector<ssize_t> strides = {16 * static_cast<ssize_t>(sizeof(double)),
                                    4 * static_cast<ssize_t>(sizeof(double)),
                                    static_cast<ssize_t>(sizeof(double))};

    return py::array_t<double>(shape, strides, result.data());
}

void init_client(py::module& m, py::module&) {
    m.doc() = R"(
    Sensor client bindings generated by pybind11.

    This module is generated directly from the C++ code and not meant to be used
    directly.
    )";

    // turn off signatures in docstrings: mypy stubs provide better types
    py::options options;
    options.disable_function_signatures();

    auto client_error =
        py::register_exception<ouster::sensor::ClientError>(m, "ClientError");
    py::register_exception<ouster::sensor::ClientTimeout>(m, "ClientTimeout",
                                                          client_error);
    py::register_exception<ouster::sensor::ClientOverflow>(m, "ClientOverflow",
                                                           client_error);

    py::enum_<ouster::core::Severity>(m, "Severity")
        .value("OUSTER_WARNING", ouster::core::Severity::OUSTER_WARNING)
        .value("OUSTER_ERROR", ouster::core::Severity::OUSTER_ERROR);

    // clang-format off
    // Packet Format
    py::class_<packet_format, std::shared_ptr<packet_format>>(m, "PacketFormat")
        .def(py::init([](const sensor_info& info) {
            return new packet_format(info);
        }))
        .def(py::init([](sensor::UDPProfileLidar profile,
                         size_t pixels_per_column,
                         size_t columns_per_packet) {
                        return new packet_format(profile, pixels_per_column,
                                                 columns_per_packet);
                    }))
        .def_static("from_metadata",
                    [](const sensor_info& info) -> const packet_format& {
                        return sensor::get_format(info);
                    }, py::return_value_policy::reference)
        .def_static("from_info",
                    [](const sensor_info& info) -> const packet_format& {
                        return sensor::get_format(info);
                    }, py::return_value_policy::reference)
        .def_static("from_profile",
                    [](sensor::UDPProfileLidar profile,
                       size_t pixels_per_column,
                       size_t columns_per_packet) -> const packet_format& {
                        return sensor::get_format(profile, pixels_per_column,
                                                  columns_per_packet);
                    }, py::return_value_policy::reference)
        .def_readonly("lidar_packet_size", &packet_format::lidar_packet_size)
        .def_readonly("imu_packet_size", &packet_format::imu_packet_size)
        .def_readonly("udp_profile_lidar", &packet_format::udp_profile_lidar)
        .def_readonly("columns_per_packet", &packet_format::columns_per_packet)
        .def_readonly("pixels_per_column", &packet_format::pixels_per_column)
        .def_readonly("packet_header_size", &packet_format::packet_header_size)
        .def_readonly("col_header_size", &packet_format::col_header_size)
        .def_readonly("col_footer_size", &packet_format::col_footer_size)
        .def_readonly("col_size", &packet_format::col_size)
        .def_readonly("packet_footer_size", &packet_format::packet_footer_size)
        .def_readonly("max_frame_id", &packet_format::max_frame_id)

        .def("field_value_mask", &packet_format::field_value_mask)
        .def("field_bitness", &packet_format::field_bitness)

        .def("crc", [](packet_format& pf, py::buffer buf) {
            return pf.crc(getptr(pf.lidar_packet_size, buf));
        })

        .def("calculate_crc", [](packet_format& pf, py::buffer buf) {
            return pf.calculate_crc(getptr(pf.lidar_packet_size, buf));
        })

        .def("packet_type", [](packet_format& pf, py::buffer buf) {
            return pf.packet_type(getptr(pf.lidar_packet_size, buf));
        })

        .def("frame_id", [](packet_format& pf, py::buffer buf) {
            return pf.frame_id(getptr(pf.lidar_packet_size, buf));
        })

        .def("prod_sn", [](packet_format& pf, py::buffer buf) {
            return pf.prod_sn(getptr(pf.lidar_packet_size, buf));
        })

        .def("init_id", [](packet_format& pf, py::buffer buf) {
            return pf.init_id(getptr(pf.lidar_packet_size, buf));
        })

        .def("alert_flags", [](packet_format& pf, py::buffer buf) {
            return pf.alert_flags(getptr(pf.lidar_packet_size, buf));
        })

        .def("countdown_thermal_shutdown", [](packet_format& pf, py::buffer buf) {
            return pf.countdown_thermal_shutdown(getptr(pf.lidar_packet_size, buf));
        })

        .def("countdown_shot_limiting", [](packet_format& pf, py::buffer buf) {
            return pf.countdown_shot_limiting(getptr(pf.lidar_packet_size, buf));
        })

        .def("thermal_shutdown", [](packet_format& pf, py::buffer buf) {
            return pf.thermal_shutdown(getptr(pf.lidar_packet_size, buf));
        })

        .def("shot_limiting", [](packet_format& pf, py::buffer buf) {
            return pf.shot_limiting(getptr(pf.lidar_packet_size, buf));
        })

        // NOTE: keep_alive seems to be ignored without cpp_function wrapper
        .def_property_readonly("fields", py::cpp_function([](const packet_format& self) {
                return py::make_key_iterator(self.begin(), self.end());
        }, py::keep_alive<0, 1>()),
        "Return an iterator of available channel fields.")

        .def("packet_field", [](packet_format& pf, const std::string& f, py::buffer buf) -> py::array {
            auto buf_ptr = getptr(pf.lidar_packet_size, buf);

            auto packet_field = [&](auto& res) -> void {
                for (int icol = 0; icol < pf.columns_per_packet; icol++) {
                    auto col = pf.nth_col(icol, buf_ptr);
                    auto dst_col = res.mutable_data(0, icol);
                    pf.col_field(col, f, dst_col, pf.columns_per_packet);
                }
            };

            std::vector<size_t> dims{static_cast<size_t>(pf.pixels_per_column),
                                     static_cast<size_t>(pf.columns_per_packet)};
            
            switch (pf.field_type(f)) {
                case sensor::ChanFieldType::UINT8: {
                    py::array_t<uint8_t> res(dims);
                    packet_field(res);
                    return std::move(res);
                }
                case sensor::ChanFieldType::UINT16: {
                    py::array_t<uint16_t> res(dims);
                    packet_field(res);
                    return std::move(res);
                }
                case sensor::ChanFieldType::UINT32: {
                    py::array_t<uint32_t> res(dims);
                    packet_field(res);
                    return std::move(res);
                }
                case sensor::ChanFieldType::UINT64: {
                    py::array_t<uint64_t> res(dims);
                    packet_field(res);
                    return std::move(res);
                }
                default:
                    throw py::key_error("Invalid type for PacketFormat");
            }
        })

        .def("packet_header", [](packet_format& pf, py::object o, py::buffer buf) {

            auto packet_header = [&](auto&& f) -> py::array {
                using T = typename std::result_of<decltype(f)(const uint8_t*)>::type;

                auto p = getptr(pf.lidar_packet_size, buf);
                auto res = py::array_t<T>(pf.columns_per_packet);

                for (int icol = 0; icol < pf.columns_per_packet; icol++)
                    res.mutable_at(icol) = f(pf.nth_col(icol, p));

                return std::move(res);
            };

            auto ind = py::int_(o).cast<int>();
            switch (ind) {
                case 0: return packet_header([&](auto col) { return pf.col_timestamp(col); });
                case 1: return packet_header([&](auto col) { return pf.col_encoder(col); });
                case 2: return packet_header([&](auto col) { return pf.col_measurement_id(col); });
                case 3: return packet_header([&](auto col) { return pf.col_status(col); });
                case 4: return packet_header([&](auto col) { return pf.col_frame_id(col); });
                default: throw py::key_error("Invalid header index for PacketFormat");
            }
        })

        // IMU packet accessors
        .def("imu_sys_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_sys_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_accel_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_accel_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_gyro_ts", [](packet_format& pf, py::buffer buf) { return pf.imu_gyro_ts(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_x", [](packet_format& pf, py::buffer buf) { return pf.imu_av_x(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_y", [](packet_format& pf, py::buffer buf) { return pf.imu_av_y(getptr(pf.imu_packet_size, buf)); })
        .def("imu_av_z", [](packet_format& pf, py::buffer buf) { return pf.imu_av_z(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_x", [](packet_format& pf, py::buffer buf) { return pf.imu_la_x(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_y", [](packet_format& pf, py::buffer buf) { return pf.imu_la_y(getptr(pf.imu_packet_size, buf)); })
        .def("imu_la_z", [](packet_format& pf, py::buffer buf) { return pf.imu_la_z(getptr(pf.imu_packet_size, buf)); });

    // PacketWriter
    py::class_<packet_writer, packet_format, std::shared_ptr<packet_writer>>(m, "PacketWriter")
        // this is safe so long as packet_writer does not carry any extra state
        .def_static("from_info",
                    [](const sensor_info& info) -> const packet_writer& {
                        const auto& pf = sensor::get_format(info);
                        return static_cast<const packet_writer&>(pf);
                    }, py::return_value_policy::reference)
        .def_static("from_profile",
                    [](sensor::UDPProfileLidar profile,
                       size_t pixels_per_column,
                       size_t columns_per_packet) -> const packet_writer& {
                        const auto& pf = sensor::get_format(profile,
                                                            pixels_per_column,
                                                            columns_per_packet);
                        return static_cast<const packet_writer&>(pf);
                    }, py::return_value_policy::reference)
        .def("set_col_status",
              [](const packet_writer& self, LidarPacket& p, int col_idx,
                 uint32_t status) {
                  if (col_idx >= self.columns_per_packet)
                      throw std::invalid_argument("col_idx out of bounds");
                  uint8_t* col_buf = self.nth_col(col_idx, p.buf.data());
                  self.set_col_status(col_buf, status);
              })
        .def("set_col_timestamp",
              [](const packet_writer& self, LidarPacket& p, int col_idx,
                 uint64_t ts) {
                  if (col_idx >= self.columns_per_packet)
                      throw std::invalid_argument("col_idx out of bounds");
                  uint8_t* col_buf = self.nth_col(col_idx, p.buf.data());
                  self.set_col_timestamp(col_buf, ts);
              })
        .def("set_col_measurement_id",
              [](const packet_writer& self, LidarPacket& p, int col_idx,
                 uint16_t m_id) {
                  if (col_idx >= self.columns_per_packet)
                      throw std::invalid_argument("col_idx out of bounds");
                  uint8_t* col_buf = self.nth_col(col_idx, p.buf.data());
                  self.set_col_measurement_id(col_buf, m_id);
              })
        .def("set_frame_id",
              [](const packet_writer& self, LidarPacket& p, uint32_t frame_id) {
                  self.set_frame_id(p.buf.data(), frame_id);
              })
        .def("set_alert_flags", [](const packet_writer& self, LidarPacket& p, uint8_t alert_flags) {
            self.set_alert_flags(p.buf.data(), alert_flags);
        })
        .def("set_shutdown_countdown", [](const packet_writer& self, LidarPacket& p, uint8_t shutdown_countdown) {
            self.set_shutdown_countdown(p.buf.data(), shutdown_countdown);
        })
        .def("set_shot_limiting_countdown", [](const packet_writer& self, LidarPacket& p, uint8_t shot_limiting_countdown) {
            self.set_shot_limiting_countdown(p.buf.data(), shot_limiting_countdown);
        })
        .def("set_field", set_field<uint8_t>{})
        .def("set_field", set_field<uint16_t>{})
        .def("set_field", set_field<uint32_t>{})
        .def("set_field", set_field<uint64_t>{})
        .def("set_field", set_field<int8_t>{})
        .def("set_field", set_field<int16_t>{})
        .def("set_field", set_field<int32_t>{})
        .def("set_field", set_field<int64_t>{})
        .def("set_field", set_field<float>{})
        .def("set_field", set_field<double>{});

    m.def("scan_to_packets",
          [](const LidarScan& ls, const packet_writer& pw, uint32_t init_id, uint64_t prod_sn) {
              std::vector<LidarPacket> packets;

              auto append_pypacket = [&](const LidarPacket& packet) {
                  packets.push_back(packet);
              };

              auto iter = make_lambda_iter(append_pypacket);
              impl::scan_to_packets(ls, pw, iter, init_id, prod_sn);
              return packets;
          });

    // Data Format
    py::class_<data_format>(m, "DataFormat")
        .def(py::init<>(), R"(
            Data Format of a packet coming from sensor

            See sensor documentation for the meaning of each property
        )")
        .def_readwrite("pixels_per_column", &data_format::pixels_per_column, R"(

        Pixels per column in the lidar packet, synonymous with the number of beams of the sensor

        :type: int
        )")
        .def_readwrite("columns_per_packet", &data_format::columns_per_packet, R"(
        Columns per packet in the lidar packet, typically 16

        :type: int
        )")
        .def_readwrite("columns_per_frame", &data_format::columns_per_frame, R"(
        Columns per frame in the lidar packet, corresponding with lidar_mode

        :type: int
        )")
        .def_readwrite("pixel_shift_by_row", &data_format::pixel_shift_by_row, R"(
        Shift of pixels by row to create natural images

        :type: List[int]

        )")
        .def_readwrite("column_window", &data_format::column_window, R"(
        Firing window of sensor, set by config.azimuth_window

        :type: Tuple[int, int]
        )")
        .def_readwrite("udp_profile_lidar", &data_format::udp_profile_lidar, R"(
        Lidar packet profile
        )")
        .def_readwrite("udp_profile_imu", &data_format::udp_profile_imu, R"(
        IMU packet profile
        )")
        .def_readwrite("fps", &data_format::fps, R"(
        Frames per second, e.g., 10 for lidar_mode 1024x10

        :type: int
        )")
        .def("valid_columns_per_frame", &data_format::valid_columns_per_frame, R"(
        Return the number of valid columns per complete frame of data with the column_window applied.
        )")
        .def("packets_per_frame", &data_format::packets_per_frame, R"(
        Return the number of valid packets actually sent per frame of data with the column_window applied.
        )");

    py::class_<calibration_status>(m, "SensorCalibration")
        .def(py::init<>(), R"(
           Sensor Calibration in a Sensor Metadata, covering reflectivity calibration and more"
        )")
        .def_readwrite("reflectivity_status", &calibration_status::reflectivity_status, "Reflectivity calibration status")
        .def_readwrite("reflectivity_timestamp", &calibration_status::reflectivity_timestamp, "Reflectivity timestamp")
        .def("__str__", [](const calibration_status& i) { return to_string(i); })
        .def("__eq__", [](const calibration_status& i, const calibration_status& j) { return i == j; });

    // Product Line
    py::class_<product_info>(m, "ProductInfo")
        .def_readonly("full_product_info", &product_info::full_product_info, R"(
        The original full product info string.

        :type: string
        )")
        .def_readonly("form_factor", &product_info::form_factor, R"(
        The form factor of the product.

        :type: string
        )")
        .def_readonly("short_range", &product_info::short_range, R"(
        If the product is a short range make.

        :type: bool
        )")
        .def_readonly("beam_config", &product_info::beam_config, R"(
        The beam configuration of the product..

        :type: string
        )")
        .def_readonly("beam_count", &product_info::beam_count, R"(
        Number of beams

        :type: int
        )")
        .def("__eq__", [](const product_info& i, const product_info& j) { return i == j; })
        .def("__str__", [](const product_info& i) {return to_string(i);});

    // Sensor Info
    py::class_<sensor_info, std::shared_ptr<sensor_info>>(m, "SensorInfo", R"(
        Sensor Info required to interpret UDP data streams.

        See the sensor documentation for the meaning of each property.
        )")
        .def(py::init<>(), R"(
            Construct an empty metadata.
        )")
        .def(py::init([](const std::string& s) {
            return new sensor_info(s);
        }), py::arg("s"),  R"(
        Args:
            s (str): json string to parse
        )")
        .def_readwrite("sn", &sensor_info::sn, "Sensor serial number.")
        .def_readwrite("fw_rev", &sensor_info::fw_rev, "Sensor firmware revision.")
        .def_readwrite("prod_line", &sensor_info::prod_line, "Product line, e.g., 'OS-1-128'.")
        .def_readwrite("format", &sensor_info::format,  "Describes the structure of a lidar packet. See class DataFormat.")
        .def_readwrite("beam_azimuth_angles", &sensor_info::beam_azimuth_angles, "Beam azimuth angles, useful for XYZ projection.")
        .def_readwrite("beam_altitude_angles", &sensor_info::beam_altitude_angles, "Beam altitude angles, useful for XYZ projection.")
        .def_readwrite("imu_to_sensor_transform", &sensor_info::imu_to_sensor_transform, "Homogenous transformation matrix representing IMU offset to Sensor Coordinate Frame.")
        .def_readwrite("lidar_to_sensor_transform", &sensor_info::lidar_to_sensor_transform, "Homogeneous transformation matrix from Lidar Coordinate Frame to Sensor Coordinate Frame.")
        .def_readwrite("lidar_origin_to_beam_origin_mm", &sensor_info::lidar_origin_to_beam_origin_mm, "Distance between lidar origin and beam origin in millimeters.")
        .def_readwrite("beam_to_lidar_transform", &sensor_info::beam_to_lidar_transform, "Homogenous transformation matrix reprsenting Beam to Lidar Transform")
        .def_readwrite("extrinsic", &sensor_info::extrinsic, "Extrinsic Matrix.")
        .def_readwrite("init_id", &sensor_info::init_id, "Initialization id.")
        .def_readwrite("build_date", &sensor_info::build_date, "Build date")
        .def_readwrite("image_rev", &sensor_info::image_rev, "Image rev")
        .def_readwrite("prod_pn", &sensor_info::prod_pn, "Prod pn")
        .def_readwrite("status", &sensor_info::status, "sensor status")
        .def_readwrite("cal", &sensor_info::cal, "sensor calibration")
        .def_readwrite("config", &sensor_info::config, "sensor config")
        .def_readwrite("user_data", &sensor_info::user_data, "sensor user data")
        .def_static("from_default", &sensor::default_sensor_info, R"(
        Create gen-1 OS-1-64 SensorInfo populated with design values.
        )")
        .def("to_json_string", &sensor_info::to_json_string, R"( Return metadata string made from current entries"
        )")
        .def("has_fields_equal", &sensor_info::has_fields_equal, R"(Compare public fields")")
        // only uncomment for debugging purposes!! story for general use and output is not filled
        //.def("__str__", [](const sensor_info& i) { return to_string(i); })
        .def_property_readonly("w", [](const sensor_info& self) { return self.w(); }, R"(returns the width of a frame (equivalent to format.columns_per_frame))")
        .def_property_readonly("h", [](const sensor_info& self) { return self.h(); }, R"(returns the height of a frame (equivalent to format.pixels_per_column))")
        .def("__eq__", [](const sensor_info& i, const sensor_info& j) { return i == j; })
        .def("__repr__", [](const sensor_info& self) {
            const auto mode = self.config.lidar_mode ? to_string(self.config.lidar_mode.value()) : std::to_string(self.format.fps) + "fps";
            return "<ouster.sdk.client.SensorInfo " + self.prod_line + " " +
                std::to_string(self.sn) + " " + self.fw_rev + " " + mode + ">";
        })
        .def("get_version", [](const sensor_info& self) { return self.get_version(); }, R"(Get parsed sensor version)")
        .def("get_product_info", [](const sensor_info& self) { return self.get_product_info(); }, R"(Get parsed product info)")
        .def("__copy__", [](const sensor_info& self) { return sensor_info{self}; })
        .def("__deepcopy__", [](const sensor_info& self, py::dict) { return sensor_info{self}; });


    // Enums
    auto lidar_mode = py::enum_<sensor::lidar_mode>(m, "LidarMode", R"(
        Possible Lidar Modes of sensor.

        Determines to horizontal and vertical resolution rates of sensor. See
        sensor documentation for details.)");
    def_enum(lidar_mode, sensor::impl::lidar_mode_strings, "MODE_");
    // TODO: this is wacky
    lidar_mode.value("MODE_UNSPEC", sensor::lidar_mode::MODE_UNSPEC);
    lidar_mode.attr("from_string") = py::cpp_function([](const std::string& s) {
        return sensor::lidar_mode_of_string(s); }, py::name("from_string"), "Create LidarMode from string.");

    auto timestamp_mode = py::enum_<sensor::timestamp_mode>(m, "TimestampMode", R"(
        Possible Timestamp modes of sensor.

        See sensor documentation for details.)");
    def_enum(timestamp_mode, sensor::impl::timestamp_mode_strings);
    timestamp_mode.value("TIME_FROM_UNSPEC", sensor::timestamp_mode::TIME_FROM_UNSPEC);
    timestamp_mode.attr("from_string") = py::cpp_function([](const std::string& s) {
        return sensor::timestamp_mode_of_string(s); }, py::name("from_string"), "Create TimestampMode from string.");

    auto OperatingMode = py::enum_<sensor::OperatingMode>(m, "OperatingMode", R"(
        Possible Operating modes of sensor.

        See sensor documentation for details.)");
    def_enum(OperatingMode, sensor::impl::operating_mode_strings, "OPERATING_");

    auto MultipurposeIOMode = py::enum_<sensor::MultipurposeIOMode>(m, "MultipurposeIOMode", R"(
        Mode of MULTIPURPOSE_IO pin.

        See sensor documentation for details.)");
    def_enum(MultipurposeIOMode, sensor::impl::multipurpose_io_mode_strings, "MULTIPURPOSE_");

    auto Polarity = py::enum_<sensor::Polarity>(m, "Polarity", R"(
        Pulse Polarity.

        Applicable to several Polarity settings on sensor.)");
    def_enum(Polarity, sensor::impl::polarity_strings, "POLARITY_");

    auto ReturnOrder = py::enum_<sensor::ReturnOrder>(m, "ReturnOrder", R"(
        Sensor return order.

        See sensor documentation for details.)");
    def_enum(ReturnOrder, sensor::impl::return_order_strings, "ORDER_");

    auto FullScaleRange = py::enum_<sensor::FullScaleRange>(m, "FullScaleRange", R"(
        IMU output scale range.

        See sensor documentation for details.)");
    def_enum(FullScaleRange, sensor::impl::full_scale_range_strings, "FSR_");

    auto NMEABaudRate = py::enum_<sensor::NMEABaudRate>(m, "NMEABaudRate", R"(
        Expected baud rate sensor attempts to decode for NMEA UART input $GPRMC messages.)");
    def_enum(NMEABaudRate, sensor::impl::nmea_baud_rate_strings);

    auto UDPProfileLidar = py::enum_<sensor::UDPProfileLidar>(m, "UDPProfileLidar", "UDP lidar profile.");
    def_enum(UDPProfileLidar, sensor::impl::udp_profile_lidar_strings, "PROFILE_LIDAR_");
    UDPProfileLidar.attr("from_string") = py::cpp_function(
        [](const std::string& s) {
            return sensor::udp_profile_lidar_of_string(s);
        },
        py::name("from_string"), "Create UDPProfileLidar from string.");
    UDPProfileLidar.def_property_readonly_static(
        "values",
        [](py::object) {
            return py::make_key_iterator(
                sensor::impl::udp_profile_lidar_strings.begin(),
                std::find_if(
                    sensor::impl::udp_profile_lidar_strings.begin(),
                    sensor::impl::udp_profile_lidar_strings.end(),
                    [](const auto& p) { return p.second == nullptr; }));
        },
        "Returns an iterator of all UDPProfileLidar enum members.");

    auto UDPProfileIMU = py::enum_<sensor::UDPProfileIMU>(m, "UDPProfileIMU", "UDP imu profile.");
    def_enum(UDPProfileIMU, sensor::impl::udp_profile_imu_strings, "PROFILE_IMU_");

    auto ShotLimitingStatus = py::enum_<sensor::ShotLimitingStatus>(m, "ShotLimitingStatus", "Shot Limiting Status.");
    def_enum(ShotLimitingStatus, sensor::impl::shot_limiting_status_strings);

    auto ThermalShutdownStatus = py::enum_<sensor::ThermalShutdownStatus>(m, "ThermalShutdownStatus", "Thermal Shutdown Status.");
    def_enum(ThermalShutdownStatus, sensor::impl::thermal_shutdown_status_strings);

    auto FieldClass = py::enum_<ouster::FieldClass>(m, "FieldClass", "LidarScan field classes", py::arithmetic());
    def_enum(FieldClass, sensor::impl::field_class_strings);

    m.def("parse_and_validate_sensor_config",
          [](const std::string& metadata)
              -> std::tuple<sensor_config,
                            ouster::ValidatorIssues> {
              sensor_config config;
              ValidatorIssues issues;
              ouster::parse_and_validate_config(metadata, config, issues);
              return std::make_pair(config, issues);
          });
    // Sensor Config
    py::class_<sensor_config>(m, "SensorConfig",  R"(
        Corresponds to sensor config parameters.

        Please see sensor documentation for the meaning of each property.
        )")
        .def(py::init<>(), "Construct an empty SensorConfig.")
        .def(py::init([](const std::string& s) {
            auto config = new sensor_config{};
            ValidatorIssues issues;
            if (!ouster::parse_and_validate_config(s, *config, issues)) {
                throw std::runtime_error(to_string(issues.critical));
            }
            return config;
        }), py::arg("s"), R"(Construct a SensorConfig from a json string.
        Args:
            s (str): json string to parse
        )")
        .def_readwrite("udp_dest", &sensor_config::udp_dest, "Destination to which sensor sends UDP traffic.")
        .def_readwrite("udp_port_lidar", &sensor_config::udp_port_lidar, "Port on UDP destination to which lidar data will be sent.")
        .def_readwrite("udp_port_imu", &sensor_config::udp_port_imu, "Port on UDP destination to which IMU data will be sent.")
        .def_readwrite("timestamp_mode", &sensor_config::timestamp_mode, "Timestamp mode of sensor. See class TimestampMode.")
        .def_readwrite("lidar_mode", &sensor_config::lidar_mode, "Horizontal and Vertical Resolution rate of sensor as mode, e.g., 1024x10. See class LidarMode.")
        .def_readwrite("operating_mode", &sensor_config::operating_mode, "Operating Mode of sensor. See class OperatingMode.")
        .def_readwrite("multipurpose_io_mode", &sensor_config::multipurpose_io_mode, "Mode of MULTIPURPOSE_IO pin. See class MultipurposeIOMode.")
        .def_readwrite("azimuth_window", &sensor_config::azimuth_window, "Tuple representing the visible region of interest of the sensor in millidegrees, .e.g., (0, 360000) for full visibility.")
        .def_readwrite("signal_multiplier", &sensor_config::signal_multiplier, "Multiplier for signal strength of sensor, corresponding to maximum allowable azimuth_window. Gen 2 Only.")
        .def_readwrite("sync_pulse_out_angle", &sensor_config::sync_pulse_out_angle, "Polarity of SYNC_PULSE_OUT output. See sensor documentation for details." )
        .def_readwrite("sync_pulse_out_pulse_width", &sensor_config::sync_pulse_out_pulse_width, "SYNC_PULSE_OUT pulse width in ms. See sensor documentation for details.")
        .def_readwrite("nmea_in_polarity", &sensor_config::nmea_in_polarity, "Polarity of NMEA UART input $GPRMC messages. See sensor documentation for details.")
        .def_readwrite("nmea_baud_rate", &sensor_config::nmea_baud_rate, "Expected baud rate sensor attempts to decode for NMEA UART input $GPRMC messages.")
        .def_readwrite("nmea_ignore_valid_char", &sensor_config::nmea_ignore_valid_char, "NMEA Ignore Valid Char. True corresponds to 1 and False to 0 for property; see sensor documentation for details.")
        .def_readwrite("nmea_leap_seconds", &sensor_config::nmea_leap_seconds, "Integer number of leap seconds that will be added to the UDP timetsamp when calculating seconds since Unix Epoch time. See sensor documentation for details.")
        .def_readwrite("sync_pulse_in_polarity", &sensor_config::sync_pulse_in_polarity, "Polarity of SYNC_PULSE_IN pin. See sensor documentation for details.")
        .def_readwrite("sync_pulse_out_polarity", &sensor_config::sync_pulse_out_polarity, "Polarity of SYNC_PULSE_OUT output. See sensor documentation for details." )
        .def_readwrite("sync_pulse_out_frequency", &sensor_config::sync_pulse_out_frequency, "SYNC_PULSE_OUT rate. See sensor documentation for details.")
        .def_readwrite("phase_lock_enable", &sensor_config::phase_lock_enable, "Enable phase lock. See sensor documentation for more details.")
        .def_readwrite("phase_lock_offset", &sensor_config::phase_lock_offset, "Angle in Lidar Coordinate Frame that sensors are locked to, in millidegrees. See sensor documentation for details.")
        .def_readwrite("columns_per_packet", &sensor_config::columns_per_packet, "Measurement blocks per UDP packet. See sensor documentation for details.")
        .def_readwrite("udp_profile_lidar", &sensor_config::udp_profile_lidar, "UDP packet format for lidar data. See sensor documentation for details.")
        .def_readwrite("udp_profile_imu", &sensor_config::udp_profile_imu, "UDP packet format for imu data. See sensor documentation for details.")
        .def_readwrite("gyro_fsr", &sensor_config::gyro_fsr, "The gyro full scale measurement range to use. See sensor documentation for details.")
        .def_readwrite("accel_fsr", &sensor_config::accel_fsr, "The accelerometer full scale measurement range to use. See sensor documentation for details.")
        .def_readwrite("return_order", &sensor_config::return_order, "The priority of sensor returns to output. See sensor documentation for details.")
        .def_readwrite("min_range_threshold_cm", &sensor_config::min_range_threshold_cm, "The minimum detection range of the sensor in cm. See sensor documentation for details.")
        .def_readwrite("extra_options", &sensor_config::extra_options, "Extra configuration options on the sensor. Each value should be stringized json.")
        .def("__str__", [](const sensor_config& i) { return to_string(i); })
        .def("__eq__", [](const sensor_config& i, const sensor_config& j) { return i == j; })
        .def("__copy__", [](const sensor_config& self) { return sensor_config{self}; })
        .def("__deepcopy__", [](const sensor_config& self, py::dict) { return sensor_config{self}; });

    m.def("init_logger",
        [](const std::string& log_level, const std::string& log_file_path,
           bool rotating, int max_size_in_bytes, int max_files) {
            return sensor::init_logger(log_level, log_file_path, rotating, max_size_in_bytes,
                                max_files);
        },
        R"(
        Initializes and configures ouster_client logs. This method should be invoked
        only once before calling any other method from the library if the user wants
        to direct the library log statements to a different medium (other than
        console which is the default).

        Args:
            log_level Control the level of log messages outputed by the client.
                Valid options are (case-sensitive): "trace", "debug", "info", "warning",
                "error", "critical" and "off".
            log_file_path (str): Path to location where log files are stored. The
                path must be in a location that the process has write access to. If an empty
                string is provided then the logs will be directed to the console. When
                an empty string is passed then the rest of parameters are ignored.
            rotating (bool): Configure the log file with rotation, rotation rules are
                specified through the two following parameters max_size_in_bytes and
                max_files. If rotating is set to false the following parameters are ignored
            max_size_in_bytes (int): Maximum number of bytes to write to a rotating log
                file before starting a new file. ignored if rotating is set to False.
            max_files (int): Maximum number of rotating files to accumlate before
                re-using the first file. ignored if rotating is set to False.

        Returns:
            returns True on success, False otherwise.
        )",
        py::arg("log_level"), py::arg("log_file_path") = "",
        py::arg("rotating") = false, py::arg("max_size_in_bytes") = 0, py::arg("max_files") = 0);

    m.def("set_config", [] (const std::string& hostname, const sensor_config& config, bool persist,  bool udp_dest_auto, bool force_reinit) {
        uint8_t config_flags = 0;
        if (persist) config_flags |= ouster::sensor::CONFIG_PERSIST;
        if (udp_dest_auto) config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
        if (force_reinit) config_flags |= ouster::sensor::CONFIG_FORCE_REINIT;
        if (!sensor::set_config(hostname, config, config_flags)) {
            throw std::runtime_error("Error setting sensor config.");
        }
    }, R"(
        Set sensor config parameters on sensor.

        Args:
            hostname (str): hostname of the sensor
            config (SensorConfig): config to set sensor parameters to
            persist (bool): persist parameters after sensor disconnection (default = False)
            udp_dest_auto (bool): automatically determine sender's IP at the time command was sent
                and set it as destination of UDP traffic. Function will error out if config has
                udp_dest member. (default = False)
            force_reinit (bool): forces the sensor to re-init during set_config even when config
                params have not changed. (default = False)
        )", py::arg("hostname"), py::arg("config"), py::arg("persist") = false, py::arg("udp_dest_auto") = false,
            py::arg("force_reinit") = false);

    m.def("get_config", [](const std::string& hostname, bool active) {
        sensor::sensor_config config;
        if (!sensor::get_config(hostname, config, active)) {
            throw std::runtime_error("Error getting sensor config.");
        }
        return config;
    }, R"(
        Returns sensor config parameters as SensorConfig.

        Args:
            hostname (str): hostname of the sensor
            active (bool): return active or staged sensor configuration
        )", py::arg("hostname"), py::arg("active") = true);

    // Version Info
    py::class_<util::version>(m, "Version")
        .def(py::init<>())
        .def("__eq__", [](const util::version& u, const util::version& v) { return u == v; })
        .def("__lt__", [](const util::version& u, const util::version& v) { return u < v; })
        .def("__le__", [](const util::version& u, const util::version& v) { return u <= v; })
        .def_readwrite("major", &util::version::major)
        .def_readwrite("minor", &util::version::minor)
        .def_readwrite("patch", &util::version::patch)
        .def_readwrite("stage", &util::version::stage)
        .def_readwrite("machine", &util::version::machine)
        .def_readwrite("prerelease", &util::version::prerelease)
        .def_readwrite("build", &util::version::build)
        .def_static("from_string", &util::version_from_string);

    m.attr("invalid_version") = util::invalid_version;

    m.attr("min_version") = sensor::min_version;

    // clang-format on

    // client
    py::class_<client_shared_ptr>(m, "SensorConnection")
        .def(py::init([](std::string hostname, int lidar_port,
                         int imu_port) -> client_shared_ptr {
                 auto cli = sensor::init_client(hostname, lidar_port, imu_port);
                 if (!cli)
                     throw std::runtime_error(
                         "Failed initializing sensor connection");
                 return cli;
             }),
             py::arg("hostname"), py::arg("lidar_port") = 7502,
             py::arg("imu_port") = 7503)
        .def(py::init([](std::string hostname, std::string udp_dest_host,
                         sensor::lidar_mode lp_mode,
                         sensor::timestamp_mode ts_mode, int lidar_port,
                         int imu_port, int timeout_sec,
                         bool persist_config) -> client_shared_ptr {
                 auto cli = sensor::init_client(
                     hostname, udp_dest_host, lp_mode, ts_mode, lidar_port,
                     imu_port, timeout_sec, persist_config);
                 if (!cli)
                     throw std::runtime_error(
                         "Failed initializing sensor connection");
                 return cli;
             }),
             py::arg("hostname"), py::arg("udp_dest_host"),
             py::arg("mode") = sensor::lidar_mode::MODE_1024x10,
             py::arg("timestamp_mode") =
                 sensor::timestamp_mode::TIME_FROM_INTERNAL_OSC,
             py::arg("lidar_port") = 0, py::arg("imu_port") = 0,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("persist_config") = false)
        .def(
            "poll",
            [](const client_shared_ptr& self,
               int timeout_sec) -> sensor::client_state {
                return sensor::poll_client(*self, timeout_sec);
            },
            py::arg("timeout_sec") = 1)
        .def("read_lidar_packet",
             [](const client_shared_ptr& self, LidarPacket& packet) -> bool {
                 return sensor::read_lidar_packet(*self, packet);
             })
        .def("read_imu_packet",
             [](const client_shared_ptr& self, ImuPacket& packet) -> bool {
                 return sensor::read_imu_packet(*self, packet);
             })
        .def_property_readonly("lidar_port",
                               [](const client_shared_ptr& self) -> int {
                                   return sensor::get_lidar_port(*self);
                               })
        .def_property_readonly("imu_port",
                               [](const client_shared_ptr& self) -> int {
                                   return sensor::get_imu_port(*self);
                               })
        .def(
            "get_metadata",
            [](client_shared_ptr& self, int timeout_sec) -> std::string {
                return sensor::get_metadata(*self, timeout_sec);
            },
            py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("shutdown", [](client_shared_ptr& self) { self.reset(); });

    // New Client
    py::enum_<sensor::ClientEvent::EventType>(m, "ClientEventType",
                                              py::arithmetic())
        .value("Error", sensor::ClientEvent::Error)
        .value("Exit", sensor::ClientEvent::Exit)
        .value("PollTimeout", sensor::ClientEvent::PollTimeout)
        .value("Packet", sensor::ClientEvent::Packet);

    py::class_<sensor::ClientEvent>(m, "ClientEvent")
        .def(py::init())
        .def_readwrite("source", &sensor::ClientEvent::source)
        .def_readwrite("type", &sensor::ClientEvent::type)
        .def("packet", [](sensor::ClientEvent& self) {
            if (self.type == sensor::ClientEvent::Packet) {
                if (self.packet().type() == sensor::PacketType::Lidar) {
                    return py::cast(
                        static_cast<sensor::LidarPacket&>(self.packet()));
                } else if (self.packet().type() == sensor::PacketType::Imu) {
                    return py::cast(
                        static_cast<sensor::ImuPacket&>(self.packet()));
                }
            }
            throw std::runtime_error("No packet available in event.");
        });

    py::class_<sensor::Sensor>(m, "Sensor")
        .def(py::init<const std::string&, const sensor_config&>(),
             py::arg("hostname"), py::arg("desired_config") = sensor_config())
        .def(
            "fetch_metadata",
            [](sensor::Sensor& self, int timeout) {
                return self.fetch_metadata(timeout);
            },
            py::arg("timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("http_client", &sensor::Sensor::http_client)
        .def("desired_config", &sensor::Sensor::desired_config)
        .def("hostname", &sensor::Sensor::hostname);

    class PacketSourceHack : public ouster::core::PacketSource {
       public:
        using ouster::core::PacketSource::close;
    };
    py::class_<ouster::core::PacketSource, PyPacketSource,
               std::shared_ptr<ouster::core::PacketSource>>(m, "PacketSource",
                                                            R"(
                PacketSource is a base class for reading packet data from various sources.

                Attributes:
                    sensor_info (List[SensorInfo]): Metadata about the sensors providing the packets.
                    is_live (bool): Indicates whether the packet source is live (actively receiving data).

                Methods:
                    close(): Closes the packet source and releases any associated resources.
                    __iter__(): Returns an iterator over the packets in the source.
                )")
        .def(py::init<>())
        .def_property_readonly("sensor_info",
                               &ouster::core::PacketSource::sensor_info,
                               R"(
                                Retrieve sensor information for all sensors in the packet source.

                                Returns:
                                    A list of `SensorInfo` objects, each containing metadata
                                    about a sensor, such as serial number, firmware version,
                                    and calibration details.
                                )")
        .def_property_readonly("is_live", &ouster::core::PacketSource::is_live,
                               R"(
                                Check if the packet source is live.)")

        .def_property_readonly(
            "metadata",
            [](const ouster::core::PacketSource& self) {
                PyErr_WarnEx(PyExc_DeprecationWarning,
                             "metadata is deprecated. To get "
                             "the sensor_info for each sensor use sensor_info."
                             " instead. metadata will be removed in Q2 2025.",
                             2);
                return self.sensor_info();
            })
        .def("close",
             [](ouster::core::PacketSource* self) {
                 ((PacketSourceHack*)self)->close();
             })
        .def("__enter__", [](ouster::core::PacketSource& /*self*/) {})
        .def("__exit__",
             [](ouster::core::PacketSource* self,
                pybind11::object& /*exc_type*/, pybind11::object& /*exc_value*/,
                pybind11::object& /*traceback*/) {
                 ((PacketSourceHack*)self)->close();
             })
        .def(
            "__iter__",
            [](const ouster::core::PacketSource& s) {
                iterator_holder<ouster::core::PacketIterator> holder{s.begin(),
                                                                     s.end()};
                return holder;
            },
            py::keep_alive<
                0,
                1>() /* Essential: keep object alive while iterator exists */);

    py::class_<iterator_holder<ouster::core::PacketIterator>>(m,
                                                              "packet_iterator")
        .def("__iter__",
             [](iterator_holder<ouster::core::PacketIterator>& self)
                 -> iterator_holder<ouster::core::PacketIterator>& {
                 return self;
             })
        .def("__next__", [](iterator_holder<ouster::core::PacketIterator>&
                                self) {
            if (!self.first_or_done) {
                py::gil_scoped_release release;
                self.iter++;
            } else {
                self.first_or_done = false;
            }
            if (self.iter == self.end) {
                self.first_or_done = true;
                throw py::stop_iteration();
            }
            auto& ptr = (*self.iter).second;

            // todo avoid the copies
            auto copy = new ouster::sensor::Packet(*ptr);
            if (ptr->type() == ouster::sensor::PacketType::Lidar) {
                std::pair<int, std::shared_ptr<ouster::sensor::LidarPacket>>
                    pair;
                pair.first = (*self.iter).first;
                pair.second.reset((ouster::sensor::LidarPacket*)copy);
                return py::cast(pair);
            } else {
                std::pair<int, std::shared_ptr<ouster::sensor::ImuPacket>> pair;
                pair.first = (*self.iter).first;
                pair.second.reset((ouster::sensor::ImuPacket*)copy);
                return py::cast(pair);
            }
        });

    py::class_<iterator_holder<ouster::core::ScanIterator>>(m, "scan_iterator")
        .def("__iter__",
             [](iterator_holder<ouster::core::ScanIterator>& self)
                 -> iterator_holder<ouster::core::ScanIterator>& {
                 return self;
             })
        .def("__next__", [](iterator_holder<ouster::core::ScanIterator>& self) {
            py::gil_scoped_release release;
            if (!self.first_or_done) {
                self.iter++;
            } else {
                self.first_or_done = false;
            }
            if (self.iter == self.end) {
                self.first_or_done = true;
                throw py::stop_iteration();
            }

            return *self.iter;
        });

    py::class_<sensor::SensorPacketSource, ouster::core::PacketSource,
               std::shared_ptr<ouster::sensor::SensorPacketSource>>(
        m, "SensorPacketSource",
        R"(
            SensorPacketSource is a class for reading packet data from a sensor.
        
            Examples:
                - Reading packets from a sensor:
                  `SensorPacketSource(sensors, config_timeout, buffer_time)`
        
            Args:
                sensors (List[Sensor]): A list of sensors to read packets from.
                config_timeout (float): Timeout for sensor configuration in seconds.
                buffer_time (float): Buffer time for packet storage in seconds.
        
            Returns:
                SensorPacketSource: An instance of the packet source for reading packets.
            )")
        .def(py::init([](std::vector<sensor::Sensor> sensors,
                         double config_timeout,
                         double buffer_time) -> sensor::SensorPacketSource* {
                 return new sensor::SensorPacketSource(sensors, config_timeout,
                                                       buffer_time);
             }),
             py::arg("sensors"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("buffer_time") = 0)
        .def(py::init([](const std::string& file, const py::kwargs& kwargs) {
                 ouster::PacketSourceOptions opts;
                 parse_packet_source_options(kwargs, opts);
                 return new ouster::sensor::SensorPacketSource(file, opts);
             }),
             py::arg("file"))
        .def(py::init([](const std::vector<std::string>& file,
                         const py::kwargs& kwargs) {
                 ouster::PacketSourceOptions opts;
                 parse_packet_source_options(kwargs, opts);
                 return new ouster::sensor::SensorPacketSource(file, opts);
             }),
             py::arg("file"))
        .def(py::init([](std::vector<sensor::Sensor> sensors,
                         std::vector<sensor::sensor_info> metadata,
                         double config_timeout,
                         double buffer_size) -> sensor::SensorPacketSource* {
                 return new sensor::SensorPacketSource(
                     sensors, metadata, config_timeout, buffer_size);
             }),
             py::arg("sensors"), py::arg("metadata"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("buffer_time") = 0)
        .def("flush", &sensor::SensorPacketSource::flush)
        .def("buffer_size", &sensor::SensorPacketSource::buffer_size)
        .def(
            "get_packet",
            [](sensor::SensorPacketSource& self, double timeout) {
                py::gil_scoped_release release;
                auto packet = self.get_packet(timeout);
                return packet;
            },
            py::arg("timeout") = 0.1);

    class ScanSourceHack : public ouster::core::AnyScanSource {
       public:
        using ouster::core::ScanSource::close;
    };
    py::class_<ouster::core::ScanSource, PyScanSource,
               std::shared_ptr<ouster::core::ScanSource>>(
        m, "ScanSource",
        "ScanSource is a base class for reading point cloud data from various "
        "sources. ")
        .def(py::init<>())
        .def_property_readonly(
            "sensors_count",
            [](const ouster::core::ScanSource& self) {
                PyErr_WarnEx(
                    PyExc_DeprecationWarning,
                    "sensors_count is deprecated. To get "
                    "the number of sensors get the length of sensor_info."
                    " sensors_count() will be removed in Q2 2025.",
                    2);
                return self.sensor_info().size();
            })
        .def_property_readonly(
            "sensor_info",
            [](const ouster::core::ScanSource& self) {
                return self.sensor_info();
            },
            R"(
            Retrieve sensor information for all sensors in the scan source.

            Returns:
                A list of `SensorInfo` objects, each containing metadata
                about a sensor, such as serial number, firmware version,
                and calibration details.
            )")
        .def_property_readonly(
            "metadata",
            [](const ouster::core::ScanSource& self) {
                PyErr_WarnEx(PyExc_DeprecationWarning,
                             "metadata is deprecated. To get "
                             "the sensor_info for each sensor use sensor_info."
                             " instead. metadata will be removed in Q2 2025.",
                             2);
                return self.sensor_info();
            })
        .def_property_readonly(
            "is_live",
            [](const ouster::core::ScanSource& self) { return self.is_live(); },
            R"(
            Check if the scan source is live.

            A live scan source indicates that it is actively receiving data from a sensor.

            Returns:
                bool: True if the scan source is live, False otherwise.
            )")
        .def_property_readonly("is_indexed",
                               [](const ouster::core::ScanSource& self) {
                                   return self.is_indexed();
                               })
        .def_property_readonly("scans_num",
                               [](const ouster::core::ScanSource& self) {
                                   return self.scans_num();
                               })
        .def("__enter__", [](ouster::core::ScanSource& /*self*/) {})
        .def("__exit__",
             [](ouster::core::ScanSource* self, pybind11::object& /*exc_type*/,
                pybind11::object& /*exc_value*/,
                pybind11::object& /*traceback*/) {
                 ((ScanSourceHack*)self)->close();
             })
        .def("close",
             [](ouster::core::ScanSource* self) {
                 ((ScanSourceHack*)self)->close();
             })
        .def("__getitem__", [](const ouster::core::ScanSource& self,
                               int index) { return self[index]; })
        .def(
            "__getitem__",
            [](const ouster::core::ScanSource& self, py::slice slice) {
                size_t start, stop, step, slice_length;
                if (!slice.compute(self.end() - self.begin(), &start, &stop,
                                   &step, &slice_length)) {
                    throw py::error_already_set();
                }
                auto slicer = new ouster::core::Slicer(self, start, stop, step);
                return std::shared_ptr<ouster::core::Slicer>(slicer);
            },
            py::keep_alive<
                0, 1>() /* Essential: keep object alive while wrapper exists */)
        .def(
            "slice",
            [](const ouster::core::ScanSource& self, py::slice slice) {
                size_t start, stop, step, slice_length;
                if (!slice.compute(self.end() - self.begin(), &start, &stop,
                                   &step, &slice_length)) {
                    throw py::error_already_set();
                }
                auto slicer = new ouster::core::Slicer(self, start, stop, step);
                return std::shared_ptr<ouster::core::Slicer>(slicer);
            },
            py::keep_alive<
                0, 1>() /* Essential: keep object alive while wrapper exists */)
        .def(
            "single",
            [](const ouster::core::ScanSource& self, int index) {
                auto singler = new ouster::core::Singler(self, index);
                return std::shared_ptr<ouster::core::Singler>(singler);
            },
            py::keep_alive<
                0, 1>() /* Essential: keep object alive while wrapper exists */)
        .def("__len__",
             [](const ouster::core::ScanSource& s) {
                 try {
                     return s.size();
                 } catch (std::exception& e) {
                     // need to rethrow as type error for python to work
                     // correctly
                     throw pybind11::type_error(
                         "Cannot get the length of an unindexed scan source.");
                 }
             })
        .def_property_readonly(
            "full_index",
            [](const ouster::core::ScanSource& self) {
                auto& data = self.full_index();
                std::vector<size_t> shape = {data.size(), 2};
                auto arr = py::array_t<uint64_t>(shape, (uint64_t*)data.data(),
                                                 py::cast(self));
                reinterpret_cast<py::detail::PyArray_Proxy*>(arr.ptr())
                    ->flags &= ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
                return arr;
            })
        .def_property_readonly(
            "individual_index",
            [](const ouster::core::ScanSource& self) {
                std::vector<py::array> out;
                auto& data = self.individual_index();
                for (const auto& index : data) {
                    std::vector<size_t> shape = {index.size(), 2};
                    auto arr = py::array_t<uint64_t>(
                        shape, (uint64_t*)index.data(), py::cast(self));
                    reinterpret_cast<py::detail::PyArray_Proxy*>(arr.ptr())
                        ->flags &= ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
                    out.push_back(arr);
                }
                return out;
            })
        .def(
            "single_iter",
            [](const ouster::core::ScanSource& s, int sensor_idx) {
                iterator_holder<ouster::core::ScanIterator> holder{
                    s.begin(sensor_idx), s.end()};
                return holder;
            },
            py::keep_alive<
                0,
                1>() /* Essential: keep object alive while iterator exists */)
        .def(
            "__iter__",
            [](const ouster::core::ScanSource& s) {
                iterator_holder<ouster::core::ScanIterator> holder{s.begin(),
                                                                   s.end()};
                return holder;
            },
            py::keep_alive<
                0,
                1>() /* Essential: keep object alive while iterator exists */);

    py::class_<sensor::SensorScanSource, ouster::core::ScanSource,
               std::shared_ptr<ouster::sensor::SensorScanSource>>(
        m, "SensorScanSource")
        .def(py::init<std::string>(), py::arg("file"))
        .def(py::init([](std::vector<sensor::Sensor> sensors, double timeout,
                         unsigned int queue_size,
                         bool soft_id_check) -> sensor::SensorScanSource* {
                 return new sensor::SensorScanSource(sensors, timeout,
                                                     queue_size, soft_id_check);
             }),
             py::arg("sensors"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def(py::init([](const std::string& file, const py::kwargs& kwargs) {
                 ouster::ScanSourceOptions opts;
                 parse_scan_source_options(kwargs, opts);
                 return new ouster::sensor::SensorScanSource(file, opts);
             }),
             py::arg("file"))
        .def(py::init([](const std::vector<std::string>& file,
                         const py::kwargs& kwargs) {
                 ouster::ScanSourceOptions opts;
                 parse_scan_source_options(kwargs, opts);
                 return new ouster::sensor::SensorScanSource(file, opts);
             }),
             py::arg("file"))
        .def(py::init([](std::vector<sensor::Sensor> sensors,
                         std::vector<sensor::sensor_info> metadata,
                         double timeout, unsigned int queue_size,
                         bool soft_id_check) -> sensor::SensorScanSource* {
                 return new sensor::SensorScanSource(sensors, metadata, timeout,
                                                     queue_size, soft_id_check);
             }),
             py::arg("sensors"), py::arg("metadata"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def(py::init([](std::vector<sensor::Sensor> sensors,
                         std::vector<sensor::sensor_info> metadata,
                         const std::vector<std::vector<FieldType>>& field_types,
                         double timeout, unsigned int queue_size,
                         bool soft_id_check) -> sensor::SensorScanSource* {
                 return new sensor::SensorScanSource(sensors, metadata,
                                                     field_types, timeout,
                                                     queue_size, soft_id_check);
             }),
             py::arg("sensors"), py::arg("metadata"), py::arg("fields"),
             py::arg("config_timeout") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("queue_size") = 2, py::arg("soft_id_check") = false)
        .def("flush", &sensor::SensorScanSource::flush)
        .def_property_readonly("dropped_scans",
                               &sensor::SensorScanSource::dropped_scans)
        .def_property_readonly("id_error_count",
                               &sensor::SensorScanSource::id_error_count)
        .def(
            "get_scan",
            [](sensor::SensorScanSource& self, double timeout) {
                py::gil_scoped_release release;
                auto packet = self.get_scan(timeout);
                return packet;
            },
            py::arg("timeout") = 0.1);

    // Client Handle
    py::enum_<sensor::client_state>(m, "ClientState", py::arithmetic())
        .value("TIMEOUT", sensor::client_state::TIMEOUT)
        .value("ERROR", sensor::client_state::CLIENT_ERROR)
        .value("LIDAR_DATA", sensor::client_state::LIDAR_DATA)
        .value("IMU_DATA", sensor::client_state::IMU_DATA)
        .value("EXIT", sensor::client_state::EXIT);

    py::class_<FieldType>(m, "FieldType", R"(
        Describes a field.
    )")
        .def(py::init(
                 [](const std::string& name, py::type dtype,
                    py::tuple extra_dims,
                    size_t flags = (size_t)ouster::FieldClass::PIXEL_FIELD) {
                     return new FieldType(
                         name, field_type_of_dtype(py::dtype::from_args(dtype)),
                         extra_dims.cast<std::vector<size_t>>(),
                         static_cast<ouster::FieldClass>(flags));
                 }),
             R"(
        Construct a FieldType.

        Args:
            name:   name of the field
            dt:     data type of the field, e.g. np.uint8.
            extra_dims: a tuple representing the number of elements.
                in the dimensions beyond width and height,
                for fields with three or more dimensions.
            field_class: indicates whether the field has an entry
                per-packet, per-column, per-scan or per-pixel.
        )",
             py::arg("name"), py::arg("dtype"),
             py::arg("extra_dims") = py::tuple(),
             py::arg("field_class") = ouster::FieldClass::PIXEL_FIELD)
        .def_readwrite("name", &FieldType::name, "The name of the field.")
        .def_readwrite(
            "field_class", &FieldType::field_class,
            R"("field_class - an enum that indicates whether the field has an entry
                per-packet, per-column, per-scan or per-pixel.)")
        .def_property(
            "extra_dims",
            [](FieldType& self) -> py::tuple {
                py::tuple extra_dims_tuple(self.extra_dims.size());
                for (size_t i = 0; i < self.extra_dims.size(); i++) {
                    extra_dims_tuple[i] = self.extra_dims[i];
                }
                return extra_dims_tuple;
            },
            [](FieldType& self, py::tuple extra_dims) {
                self.extra_dims = extra_dims.cast<std::vector<size_t>>();
            },
            R"(A tuple representing the size of extra dimensions
            (if the field is greater than 2 dimensions.))")
        .def_property(
            "element_type",
            [](FieldType& self) -> py::dtype {
                return dtype_of_field_type(self.element_type);
            },
            [](FieldType& self, py::dtype dtype) {
                self.element_type = field_type_of_dtype(dtype);
            },
            "The data type (as a numpy dtype) of the field.")
        .def("__lt__",
             [](const FieldType& u, const FieldType& v) { return u < v; })
        .def("__repr__",
             [](const FieldType& self) {
                 std::stringstream ss;
                 ss << "<ouster.sdk.client.FieldType " << to_string(self)
                    << ">";
                 return ss.str();
             })
        .def("__eq__",
             [](const FieldType& l, const FieldType& r) { return l == r; })
        .def("__str__", [](const FieldType& self) { return to_string(self); });

    // Scans
    py::class_<LidarScan, std::shared_ptr<LidarScan>>(m, "LidarScan", R"(
        Represents a single "scan" or "frame" of lidar data.

        This is a "struct of arrays" representation of lidar data. Column headers are
        stored as contiguous W element arrays, while fields are WxH arrays. Channel
        fields are staggered, so the ith column header value corresponds to the ith
        column of data in each field.
        )")
        // TODO: Python and C++ API differ in h/w order for some reason
        .def(py::init([](size_t h, size_t w) { return new LidarScan(w, h); }),
             R"(

        Default constructor creates a 0 x 0 scan

        Args:
            height: height of scan
            width: width of scan

        Returns:
            New LidarScan of 0x0 expecting fields of the LEGACY profile

        )",
             py::arg("h"), py::arg("w"))
        .def(py::init([](size_t h, size_t w, sensor::UDPProfileLidar profile,
                         size_t columns_per_packet) {
                 return new LidarScan(w, h, profile, columns_per_packet);
             }),
             R"(

        Initialize a scan with the default fields for a particular udp profile

        Args:
            height: height of LidarScan, i.e., number of channels
            width: width of LidarScan
            profile: udp profile

        Returns:
            New LidarScan of specified dimensions expecting fields of specified profile

         )",
             py::arg("h"), py::arg("w"), py::arg("profile"),
             py::arg("columns_per_packet") = DEFAULT_COLUMNS_PER_PACKET)
        .def(py::init([](size_t h, size_t w,
                         const std::vector<ouster::FieldType>& field_types,
                         size_t columns_per_packet) {
                 return new LidarScan(w, h, field_types.begin(),
                                      field_types.end(), columns_per_packet);
             }),
             R"(
        Initialize a scan with a custom set of fields

        Args:
            height: height of LidarScan, i.e., number of channels
            width: width of LidarScan
            field_types: list of FieldType that specifies which fields should be present in the scan

        Returns:
            New LidarScan of specified dimensions expecting fields specified by dict

         )",
             py::arg("w"), py::arg("h"), py::arg("field_types"),
             py::arg("columns_per_packet") = DEFAULT_COLUMNS_PER_PACKET)
        .def(py::init([](const sensor_info& sensor_info) {
                 return new LidarScan(sensor_info);
             }),
             R"(
        Initialize a scan with defaults fields and size for a given sensor_info

        Args:
            sensor_info: SensorInfo to construct a scan for

        Returns:
            New LidarScan approprate for the sensor_info

         )",
             py::arg("sensor_info"))
        .def(py::init([](std::shared_ptr<sensor_info> sensor_info) {
                 return new LidarScan(sensor_info);
             }),
             R"(
        Initialize a scan with defaults fields and size for a given sensor_info

        Args:
            sensor_info: SensorInfo to construct a scan for

        Returns:
            New LidarScan approprate for the sensor_info

         )",
             py::arg("sensor_info"))
        .def(py::init([](std::shared_ptr<sensor_info> sensor_info,
                         const std::vector<FieldType>& field_types) {
                 return new LidarScan(sensor_info, field_types);
             }),
             R"(
        Initialize a scan with defaults fields and size for a given sensor_info
        with only the specified fields

        Args:
            sensor_info: SensorInfo to construct a scan for
            field_types: list of fields to have in the new scan where keys are ChanFields
                         and values are type, e.g., FieldType(client.ChanField.SIGNAL, np.uint32)

        Returns:
            New LidarScan approprate for the sensor_info

         )",
             py::arg("sensor_info"), py::arg("field_types"))
        .def(py::init([](const LidarScan& source,
                         const std::vector<FieldType>& field_types) {
                 return new LidarScan(source, field_types);
             }),
             R"(
        Initialize a lidar scan from another with only the indicated fields.
        Casts, zero pads or removes fields from the original scan if necessary.

        Args:
            source: LidarScan to copy data from
            field_types: list of fields to have in the new scan where keys are ChanFields
                         and values are type, e.g., FieldType(client.ChanField.SIGNAL, np.uint32)

        Returns:
            New LidarScan with selected data copied over or zero padded

         )",
             py::arg("source"), py::arg("field_types"))
        .def(py::init(
                 [](const LidarScan& source) { return new LidarScan(source); }),
             R"(
        Initialize a lidar scan with a copy of the data from another.

        Args:
            source: LidarScan to copy

        Returns:
            New LidarScan with data copied over from provided scan.

         )",
             py::arg("source"))
        .def_readonly("w", &LidarScan::w,
                      "Width or horizontal resolution of the scan.")
        .def_readonly("h", &LidarScan::h,
                      "Height or vertical resolution of the scan.")
        .def_readwrite(
            "frame_id", &LidarScan::frame_id,
            "Corresponds to the frame id header in the packet format.")
        .def_readwrite(
            "frame_status", &LidarScan::frame_status,
            "Information from the packet header which corresponds to a frame.")
        .def_readwrite("shutdown_countdown", &LidarScan::shutdown_countdown,
                       "Thermal shutdown countdown. Please refer to the "
                       "firmware documentation for more information.")
        .def_readwrite("shot_limiting_countdown",
                       &LidarScan::shot_limiting_countdown,
                       "Shot-limiting countdown. Please refer to the firmware "
                       "documentation for more information.")
        .def(
            "complete",
            [](const LidarScan& self,
               nonstd::optional<sensor::ColumnWindow> window) {
                if (!window) {
                    return self.complete();
                }
                return self.complete(window.value());
            },
            py::arg("window") =
                static_cast<nonstd::optional<sensor::ColumnWindow>>(
                    nonstd::nullopt))
        .def_property_readonly(
            "packet_count", &LidarScan::packet_count,
            "The number of packets used to produce a full scan given the width "
            "in pixels and the number of columns per "
            "packet.")
        .def(
            "field",
            [](LidarScan& self, const std::string& name) {
                auto&& field = self.field(name);
                auto dt = dtype_of_field_type(field.tag());
                auto shape = shape_of_descriptor(field.desc());
                return py::array(dt, shape, field.get(), py::cast(self));
            },
            R"(
        Return a view of the specified channel field.

        Args:
            name: name of the field to return

        Returns:
            The specified field as a numpy array
        )")
        .def(
            "add_field",
            [](LidarScan& self, const std::string& name, py::type dtype,
               py::tuple shape, size_t field_class) {
                auto ft = field_type_of_dtype(py::dtype::from_args(dtype));
                Field& ptr = self.add_field(
                    FieldType(name, ft, shape.cast<std::vector<size_t>>(),
                              static_cast<ouster::FieldClass>(field_class)));

                auto dt = dtype_of_field_type(ptr.tag());
                auto sh = shape_of_descriptor(ptr.desc());
                return py::array(dt, sh, ptr.get(), py::cast(self));
            },
            R"(
        Adds a new field under specified name

        Args:
            name: name of the field to add
            shape: tuple of ints, shape of the field to add
            dtype: dtype of field to add, e.g. np.uint32
            field_class: class of the field to add, see field_class

        Returns:
            The field as a numpy array
        )",
            py::arg("name"), py::arg("dtype"), py::arg("shape") = py::tuple(),
            py::arg("field_class") = ouster::FieldClass::PIXEL_FIELD)
        .def(
            "add_field",
            [](LidarScan& self, const FieldType& type) {
                Field& ptr = self.add_field(type);

                auto dt = dtype_of_field_type(ptr.tag());
                auto sh = shape_of_descriptor(ptr.desc());
                return py::array(dt, sh, ptr.get(), py::cast(self));
            },
            R"(
        Adds a new field under specified name and type

        Args:
            type: FieldType of the field to add

        Returns:
            The field as a numpy array
        )",
            py::arg("type"))
        .def(
            "add_field",
            [](LidarScan& self, const std::string& name, const py::array& data,
               size_t field_class) -> py::array {
                if (!(data.flags() & py::array::c_style)) {
                    throw std::invalid_argument(
                        "add_field: submitted ndarray is not c-contiguous");
                }

                auto dt = data.dtype();
                std::vector<size_t> shape;
                for (int i = 0; i < data.ndim(); i++) {
                    shape.push_back(data.shape(i));
                }
                auto ft = field_type_of_dtype(dt);
                Field& ptr = self.add_field(
                    name, FieldDescriptor::array(ft, shape),
                    static_cast<ouster::FieldClass>(field_class));
                auto sh = shape_of_descriptor(ptr.desc());
                py::buffer_info info = data.request();
                memcpy(ptr.get(), info.ptr, ptr.bytes());
                return py::array(dt, sh, ptr.get(), py::cast(self));
            },
            R"(
        Adds a new field under the specified name, with the given contents.
        IMPORTANT: this will deep copy the supplied data.

        Args:
            name: the name of the new field
            data: the contents of the new field
            field_class: class of the field to add, see field_class

        Returns:
            The field as a numpy array.
            )",
            py::arg("name"), py::arg("data"),
            py::arg("field_class") = ouster::FieldClass::PIXEL_FIELD)
        .def(
            "del_field",
            [](LidarScan& self, const std::string& name) -> py::array {
                // need a heap allocated Field to pass to python
                Field* field = new Field(std::move(self.del_field(name)));
                auto dt = dtype_of_field_type(field->tag());
                auto shape = shape_of_descriptor(field->desc());

                py::capsule cleanup{field, [](void* p) {
                                        Field* ptr =
                                            reinterpret_cast<Field*>(p);
                                        delete ptr;
                                    }};
                return py::array(dt, shape, field->get(), cleanup);
            },
            R"(
        Release a field from the LidarScan and return it to the user

        Args:
            name: name of the field to drop

        Returns:
            The specified field as a numpy array
        )")
        .def("has_field", &LidarScan::has_field,
             R"(
        Returns true if the LidarScan has a field with the given name

        Args:
            name: name of the field to check for

        Returns:
            True if the field exists in the scan, false otherwise
        )",
             py::arg("name"))
        .def(
            "field_class",
            [](LidarScan& self, const std::string& name) -> ouster::FieldClass {
                return self.field(name).field_class();
            },
            R"(
        Retrieve FieldClass of field

        Args:
            name: name of the field

        Returns:
            FieldClass of the field
        )")
        .def(
            "has_field",
            [](LidarScan& self, const std::string& name) -> bool {
                return self.has_field(name);
            },
            R"(
        Check if a field with a given name exists in the LidarScan.

        Args:
            name: name of the field

        Returns:
            True if the field is present in the LidarScan.
        )")
        .def("shot_limiting", &LidarScan::shot_limiting,
             "The frame shot limiting status.")
        .def("thermal_shutdown", &LidarScan::thermal_shutdown,
             "The frame thermal shutdown status.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "timestamp",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint64_t>(), self.w,
                                 self.timestamp().data(), py::cast(self));
            },
            "The measurement timestamp header as a W-element numpy array.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "packet_timestamp",
            [](LidarScan& self) {
                return py::array(
                    py::dtype::of<uint64_t>(), self.packet_timestamp().rows(),
                    self.packet_timestamp().data(), py::cast(self));
            },
            "The host timestamp header as a numpy array with "
            "W/columns-per-packet entries.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "alert_flags",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint8_t>(),
                                 self.alert_flags().rows(),
                                 self.alert_flags().data(), py::cast(self));
            },
            "The alert flags header as a numpy array with "
            "W/columns-per-packet entries.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "measurement_id",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint16_t>(), self.w,
                                 self.measurement_id().data(), py::cast(self));
            },
            "The measurement id header as a W-element numpy array.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "status",
            [](LidarScan& self) {
                return py::array(py::dtype::of<uint32_t>(), self.w,
                                 self.status().data(), py::cast(self));
            },
            "The measurement status header as a W-element numpy array.")
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "pose",
            [](LidarScan& self) {
                auto&& field = self.pose();
                return py::array(py::dtype::of<double>(),
                                 shape_of_descriptor(field.desc()), field.get(),
                                 py::cast(self));
            },
            "The pose vector of 4x4 homogeneous matrices (per each timestamp).")
        .def_property_readonly(
            "fields",
            [](const LidarScan& self) {
                std::vector<std::string> keys;
                for (const auto& f : self.fields()) {
                    keys.push_back(f.first);
                }
                std::sort(keys.begin(), keys.end());
                return keys;
            },
            "Return an list of available fields.")
        .def_property_readonly(
            "field_types",
            [](const LidarScan& self) { return self.field_types(); },
            "Return an list of available fields.")
        .def("get_first_valid_packet_timestamp",
             &LidarScan::get_first_valid_packet_timestamp,
             "Return first valid packet timestamp in the scan.")
        .def("get_last_valid_packet_timestamp",
             &LidarScan::get_last_valid_packet_timestamp,
             "Return last valid packet timestamp in the scan.")
        .def("get_first_valid_column_timestamp",
             &LidarScan::get_first_valid_column_timestamp,
             "Return first valid column timestamp in the scan.")
        .def("get_last_valid_column_timestamp",
             &LidarScan::get_last_valid_column_timestamp,
             "Return last valid column timestamp in the scan.")
        .def("get_first_valid_column", &LidarScan::get_first_valid_column,
             "Return first valid column index in the scan.")
        .def("get_last_valid_column", &LidarScan::get_last_valid_column,
             "Return last valid column index in the scan.")
        .def_readwrite("sensor_info", &LidarScan::sensor_info,
                       "The SensorInfo associated with this LidarScan.")
        .def("__eq__",
             [](const LidarScan& l, const LidarScan& r) { return l == r; })
        .def("__copy__", [](const LidarScan& self) { return LidarScan{self}; })
        .def("__deepcopy__",
             [](const LidarScan& self, py::dict) { return LidarScan{self}; })
        .def("__repr__",
             [](const LidarScan& self) {
                 std::stringstream ss;
                 ss << "<ouster.sdk.client._client.LidarScan @" << (void*)&self
                    << ">";
                 return ss.str();
             })
        .def("__str__", [](const LidarScan& self) { return to_string(self); });

    // Destagger overloads for most numpy scalar types
    m.def("destagger_bool", &ouster::destagger<bool>);
    m.def("destagger_int8", &ouster::destagger<int8_t>);
    m.def("destagger_int16", &ouster::destagger<int16_t>);
    m.def("destagger_int32", &ouster::destagger<int32_t>);
    m.def("destagger_int64", &ouster::destagger<int64_t>);
    m.def("destagger_uint8", &ouster::destagger<uint8_t>);
    m.def("destagger_uint16", &ouster::destagger<uint16_t>);
    m.def("destagger_uint32", &ouster::destagger<uint32_t>);
    m.def("destagger_uint64", &ouster::destagger<uint64_t>);
    m.def("destagger_float", &ouster::destagger<float>);
    m.def("destagger_double", &ouster::destagger<double>);

    py::class_<SensorHttp>(m, "SensorHttp")
        .def("metadata", &SensorHttp::metadata,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("sensor_info", &SensorHttp::sensor_info,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("get_config_params", &SensorHttp::get_config_params,
             py::arg("active"),
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("set_config_param", &SensorHttp::set_config_param, py::arg("key"),
             py::arg("value"),
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("active_config_params", &SensorHttp::active_config_params,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("staged_config_params", &SensorHttp::staged_config_params,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("set_udp_dest_auto", &SensorHttp::set_udp_dest_auto,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("beam_intrinsics", &SensorHttp::beam_intrinsics,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("imu_intrinsics", &SensorHttp::imu_intrinsics,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("lidar_intrinsics", &SensorHttp::lidar_intrinsics,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("lidar_data_format", &SensorHttp::lidar_data_format,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("reinitialize", &SensorHttp::reinitialize,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("save_config_params", &SensorHttp::save_config_params,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("get_user_data", &SensorHttp::get_user_data,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        // TODO: get_user_data_and_policy is hard to bind, bind later if needed
        .def("set_user_data", &SensorHttp::set_user_data, py::arg("data"),
             py::arg("keep_on_config_delete") = true,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("delete_user_data", &SensorHttp::delete_user_data,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("network", &SensorHttp::network,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("set_static_ip", &SensorHttp::set_static_ip, py::arg("ip_address"),
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("delete_static_ip", &SensorHttp::delete_static_ip,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("auto_detected_udp_dest", &SensorHttp::auto_detected_udp_dest,
             py::arg("timeout_sec") = SHORT_HTTP_REQUEST_TIMEOUT_SECONDS,
             py::arg("original_destination") = py::none())
        .def(
            "diagnostics_dump",
            [](SensorHttp& self, int timeout_sec) {
                auto vec = self.diagnostics_dump(timeout_sec);
                return py::bytes((const char*)vec.data(), vec.size());
            },
            py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS)
        .def("firmware_version",
             [](SensorHttp& self) { return self.firmware_version(); })
        .def("hostname", &SensorHttp::hostname)
        .def_static(
            "create",
            [](const std::string& hostname, int timeout_sec) {
                return SensorHttp::create(hostname, timeout_sec);
            },
            py::arg("hostname"),
            py::arg("timeout_sec") = LONG_HTTP_REQUEST_TIMEOUT_SECONDS);

    py::class_<ScanBatcher>(m, "ScanBatcher")
        .def(py::init<int, packet_format>())
        .def(py::init<std::shared_ptr<sensor_info>>())
        .def("__call__", [](ScanBatcher& self, LidarPacket& packet,
                            LidarScan& ls) { return self(packet, ls); });

    // XYZ Projection
    py::class_<XYZLutT<float>>(m, "XYZLutFloat")
        .def(py::init([](const sensor_info& sensor, bool use_extrinsics) {
                 XYZLutT<float> lut = XYZLutT<float>(sensor, use_extrinsics);
                 return lut;
             }),
             py::arg("info"), py::arg("use_extrinsics"))
        .def("__call__",
             [](const XYZLutT<float>& self,
                const Eigen::Ref<const img_t<uint32_t>>& range) {
                 return cartesianT<float>(range, self.direction, self.offset);
             })
        .def("__call__", [](const XYZLutT<float>& self, const LidarScan& scan) {
            return cartesianT<float>(scan, self.direction, self.offset);
        });

    py::class_<XYZLutT<double>>(m, "XYZLut")
        .def(py::init([](const sensor_info& sensor, bool use_extrinsics) {
                 return make_xyz_lut(sensor, use_extrinsics);
             }),
             py::arg("info"), py::arg("use_extrinsics"))
        .def("__call__",
             [](const XYZLutT<double>& self,
                const Eigen::Ref<const img_t<uint32_t>>& range) {
                 return cartesianT<double>(range, self.direction, self.offset);
             })
        .def("__call__",
             [](const XYZLutT<double>& self, const LidarScan& scan) {
                 return cartesianT<double>(scan, self.direction, self.offset);
             });

    // Image processing
    py::class_<viz::AutoExposure>(m, "AutoExposure")
        .def(py::init<>())
        .def(py::init<int>(), py::arg("update_every"))
        .def(py::init<double, double, int>(), py::arg("lo_percentile"),
             py::arg("hi_percentile"), py::arg("update_every"))
        .def("__call__", &image_proc_call<viz::AutoExposure, float>,
             py::arg("image"), py::arg("update_state") = true)
        .def("__call__", &image_proc_call<viz::AutoExposure, double>,
             py::arg("image"), py::arg("update_state") = true);

    py::class_<viz::BeamUniformityCorrector>(m, "BeamUniformityCorrector")
        .def(py::init<>())
        .def("__call__", &image_proc_call<viz::BeamUniformityCorrector, float>,
             py::arg("image"), py::arg("update_state") = true)
        .def("__call__", &image_proc_call<viz::BeamUniformityCorrector, double>,
             py::arg("image"), py::arg("update_state") = true);

    m.def(
        "get_field_types",
        [](const sensor::sensor_info& info) {
            return ouster::get_field_types(info);
        },
        R"(
        Extracts LidarScan fields with types for a given SensorInfo

        Args:
            info: sensor metadata for which to find fields types

        Returns:
            returns field types
            )",
        py::arg("info"));

    m.def(
        "get_field_types",
        [](sensor::UDPProfileLidar profile) {
            return ouster::get_field_types(profile);
        },
        R"(
        Extracts LidarScan fields with types for a given ``udp_profile_lidar``

        Args:
            udp_profile_lidar: lidar profile from which to get field types

        Returns:
            returns field types
            )",
        py::arg("udp_profile_lidar"));

    py::class_<ouster::ValidatorIssues>(m, "ValidatorIssues")
        .def_property_readonly(
            "critical",
            [](ouster::ValidatorIssues& self) { return self.critical; },
            "Critical validator issues.")
        .def_property_readonly(
            "warning",
            [](ouster::ValidatorIssues& self) { return self.warning; },
            "Warning validator issues.")
        .def_property_readonly(
            "information",
            [](ouster::ValidatorIssues& self) { return self.information; },
            "Information validator issues");

    py::class_<ouster::ValidatorIssues::ValidatorEntry>(m, "ValidatorEntry")
        .def("__str__", &ouster::ValidatorIssues::ValidatorEntry::to_string,
             R"(
        Get the string representation of a ValidatorEntry

        Returns:
            returns the string representation of a ValidatorEntry
        )")
        .def("__repr__", &ouster::ValidatorIssues::ValidatorEntry::to_string,
             R"(
        Get the string representation of a ValidatorEntry

        Returns:
            returns the string representation of a ValidatorEntry
        )")
        .def("get_path", &ouster::ValidatorIssues::ValidatorEntry::get_path,
             R"(
        Get the entry path to the issue.

        Returns:
            returns the entry path to the issue.
        )")
        .def("get_msg", &ouster::ValidatorIssues::ValidatorEntry::get_msg,
             R"(
        Get the message of the ValidatorEntry

        Returns:
            returns the message of the ValidatorEntry
        )");

    m.def(
        "parse_and_validate_metadata",
        [](const std::string& metadata)
            -> std::tuple<nonstd::optional<ouster::sensor::sensor_info>,
                          ouster::ValidatorIssues> {
            nonstd::optional<ouster::sensor::sensor_info> sensor_info;
            ouster::ValidatorIssues issues;

            ouster::parse_and_validate_metadata(metadata, sensor_info, issues);

            return std::make_pair(sensor_info, issues);
        },
        R"(
        Parse and validate sensor metadata

        Args:
            metadata (str): The metadata json to parse and validate.

        Returns:
            returns (ValidatorIssues, SensorInfo): The list of issues that were encountered 
                                                  and the parsed SensorInfo
        )");

    py::class_<Packet, std::shared_ptr<Packet>>(m, "Packet")
        // direct access to timestamp field
        .def_readwrite("host_timestamp", &Packet::host_timestamp)
        .def_readwrite("format", &Packet::format)
        .def_property_readonly("type",
                               [](const Packet& self) { return self.type(); })
        .def("__copy__", [](const Packet& self) { return Packet(self); })
        .def("__deepcopy__",
             [](const Packet& self, py::dict) { return Packet(self); })
        // NOTE: returned array is writeable, but not reassignable
        .def_property_readonly(
            "buf",
            // we have to use cpp_function here because py::keep_alive
            // does not work with pybind11 def_property methods due to a bug:
            // https://github.com/pybind/pybind11/issues/4236
            py::cpp_function(
                [](Packet& self) {
                    return py::array(py::dtype::of<uint8_t>(), self.buf.size(),
                                     self.buf.data(), py::cast(self));
                },
                py::keep_alive<0, 1>()));

    py::enum_<sensor::PacketType>(m, "PacketType", py::arithmetic())
        .value("Unknown", sensor::PacketType::Unknown)
        .value("Lidar", sensor::PacketType::Lidar)
        .value("Imu", sensor::PacketType::Imu);

    py::enum_<sensor::PacketValidationFailure>(m, "PacketValidationFailure",
                                               py::arithmetic())
        .value("NONE", sensor::PacketValidationFailure::NONE)
        .value("PACKET_SIZE", sensor::PacketValidationFailure::PACKET_SIZE)
        .value("ID", sensor::PacketValidationFailure::ID);

    py::class_<LidarPacket, Packet, std::shared_ptr<LidarPacket>>(m,
                                                                  "LidarPacket")
        .def(py::init<int>(), py::arg("size") = 65536)
        .def("__copy__",
             [](const LidarPacket& self) { return LidarPacket(self); })
        .def("__deepcopy__", [](const LidarPacket& self,
                                py::dict) { return LidarPacket(self); })
        .def("packet_type", &LidarPacket::packet_type)
        .def("frame_id", &LidarPacket::frame_id)
        .def("init_id", &LidarPacket::init_id)
        .def("prod_sn", &LidarPacket::prod_sn)
        .def("alert_flags", &LidarPacket::alert_flags)
        .def("countdown_thermal_shutdown",
             &LidarPacket::countdown_thermal_shutdown)
        .def("countdown_shot_limiting", &LidarPacket::countdown_shot_limiting)
        .def("thermal_shutdown", &LidarPacket::thermal_shutdown)
        .def("shot_limiting", &LidarPacket::shot_limiting)
        .def("crc", &LidarPacket::crc)
        .def("calculate_crc", &LidarPacket::calculate_crc)
        .def("validate",
             [](const LidarPacket& self, const sensor_info& info,
                const ouster::sensor::packet_format& format) {
                 return self.validate(info, format);
             })
        .def("validate", [](const LidarPacket& self, const sensor_info& info) {
            return self.validate(info);
        });

    py::class_<ImuPacket, Packet, std::shared_ptr<ImuPacket>>(m, "ImuPacket")
        .def(py::init<int>(), py::arg("size") = 65536)
        .def("__copy__", [](const ImuPacket& self) { return ImuPacket(self); })
        .def("__deepcopy__",
             [](const ImuPacket& self, py::dict) { return ImuPacket(self); })
        .def("sys_ts", &ImuPacket::sys_ts)
        .def("accel_ts", &ImuPacket::accel_ts)
        .def("gyro_ts", &ImuPacket::gyro_ts)
        .def("la_x", &ImuPacket::la_x)
        .def("la_y", &ImuPacket::la_y)
        .def("la_z", &ImuPacket::la_z)
        .def("la_x", &ImuPacket::av_x)
        .def("la_y", &ImuPacket::av_y)
        .def("la_z", &ImuPacket::av_z)
        .def("validate",
             [](const ImuPacket& self, const sensor_info& info,
                const ouster::sensor::packet_format& format) {
                 return self.validate(info, format);
             })
        .def("validate", [](const ImuPacket& self, const sensor_info& info) {
            return self.validate(info);
        });

    using ouster::sensor::impl::FieldInfo;
    py::class_<FieldInfo>(m, "FieldInfo")
        .def(py::init([](py::object dt, size_t offset, uint64_t mask,
                         int shift) {
            return new FieldInfo{field_type_of_dtype(py::dtype::from_args(dt)),
                                 offset, mask, shift};
        }))
        .def_property_readonly("ty_tag",
                               [](const FieldInfo& self) {
                                   return dtype_of_field_type(self.ty_tag);
                               })
        .def_readwrite("offset", &FieldInfo::offset)
        .def_readwrite("mask", &FieldInfo::mask)
        .def_readwrite("shift", &FieldInfo::shift);

    m.def("add_custom_profile", &ouster::sensor::add_custom_profile);

    m.def("in_multicast", &ouster::sensor::in_multicast);

    m.def("dewarp",
          py::overload_cast<const py::array_t<double>&,
                            const py::array_t<double>&>(&dewarp<double>),
          R"(
	Applies a set of 4x4 pose transformations to a collection of 3D points.
	Args:
	  points: A NumPy array of shape (H, W, 3) representing the 3D points.
	  poses: A NumPy array of shape (W, 4, 4) representing the 4x4 pose

	Return:
	  A NumPy array of shape (H, W, 3) containing the dewarped 3D points
	  )",
          py::arg("points"), py::arg("poses"));

    m.def(
        "dewarp",
        py::overload_cast<const py::array_t<float>&, const py::array_t<float>&>(
            &dewarp<float>),
        R"(
	  )",
        py::arg("points"), py::arg("poses"));

    m.def("transform", &transform<float>,
          R"(
      Applies a single of 4x4 pose transformations to a collection of 3D points.
      Args:
        points: A NumPy array of shape (H, W, 3), or (N, 3)
        pose: A NumPy array of shape (4, 4) representing the 4x4 pose

      Return:
        A NumPy array of shape (H, W, 3) or (N, 3) containing the transformed 3D points
        after applying the corresponding 4x4 transformation matrices to the points
        )",
          py::arg("points"), py::arg("pose"));
    m.def("transform", &transform<double>,
          R"(
        )",
          py::arg("points"), py::arg("pose"));

    m.attr("__version__") = ouster::SDK_VERSION;

    m.attr("SHORT_HTTP_REQUEST_TIMEOUT_SECONDS") =
        py::int_(SHORT_HTTP_REQUEST_TIMEOUT_SECONDS);
    m.attr("LONG_HTTP_REQUEST_TIMEOUT_SECONDS") =
        py::int_(LONG_HTTP_REQUEST_TIMEOUT_SECONDS);

    m.def(
        "open_source",
        [](const std::string& file, bool collate, int sensor_idx,
           const py::kwargs& kwargs) {
            ScanSourceOptions opts;
            parse_scan_source_options(kwargs, opts);
            return ouster::open_source(
                       file, [&](auto& options) { options = opts; }, collate,
                       sensor_idx)
                .child();
        },
        py::arg("source"), py::arg("collate") = true,
        py::arg("sensor_idx") = -1);

    m.def(
        "open_source",
        [](const std::vector<std::string>& files, bool collate, int sensor_idx,
           const py::kwargs& kwargs) {
            ScanSourceOptions opts;
            parse_scan_source_options(kwargs, opts);
            return ouster::open_source(
                       files, [&](auto& options) { options = opts; }, collate,
                       sensor_idx)
                .child();
        },
        py::arg("source"), py::arg("collate") = true,
        py::arg("sensor_idx") = -1);

    m.def(
        "open_packet_source",
        [](const std::string& file, const py::kwargs& kwargs) {
            PacketSourceOptions opts;
            parse_packet_source_options(kwargs, opts);
            return ouster::open_packet_source(
                       file, [&](auto& options) { options = opts; })
                .child();
        },
        py::arg("source"));

    m.def(
        "open_packet_source",
        [](const std::vector<std::string>& files, const py::kwargs& kwargs) {
            PacketSourceOptions opts;
            parse_packet_source_options(kwargs, opts);
            return ouster::open_packet_source(
                       files, [&](auto& options) { options = opts; })
                .child();
        },
        py::arg("source"));

    py::class_<ouster::core::Collator, ouster::core::ScanSource,
               std::shared_ptr<ouster::core::Collator>>(m, "Collator");

    m.def(
        "collate",
        [](ouster::core::ScanSource& src, int dt) {
            return ouster::core::Collator(src, dt);
        },
        py::arg("source"), py::arg("dt") = 210000000, py::keep_alive<0, 1>(),
        R"(
        Collate scans from a scan source.

        This function creates a `Collator` object that combines scans from a scan source.

        Args:
            source (ScanSource): The scan source to collate.
            dt (int): The time delta in nanoseconds for collating scans. Default is 210000000.

        Returns:
            Collator: A collator object for the given scan source.
        )");

    py::class_<ouster::core::Singler, ouster::core::ScanSource,
               std::shared_ptr<ouster::core::Singler>>(m, "Singler",
                                                       R"(
            Singler is a class for extracting scans from a single sensor.
            )");

    py::class_<ouster::core::Slicer, ouster::core::ScanSource,
               std::shared_ptr<ouster::core::Slicer>>(m, "Slicer",
                                                      R"(
                    Slicer is a class for slicing scans from a scan source.
                )");

    m.def(
        "resolve_field_types",
        [](const std::vector<std::shared_ptr<ouster::sensor::sensor_info>>&
               metadata,
           bool raw_headers, bool raw_fields,
           const nonstd::optional<std::vector<std::string>>& field_names) {
            return ouster::resolve_field_types(metadata, raw_headers,
                                               raw_fields, field_names);
        },
        py::arg("metadata"), py::arg("raw_headers") = false,
        py::arg("raw_fields") = false,
        py::arg("field_names") = nonstd::optional<std::vector<std::string>>(),
        R"(
        Resolve field types for a given set of metadata and field names.
    
        This function determines the types of fields (e.g., signal, reflectivity) based on
        the provided sensor metadata and field names.
    
        Args:
            metadata (List[SensorInfo]): A list of sensor metadata objects.
            raw_headers (bool): Whether to include raw headers in the resolution. Default is False.
            raw_fields (bool): Whether to include raw fields in the resolution. Default is False.
            field_names (List[str]): A list of field names to resolve. Default is an empty list.
    
        Returns:
            List[FieldType]: A list of resolved field types.
        )");

    py::enum_<ouster::core::IoType>(m, "IoType", py::arithmetic(),
                                    R"(
        Enumeration of input/output types.

        This enum defines the various types of input/output formats supported by the SDK.
        )")
        .value("OSF", ouster::core::IoType::OSF)
        .value("PCAP", ouster::core::IoType::PCAP)
        .value("SENSOR", ouster::core::IoType::SENSOR)
        .value("BAG", ouster::core::IoType::BAG)
        .value("CSV", ouster::core::IoType::CSV)
        .value("PLY", ouster::core::IoType::PLY)
        .value("PCD", ouster::core::IoType::PCD)
        .value("LAS", ouster::core::IoType::LAS)
        .value("MCAP", ouster::core::IoType::MCAP)
        .value("PNG", ouster::core::IoType::PNG);

    m.def("io_type", &ouster::core::io_type, py::arg("uri"),
          R"(
        Determine the input/output type for a given URI.
        )");

    m.def("io_type_from_extension", &ouster::core::io_type_from_extension,
          py::arg("filename"),
          R"(
            Determine the input/output type based on a file extension.
          )");

    m.def("extension_from_io_type", &ouster::core::extension_from_io_type,
          py::arg("io_type"),
          R"(
          Get the file extension for a given input/output type.
          )");

    m.def(
        "populate_extrinsics",
        [](const std::string& extrinsics_file,
           std::vector<py::array_t<double>> extrinsics,
           std::vector<std::shared_ptr<sensor::sensor_info>>& sensor_infos) {
            std::vector<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> exts;
            for (auto& pose : extrinsics) {
                // Ensure the pose is a 4x4 matrix
                if (pose.ndim() != 2 || pose.shape(0) != 4 ||
                    pose.shape(1) != 4) {
                    throw std::runtime_error(
                        "pose array must have shape (4, 4)");
                }

                py::array_t<double> c_style_pose;

                // Create a C-style copy of pose if it's neither C-style nor
                // F-style const
                const py::array_t<double>* pose_ptr = &pose;
                if (!(pose.flags() & py::array::c_style)) {
                    c_style_pose =
                        py::array_t<double, py::array::c_style>(pose);
                    pose_ptr =
                        &c_style_pose;  // Use the C-style array for processing
                }

                // Convert pose to Eigen format
                Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>
                    pose_eigen(pose_ptr->data());
                exts.push_back(pose_eigen);
            }
            populate_extrinsics(extrinsics_file, exts, sensor_infos);
        },
        R"(
        Populate extrinsics for a set of sensors.
    
        This function reads extrinsics from a file and applies them to the provided
        sensor metadata.
    
        Args:
            extrinsics_file (str): Path to the file containing extrinsics data.
            extrinsics (List[np.ndarray]): A list of 4x4 pose matrices.
            sensor_infos (List[SensorInfo]): A list of sensor metadata objects to update.
    
        Returns:
            None
        )");

    m.def("euler_pose_to_matrix", &ouster::core::euler_pose_to_matrix,
          R"(
            Convert a pose given in Euler angles and translation to a 4x4 transformation matrix.

            The pose vector should contain the following elements in order:
              [roll, pitch, yaw, x, y, z]
            where roll, pitch, and yaw are in radians.

            Returns:
              A 4x4 homogeneous transformation matrix.
          )");

    m.def("quaternion_pose_to_matrix", &ouster::core::quaternion_pose_to_matrix,
          R"(
            Convert a pose given as a quaternion and translation to a 4x4 transformation matrix.

            The pose vector should contain the following elements in order:
              [qw, qx, qy, qz, x, y, z]

            Returns:
              A 4x4 homogeneous transformation matrix.
          )");

    m.def("interp_pose",
          py::overload_cast<const py::array_t<double>&,
                            const py::array_t<double>&,
                            const py::array_t<double>&>(&interp_pose),
          py::arg("x_interp"), py::arg("x_known"), py::arg("poses_known"),
          R"(
        Interpolates 4x4 pose matrices at given x-coordinate values.

        Args:
            x_interp: NumPy array of shape (N, 1) or (N,) with inquiry x-coordinate values.
            x_known: NumPy array of shape (M, 1) or (M,) with known x-coordinate values.
            poses_known: NumPy array of shape (M, 4, 4) with known 4x4 pose matrices.

        Returns:
            NumPy array of shape (N, 4, 4) containing interpolated pose matrices.
        )");

    m.def("voxel_downsample", &downsample_point_cloud, py::arg("voxel_size"),
          py::arg("pts"), py::arg("attributes"),
          py::arg("min_points_per_voxel") = 1,
          R"(
            [BETA] Downsample a pointcloud using a voxel grid of the requested resolution.

            Args:
              voxel_size: The size of the voxel grid.
              pts: Nx3 matrix of points to downsample.
              attributes: A dictionary of attributes to downsample.
              min_points_per_voxel: Minimum number of points per voxel to keep.

            Returns:
              A tuple containing the downsampled points and attributes.

            Note:
              This is a beta feature and its API may change in future releases.
          )");

    m.def("read_pointcloud", &ouster::core::read_pointcloud,
          R"(
            [BETA] Loads the 3D X Y and Z points from a PCD or PLY file and returns
            them as Nx3 matrix.

            Args:
              filename: filename to load

            Returns:
              Nx3 matrix of the resulting points.

            Note:
              This is a beta feature and its API may change in future releases.
          )");

    m.def("set_http_api_headers", &ouster::sensor::util::set_http_api_headers,
          py::arg("headers"));

    m.def("set_http_api_prefix", &ouster::sensor::util::set_http_api_prefix,
          py::arg("prefix"));
}
