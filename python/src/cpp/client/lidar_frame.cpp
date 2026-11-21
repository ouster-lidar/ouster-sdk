/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file
 * @brief ouster_pyclient
 *
 * Note: the type annotations in `client.pyi` need to be updated whenever this
 * file changes. See the mypy documentation for details.
 */

#include "ouster/core/lidar_frame.h"

#include <nanobind/make_iterator.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_map.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "eigen_dense.h"
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include <pyerrors.h>
#include <warnings.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "client_common.h"
#include "common.h"
#include "ouster/core/data_format.h"
#include "ouster/core/field.h"
#include "ouster/core/frame.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/object.h"
#include "ouster/core/packet.h"
#include "ouster/core/sensor_info.h"
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"
#include "ouster/core/zone_state.h"

using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::ColumnWindow;
using ouster::sdk::core::Field;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::FieldDescriptor;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::FieldView;
using ouster::sdk::core::Frame;
using ouster::sdk::core::FrameSet;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::LidarInfo;
using ouster::sdk::core::Object;
using ouster::sdk::core::PacketType;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::ZoneState;

// External linkage is intentional (declared in client/main.cpp); the many
// nanobind .def() bindings are inherently a single long registration routine.
// NOLINTNEXTLINE(misc-use-internal-linkage,readability-function-cognitive-complexity)
void init_client_lidar_frame(py::module_& module, py::module_& /*unused*/) {
    py::bind_map<std::unordered_map<std::string, std::vector<Object>>>(module, "DictStrObjectList");

    // Polymorphic base for LidarFrame and CameraFrame. Registered (without a
    // constructor) so that FrameSet can expose a unified, heterogeneous
    // collection of frames and nanobind can automatically downcast returned
    // base pointers to their concrete registered type.
    //
    // The API common to all frame kinds (dimensions, generic field access,
    // per-timestamp poses/timestamps, ...) is bound here so that algorithms can
    // operate on a heterogeneous FrameSet through the base Frame without
    // downcasting to a concrete type.
    py::class_<Frame>(module, "Frame", "Abstract base class for frames (LidarFrame, CameraFrame).")
        .def_ro("w", &Frame::w, "Width or horizontal resolution of the frame.")
        .def_ro("h", &Frame::h, "Height or vertical resolution of the frame.")
        // Read-only on the base: assigning here could not check that the info
        // matches the frame's sensor kind. LidarFrame and CameraFrame override
        // this with a writable, type-checked property.
        .def_prop_ro(
            "sensor_info", [](const Frame& self) { return self.sensor_info(); },
            "The SensorInfo (calibration) associated with this frame, if any. "
            "Read-only here; assign it through the concrete LidarFrame or "
            "CameraFrame, which check the metadata type.")
        .def_prop_ro("packet_count", &Frame::packet_count,
                     "The number of packets used to produce a "
                     "full frame given the width "
                     "in pixels and the number of columns per "
                     "packet.")
        .def(
            "field",
            [](Frame* self, const std::string& name) {
                // Pass self as the owner to keep the frame alive while the
                // view is used
                return field_to_pyobj(self->field(name), py::cast(self));
            },
            py::rv_policy::reference_internal, py::sig("def field(self, name: str, /) -> NDArray"),
            R"(
    Return a view of the specified channel field.

    Args:
        name: name of the field to return

    Returns:
        The specified field as a numpy array
    )")
        .def(
            "add_field",
            [](Frame* self, const std::string& name, const py::ndarray<py::ro, py::c_contig>& arr,
               size_t field_class) -> py::object {
                // Convert DLPack dtype to OusterDtype (handles basic
                // numeric types)
                OusterDtype o_dtype(arr.dtype());

                // Extract shape
                std::vector<size_t> shape;
                shape.reserve(arr.ndim());
                for (size_t i = 0; i < arr.ndim(); ++i) {
                    shape.push_back(arr.shape(i));
                }

                // Create Descriptor directly from ndarray properties
                FieldDescriptor desc = FieldDescriptor::array(o_dtype.cft(), shape);

                Field& field =
                    self->add_field(name, desc, static_cast<FieldClass>(field_class), false);

                // copy data
                memcpy(field.get(), arr.data(), field.bytes());

                return field_to_pyobj(field, py::cast(self));
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
            py::arg("name"), py::arg("data"), py::arg("field_class") = FieldClass::PIXEL_FIELD,
            py::sig("def add_field(self, name: str, data: NDArray, "
                    "field_class: FieldClass = ..., /) -> NDArray"))
        .def(
            "add_field",
            [](Frame* self, const std::string& name, const py::object& dtype,
               const py::tuple& shape, size_t field_class) {
                // NOTE: "shape" here lies, it's actually extra dims
                // TODO: fix the api or stop using FieldType here   -- Tim
                // T.
                FieldType field_type =
                    init_field_type(name, dtype, py::cast<std::vector<size_t>>(shape),
                                    static_cast<FieldClass>(field_class));

                Field& field = self->add_field(field_type);

                return field_to_pyobj(field, py::cast(self));
            },
            py::sig("def add_field(self, name: str, dtype: "
                    "numpy.typing.DTypeLike, shape: "
                    "typing.Tuple[int, ...] = (), field_class: FieldClass = "
                    "..., /) -> NDArray"),
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
            py::arg("field_class") = FieldClass::PIXEL_FIELD)
        .def(
            "add_field",
            [](Frame* self, const FieldType& type) {
                Field& field = self->add_field(type);
                return field_to_pyobj(field, py::cast(self));
            },
            R"(
    Adds a new field under specified name and type

    Args:
        type: FieldType of the field to add

    Returns:
        The field as a numpy array
    )",
            py::arg("type"), py::sig("def add_field(self, type: FieldType, /) -> NDArray"))
        .def(
            "add_field",
            [](Frame* self, const std::string& name, const py::object& array, size_t field_class) {
                // first make sure it is a recarray
                py::module_ numpy = py::module_::import_("numpy");
                if (!array.type().is(numpy.attr("recarray"))) {
                    throw std::invalid_argument("Argument 'data' needs to be a recarray.");
                }

                // this is a safe cast since a recarray is a pyarrayobject
                auto ptr = reinterpret_cast<PyArrayObject*>(array.ptr());

                OusterDtype zone_type(ChanFieldType::ZONE_STATE);

                // validate that it is the correct type
                if (!array.attr("dtype").attr("__eq__")(zone_type.otype())) {
                    throw std::invalid_argument(
                        "Data argument was unexpected dtype. Must be "
                        "equivalent to ZoneState.dtype().");
                }

                std::vector<size_t> shape;
                auto oshape = PyArray_SHAPE(ptr);
                auto ndim = PyArray_NDIM(ptr);
                shape.reserve(ndim);
                for (int i = 0; i < ndim; ++i) {
                    shape.push_back(oshape[i]);
                }

                FieldDescriptor desc = FieldDescriptor::array(zone_type.cft(), shape);

                Field& field =
                    self->add_field(name, desc, static_cast<FieldClass>(field_class), false);

                memcpy(field.get(), PyArray_DATA(ptr), field.bytes());

                return field_to_pyobj(field, py::cast(self));
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
            py::arg("name"), py::arg("data"), py::arg("field_class") = FieldClass::PIXEL_FIELD,
            py::sig("def add_field(self, name: str, data: numpy.recarray, "
                    "field_class: FieldClass = ..., /) -> NDArray"))
        .def(
            "del_field",
            [](Frame& self, const std::string& name) -> py::object {
                // need a heap allocated Field to pass to python
                Field* field = new Field(self.del_field(name));

                // Nanobind capsule with noexcept destructor
                py::capsule cleanup(field, [](void* ptr) noexcept {
                    Field* field_ptr = reinterpret_cast<Field*>(ptr);
                    delete field_ptr;
                });

                return field_to_pyobj(*field, cleanup);
            },
            py::sig("def del_field(self, name: str, /) -> NDArray"),
            R"(
            Release a field from the frame and return it to the user

            Args:
                name: name of the field to drop

            Returns:
                The specified field as a numpy array
            )")
        .def("has_field", &Frame::has_field,
             R"(
             Returns true if the frame has a field with the given name

             Args:
                 name: name of the field to check for

             Returns:
                 True if the field exists in the frame, false otherwise
             )",
             py::arg("name"))
        .def(
            "field_class",
            [](Frame& self, const std::string& name) -> FieldClass {
                return self.field(name).field_class();
            },
            R"(
            Retrieve FieldClass of field

            Args:
                name: name of the field

    Returns:
        FieldClass of the field
    )")
        // NOTE: returned array is writeable, but not reassignable
        .def_prop_ro(
            "timestamp",
            [](Frame& self) {
                auto timestamps = self.timestamp();
                auto arr = py::ndarray<uint64_t, py::ndim<1>, py::numpy>(
                    timestamps.data(), {static_cast<size_t>(timestamps.size())});
                return arr;
            },
            py::rv_policy::reference_internal,
            "The measurement timestamp header as a numpy array (length w for a "
            "LidarFrame, 1 for a CameraFrame).")
        // NOTE: returned array is writeable, but not reassignable
        .def_prop_ro(
            "receive_timestamp",
            [](Frame& self) {
                size_t size = self.receive_timestamp().rows();
                auto arr = py::ndarray<uint64_t, py::ndim<1>, py::numpy>(
                    self.receive_timestamp().data(), {size});
                return arr;
            },
            py::rv_policy::reference_internal,
            "The per-packet host timestamp header as a numpy array (length "
            "W/columns-per-packet for a LidarFrame; length may vary for a "
            "CameraFrame).")
        // NOTE: returned array is writeable, but not reassignable
        // Deprecated alias for receive_timestamp.
        .def_prop_ro(
            "packet_timestamp",
            [](Frame& self) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "packet_timestamp is deprecated, use receive_timestamp instead", 1);
                size_t size = self.receive_timestamp().rows();
                auto arr = py::ndarray<uint64_t, py::ndim<1>, py::numpy>(
                    self.receive_timestamp().data(), {size});
                return arr;
            },
            py::rv_policy::reference_internal, "Deprecated alias for receive_timestamp.")
        // NOTE: returned array is writeable, but not reassignable
        .def_prop_ro(
            "status",
            [](Frame& self) {
                size_t size = self.status().rows();
                auto arr =
                    py::ndarray<uint32_t, py::ndim<1>, py::numpy>(self.status().data(), {size});
                return arr;
            },
            py::rv_policy::reference_internal,
            "The per-timestamp status header (bit 0 = valid) as a numpy array "
            "(length w for a LidarFrame, 1 for a CameraFrame).")
        .def("get_first_valid_receive_timestamp", &Frame::get_first_valid_receive_timestamp,
             "Return first valid packet timestamp in the frame. Raises "
             "RuntimeError if no valid packets are available.")
        .def("get_last_valid_receive_timestamp", &Frame::get_last_valid_receive_timestamp,
             "Return last valid packet timestamp in the frame. Raises "
             "RuntimeError if no valid packets are available.")
        .def("get_min_valid_receive_timestamp", &Frame::get_min_valid_receive_timestamp,
             "Return min valid packet timestamp in the frame. Raises "
             "RuntimeError if no valid packets are available.")
        .def("get_max_valid_receive_timestamp", &Frame::get_max_valid_receive_timestamp,
             "Return max valid packet timestamp in the frame. Raises "
             "RuntimeError if no valid packets are available.")
        .def(
            "get_first_valid_packet_timestamp",
            [](const Frame& self) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "get_first_valid_packet_timestamp is deprecated, use "
                             "get_first_valid_receive_timestamp instead",
                             1);
                return self.get_first_valid_receive_timestamp();
            },
            "Deprecated alias for get_first_valid_receive_timestamp.")
        .def(
            "get_last_valid_packet_timestamp",
            [](const Frame& self) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "get_last_valid_packet_timestamp is deprecated, use "
                             "get_last_valid_receive_timestamp instead",
                             1);
                return self.get_last_valid_receive_timestamp();
            },
            "Deprecated alias for get_last_valid_receive_timestamp.")
        .def(
            "get_min_valid_packet_timestamp",
            [](const Frame& self) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "get_min_valid_packet_timestamp is deprecated, use "
                             "get_min_valid_receive_timestamp instead",
                             1);
                return self.get_min_valid_receive_timestamp();
            },
            "Deprecated alias for get_min_valid_receive_timestamp.")
        .def(
            "get_max_valid_packet_timestamp",
            [](const Frame& self) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "get_max_valid_packet_timestamp is deprecated, use "
                             "get_max_valid_receive_timestamp instead",
                             1);
                return self.get_max_valid_receive_timestamp();
            },
            "Deprecated alias for get_max_valid_receive_timestamp.")
        .def("get_first_valid_column", &Frame::get_first_valid_column,
             "Return first valid column index in the frame. Raises "
             "RuntimeError if no valid columns are available.")
        .def("get_last_valid_column", &Frame::get_last_valid_column,
             "Return last valid column index in the frame. Raises "
             "RuntimeError if no valid columns are available.")
        // NOTE: returned array is writeable, but not reassignable
        .def_prop_ro(
            "body_to_world",
            [](Frame* self) {
                auto&& field = self->body_to_world();
                return field_to_pyobj(field, py::cast(self));
            },
            py::rv_policy::reference_internal, py::sig("def body_to_world(self, /) -> NDArray"),
            "The body-to-world transform vector of 4x4 homogeneous "
            "matrices (per each timestamp).")
        .def_prop_ro(
            "fields",
            [](const Frame& self) {
                std::vector<std::string> keys;
                for (const auto& field_pair : self.fields()) {
                    keys.push_back(field_pair.first);
                }
                std::sort(keys.begin(), keys.end());
                return keys;
            },
            "Return a list of available fields.")
        .def_prop_ro(
            "objects",
            [](Frame& self) -> std::unordered_map<std::string, std::vector<Object>>& {
                return self.objects();
            },
            py::rv_policy::reference)
        // Bound once on the base: Frame::equals() is virtual, so every frame
        // subclass (LidarFrame, CameraFrame, ...) inherits value comparison
        // through the MRO and compares using its own concrete type. Python
        // derives __ne__ from __eq__ automatically.
        .def("__eq__", [](const Frame& self, const py::object& right) {
            if (!py::isinstance<Frame>(right)) {
                return false;
            }
            return self.equals(py::cast<const Frame&>(right));
        });

    // Polymorphic base for LidarInfo and CameraInfo. Registered here (before
    // camera_frame/data bindings) so FrameSetSource.sensor_info can expose a
    // unified, heterogeneous list of sensor infos and nanobind can downcast the
    // returned base pointers to their concrete registered type.
    //
    // Construction goes through __new__ rather than __init__ so that
    // SensorInfo(json_string) can act as a factory returning a LidarInfo or
    // CameraInfo. The derived classes must also construct via py::new_: when
    // __new__ returns an instance of a subclass, Python (3.8, or any call that
    // bypasses nanobind's type vectorcall) then calls type(obj).__init__ with
    // the same arguments, which must be the no-op __init__ py::new_ installs.
    py::class_<SensorInfo>(module, "SensorInfo",
                           "Abstract base class for sensor infos (LidarInfo, CameraInfo).")
        .def(py::new_([](const std::string& json_string) {
                 return SensorInfo::from_json_string(json_string);
             }),
             py::arg("json_string"), R"(
    Construct the concrete sensor info for a metadata JSON string.

    The sensor kind is inferred from the JSON: metadata with a top-level
    ``camera_info`` object produces a CameraInfo, anything else a LidarInfo.

    Args:
        json_string (str): json string to parse

    Returns:
        LidarInfo or CameraInfo, depending on the metadata.
    )")
        .def_static("from_json_string", &SensorInfo::from_json_string, py::arg("json_string"),
                    R"(
    Construct the concrete sensor info for a metadata JSON string.

    Equivalent to ``SensorInfo(json_string)``.

    Args:
        json_string (str): json string to parse

    Returns:
        LidarInfo or CameraInfo, depending on the metadata.
    )")
        .def_rw("sn", &SensorInfo::sn, "Sensor serial number.")
        .def_rw("sensor_to_body", &SensorInfo::sensor_to_body,
                "Homogeneous transform from the sensor coordinate frame to the body frame. "
                "Defaults to the identity.")
        .def_rw("imu_to_sensor_transform", &SensorInfo::imu_to_sensor_transform,
                "Homogeneous transform from the sensor coordinate frame to the IMU frame "
                "(zero/identity default when the sensor has no separate IMU extrinsic).")
        .def_prop_ro(
            "w", [](const SensorInfo& sensor_info) { return sensor_info.w(); },
            R"(returns the width of a frame (equivalent to format.columns_per_frame))")
        .def_prop_ro(
            "h", [](const SensorInfo& self) { return self.h(); },
            R"(returns the height of a frame (equivalent to format.pixels_per_column))")
        .def_prop_ro(
            "xyzlut_float", [](const SensorInfo& self) { return self.xyzlut<float>(); },
            "Cached ``XYZLutFloat`` for this sensor. The returned LUT stays valid after "
            "``clear_cache()``, but keeps projecting with the values it was built from.")
        .def_prop_ro(
            "xyzlut_double", [](const SensorInfo& self) { return self.xyzlut<double>(); },
            "Cached ``XYZLut`` for this sensor. The returned LUT stays valid after "
            "``clear_cache()``, but keeps projecting with the values it was built from.")
        .def_prop_ro("fps", &SensorInfo::fps, "Rate at which this sensor produces frames.")
        .def_prop_ro("latency", &SensorInfo::latency,
                     "Delay in seconds between the start of a capture and the timestamp "
                     "reported for the resulting frame.")
        .def_prop_ro("could_be_synchronized", &SensorInfo::could_be_synchronized,
                     "Whether this sensor is configured to synchronize with an external time "
                     "source, and so may share a timebase with other sensors.")
        .def("clear_cache", &SensorInfo::clear_cache,
             "Clear cached data such as xyzluts calculated from other members.")
        .def("to_json_string", &SensorInfo::to_json_string,
             "Return a JSON serialized string representation of this sensor metadata.");

    auto lidar_frame =
        py::class_<LidarFrame, Frame>(module, "LidarFrame", R"(
    Represents a single frame of lidar data.

    This is a "struct of arrays" representation of lidar data. Column headers are
    stored as contiguous W element arrays, while fields are WxH arrays. Channel
    fields are staggered, so the ith column header value corresponds to the ith
    column of data in each field.
    )")
            .def(
                "__init__", [](LidarFrame* self) { new (self) LidarFrame(); },
                R"(

    Default constructor creates an invalid 0 x 0 frame

    Returns:
        New LidarFrame of 0x0

    )")
            .def(
                "__init__",
                [](LidarFrame* self, size_t h, size_t w, const std::vector<FieldType>& field_types,
                   size_t columns_per_packet) {
                    new (self) LidarFrame(h, w, field_types, columns_per_packet);
                },
                R"(
    Initialize a frame with a custom set of fields

    Args:
        height: height of LidarFrame, i.e., number of channels
        width: width of LidarFrame
        field_types: list of FieldType that specifies which fields should be present in the frame

    Returns:
        New LidarFrame of specified dimensions expecting fields specified by dict

        )",
                py::arg("h"), py::arg("w"), py::arg("field_types"), py::arg("columns_per_packet"))
            .def(
                "__init__",
                [](LidarFrame* self, std::shared_ptr<LidarInfo> sensor_info) {
                    new (self) LidarFrame(std::move(sensor_info));
                },
                R"(
    Initialize a frame with default fields and size for a given sensor_info

    Args:
        sensor_info: LidarInfo to construct a frame for

    Returns:
        New LidarFrame appropriate for the sensor_info

        )",
                py::arg("sensor_info"))
            .def(
                "__init__",
                [](LidarFrame* self, const std::shared_ptr<LidarInfo>& sensor_info,
                   const std::vector<FieldType>& field_types) {
                    new (self) LidarFrame(sensor_info, field_types);
                },
                R"(
    Initialize a frame with default fields and size for a given sensor_info
    with only the specified fields

    Args:
        sensor_info: LidarInfo to construct a frame for
        field_types: list of fields to have in the new frame where keys are ChanFields
                        and values are type, e.g., FieldType(client.ChanField.SIGNAL, np.uint32)

    Returns:
        New LidarFrame appropriate for the sensor_info

        )",
                py::arg("sensor_info"), py::arg("field_types"))
            .def(
                "__init__",
                [](LidarFrame* self, const LidarFrame& source,
                   const std::vector<FieldType>& field_types) {
                    new (self) LidarFrame(source, field_types);
                },
                R"(
    Initialize a lidar frame from another with only the indicated fields.
    Casts, zero pads or removes fields from the original frame if necessary.

    Args:
        source: LidarFrame to copy data from
        field_types: list of fields to have in the new frame where keys are ChanFields
                        and values are type, e.g., FieldType(client.ChanField.SIGNAL, np.uint32)

    Returns:
        New LidarFrame with selected data copied over or zero padded

        )",
                py::arg("source"), py::arg("field_types"))
            .def(
                "__init__",
                [](LidarFrame* self, const LidarFrame& source) { new (self) LidarFrame(source); },
                R"(
    Initialize a lidar frame with a copy of the data from another.

    Args:
        source: LidarFrame to copy

    Returns:
        New LidarFrame with data copied over from provided frame.

        )",
                py::arg("source"))
            .def_rw("frame_id", &LidarFrame::frame_id,
                    "Corresponds to the frame id header in the packet format.")
            .def_rw("frame_status", &LidarFrame::frame_status,
                    "Information from the packet header which "
                    "corresponds to a frame.")
            .def_rw("shutdown_countdown", &LidarFrame::shutdown_countdown,
                    "Thermal shutdown countdown. Please refer to the "
                    "firmware documentation for more information.")
            .def_rw("shot_limiting_countdown", &LidarFrame::shot_limiting_countdown,
                    "Shot-limiting countdown. Please refer to the firmware "
                    "documentation for more information.")
            .def(
                "complete",
                [](const LidarFrame& self, nonstd::optional<ColumnWindow> window) {
                    if (!window) {
                        return self.complete();
                    }
                    return self.complete(window.value());
                },
                py::arg("window") = static_cast<nonstd::optional<ColumnWindow>>(nonstd::nullopt))
            .def("shot_limiting", &LidarFrame::shot_limiting, "The frame shot limiting status.")
            .def("thermal_shutdown", &LidarFrame::thermal_shutdown,
                 "The frame thermal shutdown status.")
            // NOTE: returned array is writeable, but not reassignable
            .def_prop_ro(
                "alert_flags",
                [](LidarFrame& self) {
                    size_t size = self.alert_flags().rows();
                    auto arr = py::ndarray<uint8_t, py::ndim<1>, py::numpy>(
                        self.alert_flags().data(), {size});
                    return arr;
                },
                py::rv_policy::reference_internal,
                "The alert flags header as a numpy array with "
                "W/columns-per-packet entries.")
            // NOTE: returned array is writeable, but not reassignable
            .def_prop_ro(
                "measurement_id",
                [](LidarFrame& self) {
                    auto arr = py::ndarray<uint16_t, py::ndim<1>, py::numpy>(
                        self.measurement_id().data(), {self.w});
                    return arr;
                },
                "The measurement id header as a W-element numpy array.")
            .def_prop_ro(
                "zones",
                [](LidarFrame* self) {
                    ouster::sdk::core::ArrayView1<ouster::sdk::core::ZoneState> data =
                        self->zones();

                    auto cft = ouster::sdk::core::ChanFieldType::ZONE_STATE;
                    auto fd = ouster::sdk::core::fd_array(cft, data.shape[0]);
                    FieldView field(data.data(), fd);
                    return field_to_pyobj(field, py::cast(self));
                },
                py::rv_policy::reference_internal, py::sig("def zones(self, /) -> NDArray"))
            .def_prop_ro(
                "field_types", [](const LidarFrame& self) { return self.field_types(); },
                "Return a list of available fields.")
            .def("get_first_valid_receive_timestamp",
                 py::overload_cast<>(&LidarFrame::get_first_valid_receive_timestamp, py::const_),
                 "Return first valid packet timestamp in the frame. Raises "
                 "RuntimeError if no valid packets are available.")
            .def("get_first_valid_receive_timestamp",
                 py::overload_cast<PacketType>(&LidarFrame::get_first_valid_receive_timestamp,
                                               py::const_),
                 py::arg("stream"),
                 "Return first valid packet timestamp for the given packet "
                 "type. Raises RuntimeError if no valid packets are available.")
            .def("get_last_valid_receive_timestamp",
                 py::overload_cast<>(&LidarFrame::get_last_valid_receive_timestamp, py::const_),
                 "Return last valid packet timestamp in the frame. Raises "
                 "RuntimeError if no valid packets are available.")
            .def("get_last_valid_receive_timestamp",
                 py::overload_cast<PacketType>(&LidarFrame::get_last_valid_receive_timestamp,
                                               py::const_),
                 py::arg("stream"),
                 "Return last valid packet timestamp for the given packet "
                 "type. Raises RuntimeError if no valid packets are available.")
            .def("get_min_valid_receive_timestamp",
                 py::overload_cast<>(&LidarFrame::get_min_valid_receive_timestamp, py::const_),
                 "Return min valid packet timestamp in the frame. Raises "
                 "RuntimeError if no valid packets are available.")
            .def("get_min_valid_receive_timestamp",
                 py::overload_cast<PacketType>(&LidarFrame::get_min_valid_receive_timestamp,
                                               py::const_),
                 py::arg("stream"),
                 "Return min valid packet timestamp for the given packet type. "
                 "Raises RuntimeError if no valid packets are available.")
            .def("get_max_valid_receive_timestamp",
                 py::overload_cast<>(&LidarFrame::get_max_valid_receive_timestamp, py::const_),
                 "Return max valid packet timestamp in the frame. Raises "
                 "RuntimeError if no valid packets are available.")
            .def("get_max_valid_receive_timestamp",
                 py::overload_cast<PacketType>(&LidarFrame::get_max_valid_receive_timestamp,
                                               py::const_),
                 py::arg("stream"),
                 "Return max valid packet timestamp for the given packet type. "
                 "Raises RuntimeError if no valid packets are available.")
            // Deprecated aliases for the receive_timestamp accessors.
            .def(
                "get_first_valid_packet_timestamp",
                [](const LidarFrame& self) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "get_first_valid_packet_timestamp is deprecated, use "
                                 "get_first_valid_receive_timestamp() instead",
                                 1);
                    return self.get_first_valid_receive_timestamp();
                },
                "Deprecated alias for get_first_valid_receive_timestamp.")
            .def(
                "get_first_valid_packet_timestamp",
                [](const LidarFrame& self, PacketType stream) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "get_first_valid_packet_timestamp is deprecated, use "
                                 "get_first_valid_receive_timestamp(stream) instead",
                                 1);
                    return self.get_first_valid_receive_timestamp(stream);
                },
                py::arg("stream"), "Deprecated alias for get_first_valid_receive_timestamp.")
            .def(
                "get_last_valid_packet_timestamp",
                [](const LidarFrame& self) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "get_last_valid_packet_timestamp is deprecated, use "
                                 "get_last_valid_receive_timestamp() instead",
                                 1);
                    return self.get_last_valid_receive_timestamp();
                },
                "Deprecated alias for get_last_valid_receive_timestamp.")
            .def(
                "get_last_valid_packet_timestamp",
                [](const LidarFrame& self, PacketType stream) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "get_last_valid_packet_timestamp is deprecated, use "
                                 "get_last_valid_receive_timestamp(stream) instead",
                                 1);
                    return self.get_last_valid_receive_timestamp(stream);
                },
                py::arg("stream"), "Deprecated alias for get_last_valid_receive_timestamp.")
            .def(
                "get_min_valid_packet_timestamp",
                [](const LidarFrame& self) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "get_min_valid_packet_timestamp is deprecated, use "
                                 "get_min_valid_receive_timestamp() instead",
                                 1);
                    return self.get_min_valid_receive_timestamp();
                },
                "Deprecated alias for get_min_valid_receive_timestamp.")
            .def(
                "get_min_valid_packet_timestamp",
                [](const LidarFrame& self, PacketType stream) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "get_min_valid_packet_timestamp is deprecated, use "
                                 "get_min_valid_receive_timestamp(stream) instead",
                                 1);
                    return self.get_min_valid_receive_timestamp(stream);
                },
                py::arg("stream"), "Deprecated alias for get_min_valid_receive_timestamp.")
            .def(
                "get_max_valid_packet_timestamp",
                [](const LidarFrame& self) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "get_max_valid_packet_timestamp is deprecated, use "
                                 "get_max_valid_receive_timestamp() instead",
                                 1);
                    return self.get_max_valid_receive_timestamp();
                },
                "Deprecated alias for get_max_valid_receive_timestamp.")
            .def(
                "get_max_valid_packet_timestamp",
                [](const LidarFrame& self, PacketType stream) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "get_max_valid_packet_timestamp is deprecated, use "
                                 "get_max_valid_receive_timestamp(stream) instead",
                                 1);
                    return self.get_max_valid_receive_timestamp(stream);
                },
                py::arg("stream"), "Deprecated alias for get_max_valid_receive_timestamp.")
            // __eq__ is inherited from the Frame base, which dispatches to the
            // virtual Frame::equals() and so compares lidar-specific state.
            .def("__copy__", [](const LidarFrame& self) { return LidarFrame{self}; })
            .def("__deepcopy__", [](const LidarFrame& self,
                                    const py::dict& /*unused*/) { return LidarFrame{self}; })
            .def("__repr__",
                 [](const LidarFrame& self) {
                     std::stringstream stream;
                     stream << "<ouster.sdk.client._client.LidarFrame @"
                            << static_cast<const void*>(&self) << ">";
                     return stream.str();
                 })
            .def("__str__", [](const LidarFrame& self) { return to_string(self); });
    // sensor_info lives on the Frame base as a shared_ptr<SensorInfo>; expose it
    // here as a downcasting property so Python still sees a LidarInfo on read
    // and stores it on the shared base member on write.
    lidar_frame.def_prop_rw(
        "sensor_info", [](const LidarFrame& self) { return self.sensor_info(); },
        [](LidarFrame& self, std::shared_ptr<LidarInfo> info) {
            self.set_sensor_info(std::move(info));
        },
        py::arg().none(), "The LidarInfo associated with this LidarFrame.");

    py::class_<FrameSet>(module, "FrameSet")
        .def(py::init<>())
        .def(
            "__init__",
            [](FrameSet* self, const std::vector<std::shared_ptr<Frame>>& frames) {
                new (self) FrameSet(frames);
            },
            py::arg("frames") = std::vector<std::shared_ptr<Frame>>{},
            py::sig("def __init__(self, frames: Sequence[Optional[Frame]] = []) -> None"))
        .def(
            "add_field",
            [](FrameSet* self, const std::string& name,
               const py::ndarray<py::ro, py::c_contig>& arr) {
                OusterDtype o_dtype(arr.dtype());

                std::vector<size_t> shape;
                shape.reserve(arr.ndim());
                for (size_t i = 0; i < arr.ndim(); ++i) {
                    shape.push_back(arr.shape(i));
                }

                FieldDescriptor desc = FieldDescriptor::array(o_dtype.cft(), shape);

                Field& field = self->add_field(name, desc, false);

                // copy data
                memcpy(field.get(), arr.data(), field.bytes());

                return field_to_pyobj(field, py::cast(self));
            },
            py::rv_policy::reference_internal,
            py::sig("def add_field(self, name: str, data: NDArray, /) -> NDArray"))
        .def(
            "add_field",
            [](FrameSet* self, const std::string& name, const py::object& dtype,
               const py::tuple& shape) {
                auto desc = FieldDescriptor::array(OusterDtype(dtype).cft(),
                                                   py::cast<std::vector<size_t>>(shape));

                Field& field = self->add_field(name, desc);

                return field_to_pyobj(field, py::cast(self));
            },
            py::rv_policy::reference_internal, py::arg("name"), py::arg("dtype"),
            py::arg("shape") = py::tuple(),
            py::sig("def add_field(self, name: str, dtype: type, shape: "
                    "Tuple[int, ...] = (), /) -> NDArray"))
        .def("has_field", &FrameSet::has_field)
        .def(
            "del_field",
            [](FrameSet& self, const std::string& name) -> py::object {
                // need a heap allocated Field to pass to python
                Field* field = new Field(self.del_field(name));
                py::capsule cleanup(field, [](void* ptr) noexcept {
                    Field* field_ptr = reinterpret_cast<Field*>(ptr);
                    delete field_ptr;
                });
                return field_to_pyobj(*field, cleanup);
            },
            py::sig("def del_field(self, name: str, /) -> NDArray"))
        .def(
            "field",
            [](FrameSet* self, const std::string& name) {
                return field_to_pyobj(self->field(name), py::cast(self));
            },
            py::rv_policy::reference_internal, py::sig("def field(self, name: str, /) -> NDArray"))
        .def_prop_ro("fields",
                     [](const FrameSet& self) {
                         std::vector<std::string> keys;
                         for (const auto& field_pair : self.fields()) {
                             keys.push_back(field_pair.first);
                         }
                         std::sort(keys.begin(), keys.end());
                         return keys;
                     })
        .def_prop_ro(
            "objects",
            [](FrameSet& self) -> std::unordered_map<std::string, std::vector<Object>>& {
                return self.objects();
            },
            py::rv_policy::reference)
        .def(
            "frames",
            [](const FrameSet& self) {
                return std::vector<std::shared_ptr<Frame>>(self.begin(), self.end());
            },
            "Return all frames (including null slots), of any type.")
        .def(
            "valid_frames",
            [](const FrameSet& self) {
                std::vector<std::shared_ptr<Frame>> frames;
                for (const auto& frame : self) {
                    if (frame) {
                        frames.push_back(frame);
                    }
                }
                return frames;
            },
            "Return a list of the valid (non-null) frames, of any type.")
        .def(
            "valid_lidar_frames", [](const FrameSet& self) { return self.valid_lidar_frames(); },
            "Return a list of the valid frames whose concrete type is LidarFrame.")
        .def(
            "valid_camera_frames", [](const FrameSet& self) { return self.valid_camera_frames(); },
            "Return a list of the valid frames whose concrete type is CameraFrame.")
        .def(
            "valid_indices",
            [](const FrameSet& self) {
                std::vector<size_t> indexes;
                for (size_t i = 0; i < self.size(); i++) {
                    if (self[i]) {
                        indexes.push_back(i);
                    }
                }
                return indexes;
            },
            "Return a list of the indices for frames that are valid.")
        .def("__len__", &FrameSet::size)
        .def("__repr__",
             [](const FrameSet& self) {
                 std::stringstream stream;
                 stream << "<ouster.sdk.client._client.FrameSet @"
                        << static_cast<const void*>(&self) << ">";
                 return stream.str();
             })
        .def("__copy__", [](const FrameSet& self) { return FrameSet{self}; })
        .def("__deepcopy__", [](const FrameSet& self, const py::dict&) { return self.clone(); })
        .def(
            "__iter__",
            [](FrameSet& self) {
                return py::make_iterator(py::type<FrameSet>(), "FrameSetIterator", self.begin(),
                                         self.end());
            },
            py::keep_alive<0, 1>(),
            py::sig("def __iter__(self) -> "
                    "typing.Iterator[typing.Optional[Frame]]"))
        .def("__eq__",
             [](const FrameSet& self, const py::object& right) {
                 if (!py::isinstance<FrameSet>(right)) {
                     return false;
                 }
                 return self == py::cast<const FrameSet&>(right);
             })
        .def(
            "__setitem__",
            [](FrameSet& self, size_t index, const std::shared_ptr<Frame>& value) {
                self[index] = value;
            },
            py::arg("index"), py::arg("value").none())
        .def(
            "__getitem__", [](const FrameSet& self, size_t index) { return self[index]; },
            py::sig("def __getitem__(self, index: int, /) -> "
                    "typing.Optional[Frame]"));
}
