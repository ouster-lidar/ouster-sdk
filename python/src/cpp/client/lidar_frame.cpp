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

#include <sstream>

#include "client_common.h"
#include "common.h"
#include "ouster/core/frame_set.h"
#include "ouster/core/object.h"
#include "ouster/core/types.h"
#include "ouster/core/zone_state.h"

using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::ColumnWindow;
using ouster::sdk::core::Field;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::FieldDescriptor;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::FieldView;
using ouster::sdk::core::FrameSet;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::Object;
using ouster::sdk::core::PacketType;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::UDPProfileLidar;
using ouster::sdk::core::ZoneState;

void init_client_lidar_frame(py::module_& module, py::module_& /*unused*/) {
    py::bind_map<std::unordered_map<std::string, std::vector<Object>>>(module, "DictStrObjectList");

    auto lidar_frame =
        py::class_<LidarFrame>(module, "LidarFrame", R"(
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
                [](LidarFrame* self, size_t h, size_t w) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "LidarFrame(h, w) is deprecated, use "
                                 "LidarFrame(h, w, field_types, "
                                 "columns_per_packet) instead",
                                 1);
                    new (self) LidarFrame(h, w);
                },
                R"(

        Default constructor creates a H x W frame

        Args:
            height: height of frame
            width: width of frame

        Returns:
            New LidarFrame of HxW expecting fields of the LEGACY profile

        )",
                py::arg("h"), py::arg("w"))
            .def(
                "__init__",
                [](LidarFrame* self, size_t h, size_t w,
                   const std::vector<FieldType>& field_types,
                   size_t columns_per_packet) {
                    new (self)
                        LidarFrame(h, w, field_types, columns_per_packet);
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
                py::arg("h"), py::arg("w"), py::arg("field_types"),
                py::arg("columns_per_packet"))
            .def(
                "__init__",
                [](LidarFrame* self, size_t h, size_t w,
                   UDPProfileLidar profile, size_t columns_per_packet) {
                    PyErr_WarnEx(
                        PyExc_FutureWarning,
                        "LidarFrame(h, w, profile, columns_per_packet) "
                        "is deprecated, use "
                        "LidarFrame(h, w, "
                        "field_types, columns_per_packet) instead",
                        1);
                    new (self) LidarFrame(h, w, profile, columns_per_packet);
                },
                R"(

        Initialize a frame with the default fields for a particular udp profile

        Args:
            height: height of LidarFrame, i.e., number of channels
            width: width of LidarFrame
            profile: udp profile

        Returns:
            New LidarFrame of specified dimensions expecting fields of specified
        profile

            )",
                py::arg("h"), py::arg("w"), py::arg("profile"),
                py::arg("columns_per_packet"))
            .def(
                "__init__",
                [](LidarFrame* self, std::shared_ptr<SensorInfo> sensor_info) {
                    new (self) LidarFrame(std::move(sensor_info));
                },
                R"(
    Initialize a frame with default fields and size for a given sensor_info

    Args:
        sensor_info: SensorInfo to construct a frame for

    Returns:
        New LidarFrame approprate for the sensor_info

        )",
                py::arg("sensor_info"))
            .def(
                "__init__",
                [](LidarFrame* self,
                   const std::shared_ptr<SensorInfo>& sensor_info,
                   const std::vector<FieldType>& field_types) {
                    new (self) LidarFrame(sensor_info, field_types);
                },
                R"(
    Initialize a frame with default fields and size for a given sensor_info
    with only the specified fields

    Args:
        sensor_info: SensorInfo to construct a frame for
        field_types: list of fields to have in the new frame where keys are ChanFields
                        and values are type, e.g., FieldType(client.ChanField.SIGNAL, np.uint32)

    Returns:
        New LidarFrame approprate for the sensor_info

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
                [](LidarFrame* self, const LidarFrame& source) {
                    new (self) LidarFrame(source);
                },
                R"(
    Initialize a lidar frame with a copy of the data from another.

    Args:
        source: LidarFrame to copy

    Returns:
        New LidarFrame with data copied over from provided frame.

        )",
                py::arg("source"))
            .def_ro("w", &LidarFrame::w,
                    "Width or horizontal resolution of the frame.")
            .def_ro("h", &LidarFrame::h,
                    "Height or vertical resolution of the frame.")
            .def_rw("frame_id", &LidarFrame::frame_id,
                    "Corresponds to the frame id header in the packet format.")
            .def_rw("frame_status", &LidarFrame::frame_status,
                    "Information from the packet header which "
                    "corresponds to a frame.")
            .def_rw("shutdown_countdown", &LidarFrame::shutdown_countdown,
                    "Thermal shutdown countdown. Please refer to the "
                    "firmware documentation for more information.")
            .def_rw("shot_limiting_countdown",
                    &LidarFrame::shot_limiting_countdown,
                    "Shot-limiting countdown. Please refer to the firmware "
                    "documentation for more information.")
            .def(
                "complete",
                [](const LidarFrame& self,
                   nonstd::optional<ColumnWindow> window) {
                    if (!window) {
                        return self.complete();
                    }
                    return self.complete(window.value());
                },
                py::arg("window") = static_cast<nonstd::optional<ColumnWindow>>(
                    nonstd::nullopt))
            .def_prop_ro("packet_count", &LidarFrame::packet_count,
                         "The number of packets used to produce a "
                         "full frame given the width "
                         "in pixels and the number of columns per "
                         "packet.")
            .def(
                "field",
                [](LidarFrame* self, const std::string& name) {
                    // Pass self as the owner to keep the frame alive while the
                    // view is used
                    return field_to_pyobj(self->field(name), py::cast(self));
                },
                py::rv_policy::reference_internal,
                py::sig("def field(self, name: str, /) -> NDArray"),
                R"(
    Return a view of the specified channel field.

    Args:
        name: name of the field to return

    Returns:
        The specified field as a numpy array
    )")
            .def(
                "add_field",
                [](LidarFrame* self, const std::string& name,
                   const py::ndarray<py::ro, py::c_contig>& arr,
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
                    FieldDescriptor desc =
                        FieldDescriptor::array(o_dtype.cft(), shape);

                    Field& field = self->add_field(
                        name, desc, static_cast<FieldClass>(field_class));

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
                py::arg("name"), py::arg("data"),
                py::arg("field_class") = FieldClass::PIXEL_FIELD,
                py::sig("def add_field(self, name: str, data: NDArray, "
                        "field_class: FieldClass = ..., /) -> NDArray"))
            .def(
                "add_field",
                [](LidarFrame* self, const std::string& name,
                   const py::object& dtype, const py::tuple& shape,
                   size_t field_class) {
                    // NOTE: "shape" here lies, it's actually extra dims
                    // TODO: fix the api or stop using FieldType here   -- Tim
                    // T.
                    FieldType field_type = init_field_type(
                        name, dtype, py::cast<std::vector<size_t>>(shape),
                        static_cast<FieldClass>(field_class));

                    Field& field = self->add_field(field_type);

                    return field_to_pyobj(field, py::cast(self));
                },
                py::sig(
                    "def add_field(self, name: str, dtype: "
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
                py::arg("name"), py::arg("dtype"),
                py::arg("shape") = py::tuple(),
                py::arg("field_class") = FieldClass::PIXEL_FIELD)
            .def(
                "add_field",
                [](LidarFrame* self, const FieldType& type) {
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
                py::arg("type"),
                py::sig("def add_field(self, type: FieldType, /) -> NDArray"))
            .def(
                "add_field",
                [](LidarFrame* self, const std::string& name,
                   const py::object& array, size_t field_class) {
                    // first make sure it is a recarray
                    py::module_ numpy = py::module_::import_("numpy");
                    if (!array.type().is(numpy.attr("recarray"))) {
                        throw std::invalid_argument(
                            "Argument 'data' needs to be a recarray.");
                    }

                    // this is a safe cast since a recarray is a pyarrayobject
                    auto ptr = reinterpret_cast<PyArrayObject*>(array.ptr());

                    OusterDtype zone_type(ChanFieldType::ZONE_STATE);

                    // validate that it is the correct type
                    if (!array.attr("dtype").attr("__eq__")(
                            zone_type.otype())) {
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

                    FieldDescriptor desc =
                        FieldDescriptor::array(zone_type.cft(), shape);

                    Field& field = self->add_field(
                        name, desc, static_cast<FieldClass>(field_class));

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
                py::arg("name"), py::arg("data"),
                py::arg("field_class") = FieldClass::PIXEL_FIELD,
                py::sig("def add_field(self, name: str, data: numpy.recarray, "
                        "field_class: FieldClass = ..., /) -> NDArray"))
            .def(
                "del_field",
                [](LidarFrame& self, const std::string& name) -> py::object {
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
            Release a field from the LidarFrame and return it to the user

            Args:
                name: name of the field to drop

            Returns:
                The specified field as a numpy array
            )")
            .def("has_field", &LidarFrame::has_field,
                 R"(
             Returns true if the LidarFrame has a field with the given name

             Args:
                 name: name of the field to check for

             Returns:
                 True if the field exists in the frame, false otherwise
             )",
                 py::arg("name"))
            .def(
                "field_class",
                [](LidarFrame& self, const std::string& name) -> FieldClass {
                    return self.field(name).field_class();
                },
                R"(
            Retrieve FieldClass of field

            Args:
                name: name of the field

    Returns:
        FieldClass of the field
    )")
            .def("shot_limiting", &LidarFrame::shot_limiting,
                 "The frame shot limiting status.")
            .def("thermal_shutdown", &LidarFrame::thermal_shutdown,
                 "The frame thermal shutdown status.")
            // NOTE: returned array is writeable, but not reassignable
            .def_prop_ro(
                "timestamp",
                [](LidarFrame& self) {
                    auto arr = py::ndarray<uint64_t, py::ndim<1>, py::numpy>(
                        self.timestamp().data(), {self.w});
                    return arr;
                },
                py::rv_policy::reference_internal,
                "The measurement timestamp header as a W-element numpy array.")
            // NOTE: returned array is writeable, but not reassignable
            .def_prop_ro(
                "packet_timestamp",
                [](LidarFrame& self) {
                    size_t size = self.packet_timestamp().rows();
                    auto arr = py::ndarray<uint64_t, py::ndim<1>, py::numpy>(
                        self.packet_timestamp().data(), {size});
                    return arr;
                },
                py::rv_policy::reference_internal,
                "The host timestamp header as a numpy array with "
                "W/columns-per-packet entries.")
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
            // NOTE: returned array is writeable, but not reassignable
            .def_prop_ro(
                "status",
                [](LidarFrame& self) {
                    auto arr = py::ndarray<uint32_t, py::ndim<1>, py::numpy>(
                        self.status().data(), {self.w});
                    return arr;
                },
                py::rv_policy::reference_internal,
                "The measurement status header as a W-element numpy array.")
            // NOTE: returned array is writeable, but not reassignable
            .def_prop_ro(
                "body_to_world",
                [](LidarFrame* self) {
                    auto&& field = self->body_to_world();
                    return field_to_pyobj(field, py::cast(self));
                },
                py::rv_policy::reference_internal,
                py::sig("def body_to_world(self, /) -> NDArray"),
                "The body-to-world transform vector of 4x4 homogeneous "
                "matrices (per each timestamp).")
            .def_prop_ro(
                "pose",
                [](LidarFrame* self) {
                    PyErr_WarnEx(PyExc_FutureWarning,
                                 "LidarFrame.pose is deprecated, use "
                                 "LidarFrame.body_to_world instead",
                                 1);
                    auto&& field = self->body_to_world();
                    return field_to_pyobj(field, py::cast(self));
                },
                py::rv_policy::reference_internal,
                py::sig("def pose(self, /) -> NDArray"),
                "The body-to-world transform vector of 4x4 homogeneous "
                "matrices (per each timestamp). "
                "Deprecated: use body_to_world.")
            .def_prop_ro(
                "zones",
                [](LidarFrame* self) {
                    ouster::sdk::core::ArrayView1<ouster::sdk::core::ZoneState>
                        data = self->zones();

                    auto cft = ouster::sdk::core::ChanFieldType::ZONE_STATE;
                    auto fd = ouster::sdk::core::fd_array(cft, data.shape[0]);
                    FieldView field(data.data(), fd);
                    return field_to_pyobj(field, py::cast(self));
                },
                py::rv_policy::reference_internal,
                py::sig("def zones(self, /) -> NDArray"))
            .def_prop_ro(
                "fields",
                [](const LidarFrame& self) {
                    std::vector<std::string> keys;
                    for (const auto& field_pair : self.fields()) {
                        keys.push_back(field_pair.first);
                    }
                    std::sort(keys.begin(), keys.end());
                    return keys;
                },
                "Return a list of available fields.")
            .def_prop_ro(
                "field_types",
                [](const LidarFrame& self) { return self.field_types(); },
                "Return a list of available fields.")
            .def_prop_ro(
                "objects",
                [](LidarFrame& self)
                    -> std::unordered_map<std::string, std::vector<Object>>& {
                    return self.objects();
                },
                py::rv_policy::reference)
            .def("get_first_valid_packet_timestamp",
                 py::overload_cast<>(
                     &LidarFrame::get_first_valid_packet_timestamp, py::const_),
                 "Return first valid packet timestamp in the frame. Raises "
                 "RuntimeError if no valid packets are available.")
            .def("get_first_valid_packet_timestamp",
                 py::overload_cast<PacketType>(
                     &LidarFrame::get_first_valid_packet_timestamp, py::const_),
                 py::arg("stream"),
                 "Return first valid packet timestamp for the given packet "
                 "type. Raises RuntimeError if no valid packets are available.")
            .def("get_last_valid_packet_timestamp",
                 py::overload_cast<>(
                     &LidarFrame::get_last_valid_packet_timestamp, py::const_),
                 "Return last valid packet timestamp in the frame. Raises "
                 "RuntimeError if no valid packets are available.")
            .def("get_last_valid_packet_timestamp",
                 py::overload_cast<PacketType>(
                     &LidarFrame::get_last_valid_packet_timestamp, py::const_),
                 py::arg("stream"),
                 "Return last valid packet timestamp for the given packet "
                 "type. Raises RuntimeError if no valid packets are available.")
            .def("get_min_valid_packet_timestamp",
                 py::overload_cast<>(
                     &LidarFrame::get_min_valid_packet_timestamp, py::const_),
                 "Return min valid packet timestamp in the frame. Raises "
                 "RuntimeError if no valid packets are available.")
            .def("get_min_valid_packet_timestamp",
                 py::overload_cast<PacketType>(
                     &LidarFrame::get_min_valid_packet_timestamp, py::const_),
                 py::arg("stream"),
                 "Return min valid packet timestamp for the given packet type. "
                 "Raises RuntimeError if no valid packets are available.")
            .def("get_max_valid_packet_timestamp",
                 py::overload_cast<>(
                     &LidarFrame::get_max_valid_packet_timestamp, py::const_),
                 "Return max valid packet timestamp in the frame. Raises "
                 "RuntimeError if no valid packets are available.")
            .def("get_max_valid_packet_timestamp",
                 py::overload_cast<PacketType>(
                     &LidarFrame::get_max_valid_packet_timestamp, py::const_),
                 py::arg("stream"),
                 "Return max valid packet timestamp for the given packet type. "
                 "Raises RuntimeError if no valid packets are available.")
            .def(
                "get_first_valid_lidar_packet_timestamp",
                [](const LidarFrame& self) {
                    PyErr_WarnEx(
                        PyExc_FutureWarning,
                        "get_first_valid_lidar_packet_timestamp is "
                        "deprecated, use "
                        "get_first_valid_packet_timestamp(PacketType.Lidar) "
                        "instead",
                        1);
                    return self.get_first_valid_lidar_packet_timestamp();
                },
                "Return first valid lidar packet timestamp in the frame. "
                "Deprecated: use "
                "get_first_valid_packet_timestamp(PacketType.Lidar).")
            .def(
                "get_last_valid_lidar_packet_timestamp",
                [](const LidarFrame& self) {
                    PyErr_WarnEx(
                        PyExc_FutureWarning,
                        "get_last_valid_lidar_packet_timestamp is "
                        "deprecated, use "
                        "get_last_valid_packet_timestamp(PacketType.Lidar) "
                        "instead",
                        1);
                    return self.get_last_valid_lidar_packet_timestamp();
                },
                "Return last valid lidar packet timestamp in the frame. "
                "Deprecated: use "
                "get_last_valid_packet_timestamp(PacketType.Lidar).")
            .def("get_first_valid_column", &LidarFrame::get_first_valid_column,
                 "Return first valid column index in the frame. Raises "
                 "RuntimeError if no valid columns are available.")
            .def("get_last_valid_column", &LidarFrame::get_last_valid_column,
                 "Return last valid column index in the frame. Raises "
                 "RuntimeError if no valid columns are available.")
            .def("__eq__",
                 [](const LidarFrame& self, const py::object& right) {
                     if (!py::isinstance<LidarFrame>(right)) {
                         return false;
                     }
                     return self == py::cast<const LidarFrame&>(right);
                 })
            .def("__copy__",
                 [](const LidarFrame& self) { return LidarFrame{self}; })
            .def("__deepcopy__",
                 [](const LidarFrame& self, const py::dict& /*unused*/) {
                     return LidarFrame{self};
                 })
            .def("__repr__",
                 [](const LidarFrame& self) {
                     std::stringstream stream;
                     stream << "<ouster.sdk.client._client.LidarFrame @"
                            << static_cast<const void*>(&self) << ">";
                     return stream.str();
                 })
            .def("__str__",
                 [](const LidarFrame& self) { return to_string(self); });
    def_opt(lidar_frame, LidarFrame, "sensor_info", sensor_info,
            "The SensorInfo associated with this LidarFrame.");

    py::class_<FrameSet>(module, "FrameSet")
        .def(py::init<>())
        .def(
            "__init__",
            [](FrameSet* self, const std::vector<std::shared_ptr<LidarFrame>>& frames) {
                new (self) FrameSet(frames);
            },
            py::arg("frames"),
            py::sig("def __init__(self, frames: Sequence[Optional[LidarFrame]], "
                    "/) -> None"))
        .def(
            "__init__",
            [](FrameSet* self, const std::vector<std::shared_ptr<LidarFrame>>& scans) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "Keyword argument 'scans' is deprecated, use 'frames' "
                             "instead. The last supported version for this will be 1.0.",
                             1);
                new (self) FrameSet(scans);
            },
            py::arg("scans"),
            py::sig("def __init__(self, *, scans: Sequence[Optional[LidarFrame]]) "
                    "-> None"))
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

                Field& field = self->add_field(name, desc);

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
            "valid_frames",
            [](const FrameSet& self) {
                std::vector<std::shared_ptr<LidarFrame>> frames;
                for (const auto& lidar_frame : self) {
                    if (lidar_frame) {
                        frames.push_back(lidar_frame);
                    }
                }
                return frames;
            },
            "Return a list of frames that are valid.")
        .def(
            "valid_scans",
            [](const FrameSet& self) {
                PyErr_WarnEx(PyExc_FutureWarning,
                             "valid_scans is deprecated, use valid_frames instead.", 1);
                std::vector<std::shared_ptr<LidarFrame>> frames;
                for (const auto& lidar_frame : self) {
                    if (lidar_frame) {
                        frames.push_back(lidar_frame);
                    }
                }
                return frames;
            },
            "Deprecated: use valid_frames instead.")
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
                    "typing.Iterator[typing.Optional[LidarFrame]]"))
        .def("__eq__",
             [](const FrameSet& self, const py::object& right) {
                 if (!py::isinstance<FrameSet>(right)) {
                     return false;
                 }
                 return self == py::cast<const FrameSet&>(right);
             })
        .def(
            "__setitem__",
            [](FrameSet& self, size_t index, const std::shared_ptr<LidarFrame>& value) {
                self[index] = value;
            },
            py::arg("index"), py::arg("value").none())
        .def(
            "__getitem__", [](const FrameSet& self, size_t index) { return self[index]; },
            py::sig("def __getitem__(self, index: int, /) -> "
                    "typing.Optional[LidarFrame]"));
}
