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

#include "ouster/lidar_scan.h"

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pyerrors.h>
#include <warnings.h>

#include "client_common.h"
#include "ouster/lidar_scan_set.h"
#include "ouster/types.h"
#include "ouster/zone_state.h"

namespace py = pybind11;
using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::ColumnWindow;
using ouster::sdk::core::Field;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::FieldDescriptor;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::LidarScan;
using ouster::sdk::core::LidarScanSet;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::UDPProfileLidar;
using ouster::sdk::core::ZoneState;

void init_client_lidar_scan(py::module& module, py::module& /*unused*/) {
    py::class_<LidarScan, std::shared_ptr<LidarScan>>(module, "LidarScan", R"(
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
        .def(py::init([](size_t h, size_t w, UDPProfileLidar profile,
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
                         const std::vector<FieldType>& field_types,
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
        .def(py::init([](const SensorInfo& sensor_info) {
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
        .def(py::init([](std::shared_ptr<SensorInfo> sensor_info) {
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
        .def(py::init([](std::shared_ptr<SensorInfo> sensor_info,
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
        .def_readwrite("frame_status", &LidarScan::frame_status,
                       "Information from the packet header which "
                       "corresponds to a frame.")
        .def_readwrite("shutdown_countdown", &LidarScan::shutdown_countdown,
                       "Thermal shutdown countdown. Please refer to the "
                       "firmware documentation for more information.")
        .def_readwrite("shot_limiting_countdown",
                       &LidarScan::shot_limiting_countdown,
                       "Shot-limiting countdown. Please refer to the firmware "
                       "documentation for more information.")
        .def(
            "complete",
            [](const LidarScan& self, nonstd::optional<ColumnWindow> window) {
                if (!window) {
                    return self.complete();
                }
                return self.complete(window.value());
            },
            py::arg("window") =
                static_cast<nonstd::optional<ColumnWindow>>(nonstd::nullopt))
        .def_property_readonly("packet_count", &LidarScan::packet_count,
                               "The number of packets used to produce a "
                               "full scan given the width "
                               "in pixels and the number of columns per "
                               "packet.")
        .def(
            "field",
            [](LidarScan& self, const std::string& name) {
                return field_to_pyobj(self.field(name), py::cast(self));
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
                // NOTE: "shape" here lies, it's actually extra dims
                // TODO: fix the api or stop using FieldType here   -- Tim
                // T.
                FieldType field_type =
                    init_field_type(name, py::dtype::from_args(dtype),
                                    shape.cast<std::vector<size_t>>(),
                                    static_cast<FieldClass>(field_class));

                Field& field = self.add_field(field_type);

                return field_to_pyobj(field, py::cast(self));
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
            py::arg("field_class") = FieldClass::PIXEL_FIELD)
        .def(
            "add_field",
            [](LidarScan& self, const FieldType& type) {
                Field& field = self.add_field(type);
                return field_to_pyobj(field, py::cast(self));
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
            [](LidarScan& self, const std::string& name, py::array& data,
               size_t field_class) -> py::array {
                // If this is a recarray view, then get the base array
                py::module_ np = py::module_::import("numpy");
                if (py::isinstance(data, np.attr("recarray"))) {
                    data = data.attr("base");
                }

                if (!(data.flags() & py::array::c_style)) {
                    throw std::invalid_argument(
                        "add_field: submitted ndarray is not c-contiguous");
                }

                FieldDescriptor desc = pyarray_to_descriptor(data);

                Field& field = self.add_field(
                    name, desc, static_cast<FieldClass>(field_class));

                // copy data
                py::buffer_info info = data.request();
                memcpy(field.get(), info.ptr, field.bytes());

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
            py::arg("field_class") = FieldClass::PIXEL_FIELD)
        .def(
            "del_field",
            [](LidarScan& self, const std::string& name) -> py::array {
                // need a heap allocated Field to pass to python
                Field* field = new Field(self.del_field(name));
                py::capsule cleanup{field, [](void* ptr) {
                                        Field* field_ptr =
                                            reinterpret_cast<Field*>(ptr);
                                        delete field_ptr;
                                    }};
                return field_to_pyobj(*field, cleanup);
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
            [](LidarScan& self, const std::string& name) -> FieldClass {
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
                return field_to_pyobj(field, py::cast(self));
            },
            "The pose vector of 4x4 homogeneous matrices (per each "
            "timestamp).")
        .def_property_readonly(
            "zones",
            [](LidarScan& self) {
                ouster::sdk::core::ArrayView1<ZoneState> zones = self.zones();
                py::object out =
                    py::array(py::dtype::of<ZoneState>(), zones.shape[0],
                              zones.data(), py::cast(self));

                py::module_ numpy = pybind11::module_::import("numpy");
                out = out.attr("view")(numpy.attr("recarray"));

                return out;
            })
        .def_property_readonly(
            "fields",
            [](const LidarScan& self) {
                std::vector<std::string> keys;
                for (const auto& field_pair : self.fields()) {
                    keys.push_back(field_pair.first);
                }
                std::sort(keys.begin(), keys.end());
                return keys;
            },
            "Return a list of available fields.")
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
        .def("__eq__", [](const LidarScan& left,
                          const LidarScan& right) { return left == right; })
        .def("__copy__", [](const LidarScan& self) { return LidarScan{self}; })
        .def("__deepcopy__",
             [](const LidarScan& self, py::dict) { return LidarScan{self}; })
        .def("__repr__",
             [](const LidarScan& self) {
                 std::stringstream stream;
                 stream << "<ouster.sdk.client._client.LidarScan @"
                        << (void*)&self << ">";
                 return stream.str();
             })
        .def("__str__", [](const LidarScan& self) { return to_string(self); });

    py::class_<LidarScanSet>(module, "LidarScanSet")
        .def(py::init<>())
        .def(py::init([](const std::vector<std::shared_ptr<LidarScan>>& scans) {
            return new LidarScanSet(scans);
        }))
        .def("add_field",
             [](LidarScanSet& self, const std::string& name,
                const py::array& data) -> py::array {
                 if (!(data.flags() & py::array::c_style)) {
                     throw std::invalid_argument(
                         "add_field: submitted ndarray is not c-contiguous");
                 }

                 FieldDescriptor desc = pyarray_to_descriptor(data);

                 Field& field = self.add_field(name, desc);

                 // copy data
                 py::buffer_info info = data.request();
                 memcpy(field.get(), info.ptr, field.bytes());

                 return field_to_pyobj(field, py::cast(self));
             })
        .def(
            "add_field",
            [](LidarScanSet& self, const std::string& name, py::type dtype,
               py::tuple shape) -> py::array {
                auto desc = FieldDescriptor::array(
                    cft_of_dtype(py::dtype::from_args(dtype)),
                    shape.cast<std::vector<size_t>>());

                Field& field = self.add_field(name, desc);

                return field_to_pyobj(field, py::cast(self));
            },
            py::arg("name"), py::arg("dtype"), py::arg("shape") = py::tuple())
        .def("has_field", &LidarScanSet::has_field)
        .def("del_field",
             [](LidarScanSet& self, const std::string& name) -> py::array {
                 // need a heap allocated Field to pass to python
                 Field* field = new Field(self.del_field(name));
                 py::capsule cleanup{field, [](void* ptr) {
                                         Field* field_ptr =
                                             reinterpret_cast<Field*>(ptr);
                                         delete field_ptr;
                                     }};
                 return field_to_pyobj(*field, cleanup);
             })
        .def("field",
             [](LidarScanSet& self, const std::string& name) {
                 return field_to_pyobj(self.field(name), py::cast(self));
             })
        .def_property_readonly(
            "fields",
            [](const LidarScanSet& self) {
                std::vector<std::string> keys;
                for (const auto& field_pair : self.fields()) {
                    keys.push_back(field_pair.first);
                }
                std::sort(keys.begin(), keys.end());
                return keys;
            })
        .def(
            "valid_scans",
            [](const LidarScanSet& self) {
                auto valid_scans = self.valid_scans();
                return py::make_iterator(valid_scans.begin(),
                                         valid_scans.end());
            },
            "Return an iterator for scans that are valid.")
        .def(
            "valid_indices",
            [](const LidarScanSet& self) {
                auto valid_indices = self.valid_indices();
                return py::make_iterator(valid_indices.begin(),
                                         valid_indices.end());
            },
            "Return an iterator to the indices for scans that are valid.")
        .def("__len__", &LidarScanSet::size)
        .def("__repr__",
             [](const LidarScanSet& self) {
                 std::stringstream stream;
                 stream << "<ouster.sdk.client._client.LidarScanSet @"
                        << (void*)&self << ">";
                 return stream.str();
             })
        .def("__copy__",
             [](const LidarScanSet& self) { return LidarScanSet{self}; })
        .def("__deepcopy__",
             [](const LidarScanSet& self, py::dict) { return self.clone(); })
        .def(
            "__iter__",
            [](LidarScanSet& self) {
                return py::make_iterator(self.begin(), self.end());
            },
            py::keep_alive<0, 1>())
        .def("__eq__", [](const LidarScanSet& left,
                          const LidarScanSet& right) { return left == right; })
        .def("__getitem__",
             [](const LidarScanSet& self, int index) { return self[index]; });
}
