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

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pyerrors.h>
#include <warnings.h>

#include "client_common.h"
#include "ouster/impl/profile_extension.h"
#include "ouster/lidar_scan.h"
#include "ouster/open_source.h"
#include "ouster/types.h"

namespace py = pybind11;
using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::DataFormat;
using ouster::sdk::core::Field;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::FieldDescriptor;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::UDPProfileLidar;
using ouster::sdk::core::Version;
using ouster::sdk::core::impl::FieldInfo;

namespace ouster {
namespace sdk {
namespace core {

namespace impl {
extern const Table<FieldClass, const char*, 4> FIELD_CLASS_STRINGS;
}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster

void init_client_field(py::module& module, py::module& /*unused*/) {
    auto field_class = py::enum_<FieldClass>(
        module, "FieldClass", "LidarScan field classes", py::arithmetic());
    def_enum(field_class, ouster::sdk::core::impl::FIELD_CLASS_STRINGS);
    py::class_<FieldType>(module, "FieldType", R"(
    Describes a field.
)")
        .def(py::init([](const std::string& name, py::object dtype,
                         py::tuple extra_dims,
                         size_t flags =
                             static_cast<size_t>(FieldClass::PIXEL_FIELD)) {
                 return init_field_type(name, py::dtype::from_args(dtype),
                                        extra_dims.cast<std::vector<size_t>>(),
                                        static_cast<FieldClass>(flags));
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
             py::arg("field_class") = FieldClass::PIXEL_FIELD)
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
                if (self.element_type == ChanFieldType::CHAR) {
                    return py::dtype(std::string("S") +
                                     std::to_string(self.extra_dims.back()));
                } else {
                    return dtype_of_field_type(self.element_type);
                }
            },
            [](FieldType& self, py::dtype dtype) {
                // if previous dtype was fixed string, pop the extra dim
                if (self.element_type == ChanFieldType::CHAR) {
                    self.extra_dims.pop_back();
                }

                auto cft = cft_of_dtype(dtype);
                // if new dtype is fixed string, push the extra dim
                if (cft == ChanFieldType::CHAR && dtype.itemsize() > 0) {
                    self.extra_dims.push_back(
                        static_cast<size_t>(dtype.itemsize()));
                }

                self.element_type = cft;
            },
            "The data type (as a numpy dtype) of the field.")
        .def("__lt__", [](const FieldType& left,
                          const FieldType& right) { return left < right; })
        .def("__repr__",
             [](const FieldType& self) {
                 std::stringstream stream;
                 stream << "<ouster.sdk.client.FieldType " << to_string(self)
                        << ">";
                 return stream.str();
             })
        .def("__eq__", [](const FieldType& left,
                          const FieldType& right) { return left == right; })
        .def("__str__", [](const FieldType& self) { return to_string(self); });

    py::class_<FieldInfo>(module, "FieldInfo")
        .def(py::init(
            [](py::object dtype, size_t offset, uint64_t mask, int shift) {
                return new FieldInfo{cft_of_dtype(py::dtype::from_args(dtype)),
                                     offset, mask, shift};
            }))
        .def_property_readonly("ty_tag",
                               [](const FieldInfo& self) {
                                   return dtype_of_field_type(self.ty_tag);
                               })
        .def_readwrite("offset", &FieldInfo::offset)
        .def_readwrite("mask", &FieldInfo::mask)
        .def_readwrite("shift", &FieldInfo::shift);

    module.def(
        "resolve_field_types",
        [](const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>&
               metadata,
           bool raw_headers, bool raw_fields,
           const nonstd::optional<std::vector<std::string>>& field_names) {
            return ouster::sdk::core::resolve_field_types(
                metadata, raw_headers, raw_fields, field_names);
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

    module.def(
        "get_field_types",
        [](const SensorInfo& info) {
            return ouster::sdk::core::get_field_types(info);
        },
        R"(
    Extracts LidarScan fields with types for a given SensorInfo

    Args:
        info: sensor metadata for which to find fields types

    Returns:
        returns field types
        )",
        py::arg("info"));

    module.def(
        "get_field_types",
        [](const DataFormat& format, const Version& fw_version) {
            return ouster::sdk::core::get_field_types(format, fw_version);
        },
        R"(
    Extracts LidarScan fields with types for a given SensorInfo

    Args:
        info: sensor data format for which to find field types
        fw_version: sensor firmware version

    Returns:
        returns field types
        )",
        py::arg("format"), py::arg("version"));

    module.def(
        "get_field_types",
        [](UDPProfileLidar profile) {
            PyErr_WarnEx(PyExc_DeprecationWarning,
                         "get_field_types(UDPProfileLidar) is deprecated, use "
                         "get_field_types(SensorInfo) instead",
                         2);
            return ouster::sdk::core::get_field_types(profile);
        },
        R"(
    Extracts LidarScan fields with types for a given ``udp_profile_lidar``

    Args:
        udp_profile_lidar: lidar profile from which to get field types

    Returns:
        returns field types
        )",
        py::arg("udp_profile_lidar"));
}
