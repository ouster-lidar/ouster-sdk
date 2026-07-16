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

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <pyerrors.h>
#include <warnings.h>

#include <sstream>

#include "client_common.h"
#include "common.h"
#include "eigen_dense.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/open_source.h"
#include "ouster/core/profile_extension.h"
#include "ouster/core/types.h"

using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::DataFormat;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::FieldDecodeInfo;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::Version;

namespace ouster {
namespace sdk {
namespace core {

namespace impl {
extern OUSTER_API_VAR const Table<FieldClass, const char*, 5> FIELD_CLASS_STRINGS;
}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster

void init_client_field(py::module_& module, py::module_& /*unused*/) {
    auto field_class = py::enum_<FieldClass>(module, "FieldClass", "LidarFrame field classes",
                                             py::is_arithmetic());
    def_enum(field_class, ouster::sdk::core::impl::FIELD_CLASS_STRINGS, "FieldClass");
    // Deprecated alias: SCAN_FIELD was renamed to FRAME_FIELD
    field_class.value("SCAN_FIELD", FieldClass::FRAME_FIELD);

    py::class_<FieldType>(module, "FieldType", R"(
    Describes a field.
)")
        .def(
            "__init__",
            [](FieldType* self, const std::string& name, const py::object& dtype_arg,
               const py::tuple& extra_dims, size_t flags) {
                // In nanobind, there is no direct dtype::from_args.
                // We call numpy.dtype(arg) to normalize the input.
                py::object np_dtype = py::module_::import_("numpy").attr("dtype");
                py::object dtype = np_dtype(dtype_arg);

                // Convert tuple to vector<size_t> manually to be safe
                std::vector<size_t> dims_vec;
                dims_vec.reserve(extra_dims.size());
                for (auto item : extra_dims) {
                    dims_vec.push_back(py::cast<size_t>(item));
                }

                new (self) FieldType(
                    init_field_type(name, dtype, dims_vec, static_cast<FieldClass>(flags)));
            },
            R"(
    Construct a FieldType.

    Args:
        name:   name of the field
        dt:     data type of the field, e.g. np.uint8.
        extra_dims: a tuple representing the number of elements.
            in the dimensions beyond width and height,
            for fields with three or more dimensions.
        field_class: indicates whether the field has an entry
            per-packet, per-column, per-frame or per-pixel.
    )",
            py::arg("name"), py::arg("dtype"), py::arg("extra_dims") = py::tuple(),
            py::arg("field_class") = FieldClass::PIXEL_FIELD)
        .def_rw("name", &FieldType::name, "The name of the field.")
        .def_rw("field_class", &FieldType::field_class,
                R"("field_class - an enum that indicates whether the field has an entry
            per-packet, per-column, per-frame or per-pixel.)")
        .def_prop_rw(
            "extra_dims",
            [](FieldType& self) -> py::tuple {
                py::list extra_dims_list;
                for (size_t dim : self.extra_dims) {
                    extra_dims_list.append(dim);
                }
                return py::tuple(extra_dims_list);
            },
            [](FieldType& self, const py::tuple& extra_dims) {
                // Manually cast tuple elements to size_t vector
                std::vector<size_t> vec;
                vec.reserve(extra_dims.size());
                for (auto handle : extra_dims) {
                    vec.push_back(py::cast<size_t>(handle));
                }
                self.extra_dims = vec;
            },
            R"(A tuple representing the size of extra dimensions
        (if the field is greater than 2 dimensions.))")
        .def_prop_rw(
            "element_type",
            [](FieldType& self) -> py::object { return OusterDtype(self.element_type).otype(); },
            [](FieldType& self, const py::object& dtype) {
                // if previous dtype was fixed string, pop the extra dim
                if (self.element_type == ChanFieldType::CHAR) {
                    if (!self.extra_dims.empty()) {
                        self.extra_dims.pop_back();
                    }
                }
                auto cft = OusterDtype(dtype).cft();
                int itemsize = py::cast<int>(dtype.attr("itemsize"));

                // if new dtype is fixed string, push the extra dim
                if (cft == ChanFieldType::CHAR && itemsize > 0) {
                    self.extra_dims.push_back(static_cast<size_t>(itemsize));
                }

                self.element_type = cft;
            },
            py::for_getter(py::sig("def element_type(self, /) -> numpy.dtype")),
            "The data type (as a numpy dtype) of the field.")
        .def("__lt__", [](const FieldType& left, const FieldType& right) { return left < right; })
        .def("__repr__",
             [](const FieldType& self) {
                 std::stringstream stream;
                 stream << "<ouster.sdk.client.FieldType " << to_string(self) << ">";
                 return stream.str();
             })
        .def("__eq__",
             [](const FieldType& left, const py::object& right) {
                 if (!py::isinstance<FieldType>(right)) {
                     return false;
                 }
                 return left == py::cast<const FieldType&>(right);
             })
        .def("__str__", [](const FieldType& self) { return to_string(self); });

    py::class_<FieldDecodeInfo>(module, "FieldDecodeInfo")
        .def(
            "__init__",
            [](FieldDecodeInfo* self, const py::object& dtype_arg, size_t offset, uint64_t mask,
               int shift, int num_elements) {
                py::object np_dtype = py::module_::import_("numpy").attr("dtype");
                py::object dtype = np_dtype(dtype_arg);

                new (self)
                    FieldDecodeInfo{OusterDtype(dtype).cft(), offset, mask, shift, num_elements};
            },
            py::arg("dtype_arg"), py::arg("offset"), py::arg("mask"), py::arg("shift"),
            py::arg("num_elements") = 1)
        .def_prop_ro("ty_tag",
                     [](const FieldDecodeInfo& self) { return OusterDtype(self.ty_tag).otype(); })
        .def_rw("offset", &FieldDecodeInfo::offset)
        .def_rw("mask", &FieldDecodeInfo::mask)
        .def_rw("shift", &FieldDecodeInfo::shift)
        .def_rw("num_elements", &FieldDecodeInfo::num_elements);

    module.def(
        "resolve_field_types",
        [](const std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>& metadata,
           bool raw_headers, bool raw_fields,
           const nonstd::optional<std::vector<std::string>>& field_names) {
            return ouster::sdk::core::resolve_field_types(metadata, raw_headers, raw_fields,
                                                          field_names);
        },
        py::arg("metadata"), py::arg("raw_headers") = false, py::arg("raw_fields") = false,
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
        [](const SensorInfo& info) { return ouster::sdk::core::get_field_types(info); },
        R"(
    Extracts LidarFrame fields with types for a given SensorInfo

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
    Extracts LidarFrame fields with types for a given SensorInfo

    Args:
        info: sensor data format for which to find field types
        fw_version: sensor firmware version

    Returns:
        returns field types
        )",
        py::arg("format"), py::arg("version"));
}
