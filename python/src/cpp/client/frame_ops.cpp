#include "ouster/core/frame_ops.h"

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <Eigen/Core>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"

namespace py = nanobind;

using ouster::sdk::core::img_t;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::SensorInfo;

namespace {

const std::vector<std::string>* optional_fields(const std::vector<std::string>& fields,
                                                bool has_fields) {
    return has_fields ? &fields : nullptr;
}

}  // namespace

void init_client_frame_ops(py::module_& module, py::module_& /*unused*/) {
    namespace fops = ouster::sdk::core::frame_ops;

    module.def("clip", &fops::clip, py::arg("frame"), py::arg("fields"), py::arg("lower"),
               py::arg("upper"), py::arg("invalid") = 0,
               R"doc(
Limits the values of the specified set of pixel fields to within the range
[lower, upper]. Any value outside this range is replaced by the supplied
invalid value (default is zero).
)doc");

    module.def(
        "filter_field",
        [](LidarFrame& frame, const std::string& field, double lower, double upper, double invalid,
           const std::optional<std::vector<std::string>>& filtered_fields) {
            const auto* fields_ptr = filtered_fields ? &filtered_fields.value() : nullptr;
            fops::filter_field(frame, field, lower, upper, invalid, fields_ptr);
        },
        py::arg("frame"), py::arg("field"), py::arg("lower"), py::arg("upper"),
        py::arg("invalid") = 0, py::arg("filtered_fields") = py::none(),
        R"doc(
Filters frame pixel fields based on the values of another pixel field.
Pixels whose filter field values fall in the range [lower, upper] are
replaced by the supplied invalid value (default is zero).

Parameters:
- frame: LidarFrame
- field: str; the pixel field to be used as the filter mask source
- lower: float; lower bound
- upper: float; upper bound
- invalid: int; the invalid value to use, default is 0
- filtered_fields: Optional[List[str]]; optional list of fields to filter
)doc");

    module.def(
        "_frame_ops_filter_uv",
        [](LidarFrame& frame, const std::string& coord_2d, size_t lower, size_t upper,
           double invalid, const std::vector<std::string>& fields, bool has_fields) {
            fops::filter_uv(frame, coord_2d, lower, upper, invalid,
                            optional_fields(fields, has_fields));
        },
        py::arg("frame"), py::arg("coord_2d"), py::arg("lower"), py::arg("upper"),
        py::arg("invalid") = 0, py::arg("filtered_fields") = std::vector<std::string>{},
        py::arg("has_filtered_fields") = false);

    module.def(
        "_frame_ops_mask",
        [](LidarFrame& frame, const std::vector<std::string>& fields,
           const py::ndarray<const uint8_t, py::ndim<2>, py::c_contig>& mask) {
            Eigen::Map<const img_t<uint8_t>> mask_eigen(mask.data(),
                                                        static_cast<Eigen::Index>(mask.shape(0)),
                                                        static_cast<Eigen::Index>(mask.shape(1)));
            fops::mask(frame, fields, mask_eigen);
        },
        py::arg("frame"), py::arg("fields"), py::arg("mask"));

    module.def("select_by_index_metadata", &fops::select_by_index_metadata, py::arg("metadata"),
               py::arg("indices"));
    module.def("select_by_index", &fops::select_by_index, py::arg("frame"), py::arg("indices"),
               py::arg("update_metadata") = false);
    module.def("reduce_by_factor_metadata", &fops::reduce_by_factor_metadata, py::arg("metadata"),
               py::arg("factor"));
    module.def("reduce_by_factor", &fops::reduce_by_factor, py::arg("frame"), py::arg("factor"),
               py::arg("update_metadata") = false);
}
