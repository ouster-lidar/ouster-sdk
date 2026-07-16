#include "ouster/core/frame_ops.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <unordered_set>

#include "ouster/core/field.h"
#include "ouster/core/impl/lidar_frame_impl.h"

namespace ouster {
namespace sdk {
namespace core {
namespace frame_ops {
namespace {

std::vector<std::string> resolve_pixel_fields(const LidarFrame& frame,
                                              const std::vector<std::string>* filtered_fields) {
    std::unordered_set<std::string> pixel_fields;
    for (const auto& ft : frame.field_types()) {
        if (ft.field_class == FieldClass::PIXEL_FIELD) {
            pixel_fields.insert(ft.name);
        }
    }

    std::vector<std::string> requested;
    if (filtered_fields) {
        requested = *filtered_fields;
    } else {
        for (const auto& item : frame.fields()) {
            requested.push_back(item.first);
        }
    }

    std::vector<std::string> present;
    std::vector<std::string> non_pixel;
    for (const auto& field : requested) {
        if (!frame.has_field(field)) {
            continue;
        }
        if (!pixel_fields.count(field)) {
            non_pixel.push_back(field);
            continue;
        }
        present.push_back(field);
    }

    if (filtered_fields && !non_pixel.empty()) {
        std::string message =
            "Only PIXEL_FIELD frame fields are supported here; requested "
            "non-pixel fields: [";
        for (size_t i = 0; i < non_pixel.size(); ++i) {
            if (i) message += ", ";
            message += non_pixel[i];
        }
        message += "]";
        throw std::invalid_argument(message);
    }
    return present;
}

void validate_beam_indices(const std::vector<size_t>& indices, size_t height) {
    if (indices.empty()) {
        throw std::invalid_argument("beam indices can't be empty");
    }
    std::unordered_set<size_t> seen;
    std::vector<size_t> invalid_indices;
    for (auto index : indices) {
        if (!seen.insert(index).second) {
            throw std::invalid_argument("beam indices can't contain duplicates");
        }
        if (index >= height) {
            invalid_indices.push_back(index);
        }
    }
    if (!invalid_indices.empty()) {
        std::string message = "beam indices [";
        for (size_t i = 0; i < invalid_indices.size(); ++i) {
            if (i) message += ", ";
            message += std::to_string(invalid_indices[i]);
        }
        message += "] must be in the range [0, " + std::to_string(height) + ")";
        throw std::invalid_argument(message);
    }
}

template <typename T>
std::vector<T> select_vector(const std::vector<T>& values, const std::vector<size_t>& indices) {
    std::vector<T> out;
    out.reserve(indices.size());
    for (auto index : indices) {
        out.push_back(values.at(index));
    }
    return out;
}

template <typename Derived>
auto select_array_rows(const Eigen::ArrayBase<Derived>& values, const std::vector<size_t>& indices)
    -> Eigen::Array<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> {
    Eigen::Array<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> out(
        indices.size(), values.cols());
    for (size_t i = 0; i < indices.size(); ++i) {
        out.row(static_cast<Eigen::Index>(i)) = values.row(static_cast<Eigen::Index>(indices[i]));
    }
    return out;
}

std::string form_factor_prod_line(const SensorInfo& metadata, size_t v_res) {
    const auto pi = metadata.get_product_info();
    auto form_factor = pi.form_factor;
    if (form_factor.find("MAX") != std::string::npos) {
        form_factor = "OS" + form_factor.substr(2, 1) + "MAX";
    } else if (!form_factor.empty() &&
               std::isdigit(static_cast<unsigned char>(form_factor.back()))) {
        form_factor = form_factor.substr(0, form_factor.size() - 1) + "-" + form_factor.back();
    }
    form_factor = form_factor + "-" + std::to_string(v_res);
    if (pi.rgb) {
        form_factor += "-RGB";
    }
    return form_factor;
}

size_t product_from(const std::vector<size_t>& values, size_t start) {
    size_t result = 1;
    for (size_t i = start; i < values.size(); ++i) {
        result *= values[i];
    }
    return result;
}

void copy_selected_rows(const FieldView& src, FieldView& dst, const std::vector<size_t>& indices) {
    const auto& src_shape = src.shape();
    const auto& dst_shape = dst.shape();
    if (src_shape.empty() || dst_shape.empty()) {
        throw std::invalid_argument("cannot select rows from non-array fields");
    }
    if (dst_shape[0] != indices.size()) {
        throw std::invalid_argument("selected field height mismatch");
    }
    const auto row_bytes = product_from(src_shape, 1) * src.desc().element_size;
    const auto* src_bytes = static_cast<const uint8_t*>(src.get());
    auto* dst_bytes = static_cast<uint8_t*>(dst.get());
    for (size_t row = 0; row < indices.size(); ++row) {
        std::memcpy(dst_bytes + row * row_bytes, src_bytes + indices[row] * row_bytes, row_bytes);
    }
}

struct ClipOp {
    double lower;
    double upper;
    double invalid;

    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field) const {
        // Compare in double: bounds (e.g. +/-inf from open-ended CLI ranges) may
        // not be representable in T, and casting them to T is undefined behavior.
        const auto values = field.template cast<double>();
        field = ((values >= lower) && (values <= upper)).select(field, static_cast<T>(invalid));
    }
};

struct BuildFilterMaskOp {
    double lower;
    double upper;
    std::vector<uint8_t>& mask;

    template <typename T>
    void operator()(Eigen::Ref<const img_t<T>> field) const {
        mask.resize(static_cast<size_t>(field.rows() * field.cols()));
        for (Eigen::Index r = 0; r < field.rows(); ++r) {
            for (Eigen::Index c = 0; c < field.cols(); ++c) {
                // Compare in double: casting out-of-range bounds to T is UB.
                // mask == 0 marks pixels to invalidate: those inside [lower, upper].
                const double value = static_cast<double>(field(r, c));
                mask[static_cast<size_t>(r * field.cols() + c)] =
                    !(value >= lower && value <= upper);
            }
        }
    }
};

struct ApplyMaskOp {
    Eigen::Ref<const img_t<uint8_t>> mask;
    double invalid;

    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field) const {
        for (Eigen::Index r = 0; r < field.rows(); ++r) {
            for (Eigen::Index c = 0; c < field.cols(); ++c) {
                if (!mask(r, c)) {
                    field(r, c) = static_cast<T>(invalid);
                }
            }
        }
    }
};

void apply_mask_to_fields(LidarFrame& frame, const std::vector<std::string>& fields,
                          Eigen::Ref<const img_t<uint8_t>> mask, double invalid) {
    for (const auto& field_name : fields) {
        auto& field = frame.field(field_name);
        impl::visit_field_2d(field, ApplyMaskOp{mask, invalid});
    }
}

}  // namespace

void clip(LidarFrame& frame, const std::vector<std::string>& fields, double lower, double upper,
          double invalid) {
    const auto* filter = fields.empty() ? nullptr : &fields;
    for (const auto& field_name : resolve_pixel_fields(frame, filter)) {
        auto& field = frame.field(field_name);
        impl::visit_field_2d(field, ClipOp{lower, upper, invalid});
    }
}

void filter_field(LidarFrame& frame, const std::string& field, double lower, double upper,
                  double invalid, const std::vector<std::string>* filtered_fields) {
    const auto& filter_view = frame.field(field);
    if (filter_view.shape().size() != 2 || filter_view.shape()[0] != frame.h ||
        filter_view.shape()[1] != frame.w) {
        throw std::invalid_argument(
            "filter_field requires a pixel field with shape (h, w) to build a "
            "mask");
    }

    std::vector<uint8_t> filter_mask;
    impl::visit_field_2d(static_cast<const FieldView&>(filter_view),
                         BuildFilterMaskOp{lower, upper, filter_mask});
    Eigen::Map<const img_t<uint8_t>> filter_mask_eigen(filter_mask.data(), frame.h, frame.w);
    apply_mask_to_fields(frame, resolve_pixel_fields(frame, filtered_fields), filter_mask_eigen,
                         invalid);
}

void filter_uv(LidarFrame& frame, const std::string& coord_2d, size_t lower, size_t upper,
               double invalid, const std::vector<std::string>* filtered_fields) {
    if (coord_2d != "u" && coord_2d != "v") {
        throw std::invalid_argument("coord_2d == " + coord_2d + " must be either 'u' or 'v'");
    }

    const auto coord_size = coord_2d == "u" ? frame.h : frame.w;
    if (lower > coord_size || upper > coord_size) {
        throw std::invalid_argument(
            "lower == " + std::to_string(lower) + " and upper == " + std::to_string(upper) +
            " must be in the range [0, " + std::to_string(coord_size) + "]");
    }
    if (lower > upper) {
        throw std::invalid_argument("lower == " + std::to_string(lower) +
                                    " must be less than upper == " + std::to_string(upper));
    }

    img_t<uint8_t> filter_mask = img_t<uint8_t>::Ones(frame.h, frame.w);
    if (coord_2d == "u") {
        for (size_t r = lower; r < upper; ++r) {
            filter_mask.row(r).setZero();
        }
        apply_mask_to_fields(frame, resolve_pixel_fields(frame, filtered_fields), filter_mask,
                             invalid);
        return;
    }

    for (const auto& field_name : resolve_pixel_fields(frame, filtered_fields)) {
        auto destaggered = destagger(*frame.sensor_info, frame.field(field_name));
        img_t<uint8_t> v_mask = img_t<uint8_t>::Ones(frame.h, frame.w);
        for (size_t r = 0; r < frame.h; ++r) {
            for (size_t c = lower; c < upper; ++c) {
                v_mask(r, c) = 0;
            }
        }
        impl::visit_field_2d(destaggered, ApplyMaskOp{v_mask, invalid});
        frame.field(field_name) = destagger(*frame.sensor_info, destaggered, true);
    }
}

void mask(LidarFrame& frame, const std::vector<std::string>& fields,
          Eigen::Ref<const img_t<uint8_t>> mask) {
    if (mask.rows() != static_cast<Eigen::Index>(frame.h) ||
        mask.cols() != static_cast<Eigen::Index>(frame.w)) {
        throw std::invalid_argument("Used mask size doesn't match frame size");
    }
    const auto* filter = fields.empty() ? nullptr : &fields;
    apply_mask_to_fields(frame, resolve_pixel_fields(frame, filter), mask, 0);
}

std::vector<size_t> reduce_factor_to_indices(size_t factor, size_t height) {
    if (factor == 0) {
        throw std::invalid_argument("factor == 0 can't be negative");
    }
    if (height % factor != 0) {
        throw std::invalid_argument("factor == " + std::to_string(factor) +
                                    " must be a divisor of " + std::to_string(height));
    }
    if (factor == height) {
        return {height / 2};
    }
    std::vector<size_t> indices;
    for (size_t i = 0; i < height; i += factor) {
        indices.push_back(i);
    }
    return indices;
}

SensorInfo select_by_index_metadata(const SensorInfo& metadata,
                                    const std::vector<size_t>& indices) {
    validate_beam_indices(indices, metadata.h());
    auto out = metadata;
    out.prod_line = form_factor_prod_line(metadata, indices.size());
    out.format.pixels_per_column = static_cast<int>(indices.size());
    out.format.pixel_shift_by_row = select_vector(metadata.format.pixel_shift_by_row, indices);
    out.beam_azimuth_angles = select_vector(metadata.beam_azimuth_angles, indices);
    out.beam_altitude_angles = select_vector(metadata.beam_altitude_angles, indices);

    if (out.zone_set) {
        for (auto& item : out.zone_set->zones) {
            auto& zone = item.second;
            if (!zone.zrb) {
                continue;
            }
            zone.zrb->far_range_mm = select_array_rows(zone.zrb->far_range_mm, indices);
            zone.zrb->near_range_mm = select_array_rows(zone.zrb->near_range_mm, indices);
        }
    }
    return out;
}

LidarFrame select_by_index(const LidarFrame& frame, const std::vector<size_t>& indices,
                           bool update_metadata) {
    validate_beam_indices(indices, frame.h);
    if (!frame.sensor_info) {
        throw std::invalid_argument("select_by_index requires frame.sensor_info");
    }

    LidarFrame result(indices.size(), frame.w, frame.field_types(),
                      frame.sensor_info->format.columns_per_packet);
    result.frame_id = frame.frame_id;
    result.frame_status = frame.frame_status;
    result.shutdown_countdown = frame.shutdown_countdown;
    result.shot_limiting_countdown = frame.shot_limiting_countdown;
    result.timestamp() = frame.timestamp();
    result.packet_timestamp() = frame.packet_timestamp();
    result.measurement_id() = frame.measurement_id();
    result.status() = frame.status();
    result.body_to_world() = frame.body_to_world();

    for (const auto& ft : frame.field_types()) {
        if (ft.field_class != FieldClass::PIXEL_FIELD) {
            result.field(ft.name) = frame.field(ft.name);
        } else {
            copy_selected_rows(frame.field(ft.name), result.field(ft.name), indices);
        }
    }
    if (update_metadata) {
        result.sensor_info =
            std::make_shared<SensorInfo>(select_by_index_metadata(*frame.sensor_info, indices));
    }
    return result;
}

SensorInfo reduce_by_factor_metadata(const SensorInfo& metadata, size_t factor) {
    return select_by_index_metadata(metadata, reduce_factor_to_indices(factor, metadata.h()));
}

LidarFrame reduce_by_factor(const LidarFrame& frame, size_t factor, bool update_metadata) {
    return select_by_index(frame, reduce_factor_to_indices(factor, frame.h), update_metadata);
}

}  // namespace frame_ops
}  // namespace core
}  // namespace sdk
}  // namespace ouster
