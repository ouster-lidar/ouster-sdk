/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

// Companion implementation header: it is included from the bottom of
// lidar_frame.h and relies on the declarations above that point, so it cannot be
// included on its own.
// IWYU pragma: private, include "ouster/core/lidar_frame.h"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/core/field.h"
#include "ouster/core/types.h"
#include "ouster/core/visibility.h"
#include "ouster/core/zone_state.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

/**
 * A packed triple of 16-bit float values.
 *
 * Stores three float16_t components (a, b, c) in a single tightly-packed
 * structure, used for RGB or similar 3-channel float16 field data.
 */
#pragma pack(push, 1)
struct OUSTER_API_CLASS float3x16_t {
    uint16_t a; /**< First float16 component. */
    uint16_t b; /**< Second float16 component. */
    uint16_t c; /**< Third float16 component. */
};
#pragma pack(pop)

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the Field `field`
 * NOTE: requested field must be two dimensional
 * Example code for the operation<T>:
 * \code
 * struct print_field_size {
 *   template <typename T>
 *   void operator()(Eigen::Ref<img_t<T>> field) {
 *       std::cout << "Rows: " << field.rows() << std::endl;
 *       std::cout << "Cols: " << field.cols() << std::endl;
 *   }
 * };
 * \endcode
 */
template <typename OP, typename... Args>
void visit_field_2d(FieldView& field, OP&& op, Args&&... args) {
    switch (field.tag()) {
        case ChanFieldType::UINT8:
            op.template operator()(Eigen::Ref<img_t<uint8_t>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT16:
            op.template operator()(Eigen::Ref<img_t<uint16_t>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT32:
            op.template operator()(Eigen::Ref<img_t<uint32_t>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT64:
            op.template operator()(Eigen::Ref<img_t<uint64_t>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT8:
            op.template operator()(Eigen::Ref<img_t<int8_t>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT16:
            op.template operator()(Eigen::Ref<img_t<int16_t>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT32:
            op.template operator()(Eigen::Ref<img_t<int32_t>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT64:
            op.template operator()(Eigen::Ref<img_t<int64_t>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::FLOAT16:
            // op.template operator()(Eigen::Ref<img_t<float16_t>>(field),
            //                        std::forward<Args>(args)...);
            break;
        case ChanFieldType::FLOAT32:
            op.template operator()(Eigen::Ref<img_t<float>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::FLOAT64:
            op.template operator()(Eigen::Ref<img_t<double>>(field), std::forward<Args>(args)...);
            break;
        case ChanFieldType::ZONE_STATE:
        case ChanFieldType::CHAR:
        case ChanFieldType::VOID:
        case ChanFieldType::UNREGISTERED:
            // These types are not Eigen scalar types or are not supported,
            // so we silently skip them rather than crash.
            break;
        default:
            throw std::invalid_argument("Invalid field for LidarFrame");
    }
}

// @copydoc visit_field_2d()
template <typename OP, typename... Args>
void visit_field_2d(const FieldView& field, OP&& op, Args&&... args) {
    switch (field.tag()) {
        case ChanFieldType::UINT8:
            op.template operator()(Eigen::Ref<const img_t<uint8_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT16:
            op.template operator()(Eigen::Ref<const img_t<uint16_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT32:
            op.template operator()(Eigen::Ref<const img_t<uint32_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT64:
            op.template operator()(Eigen::Ref<const img_t<uint64_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT8:
            op.template operator()(Eigen::Ref<const img_t<int8_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT16:
            op.template operator()(Eigen::Ref<const img_t<int16_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT32:
            op.template operator()(Eigen::Ref<const img_t<int32_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT64:
            op.template operator()(Eigen::Ref<const img_t<int64_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::FLOAT16:
            // op.template operator()(Eigen::Ref<const img_t<float16_t>>(field),
            //                        std::forward<Args>(args)...);
            break;
        case ChanFieldType::FLOAT32:
            op.template operator()(Eigen::Ref<const img_t<float>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::FLOAT64:
            op.template operator()(Eigen::Ref<const img_t<double>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::ZONE_STATE:
        case ChanFieldType::CHAR:
        case ChanFieldType::VOID:
        case ChanFieldType::UNREGISTERED:
            // These types are not Eigen scalar types or are not supported,
            // so we silently skip them rather than crash.
            break;
        default:
            throw std::invalid_argument("Invalid field for LidarFrame");
    }
}

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the Field `field`
 * Example code for the operation<T>:
 * \code
 * struct print_field_size {
 *   template <typename T, size_t N>
 *   void operator()(ArrayView<T, N> field) {
 *       // Do thing on field here.
 *   }
 * };
 * \endcode
 */
template <typename OP, typename... Args>
void visit_field_ndim(FieldView& field, OP&& op, Args&&... args) {
    if (field.shape().size() == 3) {
        switch (field.tag()) {
            case ChanFieldType::FLOAT16:
                op.template operator()(ArrayView<float16_t, 3>(field), std::forward<Args>(args)...);
                return;
            case ChanFieldType::FLOAT32:
                op.template operator()(ArrayView<float, 3>(field), std::forward<Args>(args)...);
                return;
            case ChanFieldType::FLOAT64:
                op.template operator()(ArrayView<double, 3>(field), std::forward<Args>(args)...);
                return;
            default:
                throw std::invalid_argument("Invalid field for LidarFrame");
        }
        throw std::invalid_argument("Unhandled field shape.");
    } else if (field.shape().size() == 2) {
        switch (field.tag()) {
            case ChanFieldType::UINT8:
                op.template operator()(ArrayView<uint8_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::UINT16:
                op.template operator()(ArrayView<uint16_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::UINT32:
                op.template operator()(ArrayView<uint32_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::UINT64:
                op.template operator()(ArrayView<uint64_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::INT8:
                op.template operator()(ArrayView<int8_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::INT16:
                op.template operator()(ArrayView<int16_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::INT32:
                op.template operator()(ArrayView<int32_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::INT64:
                op.template operator()(ArrayView<int64_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::FLOAT16:
                op.template operator()(ArrayView<float16_t, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::FLOAT32:
                op.template operator()(ArrayView<float, 2>(field), std::forward<Args>(args)...);
                break;
            case ChanFieldType::FLOAT64:
                op.template operator()(ArrayView<double, 2>(field), std::forward<Args>(args)...);
                break;
            default:
                throw std::invalid_argument("Invalid field for LidarFrame");
        }
    } else {
        throw std::invalid_argument("Invalid field shape.");
    }
}

// @copydoc visit_field_ndim()
template <typename OP, typename... Args>
void visit_field_ndim(const FieldView& field, OP&& op, Args&&... args) {
    if (field.shape().size() == 3) {
        switch (field.tag()) {
            case ChanFieldType::FLOAT16:
                op.template operator()(ConstArrayView<float16_t, 3>(field),
                                       std::forward<Args>(args)...);
                return;
            case ChanFieldType::FLOAT32:
                op.template operator()(ConstArrayView<float, 3>(field),
                                       std::forward<Args>(args)...);
                return;
            case ChanFieldType::FLOAT64:
                op.template operator()(ConstArrayView<double, 3>(field),
                                       std::forward<Args>(args)...);
                return;
            default:
                throw std::invalid_argument("Invalid field for LidarFrame");
        }
        throw std::invalid_argument("Unhandled field shape.");
    } else if (field.shape().size() == 2) {
        switch (field.tag()) {
            case ChanFieldType::UINT8:
                op.template operator()(ConstArrayView<uint8_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::UINT16:
                op.template operator()(ConstArrayView<uint16_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::UINT32:
                op.template operator()(ConstArrayView<uint32_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::UINT64:
                op.template operator()(ConstArrayView<uint64_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::INT8:
                op.template operator()(ConstArrayView<int8_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::INT16:
                op.template operator()(ConstArrayView<int16_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::INT32:
                op.template operator()(ConstArrayView<int32_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::INT64:
                op.template operator()(ConstArrayView<int64_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::FLOAT16:
                op.template operator()(ConstArrayView<float16_t, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::FLOAT32:
                op.template operator()(ConstArrayView<float, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            case ChanFieldType::FLOAT64:
                op.template operator()(ConstArrayView<double, 2>(field),
                                       std::forward<Args>(args)...);
                break;
            default:
                throw std::invalid_argument("Invalid field for LidarFrame");
        }
    } else {
        throw std::invalid_argument("Invalid field shape.");
    }
}

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the LidarFrame channel field f
 * NOTE: requested field must be two dimensional
 * Example code for the operation<T>:
 * \code
 * struct print_field_size {
 *   template <typename T>
 *   void operator()(Eigen::Ref<img_t<T>> field) {
 *       std::cout << "Rows: " + field.rows() << std::endl;
 *       std::cout << "Cols: " + field.cols() << std::endl;
 *   }
 * };
 * \endcode
 */
template <typename FRAME, typename OP, typename... Args>
void visit_field(FRAME&& frame, const std::string& name, OP&& op, Args&&... args) {
    // throw early as python downstream expects ValueError
    if (!frame.has_field(name)) {
        throw std::invalid_argument("Invalid field for LidarFrame");
    }

    visit_field_2d(frame.field(name), std::forward<OP>(op), std::forward<Args>(args)...);
}

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the LidarFrame channel field f
 * Example code for the operation<T>:
 * \code
 * struct print_field_size {
 *   template <typename T, size_t N>
 *   void operator()(ArrayView<T, N> field) {
 *       // Do thing on field here.
 *   }
 * };
 * \endcode
 */
template <typename FRAME, typename OP, typename... Args>
void visit_field_ndim(FRAME&& frame, const std::string& name, OP&& op, Args&&... args) {
    // throw early as python downstream expects ValueError
    if (!frame.has_field(name)) {
        throw std::invalid_argument("Invalid field for LidarFrame");
    }

    visit_field_ndim(frame.field(name), std::forward<OP>(op), std::forward<Args>(args)...);
}

/*
 * Call a generic operation op<T>(f, Args...) for each parsed channel field of
 * the lidar frame with type parameter T having the correct field type
 */
template <typename FRAME, typename OP, typename... Args>
void foreach_channel_field(FRAME&& frame, const PacketFormat& pf, OP&& op, Args&&... args) {
    for (const auto& ft : pf) {
        if (frame.has_field(ft.first)) {
            visit_field(frame, ft.first, std::forward<OP>(op), ft.first,
                        std::forward<Args>(args)...);
        }
    }
}

/*
 * Call a generic operation op<T>(f, Args...) for each parsed channel field of
 * the lidar frame with type parameter T having the correct field type
 */
template <typename FRAME, typename OP, typename... Args>
void foreach_channel_field_ndim(FRAME&& frame, const PacketFormat& pf, OP&& op, Args&&... args) {
    for (const auto& ft : pf) {
        if (frame.has_field(ft.first)) {
            visit_field_ndim(frame, ft.first, std::forward<OP>(op), ft.first,
                             std::forward<Args>(args)...);
        }
    }
}

// Read LidarFrame field and cast to the destination
struct OUSTER_API_CLASS read_and_cast {
    template <typename T, typename U>
    void operator()(Eigen::Ref<const img_t<T>> src, Eigen::Ref<img_t<U>> dest) {
        dest = src.template cast<U>();
    }
    template <typename T, typename U>
    void operator()(Eigen::Ref<img_t<T>> src, Eigen::Ref<img_t<U>> dest) {
        dest = src.template cast<U>();
    }
    template <typename T, typename U>
    void operator()(Eigen::Ref<img_t<T>> src, img_t<U>& dest) {
        dest = src.template cast<U>();
    }
    template <typename T, typename U>
    void operator()(Eigen::Ref<const img_t<T>> src, img_t<U>& dest) {
        dest = src.template cast<U>();
    }
};

// Copy fields from `ls_source` (any frame kind) to `field_dest` img with
// casting to the img_t<T> type of `field_dest`.
struct OUSTER_API_CLASS copy_and_cast {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field_dest, const Frame& ls_source,
                    const std::string& ls_source_field) {
        visit_field(ls_source, ls_source_field, read_and_cast(), field_dest);
    }
};

/**
 * Zeros fields in LidarFrames
 */
struct OUSTER_API_CLASS zero_field {
    /**
     * Zeros the field dest.
     *
     * @tparam T The type of data inside of the eigen array.
     * @param[in,out] field_dest The field to zero.
     */
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field_dest) {
        field_dest.setZero();
    }
};

/**
 * Checks whether RAW_HEADERS field is present and can be used to store headers.
 *
 * @param[in] pf packet format
 * @param[in] ls lidar frame to check for RAW_HEADERS field presence.
 */
OUSTER_API_FUNCTION
bool raw_headers_enabled(const PacketFormat& pf, const LidarFrame& ls);

}  // namespace impl

/**
 * Destagger an image by applying pixel shifts to each row.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * LidarInfo.
 * @param[in] pixel_shift_by_row The pixel shifts to apply to each row. Must
 * have size matching the image height.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @param[out] destaggered The destaggered image. Must have dimensions matching
 * the pixel shifts and image height.
 * @return A new destaggered image.
 */
template <typename T>
inline void destagger_into(const Eigen::Ref<const img_t<T>>& img,
                           const std::vector<int>& pixel_shift_by_row, bool inverse,
                           Eigen::Ref<img_t<T>> destaggered) {
    const size_t h = img.rows();
    const size_t w = img.cols();

    if (pixel_shift_by_row.size() != h) {
        throw std::invalid_argument{"image height does not match shifts size"};
    }

    if (h != static_cast<size_t>(destaggered.rows()) ||
        w != static_cast<size_t>(destaggered.cols())) {
        throw std::invalid_argument{"image and destaggered must have the same shape"};
    }
    int sign = inverse ? -1 : +1;

    const auto* const g = img.data();
    const auto d = destaggered.data();

    for (size_t u = 0; u < h; ++u) {
        const auto g_row = g + (u * w);
        const auto d_row = d + (u * w);
        const int offset = (w + sign * pixel_shift_by_row[u] % w) % w;
        memcpy(d_row, g_row + (w - offset), offset * sizeof(T));
        memcpy(d_row + offset, g_row, (w - offset) * sizeof(T));
    }
}

/**
 * Destagger an image by applying pixel shifts to each row.
 *
 * @tparam T The type of data inside of the eigen array.
 * @tparam ndim The number of dimensions in the eigen tensor.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * LidarInfo.
 * @param[in] pixel_shift_by_row The pixel shifts to apply to each row. Must
 * have size matching the image height.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @param[out] destaggered The destaggered image. Must have dimensions matching
 * the pixel shifts and image height.
 */
template <typename T, int ndim>
inline void destagger_into(
    const Eigen::TensorRef<const Eigen::Tensor<T, ndim, Eigen::RowMajor>>& img,
    const std::vector<int>& pixel_shift_by_row, bool inverse,
    Eigen::TensorRef<Eigen::Tensor<T, ndim, Eigen::RowMajor>> destaggered) {
    const size_t h = img.dimension(0);
    const size_t w = img.dimension(1);

    if (pixel_shift_by_row.size() != h) {
        throw std::invalid_argument{"image height does not match shifts size"};
    }

    for (int dim_idx = 0; dim_idx < ndim; dim_idx++) {
        if (img.dimension(dim_idx) != destaggered.dimension(dim_idx)) {
            throw std::invalid_argument{"image and destaggered must have the same shape"};
        }
    }

    int sign = inverse ? -1 : +1;

    const auto* const g = (const T*)img.data();
    const auto d = (T*)destaggered.data();

    auto n_elements = 1;
    for (int i = 2; i < ndim; i++) {
        n_elements *= img.dimension(i);
    }

    for (size_t u = 0; u < h; ++u) {
        const auto g_row = g + (u * w) * n_elements;
        const auto d_row = d + (u * w) * n_elements;
        const int offset = ((w + sign * pixel_shift_by_row[u] % w) % w) * n_elements;
        memcpy(d_row, g_row + (w * n_elements - offset), offset * sizeof(T));
        memcpy(d_row + offset, g_row, (w * n_elements - offset) * sizeof(T));
    }
}

/**
 * Destagger an image by applying pixel shifts to each row.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * LidarInfo.
 * @param[in] pixel_shift_by_row The pixel shifts to apply to each row. Must
 * have size matching the image height.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @return A new destaggered image.
 */
template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& pixel_shift_by_row, bool inverse) {
    const size_t h = img.rows();
    const size_t w = img.cols();

    img_t<T> destaggered{h, w};
    destagger_into<T>(img, pixel_shift_by_row, inverse, destaggered);
    return destaggered;
}

/**
 * Destagger an image by applying pixel shifts to each row.
 *
 * @tparam T The type of data inside of the eigen tensor.
 * @tparam ndim The number of dimensions in the eigen tensor.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * LidarInfo.
 * @param[in] pixel_shift_by_row The pixel shifts to apply to each row. Must
 * have size matching the image height.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @return A new destaggered image.
 */
template <typename T, int ndim>
inline Eigen::Tensor<T, ndim, Eigen::RowMajor> destagger(
    const Eigen::TensorRef<const Eigen::Tensor<T, ndim, Eigen::RowMajor>>& img,
    const std::vector<int>& pixel_shift_by_row, bool inverse) {
    std::array<Eigen::Index, ndim> size_array;
    for (size_t i = 0; i < ndim; i++) {
        size_array[i] = img.dimension(i);
    }
    Eigen::Tensor<T, ndim, Eigen::RowMajor> destaggered(size_array);
    destagger_into<T, ndim>(img, pixel_shift_by_row, inverse, destaggered);
    return destaggered;
}

/**
 * Destagger an image by applying pixel shifts to each row according to the
 * LidarInfo.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] info LidarInfo containing the pixel shifts to apply.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * LidarInfo.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @param[out] destaggered The destaggered image. Must have dimensions matching
 * the LidarInfo.
 */
template <typename T>
inline void destagger_into(const LidarInfo& info, const Eigen::Ref<const img_t<T>>& img,
                           bool inverse, Eigen::Ref<img_t<T>> destaggered) {
    if (img.rows() != info.format.pixels_per_column ||
        img.cols() != info.format.columns_per_frame ||
        img.rows() != info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument{"Image resolution must match LidarInfo."};
    }
    return destagger_into<T>(img, info.format.pixel_shift_by_row, inverse, destaggered);
}

/**
 * Destagger an image by applying pixel shifts to each row according to the
 * LidarInfo.
 *
 * @tparam T The type of data inside of the eigen tensor.
 * @tparam ndim The number of dimensions in the eigen tensor.
 * @param[in] info LidarInfo containing the pixel shifts to apply.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * LidarInfo.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @param[out] destaggered The destaggered image. Must have dimensions matching
 * the LidarInfo.
 */
template <typename T, int ndim>
inline void destagger_into(
    const LidarInfo& info,
    const Eigen::TensorRef<const Eigen::Tensor<T, ndim, Eigen::RowMajor>>& img, bool inverse,
    Eigen::TensorRef<Eigen::Tensor<T, ndim, Eigen::RowMajor>> destaggered) {
    if (img.rows() != info.format.pixels_per_column ||
        img.cols() != info.format.columns_per_frame ||
        img.rows() != info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument{"Image resolution must match LidarInfo."};
    }
    return destagger_into<T, ndim>(img, info.format.pixel_shift_by_row, inverse, destaggered);
}

/**
 * Destagger an image by applying pixel shifts to each row according to the
 * LidarInfo.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] info LidarInfo containing the pixel shifts to apply.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * LidarInfo.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @return A new destaggered image.
 */
template <typename T>
inline img_t<T> destagger(const LidarInfo& info, const Eigen::Ref<const img_t<T>>& img,
                          bool inverse) {
    if (img.rows() != info.format.pixels_per_column ||
        img.cols() != info.format.columns_per_frame ||
        img.rows() != info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument{"Image resolution must match LidarInfo."};
    }
    return destagger(img, info.format.pixel_shift_by_row, inverse);
}

/**
 * Destagger an image by applying pixel shifts to each row according to the
 * LidarInfo.
 *
 * @tparam T The type of data inside of the eigen tensor.
 * @tparam ndim The number of dimensions in the eigen tensor.
 * @param[in] info LidarInfo containing the pixel shifts to apply.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * LidarInfo.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @return A new destaggered image.
 */
template <typename T, int ndim>
inline Eigen::Tensor<T, ndim, Eigen::RowMajor> destagger(
    const LidarInfo& info,
    const Eigen::TensorRef<const Eigen::Tensor<T, ndim, Eigen::RowMajor>>& img, bool inverse) {
    if (img.rows() != info.format.pixels_per_column ||
        img.cols() != info.format.columns_per_frame ||
        img.rows() != info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument{"Image resolution must match LidarInfo."};
    }
    return destagger(img, info.format.pixel_shift_by_row, inverse);
}

/**
 * Stagger an image by applying the inverse of the destagger pixel shifts.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] info LidarInfo containing the pixel shifts to apply.
 * @param[in] img The image to stagger. Must have dimensions matching the
 * LidarInfo.
 * @return A new staggered image.
 */
template <typename T>
inline img_t<T> stagger(const LidarInfo& info, const Eigen::Ref<const img_t<T>>& img) {
    return destagger(info, img, true);
}

/**
 * Stagger an image by applying the inverse of the destagger pixel shifts.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] info LidarInfo containing the pixel shifts to apply.
 * @param[in] img The image to stagger. Must have dimensions matching the
 * LidarInfo.
 * @return A new staggered image.
 */
template <typename T, int ndim>
inline Eigen::Tensor<T, ndim, Eigen::RowMajor> stagger(
    const LidarInfo& info,
    const Eigen::TensorRef<const Eigen::Tensor<T, ndim, Eigen::RowMajor>>& img) {
    return destagger(info, img, true);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
