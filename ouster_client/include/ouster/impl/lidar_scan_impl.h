/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <stdexcept>

#include "ouster/field.h"
#include "ouster/impl/packet_writer.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

namespace ouster {
namespace impl {

using sensor::ChanFieldType;

template <typename T>
struct FieldTag;

template <>
struct FieldTag<uint8_t> {
    static constexpr ChanFieldType tag = ChanFieldType::UINT8;
};

template <>
struct FieldTag<uint16_t> {
    static constexpr ChanFieldType tag = ChanFieldType::UINT16;
};

template <>
struct FieldTag<uint32_t> {
    static constexpr ChanFieldType tag = ChanFieldType::UINT32;
};

template <>
struct FieldTag<uint64_t> {
    static constexpr ChanFieldType tag = ChanFieldType::UINT64;
};

template <>
struct FieldTag<int8_t> {
    static constexpr ChanFieldType tag = ChanFieldType::INT8;
};

template <>
struct FieldTag<int16_t> {
    static constexpr ChanFieldType tag = ChanFieldType::INT16;
};

template <>
struct FieldTag<int32_t> {
    static constexpr ChanFieldType tag = ChanFieldType::INT32;
};

template <>
struct FieldTag<int64_t> {
    static constexpr ChanFieldType tag = ChanFieldType::INT64;
};

template <>
struct FieldTag<float> {
    static constexpr ChanFieldType tag = ChanFieldType::FLOAT32;
};

template <>
struct FieldTag<double> {
    static constexpr ChanFieldType tag = ChanFieldType::FLOAT64;
};

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the Field `field`
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
template <typename OP, typename... Args>
void visit_field_2d(FieldView& field, OP&& op, Args&&... args) {
    switch (field.tag()) {
        case sensor::ChanFieldType::UINT8:
            op.template operator()(Eigen::Ref<img_t<uint8_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT16:
            op.template operator()(Eigen::Ref<img_t<uint16_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT32:
            op.template operator()(Eigen::Ref<img_t<uint32_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT64:
            op.template operator()(Eigen::Ref<img_t<uint64_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::INT8:
            op.template operator()(Eigen::Ref<img_t<int8_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::INT16:
            op.template operator()(Eigen::Ref<img_t<int16_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::INT32:
            op.template operator()(Eigen::Ref<img_t<int32_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::INT64:
            op.template operator()(Eigen::Ref<img_t<int64_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::FLOAT32:
            op.template operator()(Eigen::Ref<img_t<float>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::FLOAT64:
            op.template operator()(Eigen::Ref<img_t<double>>(field),
                                   std::forward<Args>(args)...);
            break;
        default:
            throw std::invalid_argument("Invalid field for LidarScan");
    }
}

// @copydoc visit_field_2d()
template <typename OP, typename... Args>
void visit_field_2d(const FieldView& field, OP&& op, Args&&... args) {
    switch (field.tag()) {
        case sensor::ChanFieldType::UINT8:
            op.template operator()(Eigen::Ref<const img_t<uint8_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT16:
            op.template operator()(Eigen::Ref<const img_t<uint16_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT32:
            op.template operator()(Eigen::Ref<const img_t<uint32_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT64:
            op.template operator()(Eigen::Ref<const img_t<uint64_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::INT8:
            op.template operator()(Eigen::Ref<const img_t<int8_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::INT16:
            op.template operator()(Eigen::Ref<const img_t<int16_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::INT32:
            op.template operator()(Eigen::Ref<const img_t<int32_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::INT64:
            op.template operator()(Eigen::Ref<const img_t<int64_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::FLOAT32:
            op.template operator()(Eigen::Ref<const img_t<float>>(field),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::FLOAT64:
            op.template operator()(Eigen::Ref<const img_t<double>>(field),
                                   std::forward<Args>(args)...);
            break;
        default:
            throw std::invalid_argument("Invalid field for LidarScan");
    }
}

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the LidarScan channel field f
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
template <typename SCAN, typename OP, typename... Args>
void visit_field(SCAN&& ls, const std::string& name, OP&& op, Args&&... args) {
    // throw early as python downstream expects ValueError
    if (!ls.has_field(name))
        throw std::invalid_argument("Invalid field for LidarScan");

    visit_field_2d(ls.field(name), std::forward<OP>(op),
                   std::forward<Args>(args)...);
}

/*
 * Call a generic operation op<T>(f, Args...) for each field of the lidar scan
 * with type parameter T having the correct field type
 */
template <typename SCAN, typename OP, typename... Args>
[[deprecated("Use either ls.fields() or foreach_channel_field instead")]] void
foreach_field(SCAN&& ls, OP&& op, Args&&... args) {
    for (const auto& ft : ls)
        visit_field(std::forward<SCAN>(ls), ft.first, std::forward<OP>(op),
                    ft.first, std::forward<Args>(args)...);
}

/*
 * Call a generic operation op<T>(f, Args...) for each parsed channel field of
 * the lidar scan with type parameter T having the correct field type
 */
template <typename SCAN, typename OP, typename... Args>
void foreach_channel_field(SCAN&& ls, const sensor::packet_format& pf, OP&& op,
                           Args&&... args) {
    for (const auto& ft : pf) {
        if (ls.has_field(ft.first)) {
            visit_field(ls, ft.first, std::forward<OP>(op), ft.first,
                        std::forward<Args>(args)...);
        }
    }
}

// Read LidarScan field and cast to the destination
struct read_and_cast {
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

// Copy fields from `ls_source` LidarScan to `field_dest` img with casting
// to the img_t<T> type of `field_dest`.
struct copy_and_cast {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field_dest, const LidarScan& ls_source,
                    const std::string& ls_source_field) {
        visit_field(ls_source, ls_source_field, read_and_cast(), field_dest);
    }
};

/**
 * Zeros fields in LidarScans
 */
struct zero_field {
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
 * @param[in] ls lidar scan to check for RAW_HEADERS field presence.
 */
bool raw_headers_enabled(const sensor::packet_format& pf, const LidarScan& ls);

/**
 * OutputItT - STL compatible output iterator over LidarPacket value type
 */
template <typename OutputItT>
void scan_to_packets(const LidarScan& ls,
                     const ouster::sensor::impl::packet_writer& pw,
                     OutputItT iter, uint32_t init_id, uint64_t prod_sn) {
    size_t total_packets = ls.packet_timestamp().size();
    auto columns_per_packet = pw.columns_per_packet;

    if (ls.w / columns_per_packet != total_packets) {
        std::string err =
            "Mismatch between expected number of packets and "
            "packet_writer.columns_per_frame";
        throw std::invalid_argument(err);
    }

    using ouster::sensor::LidarPacket;

    // TODO: switch to strings
    auto pack_field = [&pw](auto ref_field, const std::string& i,
                            LidarPacket& packet) {
        pw.set_block(ref_field, i, packet.buf.data());
    };

    auto unpack_raw_headers = [&pw](auto ref_field, LidarPacket& packet) {
        pw.unpack_raw_headers(ref_field, packet.buf.data());
    };

    auto frame_id = ls.frame_id;
    LidarPacket packet(pw.lidar_packet_size);

    for (size_t p_id = 0; p_id < total_packets; ++p_id) {
        uint8_t* lidar_buf = packet.buf.data();
        std::memset(packet.buf.data(), 0, packet.buf.size());
        packet.host_timestamp = ls.packet_timestamp()[p_id];

        // Set alert flags, which may vary from packet to packet
        pw.set_alert_flags(lidar_buf, ls.alert_flags()[p_id]);

        // Set shot-limiting and shutdown fields, which should be the same for
        // all packets in the scan
        pw.set_shutdown(lidar_buf, ls.thermal_shutdown());
        pw.set_shot_limiting(lidar_buf, ls.shot_limiting());
        pw.set_shutdown_countdown(lidar_buf, ls.shutdown_countdown);
        pw.set_shot_limiting_countdown(lidar_buf, ls.shot_limiting_countdown);

        // Set other scan-level attributes

        // TODO(dguridi): add an official PacketType enum in types.h
        pw.set_packet_type(lidar_buf, 0x1);
        pw.set_frame_id(lidar_buf, frame_id);
        pw.set_init_id(lidar_buf, init_id);
        pw.set_prod_sn(lidar_buf, prod_sn);

        bool any_valid = false;
        for (int icol = 0; icol < columns_per_packet; ++icol) {
            uint8_t* col_buf = pw.nth_col(icol, lidar_buf);

            auto id = p_id * columns_per_packet + icol;

            pw.set_col_status(col_buf, ls.status()[id]);
            pw.set_col_measurement_id(col_buf, id);
            pw.set_col_timestamp(col_buf, ls.timestamp()[id]);

            any_valid |= (ls.status()[id] & 0x01);
        }

        // do not emit packet if ts == 0 and none of the columns are valid
        if (!any_valid && !packet.host_timestamp) continue;

        foreach_channel_field(ls, pw, pack_field, packet);

        if (raw_headers_enabled(pw, ls)) {
            visit_field(ls, sensor::ChanField::RAW_HEADERS, unpack_raw_headers,
                        packet);
        } else if (pw.udp_profile_lidar !=
                   ouster::sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
            assert(packet.buf.size() > sizeof(uint64_t));
            uint64_t crc = pw.calculate_crc(packet.buf.data());
            memcpy(packet.buf.data() + packet.buf.size() - sizeof(crc), &crc,
                   sizeof(crc));
        }

        *iter++ = packet;
    }
}

}  // namespace impl

template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& pixel_shift_by_row,
                          bool inverse) {
    const size_t h = img.rows();
    const size_t w = img.cols();

    if (pixel_shift_by_row.size() != h)
        throw std::invalid_argument{"image height does not match shifts size"};

    img_t<T> destaggered{h, w};
    for (size_t u = 0; u < h; u++) {
        const std::ptrdiff_t offset =
            ((inverse ? -1 : 1) * pixel_shift_by_row[u] + w) % w;

        destaggered.row(u).segment(offset, w - offset) =
            img.row(u).segment(0, w - offset);
        destaggered.row(u).segment(0, offset) =
            img.row(u).segment(w - offset, offset);
    }
    return destaggered;
}

}  // namespace ouster
