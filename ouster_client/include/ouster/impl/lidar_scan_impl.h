/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <stdexcept>

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

/*
 * Tagged union for LidarScan fields
 */
struct FieldSlot {
    ChanFieldType tag;
    union {
        img_t<uint8_t> f8;
        img_t<uint16_t> f16;
        img_t<uint32_t> f32;
        img_t<uint64_t> f64;
    };

    FieldSlot(ChanFieldType t, size_t w, size_t h) : tag{t} {
        switch (t) {
            case ChanFieldType::VOID:
                break;
            case ChanFieldType::UINT8:
                new (&f8) img_t<uint8_t>{h, w};
                f8.setZero();
                break;
            case ChanFieldType::UINT16:
                new (&f16) img_t<uint16_t>{h, w};
                f16.setZero();
                break;
            case ChanFieldType::UINT32:
                new (&f32) img_t<uint32_t>{h, w};
                f32.setZero();
                break;
            case ChanFieldType::UINT64:
                new (&f64) img_t<uint64_t>{h, w};
                f64.setZero();
                break;
        }
    }

    FieldSlot() : FieldSlot{ChanFieldType::VOID, 0, 0} {};

    ~FieldSlot() { clear(); }

    FieldSlot(const FieldSlot& other) {
        switch (other.tag) {
            case ChanFieldType::VOID:
                break;
            case ChanFieldType::UINT8:
                new (&f8) img_t<uint8_t>{other.f8};
                break;
            case ChanFieldType::UINT16:
                new (&f16) img_t<uint16_t>{other.f16};
                break;
            case ChanFieldType::UINT32:
                new (&f32) img_t<uint32_t>{other.f32};
                break;
            case ChanFieldType::UINT64:
                new (&f64) img_t<uint64_t>{other.f64};
                break;
        }
        tag = other.tag;
    }

    FieldSlot(FieldSlot&& other) { set_from(other); }

    FieldSlot& operator=(FieldSlot other) {
        clear();
        set_from(other);
        return *this;
    }

    template <typename T>
    Eigen::Ref<img_t<T>> get() {
        if (tag == FieldTag<T>::tag)
            return get_unsafe<T>();
        else
            throw std::invalid_argument("Accessed field at wrong type");
    }

    template <typename T>
    Eigen::Ref<const img_t<T>> get() const {
        if (tag == FieldTag<T>::tag)
            return get_unsafe<T>();
        else
            throw std::invalid_argument("Accessed field at wrong type");
    }

    friend bool operator==(const FieldSlot& l, const FieldSlot& r) {
        if (l.tag != r.tag) return false;
        switch (l.tag) {
            case ChanFieldType::VOID:
                return true;
            case ChanFieldType::UINT8:
                return (l.f8 == r.f8).all();
            case ChanFieldType::UINT16:
                return (l.f16 == r.f16).all();
            case ChanFieldType::UINT32:
                return (l.f32 == r.f32).all();
            case ChanFieldType::UINT64:
                return (l.f64 == r.f64).all();
            default:
                assert(false);
        }
        // unreachable, appease older gcc
        return false;
    }

   private:
    void set_from(FieldSlot& other) {
        switch (other.tag) {
            case ChanFieldType::VOID:
                break;
            case ChanFieldType::UINT8:
                new (&f8) img_t<uint8_t>{std::move(other.f8)};
                break;
            case ChanFieldType::UINT16:
                new (&f16) img_t<uint16_t>{std::move(other.f16)};
                break;
            case ChanFieldType::UINT32:
                new (&f32) img_t<uint32_t>{std::move(other.f32)};
                break;
            case ChanFieldType::UINT64:
                new (&f64) img_t<uint64_t>{std::move(other.f64)};
                break;
        }
        tag = other.tag;
        other.clear();
    }

    void clear() {
        switch (tag) {
            case ChanFieldType::VOID:
                break;
            case ChanFieldType::UINT8:
                f8.~img_t<uint8_t>();
                break;
            case ChanFieldType::UINT16:
                f16.~img_t<uint16_t>();
                break;
            case ChanFieldType::UINT32:
                f32.~img_t<uint32_t>();
                break;
            case ChanFieldType::UINT64:
                f64.~img_t<uint64_t>();
                break;
        }
        tag = ChanFieldType::VOID;
    }

    template <typename T>
    Eigen::Ref<img_t<T>> get_unsafe();

    template <typename T>
    Eigen::Ref<const img_t<T>> get_unsafe() const;
};

template <>
inline Eigen::Ref<img_t<uint8_t>> FieldSlot::get_unsafe() {
    return f8;
}

template <>
inline Eigen::Ref<img_t<uint16_t>> FieldSlot::get_unsafe() {
    return f16;
}

template <>
inline Eigen::Ref<img_t<uint32_t>> FieldSlot::get_unsafe() {
    return f32;
}

template <>
inline Eigen::Ref<img_t<uint64_t>> FieldSlot::get_unsafe() {
    return f64;
}

template <>
inline Eigen::Ref<const img_t<uint8_t>> FieldSlot::get_unsafe() const {
    return f8;
}

template <>
inline Eigen::Ref<const img_t<uint16_t>> FieldSlot::get_unsafe() const {
    return f16;
}

template <>
inline Eigen::Ref<const img_t<uint32_t>> FieldSlot::get_unsafe() const {
    return f32;
}

template <>
inline Eigen::Ref<const img_t<uint64_t>> FieldSlot::get_unsafe() const {
    return f64;
}

/*
 * Call a generic operation op<T>(f, Args..) with the type parameter T having
 * the correct (dynamic) field type for the LidarScan channel field f
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
void visit_field(SCAN&& ls, sensor::ChanField f, OP&& op, Args&&... args) {
    switch (ls.field_type(f)) {
        case sensor::ChanFieldType::UINT8:
            op.template operator()(ls.template field<uint8_t>(f),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT16:
            op.template operator()(ls.template field<uint16_t>(f),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT32:
            op.template operator()(ls.template field<uint32_t>(f),
                                   std::forward<Args>(args)...);
            break;
        case sensor::ChanFieldType::UINT64:
            op.template operator()(ls.template field<uint64_t>(f),
                                   std::forward<Args>(args)...);
            break;
        default:
            throw std::invalid_argument("Invalid field for LidarScan");
    }
}

/*
 * Call a generic operation op<T>(f, Args...) for each field of the lidar scan
 * with type parameter T having the correct field type
 */
template <typename SCAN, typename OP, typename... Args>
void foreach_field(SCAN&& ls, OP&& op, Args&&... args) {
    for (const auto& ft : ls)
        visit_field(std::forward<SCAN>(ls), ft.first, std::forward<OP>(op),
                    ft.first, std::forward<Args>(args)...);
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
                    sensor::ChanField ls_source_field) {
        visit_field(ls_source, ls_source_field, read_and_cast(), field_dest);
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
                     OutputItT iter) {
    int total_packets = ls.packet_timestamp().size();
    auto columns_per_packet = pw.columns_per_packet;

    if (ls.w / columns_per_packet != total_packets) {
        std::string err =
            "Mismatch between expected number of packets and "
            "packet_writer.columns_per_frame";
        throw std::invalid_argument(err);
    }

    using ouster::sensor::ChanField;
    using ouster::sensor::LidarPacket;

    auto pack_field = [&pw](auto ref_field, ChanField i, LidarPacket& packet) {
        // skip over RAW_HEADERS, RAW32_WORD* and CUSTOM* fields
        if (i >= ChanField::RAW_HEADERS && i <= ChanField::CHAN_FIELD_MAX)
            return;

        pw.set_block(ref_field, i, packet.buf.data());
    };

    auto unpack_raw_headers = [&pw](auto ref_field, LidarPacket& packet) {
        pw.unpack_raw_headers(ref_field, packet.buf.data());
    };

    auto frame_id = ls.frame_id;
    LidarPacket packet(pw.lidar_packet_size);

    for (int p_id = 0; p_id < total_packets; ++p_id) {
        uint8_t* lidar_buf = packet.buf.data();
        std::memset(packet.buf.data(), 0, packet.buf.size());
        packet.host_timestamp = ls.packet_timestamp()[p_id];

        pw.set_frame_id(lidar_buf, frame_id);

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

        foreach_field(ls, pack_field, packet);

        if (raw_headers_enabled(pw, ls)) {
            visit_field(ls, ChanField::RAW_HEADERS, unpack_raw_headers, packet);
        } else if (pw.udp_profile_lidar !=
                   ouster::sensor::UDPProfileLidar::PROFILE_LIDAR_LEGACY) {
            uint32_t* ptr = reinterpret_cast<uint32_t*>(packet.buf.data() +
                                                        packet.buf.size() - 4);
            *ptr = 0xdeadbeef;  // eUDP packets end in 0xdeadbeef
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
