/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

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

#include "ouster/deprecation.h"
#include "ouster/field.h"
#include "ouster/impl/packet_writer.h"
#include "ouster/visibility.h"
#include "ouster/zone_state.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

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
            op.template operator()(Eigen::Ref<img_t<uint8_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT16:
            op.template operator()(Eigen::Ref<img_t<uint16_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT32:
            op.template operator()(Eigen::Ref<img_t<uint32_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::UINT64:
            op.template operator()(Eigen::Ref<img_t<uint64_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT8:
            op.template operator()(Eigen::Ref<img_t<int8_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT16:
            op.template operator()(Eigen::Ref<img_t<int16_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT32:
            op.template operator()(Eigen::Ref<img_t<int32_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::INT64:
            op.template operator()(Eigen::Ref<img_t<int64_t>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::FLOAT32:
            op.template operator()(Eigen::Ref<img_t<float>>(field),
                                   std::forward<Args>(args)...);
            break;
        case ChanFieldType::FLOAT64:
            op.template operator()(Eigen::Ref<img_t<double>>(field),
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
            throw std::invalid_argument("Invalid field for LidarScan");
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
    if (!ls.has_field(name)) {
        throw std::invalid_argument("Invalid field for LidarScan");
    }

    visit_field_2d(ls.field(name), std::forward<OP>(op),
                   std::forward<Args>(args)...);
}

/*
 * Call a generic operation op<T>(f, Args...) for each parsed channel field of
 * the lidar scan with type parameter T having the correct field type
 */
template <typename SCAN, typename OP, typename... Args>
void foreach_channel_field(SCAN&& ls, const PacketFormat& pf, OP&& op,
                           Args&&... args) {
    for (const auto& ft : pf) {
        if (ls.has_field(ft.first)) {
            visit_field(ls, ft.first, std::forward<OP>(op), ft.first,
                        std::forward<Args>(args)...);
        }
    }
}

// Read LidarScan field and cast to the destination
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

// Copy fields from `ls_source` LidarScan to `field_dest` img with casting
// to the img_t<T> type of `field_dest`.
struct OUSTER_API_CLASS copy_and_cast {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field_dest, const LidarScan& ls_source,
                    const std::string& ls_source_field) {
        visit_field(ls_source, ls_source_field, read_and_cast(), field_dest);
    }
};

/**
 * Zeros fields in LidarScans
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
 * @param[in] ls lidar scan to check for RAW_HEADERS field presence.
 */
OUSTER_API_FUNCTION
bool raw_headers_enabled(const PacketFormat& pf, const LidarScan& ls);

/**
 * OutputItT - STL compatible output iterator over Packet value type
 */
template <typename OutputItT>
void scan_to_packets(const LidarScan& ls,
                     const ouster::sdk::core::impl::PacketWriter& pw,
                     OutputItT iter, uint32_t init_id, uint64_t prod_sn) {
    // this bit will not work with UDPProfileLidar::OFF
    // TODO: fix if that is back on the menu -- Tim T.
    size_t total_lidar_packets = ls.packet_timestamp().size();

    if (ls.w / pw.columns_per_packet != total_lidar_packets) {
        std::string err =
            "Mismatch between expected number of packets and "
            "PacketWriter.columns_per_frame";
        throw std::invalid_argument(err);
    }

    auto frame_id = ls.frame_id;

    using namespace ouster::sdk::core::ChanField;

    auto set_header = [&pw, &ls, frame_id, init_id, prod_sn](uint8_t* buffer) {
        // Set shot-limiting and shutdown fields, which should be the same for
        // all packets in the scan
        pw.set_shutdown(buffer, static_cast<uint8_t>(ls.thermal_shutdown()));
        pw.set_shot_limiting(buffer, static_cast<uint8_t>(ls.shot_limiting()));
        pw.set_shutdown_countdown(buffer, ls.shutdown_countdown);
        pw.set_shot_limiting_countdown(buffer, ls.shot_limiting_countdown);

        // Set other scan-level attributes
        pw.set_frame_id(buffer, frame_id);
        pw.set_init_id(buffer, init_id);
        pw.set_prod_sn(buffer, prod_sn);
    };

    auto packet_format = std::make_shared<PacketFormat>(pw);

    auto emit_lidar_packet = [&pw, &ls, &set_header, &iter,
                              &packet_format](size_t packet_id) {
        LidarPacket lidar_packet(pw.lidar_packet_size);
        lidar_packet.format = packet_format;

        uint8_t* lidar_buf = lidar_packet.buf.data();
        lidar_packet.host_timestamp = ls.packet_timestamp()[packet_id];

        set_header(lidar_buf);
        pw.set_packet_type(lidar_buf, 0x1);

        // Set alert flags, which may vary from packet to packet
        pw.set_alert_flags(lidar_buf, ls.alert_flags()[packet_id]);

        bool any_valid = false;
        auto columns_per_packet = pw.columns_per_packet;
        for (int icol = 0; icol < columns_per_packet; ++icol) {
            uint8_t* col_buf = pw.nth_col(icol, lidar_buf);

            auto id = (packet_id * columns_per_packet) + icol;

            pw.set_col_status(col_buf, ls.status()[id]);
            pw.set_col_measurement_id(col_buf, id);
            pw.set_col_timestamp(col_buf, ls.timestamp()[id]);

            any_valid |= (ls.status()[id] & 0x01);
        }

        // do not emit packet if ts == 0 and none of the columns are valid
        if (!any_valid && !lidar_packet.host_timestamp) {
            return;
        }

        auto pack_field = [&pw](auto ref_field, const std::string& i,
                                LidarPacket& packet) {
            pw.set_block(ref_field, i, packet.buf.data());
        };
        foreach_channel_field(ls, pw, pack_field, lidar_packet);

        if (raw_headers_enabled(pw, ls)) {
            auto unpack_raw_headers = [&pw](auto ref_field,
                                            LidarPacket& packet) {
                pw.unpack_raw_headers(ref_field, packet.buf.data());
            };
            visit_field(ls, RAW_HEADERS, unpack_raw_headers, lidar_packet);
        } else if (pw.udp_profile_lidar != UDPProfileLidar::LEGACY &&
                   pw.header_type == HeaderType::STANDARD) {
            assert(lidar_packet.buf.size() > sizeof(uint64_t));
            uint64_t crc = pw.calculate_crc(lidar_packet.buf.data(),
                                            lidar_packet.buf.size());
            memcpy(
                lidar_packet.buf.data() + lidar_packet.buf.size() - sizeof(crc),
                &crc, sizeof(crc));
        }

        *iter++ = lidar_packet;
    };

    auto emit_imu_packet = [&pw, &ls, &set_header, &iter,
                            &packet_format](size_t packet_id) {
        ImuPacket imu_packet(pw.imu_packet_size);
        imu_packet.format = packet_format;

        uint8_t* imu_buf = imu_packet.buf.data();

        set_header(imu_buf);
        pw.set_packet_type(imu_buf, 0x2);

        if (ls.has_field(IMU_ALERT_FLAGS)) {
            ConstArrayView1<uint8_t> alert_flags = ls.field(IMU_ALERT_FLAGS);
            pw.set_alert_flags(imu_buf, alert_flags(packet_id));
        }

        if (ls.has_field(IMU_PACKET_TIMESTAMP)) {
            ConstArrayView1<uint64_t> packet_timestamp =
                ls.field(IMU_PACKET_TIMESTAMP);
            imu_packet.host_timestamp = packet_timestamp(packet_id);
        }

        if (ls.has_field(POSITION_STRING)) {
            ConstArrayView2<char> nmea_sentences = ls.field(POSITION_STRING);
            pw.set_imu_nmea_sentence(imu_buf,
                                     nmea_sentences.subview(packet_id).data());
        }

        if (ls.has_field(POSITION_TIMESTAMP)) {
            ConstArrayView1<uint64_t> nmea_ts = ls.field(POSITION_TIMESTAMP);
            pw.set_imu_nmea_ts(imu_buf, nmea_ts(packet_id));
        }

        const FieldView empty{};
        const FieldView imu_ts_fview =
            ls.has_field(IMU_TIMESTAMP) ? ls.field(IMU_TIMESTAMP) : empty;
        const FieldView imu_m_id_fview = ls.has_field(IMU_MEASUREMENT_ID)
                                             ? ls.field(IMU_MEASUREMENT_ID)
                                             : empty;
        const FieldView imu_status_fview =
            ls.has_field(IMU_STATUS) ? ls.field(IMU_STATUS) : empty;

        const FieldView imu_acc_fview =
            ls.has_field(IMU_ACC) ? ls.field(IMU_ACC) : empty;

        const FieldView imu_gyro_fview =
            ls.has_field(IMU_GYRO) ? ls.field(IMU_GYRO) : empty;

        size_t col_offset = packet_id * pw.imu_measurements_per_packet;
        for (size_t i = 0; i < pw.imu_measurements_per_packet; ++i) {
            uint8_t* col_buf = pw.imu_nth_measurement(i, imu_buf);

            if (imu_ts_fview) {
                ConstArrayView1<uint64_t> imu_timestamp = imu_ts_fview;
                pw.set_col_timestamp(col_buf, imu_timestamp(col_offset + i));
            }

            if (imu_m_id_fview) {
                ConstArrayView1<uint16_t> imu_m_id = imu_m_id_fview;
                pw.set_col_measurement_id(col_buf, imu_m_id(col_offset + i));
            }

            if (imu_status_fview) {
                ConstArrayView1<uint16_t> imu_status = imu_status_fview;
                pw.set_col_status(col_buf, imu_status(col_offset + i));
            }

            if (imu_acc_fview) {
                ConstArrayView2<float> acc = imu_acc_fview;
                pw.set_imu_la_x(col_buf, acc(col_offset + i, 0));
                pw.set_imu_la_y(col_buf, acc(col_offset + i, 1));
                pw.set_imu_la_z(col_buf, acc(col_offset + i, 2));
            }

            if (imu_gyro_fview) {
                ConstArrayView2<float> gyro = imu_gyro_fview;
                pw.set_imu_av_x(col_buf, gyro(col_offset + i, 0));
                pw.set_imu_av_y(col_buf, gyro(col_offset + i, 1));
                pw.set_imu_av_z(col_buf, gyro(col_offset + i, 2));
            }
        }

        if (pw.header_type == HeaderType::STANDARD) {
            uint64_t crc =
                pw.calculate_crc(imu_packet.buf.data(), imu_packet.buf.size());
            memcpy(imu_packet.buf.data() + imu_packet.buf.size() - sizeof(crc),
                   &crc, sizeof(crc));
        }

        *iter++ = imu_packet;
    };

    auto emit_zm_packet = [&pw, &ls, &set_header, &iter, &packet_format]() {
        ZonePacket zone_packet(pw.zone_packet_size);
        zone_packet.format = packet_format;
        uint8_t* zone_buf = zone_packet.buf.data();
        set_header(zone_buf);
        pw.set_packet_type(zone_buf, 0x3);

        if (ls.has_field(ZONE_ALERT_FLAGS)) {
            ConstArrayView1<uint8_t> alert_flags = ls.field(ZONE_ALERT_FLAGS);
            pw.set_alert_flags(zone_buf, alert_flags(0));
        }

        if (ls.has_field(ZONE_TIMESTAMP)) {
            ConstArrayView1<uint64_t> zone_ts = ls.field(ZONE_TIMESTAMP);
            pw.set_zone_timestamp(zone_buf, zone_ts(0));
        }

        if (ls.has_field(ZONE_PACKET_TIMESTAMP)) {
            ConstArrayView1<uint64_t> zone_packet_ts =
                ls.field(ZONE_PACKET_TIMESTAMP);
            zone_packet.host_timestamp = zone_packet_ts(0);
        }

        if (ls.has_field(LIVE_ZONESET_HASH)) {
            pw.set_live_zoneset_hash(zone_buf, ls.field(LIVE_ZONESET_HASH));
        }

        if (ls.has_field(ZONE_STATES)) {
            ConstArrayView1<ZoneState> zones = ls.field(ZONE_STATES);
            for (size_t i = 0; i < zones.shape[0]; ++i) {
                uint8_t* buf = pw.zone_nth_measurement(i, zone_buf);
                pw.set_zone_state(buf, zones(i));
            }
        }

        if (pw.header_type == HeaderType::STANDARD) {
            uint64_t crc = pw.calculate_crc(zone_packet.buf.data(),
                                            zone_packet.buf.size());
            memcpy(
                zone_packet.buf.data() + zone_packet.buf.size() - sizeof(crc),
                &crc, sizeof(crc));
        }

        *iter++ = zone_packet;
    };

    std::vector<std::tuple<uint64_t, size_t, PacketType>> packet_order;
    packet_order.reserve(total_lidar_packets);
    for (size_t p_id = 0; p_id < total_lidar_packets; ++p_id) {
        packet_order.emplace_back(ls.packet_timestamp()[p_id], p_id,
                                  PacketType::Lidar);
    }

    if (pw.udp_profile_imu == UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
        const FieldView empty{};
        const FieldView ts_fv = ls.has_field(IMU_PACKET_TIMESTAMP)
                                    ? ls.field(IMU_PACKET_TIMESTAMP)
                                    : empty;

        for (size_t p_id = 0; p_id < pw.imu_packets_per_frame; ++p_id) {
            uint64_t imu_packet_ts;
            if (ts_fv) {
                ConstArrayView1<uint64_t> ts_v = ts_fv;
                imu_packet_ts = ts_v(p_id);
            } else {
                imu_packet_ts = 0;
            }

            if (imu_packet_ts > 0) {
                packet_order.emplace_back(imu_packet_ts, p_id, PacketType::Imu);
            }
        }
    }

    if (pw.zone_monitoring_enabled) {
        uint64_t zone_packet_ts;
        if (ls.has_field(ZONE_PACKET_TIMESTAMP)) {
            ConstArrayView1<uint64_t> ts_v = ls.field(ZONE_PACKET_TIMESTAMP);
            zone_packet_ts = ts_v(0);
        } else {
            zone_packet_ts = 0;
        }

        if (zone_packet_ts > 0) {
            packet_order.emplace_back(zone_packet_ts, 0, PacketType::Zone);
        }
    }

    std::sort(packet_order.begin(), packet_order.end());

    for (auto&& t : packet_order) {
        switch (std::get<2>(t)) {
            case PacketType::Lidar:
                emit_lidar_packet(std::get<1>(t));
                break;
            case PacketType::Imu:
                emit_imu_packet(std::get<1>(t));
                break;
            case PacketType::Zone:
                emit_zm_packet();
                break;
            default:
                break;
        }
    }
}

}  // namespace impl

// destagger into an existing array
template <typename T>
inline void destagger_into(const Eigen::Ref<const img_t<T>>& img,
                           const std::vector<int>& pixel_shift_by_row,
                           bool inverse, Eigen::Ref<img_t<T>> destaggered) {
    const size_t h = img.rows();
    const size_t w = img.cols();

    if (pixel_shift_by_row.size() != h) {
        throw std::invalid_argument{"image height does not match shifts size"};
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

// destagger into a new array
template <typename T>
inline img_t<T> destagger(const Eigen::Ref<const img_t<T>>& img,
                          const std::vector<int>& pixel_shift_by_row,
                          bool inverse) {
    const size_t h = img.rows();
    const size_t w = img.cols();

    img_t<T> destaggered{h, w};
    destagger_into<T>(img, pixel_shift_by_row, inverse, destaggered);
    return destaggered;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
