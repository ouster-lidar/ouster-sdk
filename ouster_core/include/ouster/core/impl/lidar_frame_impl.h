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

// Copy fields from `ls_source` LidarFrame to `field_dest` img with casting
// to the img_t<T> type of `field_dest`.
struct OUSTER_API_CLASS copy_and_cast {
    template <typename T>
    void operator()(Eigen::Ref<img_t<T>> field_dest, const LidarFrame& ls_source,
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

/**
 * OutputItT - STL compatible output iterator over Packet value type
 */
template <typename OutputItT>
void frame_to_packets(const LidarFrame& ls,
                      std::shared_ptr<ouster::sdk::core::PacketFormat> packet_format,
                      OutputItT iter, uint32_t init_id, uint64_t prod_sn) {
    if (!packet_format) {
        throw std::invalid_argument("Null PacketFormat pointer");
    }

    // this bit will not work with UDPProfileLidar::OFF
    // TODO: fix if that is back on the menu -- Tim T.
    size_t total_lidar_packets = ls.packet_timestamp().size();

    if (ls.w / packet_format->columns_per_packet != total_lidar_packets) {
        std::string err =
            "Mismatch between expected number of packets and "
            "PacketFormat.columns_per_packet";
        throw std::invalid_argument(err);
    }

    auto frame_id = ls.frame_id;

    using namespace ouster::sdk::core::ChanField;

    auto set_header = [&packet_format, &ls, frame_id, init_id, prod_sn](uint8_t* buffer) {
        // Set shot-limiting and shutdown fields, which should be the same for
        // all packets in the frame
        packet_format->set_shutdown(buffer, static_cast<uint8_t>(ls.thermal_shutdown()));
        packet_format->set_shot_limiting(buffer, static_cast<uint8_t>(ls.shot_limiting()));
        packet_format->set_shutdown_countdown(buffer, ls.shutdown_countdown);
        packet_format->set_shot_limiting_countdown(buffer, ls.shot_limiting_countdown);

        // Set other frame-level attributes
        packet_format->set_frame_id(buffer, frame_id);
        packet_format->set_init_id(buffer, init_id);
        packet_format->set_prod_sn(buffer, prod_sn);
    };

    auto emit_lidar_packet = [&packet_format, &ls, &set_header, &iter](size_t packet_id) {
        LidarPacket lidar_packet(packet_format);

        uint8_t* lidar_buf = lidar_packet.buf.data();
        lidar_packet.host_timestamp = ls.packet_timestamp()[packet_id];

        set_header(lidar_buf);
        packet_format->set_packet_type(lidar_buf, 0x1);

        // Set alert flags, which may vary from packet to packet
        packet_format->set_alert_flags(lidar_buf, ls.alert_flags()[packet_id]);

        bool any_valid = false;
        auto columns_per_packet = packet_format->columns_per_packet;
        for (uint32_t icol = 0; icol < columns_per_packet; ++icol) {
            uint8_t* col_buf = packet_format->nth_col(icol, lidar_buf);

            auto id = (packet_id * columns_per_packet) + icol;

            packet_format->set_col_status(col_buf, ls.status()[id]);
            packet_format->set_col_measurement_id(col_buf, id);
            packet_format->set_col_timestamp(col_buf, ls.timestamp()[id]);

            any_valid |= (ls.status()[id] & 0x01);
        }

        // do not emit packet if ts == 0 and none of the columns are valid
        if (!any_valid && !lidar_packet.host_timestamp) {
            return;
        }

        auto pack_field = [&packet_format](auto ref_field, const std::string& i,
                                           LidarPacket& packet) {
            if (sizeof(ref_field.shape) / sizeof(ref_field.shape[0]) != 2) {
                // printf("writing 3xfloat16\n");
                auto ptr = reinterpret_cast<const float3x16_t*>(ref_field.data());
                packet_format->set_block(ptr, ref_field.shape[1], i, packet.buf.data());
            } else {
                packet_format->set_block(ref_field.data(), ref_field.shape[1], i,
                                         packet.buf.data());
            }
        };
        foreach_channel_field_ndim(ls, *packet_format, pack_field, lidar_packet);

        if (raw_headers_enabled(*packet_format, ls)) {
            auto unpack_raw_headers = [&packet_format](auto ref_field, LidarPacket& packet) {
                packet_format->unpack_raw_headers(ref_field, packet.buf.data());
            };
            visit_field(ls, RAW_HEADERS, unpack_raw_headers, lidar_packet);
        } else if (packet_format->udp_profile_lidar != UDPProfileLidar::LEGACY &&
                   packet_format->header_type == HeaderType::STANDARD) {
            assert(lidar_packet.buf.size() > sizeof(uint64_t));
            uint64_t crc =
                packet_format->calculate_crc(lidar_packet.buf.data(), lidar_packet.buf.size());
            memcpy(lidar_packet.buf.data() + lidar_packet.buf.size() - sizeof(crc), &crc,
                   sizeof(crc));
        }

        *iter++ = lidar_packet;
    };

    auto emit_imu_packet = [&packet_format, &ls, &set_header, &iter](size_t packet_id) {
        ImuPacket imu_packet(packet_format);

        uint8_t* imu_buf = imu_packet.buf.data();

        set_header(imu_buf);
        packet_format->set_packet_type(imu_buf, 0x2);

        if (ls.has_field(IMU_ALERT_FLAGS)) {
            ConstArrayView1<uint8_t> alert_flags = ls.field(IMU_ALERT_FLAGS);
            packet_format->set_alert_flags(imu_buf, alert_flags(packet_id));
        }

        if (ls.has_field(IMU_PACKET_TIMESTAMP)) {
            ConstArrayView1<uint64_t> packet_timestamp = ls.field(IMU_PACKET_TIMESTAMP);
            imu_packet.host_timestamp = packet_timestamp(packet_id);
        }

        if (ls.has_field(POSITION_STRING)) {
            ConstArrayView2<char> nmea_sentences = ls.field(POSITION_STRING);
            packet_format->set_imu_nmea_sentence(imu_buf, nmea_sentences.subview(packet_id).data());
        }

        if (ls.has_field(POSITION_TIMESTAMP)) {
            ConstArrayView1<uint64_t> nmea_ts = ls.field(POSITION_TIMESTAMP);
            packet_format->set_imu_nmea_ts(imu_buf, nmea_ts(packet_id));
        }

        const FieldView empty{};
        const FieldView imu_ts_fview =
            ls.has_field(IMU_TIMESTAMP) ? ls.field(IMU_TIMESTAMP) : empty;
        const FieldView imu_m_id_fview =
            ls.has_field(IMU_MEASUREMENT_ID) ? ls.field(IMU_MEASUREMENT_ID) : empty;
        const FieldView imu_status_fview = ls.has_field(IMU_STATUS) ? ls.field(IMU_STATUS) : empty;

        const FieldView imu_acc_fview = ls.has_field(IMU_ACC) ? ls.field(IMU_ACC) : empty;

        const FieldView imu_gyro_fview = ls.has_field(IMU_GYRO) ? ls.field(IMU_GYRO) : empty;

        size_t col_offset = packet_id * packet_format->imu_measurements_per_packet;
        for (size_t i = 0; i < packet_format->imu_measurements_per_packet; ++i) {
            uint8_t* col_buf = packet_format->imu_nth_measurement(i, imu_buf);

            if (imu_ts_fview) {
                ConstArrayView1<uint64_t> imu_timestamp = imu_ts_fview;
                packet_format->set_col_timestamp(col_buf, imu_timestamp(col_offset + i));
            }

            if (imu_m_id_fview) {
                ConstArrayView1<uint16_t> imu_m_id = imu_m_id_fview;
                packet_format->set_col_measurement_id(col_buf, imu_m_id(col_offset + i));
            }

            if (imu_status_fview) {
                ConstArrayView1<uint16_t> imu_status = imu_status_fview;
                packet_format->set_col_status(col_buf, imu_status(col_offset + i));
            }

            if (imu_acc_fview) {
                ConstArrayView2<float> acc = imu_acc_fview;
                packet_format->set_imu_la_x(col_buf, acc(col_offset + i, 0));
                packet_format->set_imu_la_y(col_buf, acc(col_offset + i, 1));
                packet_format->set_imu_la_z(col_buf, acc(col_offset + i, 2));
            }

            if (imu_gyro_fview) {
                ConstArrayView2<float> gyro = imu_gyro_fview;
                packet_format->set_imu_av_x(col_buf, gyro(col_offset + i, 0));
                packet_format->set_imu_av_y(col_buf, gyro(col_offset + i, 1));
                packet_format->set_imu_av_z(col_buf, gyro(col_offset + i, 2));
            }
        }

        if (packet_format->header_type == HeaderType::STANDARD) {
            uint64_t crc =
                packet_format->calculate_crc(imu_packet.buf.data(), imu_packet.buf.size());
            memcpy(imu_packet.buf.data() + imu_packet.buf.size() - sizeof(crc), &crc, sizeof(crc));
        }

        *iter++ = imu_packet;
    };

    auto emit_zm_packet = [&packet_format, &ls, &set_header, &iter]() {
        ZonePacket zone_packet(packet_format);
        uint8_t* zone_buf = zone_packet.buf.data();
        set_header(zone_buf);
        packet_format->set_packet_type(zone_buf, 0x3);

        if (ls.has_field(ZONE_ALERT_FLAGS)) {
            ConstArrayView1<uint8_t> alert_flags = ls.field(ZONE_ALERT_FLAGS);
            packet_format->set_alert_flags(zone_buf, alert_flags(0));
        }

        if (ls.has_field(ZONE_TIMESTAMP)) {
            ConstArrayView1<uint64_t> zone_ts = ls.field(ZONE_TIMESTAMP);
            packet_format->set_zone_timestamp(zone_buf, zone_ts(0));
        }

        if (ls.has_field(ZONE_PACKET_TIMESTAMP)) {
            ConstArrayView1<uint64_t> zone_packet_ts = ls.field(ZONE_PACKET_TIMESTAMP);
            zone_packet.host_timestamp = zone_packet_ts(0);
        }

        if (ls.has_field(LIVE_ZONESET_HASH)) {
            packet_format->set_live_zoneset_hash(zone_buf, ls.field(LIVE_ZONESET_HASH));
        }

        if (ls.has_field(ZONE_STATES)) {
            ConstArrayView1<ZoneState> zones = ls.field(ZONE_STATES);
            for (size_t i = 0; i < zones.shape[0]; ++i) {
                uint8_t* buf = packet_format->zone_nth_measurement(i, zone_buf);
                packet_format->set_zone_state(buf, zones(i));
            }
        }

        if (packet_format->header_type == HeaderType::STANDARD) {
            uint64_t crc =
                packet_format->calculate_crc(zone_packet.buf.data(), zone_packet.buf.size());
            memcpy(zone_packet.buf.data() + zone_packet.buf.size() - sizeof(crc), &crc,
                   sizeof(crc));
        }

        *iter++ = zone_packet;
    };

    std::vector<std::tuple<uint64_t, size_t, PacketType>> packet_order;
    packet_order.reserve(total_lidar_packets);
    for (size_t p_id = 0; p_id < total_lidar_packets; ++p_id) {
        packet_order.emplace_back(ls.packet_timestamp()[p_id], p_id, PacketType::Lidar);
    }

    if (packet_format->udp_profile_imu == UDPProfileIMU::ACCEL32_GYRO32_NMEA) {
        const FieldView empty{};
        const FieldView ts_fv =
            ls.has_field(IMU_PACKET_TIMESTAMP) ? ls.field(IMU_PACKET_TIMESTAMP) : empty;

        for (size_t p_id = 0; p_id < packet_format->imu_packets_per_frame; ++p_id) {
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

    if (packet_format->zone_monitoring_enabled) {
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

/**
 * Destagger an image by applying pixel shifts to each row.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * SensorInfo.
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
 * SensorInfo.
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
 * SensorInfo.
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
 * SensorInfo.
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
 * SensorInfo.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] info SensorInfo containing the pixel shifts to apply.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * SensorInfo.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @param[out] destaggered The destaggered image. Must have dimensions matching
 * the SensorInfo.
 */
template <typename T>
inline void destagger_into(const SensorInfo& info, const Eigen::Ref<const img_t<T>>& img,
                           bool inverse, Eigen::Ref<img_t<T>> destaggered) {
    if (img.rows() != info.format.pixels_per_column ||
        img.cols() != info.format.columns_per_frame ||
        img.rows() != info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument{"Image resolution must match SensorInfo."};
    }
    return destagger_into<T>(img, info.format.pixel_shift_by_row, inverse, destaggered);
}

/**
 * Destagger an image by applying pixel shifts to each row according to the
 * SensorInfo.
 *
 * @tparam T The type of data inside of the eigen tensor.
 * @tparam ndim The number of dimensions in the eigen tensor.
 * @param[in] info SensorInfo containing the pixel shifts to apply.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * SensorInfo.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @param[out] destaggered The destaggered image. Must have dimensions matching
 * the SensorInfo.
 */
template <typename T, int ndim>
inline void destagger_into(
    const SensorInfo& info,
    const Eigen::TensorRef<const Eigen::Tensor<T, ndim, Eigen::RowMajor>>& img, bool inverse,
    Eigen::TensorRef<Eigen::Tensor<T, ndim, Eigen::RowMajor>> destaggered) {
    if (img.rows() != info.format.pixels_per_column ||
        img.cols() != info.format.columns_per_frame ||
        img.rows() != info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument{"Image resolution must match SensorInfo."};
    }
    return destagger_into<T, ndim>(img, info.format.pixel_shift_by_row, inverse, destaggered);
}

/**
 * Destagger an image by applying pixel shifts to each row according to the
 * SensorInfo.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] info SensorInfo containing the pixel shifts to apply.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * SensorInfo.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @return A new destaggered image.
 */
template <typename T>
inline img_t<T> destagger(const SensorInfo& info, const Eigen::Ref<const img_t<T>>& img,
                          bool inverse) {
    if (img.rows() != info.format.pixels_per_column ||
        img.cols() != info.format.columns_per_frame ||
        img.rows() != info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument{"Image resolution must match SensorInfo."};
    }
    return destagger(img, info.format.pixel_shift_by_row, inverse);
}

/**
 * Destagger an image by applying pixel shifts to each row according to the
 * SensorInfo.
 *
 * @tparam T The type of data inside of the eigen tensor.
 * @tparam ndim The number of dimensions in the eigen tensor.
 * @param[in] info SensorInfo containing the pixel shifts to apply.
 * @param[in] img The image to destagger. Must have dimensions matching the
 * SensorInfo.
 * @param[in] inverse If true, applies the inverse of the destagger pixel
 * shifts, effectively staggering the image instead of destaggering it.
 * @return A new destaggered image.
 */
template <typename T, int ndim>
inline Eigen::Tensor<T, ndim, Eigen::RowMajor> destagger(
    const SensorInfo& info,
    const Eigen::TensorRef<const Eigen::Tensor<T, ndim, Eigen::RowMajor>>& img, bool inverse) {
    if (img.rows() != info.format.pixels_per_column ||
        img.cols() != info.format.columns_per_frame ||
        img.rows() != info.format.pixel_shift_by_row.size()) {
        throw std::invalid_argument{"Image resolution must match SensorInfo."};
    }
    return destagger(img, info.format.pixel_shift_by_row, inverse);
}

/**
 * Stagger an image by applying the inverse of the destagger pixel shifts.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] info SensorInfo containing the pixel shifts to apply.
 * @param[in] img The image to stagger. Must have dimensions matching the
 * SensorInfo.
 * @return A new staggered image.
 */
template <typename T>
inline img_t<T> stagger(const SensorInfo& info, const Eigen::Ref<const img_t<T>>& img) {
    return destagger(info, img, true);
}

/**
 * Stagger an image by applying the inverse of the destagger pixel shifts.
 *
 * @tparam T The type of data inside of the eigen array.
 * @param[in] info SensorInfo containing the pixel shifts to apply.
 * @param[in] img The image to stagger. Must have dimensions matching the
 * SensorInfo.
 * @return A new staggered image.
 */
template <typename T, int ndim>
inline Eigen::Tensor<T, ndim, Eigen::RowMajor> stagger(
    const SensorInfo& info,
    const Eigen::TensorRef<const Eigen::Tensor<T, ndim, Eigen::RowMajor>>& img) {
    return destagger(info, img, true);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
