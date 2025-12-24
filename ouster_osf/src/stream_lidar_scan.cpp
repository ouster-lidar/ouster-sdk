/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/osf/stream_lidar_scan.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <future>
#include <iterator>
#include <memory>
#include <mutex>
#include <nonstd/optional.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fb_common.h"
#include "ouster/impl/logging.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/impl/basics.h"
#include "ouster/osf/impl/png_tools.h"
#include "ouster/types.h"

using namespace ouster::sdk::core;

namespace ouster {
namespace sdk {
namespace osf {

bool poses_present(const LidarScan& lidar_scan) {
    auto& pose = lidar_scan.pose();
    auto ident = mat4d::Identity().eval();
    auto ptr = pose.get<double>();
    for (size_t i = 0, end = pose.shape()[0]; i < end; ++i) {
        if (std::memcmp(ident.data(), ptr + (i * 16), sizeof(double) * 16) !=
            0) {
            return true;
        }
    }
    return false;
}

LidarScan slice_with_cast(const LidarScan& ls_src,
                          const LidarScanFieldTypes& field_types) {
    LidarScan ls_dest{static_cast<std::size_t>(ls_src.w),
                      static_cast<std::size_t>(ls_src.h), field_types.begin(),
                      field_types.end()};

    ls_dest.frame_status = ls_src.frame_status;
    ls_dest.shutdown_countdown = ls_src.shutdown_countdown;
    ls_dest.shot_limiting_countdown = ls_src.shot_limiting_countdown;
    ls_dest.frame_id = ls_src.frame_id;

    // Copy headers
    ls_dest.timestamp() = ls_src.timestamp();
    ls_dest.measurement_id() = ls_src.measurement_id();
    ls_dest.status() = ls_src.status();
    ls_dest.packet_timestamp() = ls_src.packet_timestamp();
    ls_dest.alert_flags() = ls_src.alert_flags();
    ls_dest.pose() = ls_src.pose();

    // Copy fields
    for (const auto& field_type : field_types) {
        if (ls_src.has_field(field_type.name)) {
            ouster::sdk::core::impl::visit_field(
                ls_dest, field_type.name,
                ouster::sdk::core::impl::copy_and_cast(), ls_src,
                field_type.name);
        } else {
            throw std::invalid_argument("Required field '" + field_type.name +
                                        "' is missing from scan.");
        }
    }

    return ls_dest;
}

flatbuffers::Offset<impl::gen::LidarScanMsg> create_lidar_scan_msg(
    flatbuffers::FlatBufferBuilder& fbb, const LidarScanEncoder& encoder,
    const LidarScan& lidar_scan, const SensorInfo& info,
    const LidarScanFieldTypes meta_field_types) {
    const auto& lidar_scan_ref = lidar_scan;

    // Prepare field_types for LidarScanMsg
    std::vector<std::pair<impl::gen::CHAN_FIELD, std::string>>
        standard_fields_to_sort;
    std::vector<std::pair<std::string, const Field*>> custom_field_names;
    for (const auto& field : lidar_scan_ref.fields()) {
        // if we have meta_field_types ignore fields not in it
        if (!meta_field_types.empty()) {
            bool found = false;
            for (const auto& mft : meta_field_types) {
                if (mft.name == field.first) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                continue;
            }
        }
        // if the field is custom, add it as custom
        auto enum_value = to_osf_enum(field.first);  // optional
        if (!enum_value) {
            // todo maybe do single fields in this thread
            custom_field_names.emplace_back(field.first, &field.second);
        } else {
            // otherwise add it to field types and then scan encode those
            standard_fields_to_sort.emplace_back(enum_value.value(),
                                                 field.first);
        }
    }

    // because older OSF readers are broken, we need to sort the standard fields
    // by value...
    std::sort(standard_fields_to_sort.begin(), standard_fields_to_sort.end());

    // now actually build our arrays from the sorted one
    std::vector<ouster::sdk::osf::impl::gen::ChannelField> field_types;
    LidarScanFieldTypes standard_fields;
    for (const auto& field_pair : standard_fields_to_sort) {
        const auto& field = lidar_scan_ref.field(field_pair.second);
        field_types.emplace_back(field_pair.first, to_osf_enum(field.tag()));
        standard_fields.emplace_back(field_pair.second, field.tag());
    }

    auto field_types_off =
        osf::create_vector_of_structs<impl::gen::ChannelField>(
            fbb, field_types.data(), field_types.size());
    auto channels_off =
        fb_save_scan_channels(fbb, encoder, lidar_scan_ref, standard_fields,
                              info.format.pixel_shift_by_row);
    auto custom_fields_off = fb_save_fields(fbb, encoder, custom_field_names);
    auto timestamp_off = fbb.CreateVector<uint64_t>(
        lidar_scan_ref.timestamp().data(), lidar_scan_ref.timestamp().size());
    auto measurement_id_off = fbb.CreateVector<uint16_t>(
        lidar_scan_ref.measurement_id().data(), lidar_scan_ref.w);
    auto status_off = fbb.CreateVector<uint32_t>(lidar_scan_ref.status().data(),
                                                 lidar_scan_ref.w);
    flatbuffers::Offset<flatbuffers::Vector<double>> pose_off = 0;
    if (poses_present(lidar_scan_ref)) {
        pose_off = fbb.CreateVector<double>(
            lidar_scan_ref.pose().get<double>(),
            lidar_scan_ref.pose().bytes() / sizeof(double));
    }
    auto packet_timestamp_id_off =
        fbb.CreateVector<uint64_t>(lidar_scan_ref.packet_timestamp().data(),
                                   lidar_scan_ref.packet_timestamp().size());

    auto alert_flags_off =
        fbb.CreateVector<uint8_t>(lidar_scan_ref.alert_flags().data(),
                                  lidar_scan_ref.alert_flags().size());

    return impl::gen::CreateLidarScanMsg(
        fbb, channels_off, field_types_off, timestamp_off, measurement_id_off,
        status_off, lidar_scan_ref.frame_id, pose_off, packet_timestamp_id_off,
        custom_fields_off, lidar_scan_ref.frame_status,
        lidar_scan_ref.shutdown_countdown,
        lidar_scan_ref.shot_limiting_countdown, alert_flags_off);
}

/**
 * Copy the contents of the LidarScanMsg into a LidarScan.
 *
 * NOTE: LidarScan isn't inherently flatbuffers-based, which unfortunately means
 * we're not taking advantage of one of fb's primary benefits - the ability to
 * read data from messages without copying or allocating new memory for it.
 * Moreover, the OSF representation of LidarScan is compressed and the
 * deserialized version is not.
 *
 * As such, making use of the FB directly would require revisiting the design
 * and a significant refactor.
 */
std::unique_ptr<LidarScan> restore_lidar_scan(
    const MessageRef& msg, const SensorInfo& info,
    const nonstd::optional<std::vector<std::string>>& fields_to_decode) {
    const auto& buf = msg.buffer();
    auto ls_msg = flatbuffers::GetSizePrefixedRoot<
        ouster::sdk::osf::impl::gen::LidarScanMsg>(buf.data());

    uint32_t width = info.format.columns_per_frame;
    uint32_t height = info.format.pixels_per_column;

    // read field_types
    LidarScanFieldTypes field_types;
    if ((ls_msg->field_types() != nullptr) &&
        (ls_msg->field_types()->size() != 0u)) {
        for (auto it = ls_msg->field_types()->begin();
             it != ls_msg->field_types()->end(); it++) {
            auto field_type = FieldType{from_osf_enum(it->chan_field()),
                                        from_osf_enum(it->chan_field_type()),
                                        {},
                                        FieldClass::PIXEL_FIELD};
            field_types.push_back(field_type);
        }
    }

    // Init lidar scan with recovered fields
    LidarScanFieldTypes desired_field_types =
        !fields_to_decode.has_value() ? field_types : LidarScanFieldTypes();
    if (fields_to_decode) {
        for (const auto& field_name : fields_to_decode.value()) {
            for (const auto& field_type : field_types) {
                if (field_type.name == field_name) {
                    desired_field_types.push_back(field_type);
                    break;
                }
            }
        }
    }
    auto lidar_scan = std::make_unique<LidarScan>(
        width, height, desired_field_types.begin(), desired_field_types.end(),
        info.format.columns_per_packet);

    // set frame status - unfortunately since this is a new field in the FB
    // schema and since LidarScan::frame_status is an integer we have no way to
    // differentiate between whether the value was zero when written or wasn't
    // provided by the writer.
    lidar_scan->frame_status = ls_msg->frame_status();
    lidar_scan->shutdown_countdown = ls_msg->shutdown_countdown();
    lidar_scan->shot_limiting_countdown = ls_msg->shot_limiting_countdown();

    lidar_scan->frame_id = ls_msg->frame_id();

    // Set timestamps per column
    auto msg_ts_vec = ls_msg->header_timestamp();
    if (msg_ts_vec != nullptr) {
        if (static_cast<uint32_t>(lidar_scan->w) == msg_ts_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                lidar_scan->timestamp()[i] = msg_ts_vec->Get(i);
            }
        } else if (msg_ts_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has header_timestamp of length: "
                "{}, expected: {}",
                msg_ts_vec->size(), lidar_scan->w);
            return nullptr;
        }
    }

    // Set measurement_id per column
    auto msg_mid_vec = ls_msg->header_measurement_id();
    if (msg_mid_vec != nullptr) {
        if (static_cast<uint32_t>(lidar_scan->w) == msg_mid_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                lidar_scan->measurement_id()[i] = msg_mid_vec->Get(i);
            }
        } else if (msg_mid_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has header_measurement_id of length: "
                "{}, expected: {}",
                msg_mid_vec->size(), lidar_scan->w);
            return nullptr;
        }
    }

    // Set status per column
    auto msg_status_vec = ls_msg->header_status();
    if (msg_status_vec != nullptr) {
        if (static_cast<uint32_t>(lidar_scan->w) == msg_status_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                lidar_scan->status()[i] = msg_status_vec->Get(i);
            }
        } else if (msg_status_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has header_status of length: "
                "{}, expected: {}",
                msg_status_vec->size(), lidar_scan->w);
            return nullptr;
        }
    }

    // Set poses per column
    auto pose_vec = ls_msg->pose();
    if (pose_vec != nullptr) {
        auto& pose = lidar_scan->pose();
        if (static_cast<uint32_t>(pose.size()) == pose_vec->size()) {
            std::memcpy(pose.get<double>(), pose_vec->Data(), pose.bytes());
        } else if (pose_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has pose of length: "
                "{}, expected: {}",
                pose_vec->size(), pose.size());
            return nullptr;
        }
    }

    // Set packet timestamp per lidar packet
    auto packet_ts_vec = ls_msg->packet_timestamp();
    if (packet_ts_vec != nullptr) {
        if (static_cast<uint32_t>(lidar_scan->packet_timestamp().size()) ==
            packet_ts_vec->size()) {
            for (uint32_t i = 0; i < packet_ts_vec->size(); ++i) {
                lidar_scan->packet_timestamp()[i] = packet_ts_vec->Get(i);
            }
        } else if (packet_ts_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has packet_timestamp of length: "
                "{}, expected: {}",
                packet_ts_vec->size(), lidar_scan->packet_timestamp().size());
            return nullptr;
        }
    }

    // Set alert flags per lidar packet
    auto alert_flags_vec = ls_msg->alert_flags();
    if (alert_flags_vec != nullptr) {
        if (static_cast<size_t>(lidar_scan->alert_flags().size()) ==
            alert_flags_vec->size()) {
            for (size_t i = 0; i < alert_flags_vec->size(); ++i) {
                lidar_scan->alert_flags()[i] = alert_flags_vec->Get(i);
            }
        } else if (alert_flags_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has alert_flags of length: "
                "{}, expected: {}",
                alert_flags_vec->size(), lidar_scan->alert_flags().size());
            return nullptr;
        }
    }

    // Fill Scan Data with scan channels
    auto msg_scan_channels = ls_msg->channels();
    if (msg_scan_channels == nullptr) {
        logger().error("ERROR: lidar_scan msg doesn't have scan fields.");
        return nullptr;
    }

    fb_restore_channels(msg_scan_channels, field_types,
                        info.format.pixel_shift_by_row, *lidar_scan,
                        msg.error_handler());

    auto add_field = [&lidar_scan](const std::string& name,
                                   const FieldDescriptor& desc,
                                   FieldClass field_class) -> Field& {
        return lidar_scan->add_field(name, desc, field_class);
    };

    fb_restore_fields(ls_msg->custom_fields(), fields_to_decode, add_field,
                      msg.error_handler());

    // error if any of the requested fields did not end up in the lidar scan
    if (fields_to_decode) {
        for (const auto& field : fields_to_decode.value()) {
            if (!lidar_scan->has_field(field)) {
                throw std::runtime_error("Requested field '" + field +
                                         "' does not exist in OSF.");
            }
        }
    }

    return lidar_scan;
}

LidarScanStreamMeta::LidarScanStreamMeta(const uint32_t sensor_meta_id,
                                         const LidarScanFieldTypes field_types)
    : sensor_meta_id_{sensor_meta_id},
      field_types_{field_types.begin(), field_types.end()} {}

uint32_t LidarScanStreamMeta::sensor_meta_id() const { return sensor_meta_id_; }

std::vector<uint8_t> LidarScanStreamMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);

    // Make and store field_types with details for LidarScanStream
    std::vector<ouster::sdk::osf::impl::gen::ChannelField> field_types;
    for (const auto& field_type : field_types_) {
        auto field_enum = to_osf_enum(field_type.name);
        if (field_enum) {
            field_types.emplace_back(field_enum.value(),
                                     to_osf_enum(field_type.element_type));
        }
    }

    auto field_types_off =
        osf::create_vector_of_structs<impl::gen::ChannelField>(
            fbb, field_types.data(), field_types.size());

    auto lss_offset = ouster::sdk::osf::impl::gen::CreateLidarScanStream(
        fbb, sensor_meta_id_, field_types_off);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> LidarScanStreamMeta::from_buffer(
    const OsfBuffer buf) {
    auto lidar_scan_stream =
        impl::gen::GetSizePrefixedLidarScanStream(buf.data());
    auto sensor_meta_id = lidar_scan_stream->sensor_id();

    auto field_types_vec = lidar_scan_stream->field_types();
    LidarScanFieldTypes field_types;
    if ((field_types_vec != nullptr) && (field_types_vec->size() != 0u)) {
        std::transform(field_types_vec->begin(), field_types_vec->end(),
                       std::back_inserter(field_types),
                       [](const impl::gen::ChannelField* channel_field) {
                           return FieldType{
                               from_osf_enum(channel_field->chan_field()),
                               from_osf_enum(channel_field->chan_field_type()),
                               {},
                               FieldClass::PIXEL_FIELD};
                       });
    }

    // auto frame_mode = lidar_scan_stream->lidar_frame_mode();
    return std::make_unique<LidarScanStreamMeta>(sensor_meta_id, field_types);
}

std::string LidarScanStreamMeta::repr() const {
    std::stringstream string_stream;
    string_stream << "LidarScanStreamMeta: sensor_id = " << sensor_meta_id_
                  << ", field_types = {";
    bool first = true;
    for (const auto& field : field_types_) {
        if (!first) {
            string_stream << ", ";
        }
        string_stream << field.name << ":"
                      << ouster::sdk::core::to_string(field.element_type);
        first = false;
    }
    string_stream << "}";
    return string_stream.str();
}

// ============== LidarScan Stream ops ===========================
LidarScanStream::LidarScanStream(Token /*key*/, Writer& writer,
                                 const uint32_t sensor_meta_id,
                                 const LidarScanFieldTypes& field_types)
    : writer_{writer},
      meta_(sensor_meta_id, field_types),
      sensor_meta_id_(sensor_meta_id),
      field_types_(field_types) {
    // Note key is ignored and just used to gatekeep.
    // Check sensor and get sensor_info
    auto sensor_meta_entry = writer.get_metadata<LidarSensor>(sensor_meta_id_);
    if (sensor_meta_entry == nullptr) {
        std::stringstream string_stream;
        string_stream << "ERROR: can't find sensor_meta_id = "
                      << sensor_meta_id;
        throw std::logic_error(string_stream.str());
    }

    sensor_info_ = sensor_meta_entry->info();

    writer_.add_metadata(meta_);
}

// TODO[pb]: Every save func in Streams is uniform, need to nicely extract
// it and remove close dependence on Writer? ...
void LidarScanStream::save(const ouster::sdk::osf::ts_t receive_ts,
                           const ouster::sdk::osf::ts_t sensor_ts,
                           const LidarScan& lidar_scan,
                           const LidarScanFieldTypes& field_types) {
    auto msg_buf = make_msg(lidar_scan, field_types);
    writer_.save_message(meta_.id(), receive_ts, sensor_ts, msg_buf,
                         MetadataTraits<LidarScanStreamMeta>::type());
}

std::vector<uint8_t> LidarScanStream::make_msg(
    const LidarScan& lidar_scan, const LidarScanFieldTypes& field_types) {
    if (lidar_scan.w != sensor_info_.w() || lidar_scan.h != sensor_info_.h()) {
        std::stringstream exception_msg_stream;
        exception_msg_stream
            << "lidar scan size (" << lidar_scan.w << ", " << lidar_scan.h
            << ") does not match the sensor info resolution ("
            << sensor_info_.w() << ", " << sensor_info_.h() << ")";
        throw std::invalid_argument(exception_msg_stream.str());
    }
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    const auto& encoder = writer_.encoder().lidar_scan_encoder();
    auto ls_msg_offset = create_lidar_scan_msg(fbb, encoder, lidar_scan,
                                               sensor_info_, field_types);
    fbb.FinishSizePrefixed(ls_msg_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};  // FIXME[tws] a copy
}

/**
 * Decode the MessageRef, ultimately created from a
 * StampedMessage - see fb/chunk.fbs) as the type appropriate for this
 * LidarScanStream (at this time, this is only ever a LidarScan.)
 *
 * IMPORTANT: this method allocates a LidarScan, copies the data from the
 * buffer, and returns it wrapped in a unique_ptr. It returns nullptr if the
 * message couldn't be decoded properly.  Overall, this could be more efficient
 * and should probably be using value semantics. See restore_lidar_scan for
 * details.
 */
std::unique_ptr<LidarScanStream::obj_type> LidarScanStream::decode_msg(
    const MessageRef& msg, const LidarScanStream::meta_type& meta,
    const MetadataStore& meta_provider,
    const nonstd::optional<std::vector<std::string>>& fields) {
    auto sensor = meta_provider.get<LidarSensor>(meta.sensor_meta_id());
    return restore_lidar_scan(msg, sensor->info(), fields);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
