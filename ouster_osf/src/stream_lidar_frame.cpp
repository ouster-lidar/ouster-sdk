/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */
#include "ouster/osf/stream_lidar_frame.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <nonstd/optional.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "fb_common.h"
#include "ouster/core/field_type.h"
#include "ouster/core/frame.h"
#include "ouster/core/impl/logging.h"
#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/osf/basics.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/frame_encoder.h"
#include "ouster/osf/reader.h"

// Auto-generated flatbuffer headers.
#include "os_sensor/lidar_scan_stream_generated.h"

using namespace ouster::sdk::core;
namespace ouster {
namespace sdk {
namespace osf {

bool poses_present(const LidarFrame& lidar_frame) {
    auto& pose = lidar_frame.body_to_world();
    auto ident = mat4d::Identity().eval();
    auto ptr = pose.get<double>();
    for (size_t i = 0, end = pose.shape()[0]; i < end; ++i) {
        if (std::memcmp(ident.data(), ptr + (i * 16), sizeof(double) * 16) != 0) {
            return true;
        }
    }
    return false;
}

std::shared_ptr<Frame> slice_and_cast(const Frame& src, const LidarFrameFieldTypes& field_types) {
    // Copying through clone() is what makes this work for any frame kind: the
    // result keeps src's concrete type along with every header that type owns,
    // so there is nothing here that has to know whether it was handed a
    // LidarFrame or a CameraFrame.
    auto dest = src.clone();

    // Drop the fields that were not asked for.
    std::vector<std::string> unwanted;
    for (const auto& field : dest->fields()) {
        const bool requested =
            std::any_of(field_types.begin(), field_types.end(),
                        [&field](const FieldType& type) { return type.name == field.first; });
        if (!requested) {
            unwanted.push_back(field.first);
        }
    }
    for (const auto& name : unwanted) {
        dest->del_field(name);
    }

    // Re-create the fields whose requested type differs from what src holds and
    // cast the data into them. The rest came across unchanged with the clone.
    for (const auto& field_type : field_types) {
        if (!src.has_field(field_type.name)) {
            throw std::invalid_argument("Required field '" + field_type.name +
                                        "' is missing from frame.");
        }
        if (src.field_type(field_type.name) == field_type) {
            continue;
        }
        dest->del_field(field_type.name);
        dest->add_field(field_type);
        ouster::sdk::core::impl::visit_field(
            *dest, field_type.name, ouster::sdk::core::impl::copy_and_cast(), src, field_type.name);
    }

    return dest;
}

LidarFrame slice_and_cast(const LidarFrame& ls_src, const LidarFrameFieldTypes& field_types) {
    // clone() of a LidarFrame is a LidarFrame, so the cast cannot fail.
    auto ls_dest = std::dynamic_pointer_cast<LidarFrame>(
        slice_and_cast(static_cast<const Frame&>(ls_src), field_types));
    return std::move(*ls_dest);
}

namespace {

flatbuffers::Offset<impl::gen::LidarScanMsg> create_lidar_frame_msg(
    flatbuffers::FlatBufferBuilder& fbb, const FrameEncoder& encoder, const LidarFrame& lidar_frame,
    const LidarInfo& info, const LidarFrameFieldTypes& meta_field_types) {
    // Prepare field_types for LidarFrameMsg
    std::vector<std::pair<impl::gen::CHAN_FIELD, std::string>> standard_fields_to_sort;
    std::vector<std::pair<std::string, const Field*>> custom_field_names;
    for (const auto& field : lidar_frame.fields()) {
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
            // otherwise add it to field types and then frame encode those
            standard_fields_to_sort.emplace_back(enum_value.value(), field.first);
        }
    }

    // because older OSF readers are broken, we need to sort the standard fields
    // by value...
    std::sort(standard_fields_to_sort.begin(), standard_fields_to_sort.end());

    // now actually build our arrays from the sorted one
    std::vector<ouster::sdk::osf::impl::gen::ChannelField> field_types;
    LidarFrameFieldTypes standard_fields;
    for (const auto& field_pair : standard_fields_to_sort) {
        const auto& field = lidar_frame.field(field_pair.second);
        field_types.emplace_back(field_pair.first, to_osf_enum(field.tag()));
        standard_fields.emplace_back(field_pair.second, field.tag());
    }

    auto field_types_off = osf::create_vector_of_structs<impl::gen::ChannelField>(
        fbb, field_types.data(), field_types.size());
    auto channels_off = fb_save_frame_channels(fbb, encoder, lidar_frame, standard_fields,
                                               info.format.pixel_shift_by_row);
    auto custom_fields_off = fb_save_fields(fbb, encoder, custom_field_names);
    auto timestamp_off =
        fbb.CreateVector<uint64_t>(lidar_frame.timestamp().data(), lidar_frame.timestamp().size());
    auto measurement_id_off =
        fbb.CreateVector<uint16_t>(lidar_frame.measurement_id().data(), lidar_frame.w);
    auto status_off = fbb.CreateVector<uint32_t>(lidar_frame.status().data(), lidar_frame.w);
    flatbuffers::Offset<flatbuffers::Vector<double>> pose_off = 0;
    if (poses_present(lidar_frame)) {
        pose_off = fbb.CreateVector<double>(lidar_frame.body_to_world().get<double>(),
                                            lidar_frame.body_to_world().bytes() / sizeof(double));
    }
    auto receive_timestamp_id_off = fbb.CreateVector<uint64_t>(
        lidar_frame.receive_timestamp().data(), lidar_frame.receive_timestamp().size());

    auto alert_flags_off = fbb.CreateVector<uint8_t>(lidar_frame.alert_flags().data(),
                                                     lidar_frame.alert_flags().size());

    auto obj_lists_off = fb_save_object_lists(fbb, lidar_frame.objects());

    return impl::gen::CreateLidarScanMsg(
        fbb, channels_off, field_types_off, timestamp_off, measurement_id_off, status_off,
        static_cast<int32_t>(lidar_frame.frame_id), pose_off, receive_timestamp_id_off,
        custom_fields_off, lidar_frame.frame_status, lidar_frame.shutdown_countdown,
        lidar_frame.shot_limiting_countdown, alert_flags_off, obj_lists_off);
}

}  // namespace

namespace {

/**
 * Copy the contents of the LidarFrameMsg into a LidarFrame.
 *
 * NOTE: LidarFrame isn't inherently flatbuffers-based, which unfortunately
 * means we're not taking advantage of one of fb's primary benefits - the
 * ability to read data from messages without copying or allocating new memory
 * for it. Moreover, the OSF representation of LidarFrame is compressed and the
 * deserialized version is not.
 *
 * As such, making use of the FB directly would require revisiting the design
 * and a significant refactor.
 */
// Sequential OSF field-restoration path; splitting would obscure the decode
// logic.
// NOLINTNEXTLINE(readability-function-cognitive-complexity)
std::unique_ptr<LidarFrame> restore_lidar_frame(
    const MessageRef& msg, const LidarInfo& info,
    const nonstd::optional<std::vector<std::string>>& fields_to_decode) {
    auto ls_msg =
        flatbuffers::GetSizePrefixedRoot<ouster::sdk::osf::impl::gen::LidarScanMsg>(msg.data());

    uint32_t width = info.format.columns_per_frame;
    uint32_t height = info.format.pixels_per_column;

    // read field_types, skipping (with a warning) any that were written by a
    // newer SDK with a field or field type this version doesn't know about
    LidarFrameFieldTypes field_types;
    std::vector<size_t> channel_indices;
    if ((ls_msg->field_types() != nullptr) && (ls_msg->field_types()->size() != 0u)) {
        for (size_t i = 0; i < ls_msg->field_types()->size(); ++i) {
            const auto* field_type = ls_msg->field_types()->Get(i);
            auto name = try_from_osf_enum(field_type->chan_field());
            if (!name) {
                msg.error_handler()(Severity::OUSTER_WARNING,
                                    "Skipping field with unsupported channel field id (" +
                                        std::to_string(static_cast<int>(field_type->chan_field())) +
                                        ")");
                continue;
            }
            auto element_type = try_from_osf_enum(field_type->chan_field_type());
            if (!element_type) {
                msg.error_handler()(
                    Severity::OUSTER_WARNING,
                    "Skipping field '" + name.value() + "' with unsupported field type (tag=" +
                        std::to_string(static_cast<int>(field_type->chan_field_type())) + ")");
                continue;
            }
            field_types.emplace_back(name.value(), element_type.value(), std::vector<size_t>{},
                                     FieldClass::PIXEL_FIELD);
            channel_indices.push_back(i);
        }
    }

    // Init lidar frame with recovered fields
    LidarFrameFieldTypes desired_field_types =
        !fields_to_decode.has_value() ? field_types : LidarFrameFieldTypes();
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
    LidarFrameFieldTypes empty_field_types;
    auto lidar_frame = std::make_unique<LidarFrame>(height, width, empty_field_types,
                                                    info.format.columns_per_packet);
    // add fields here rather than above so can avoid zero initialization
    for (const auto& field : desired_field_types) {
        lidar_frame->add_field(field, false);
    }

    // set frame status - unfortunately since this is a new field in the FB
    // schema and since LidarFrame::frame_status is an integer we have no way to
    // differentiate between whether the value was zero when written or wasn't
    // provided by the writer.
    lidar_frame->frame_status = ls_msg->frame_status();
    lidar_frame->shutdown_countdown = ls_msg->shutdown_countdown();
    lidar_frame->shot_limiting_countdown = ls_msg->shot_limiting_countdown();

    lidar_frame->frame_id = ls_msg->frame_id();

    // Set timestamps per column
    auto msg_ts_vec = ls_msg->header_timestamp();
    if (msg_ts_vec != nullptr) {
        if (static_cast<uint32_t>(lidar_frame->w) == msg_ts_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                lidar_frame->timestamp()[i] = msg_ts_vec->Get(i);
            }
        } else if (msg_ts_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has header_timestamp of length: "
                "{}, expected: {}",
                msg_ts_vec->size(), lidar_frame->w);
            return nullptr;
        }
    }

    // Set measurement_id per column
    auto msg_mid_vec = ls_msg->header_measurement_id();
    if (msg_mid_vec != nullptr) {
        if (static_cast<uint32_t>(lidar_frame->w) == msg_mid_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                lidar_frame->measurement_id()[i] = msg_mid_vec->Get(i);
            }
        } else if (msg_mid_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has header_measurement_id of length: "
                "{}, expected: {}",
                msg_mid_vec->size(), lidar_frame->w);
            return nullptr;
        }
    }

    // Set status per column
    auto msg_status_vec = ls_msg->header_status();
    if (msg_status_vec != nullptr) {
        if (static_cast<uint32_t>(lidar_frame->w) == msg_status_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                lidar_frame->status()[i] = msg_status_vec->Get(i);
            }
        } else if (msg_status_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has header_status of length: "
                "{}, expected: {}",
                msg_status_vec->size(), lidar_frame->w);
            return nullptr;
        }
    }

    // Set poses per column
    auto pose_vec = ls_msg->pose();
    if (pose_vec != nullptr) {
        auto& pose = lidar_frame->body_to_world();
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
        if (static_cast<uint32_t>(lidar_frame->receive_timestamp().size()) ==
            packet_ts_vec->size()) {
            for (uint32_t i = 0; i < packet_ts_vec->size(); ++i) {
                lidar_frame->receive_timestamp()[i] = packet_ts_vec->Get(i);
            }
        } else if (packet_ts_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has receive_timestamp of length: "
                "{}, expected: {}",
                packet_ts_vec->size(), lidar_frame->receive_timestamp().size());
            return nullptr;
        }
    }

    // Set alert flags per lidar packet
    auto alert_flags_vec = ls_msg->alert_flags();
    if (alert_flags_vec != nullptr) {
        if (static_cast<size_t>(lidar_frame->alert_flags().size()) == alert_flags_vec->size()) {
            for (size_t i = 0; i < alert_flags_vec->size(); ++i) {
                lidar_frame->alert_flags()[static_cast<Eigen::Index>(i)] = alert_flags_vec->Get(i);
            }
        } else if (alert_flags_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has alert_flags of length: "
                "{}, expected: {}",
                alert_flags_vec->size(), lidar_frame->alert_flags().size());
            return nullptr;
        }
    }

    // Fill Frame Data with frame channels
    auto msg_frame_channels = ls_msg->channels();
    if (msg_frame_channels == nullptr) {
        logger().error("ERROR: lidar_frame msg doesn't have frame fields.");
        return nullptr;
    }

    auto chan_futures = fb_restore_channels(msg_frame_channels, field_types, channel_indices,
                                            info.format.pixel_shift_by_row, *lidar_frame);

    auto add_field = [&lidar_frame](const std::string& name, const FieldDescriptor& desc,
                                    FieldClass field_class) -> Field& {
        return lidar_frame->add_field(name, desc, field_class, false);
    };

    // make sure to keep objects in scope until futures are resolved
    auto custom_fields = ls_msg->custom_fields();
    auto field_futures =
        fb_restore_fields(custom_fields, fields_to_decode, add_field, msg.error_handler());

    fb_restore_object_lists(ls_msg->objects(), lidar_frame->objects(), msg.error_handler());

    // Finally wait for the futures and catch any exceptions, passing them to the error handler
    for (auto& future : chan_futures) {
        try {
            future.get();
        } catch (std::runtime_error& error) {
            msg.error_handler()(Severity::OUSTER_WARNING, error.what());
        }
    }

    for (auto& future : field_futures) {
        try {
            future.get();
        } catch (std::runtime_error& error) {
            msg.error_handler()(Severity::OUSTER_WARNING, error.what());
        }
    }

    // error if any of the requested fields did not end up in the lidar frame
    if (fields_to_decode) {
        for (const auto& field : fields_to_decode.value()) {
            if (!lidar_frame->has_field(field)) {
                throw std::runtime_error("Requested field '" + field + "' does not exist in OSF.");
            }
        }
    }

    return lidar_frame;
}

}  // namespace

LidarFrameStreamMeta::LidarFrameStreamMeta(const uint32_t sensor_meta_id,
                                           const LidarFrameFieldTypes& field_types)
    : sensor_meta_id_{sensor_meta_id}, field_types_{field_types.begin(), field_types.end()} {}

uint32_t LidarFrameStreamMeta::sensor_meta_id() const {
    return sensor_meta_id_;
}

std::vector<uint8_t> LidarFrameStreamMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);

    // Make and store field_types with details for LidarFrameStream
    std::vector<ouster::sdk::osf::impl::gen::ChannelField> field_types;
    for (const auto& field_type : field_types_) {
        auto field_enum = to_osf_enum(field_type.name);
        if (field_enum) {
            field_types.emplace_back(field_enum.value(), to_osf_enum(field_type.element_type));
        }
    }

    auto field_types_off = osf::create_vector_of_structs<impl::gen::ChannelField>(
        fbb, field_types.data(), field_types.size());

    auto lss_offset =
        ouster::sdk::osf::impl::gen::CreateLidarScanStream(fbb, sensor_meta_id_, field_types_off);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> LidarFrameStreamMeta::from_buffer(const OsfBuffer& buf) {
    auto lidar_frame_stream = impl::gen::GetSizePrefixedLidarScanStream(buf.data());
    auto sensor_meta_id = lidar_frame_stream->sensor_id();

    auto field_types_vec = lidar_frame_stream->field_types();
    LidarFrameFieldTypes field_types;
    if (field_types_vec != nullptr) {
        for (const auto* channel_field : *field_types_vec) {
            auto name = try_from_osf_enum(channel_field->chan_field());
            auto element_type = try_from_osf_enum(channel_field->chan_field_type());
            if (!name || !element_type) {
                logger().warn(
                    "LidarScanStream metadata: skipping field with unsupported channel field id "
                    "({}) or field type (tag={})",
                    static_cast<int>(channel_field->chan_field()),
                    static_cast<int>(channel_field->chan_field_type()));
                continue;
            }
            field_types.emplace_back(name.value(), element_type.value(), std::vector<size_t>{},
                                     FieldClass::PIXEL_FIELD);
        }
    }

    // auto frame_mode = lidar_frame_stream->lidar_frame_mode();
    return std::make_unique<LidarFrameStreamMeta>(sensor_meta_id, field_types);
}

std::string LidarFrameStreamMeta::repr() const {
    std::stringstream string_stream;
    string_stream << "LidarFrameStreamMeta: sensor_id = " << sensor_meta_id_ << ", field_types = {";
    bool first = true;
    for (const auto& field : field_types_) {
        if (!first) {
            string_stream << ", ";
        }
        string_stream << field.name << ":" << ouster::sdk::core::to_string(field.element_type);
        first = false;
    }
    string_stream << "}";
    return string_stream.str();
}

// ============== LidarFrame Stream ops ===========================
LidarFrameStream::LidarFrameStream(Writer& writer, const uint32_t sensor_meta_id,
                                   const LidarFrameFieldTypes& field_types)
    : writer_{writer},
      meta_(sensor_meta_id, field_types),
      sensor_meta_id_(sensor_meta_id),
      field_types_(field_types) {
    // Note key is ignored and just used to gatekeep.
    // Check sensor and get sensor_info
    auto sensor_meta_entry = writer.get_metadata<LidarSensor>(sensor_meta_id_);
    if (sensor_meta_entry == nullptr) {
        std::stringstream string_stream;
        string_stream << "ERROR: can't find sensor_meta_id = " << sensor_meta_id;
        throw std::logic_error(string_stream.str());
    }

    sensor_info_ = sensor_meta_entry->info();

    writer_.add_metadata(meta_);
}

// TODO[pb]: Every save func in Streams is uniform, need to nicely extract
// it and remove close dependence on Writer? ...
void LidarFrameStream::save(const ouster::sdk::osf::ts_t receive_ts,
                            const ouster::sdk::osf::ts_t sensor_ts, const LidarFrame& lidar_frame,
                            const LidarFrameFieldTypes& field_types) {
    auto msg_buf = make_msg(lidar_frame, field_types);
    writer_.save_message(meta_.id(), receive_ts, sensor_ts, msg_buf,
                         MetadataTraits<LidarFrameStreamMeta>::type());
}

std::vector<uint8_t> LidarFrameStream::make_msg(const LidarFrame& lidar_frame,
                                                const LidarFrameFieldTypes& field_types) {
    if (lidar_frame.w != sensor_info_.w() || lidar_frame.h != sensor_info_.h()) {
        std::stringstream exception_msg_stream;
        exception_msg_stream << "lidar frame size (" << lidar_frame.w << ", " << lidar_frame.h
                             << ") does not match the sensor info resolution (" << sensor_info_.w()
                             << ", " << sensor_info_.h() << ")";
        throw std::invalid_argument(exception_msg_stream.str());
    }
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    const auto& encoder = writer_.encoder().frame_encoder();
    auto ls_msg_offset =
        create_lidar_frame_msg(fbb, encoder, lidar_frame, sensor_info_, field_types);
    fbb.FinishSizePrefixed(ls_msg_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};  // FIXME[tws] a copy
}

/**
 * Decode the MessageRef, ultimately created from a
 * StampedMessage - see fb/chunk.fbs) as the type appropriate for this
 * LidarFrameStream (at this time, this is only ever a LidarFrame.)
 *
 * IMPORTANT: this method allocates a LidarFrame, copies the data from the
 * buffer, and returns it wrapped in a unique_ptr. It returns nullptr if the
 * message couldn't be decoded properly.  Overall, this could be more efficient
 * and should probably be using value semantics. See restore_lidar_frame for
 * details.
 */
std::unique_ptr<LidarFrameStream::obj_type> LidarFrameStream::decode_msg(
    const MessageRef& msg, const LidarFrameStream::meta_type& meta,
    const MetadataStore& meta_provider, const nonstd::optional<std::vector<std::string>>& fields) {
    auto sensor = meta_provider.get<LidarSensor>(meta.sensor_meta_id());
    return restore_lidar_frame(msg, sensor->info(), fields);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
