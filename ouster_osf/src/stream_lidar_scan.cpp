/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/stream_lidar_scan.h"

#include <algorithm>

#include "ouster/lidar_scan.h"
#include "ouster/osf/basics.h"
#include "ouster/types.h"
#include "png_tools.h"

namespace ouster {
namespace osf {

namespace {

gen::CHAN_FIELD to_osf_enum(sensor::ChanField f) {
    // TODO[pb]: When we start diverging add a better mapping.
    return static_cast<gen::CHAN_FIELD>(f);
}

sensor::ChanField from_osf_enum(gen::CHAN_FIELD f) {
    // TODO[pb]: When we start diverging add a better mapping.
    return static_cast<sensor::ChanField>(f);
}

gen::CHAN_FIELD_TYPE to_osf_enum(sensor::ChanFieldType f) {
    // TODO[pb]: When we start diverging add a better mapping.
    return static_cast<gen::CHAN_FIELD_TYPE>(f);
}

sensor::ChanFieldType from_osf_enum(gen::CHAN_FIELD_TYPE ft) {
    // TODO[pb]: When we start diverging add a better mapping.
    return static_cast<sensor::ChanFieldType>(ft);
}

}  // namespace

bool poses_present(const LidarScan& ls) {
    return std::find_if_not(ls.pose().begin(), ls.pose().end(),
                            [](const mat4d& m) { return m.isIdentity(); }) !=
           ls.pose().end();
}

// TODO[pb]: Error if field_types is not subset of fields in ls?
LidarScan slice_with_cast(const LidarScan& ls_src,
                          const LidarScanFieldTypes& field_types) {
    LidarScan ls_dest{static_cast<std::size_t>(ls_src.w),
                      static_cast<std::size_t>(ls_src.h), field_types.begin(),
                      field_types.end()};

    ls_dest.frame_id = ls_src.frame_id;

    // Copy headers
    ls_dest.timestamp() = ls_src.timestamp();
    ls_dest.measurement_id() = ls_src.measurement_id();
    ls_dest.status() = ls_src.status();
    ls_dest.pose() = ls_src.pose();
    ls_dest.packet_timestamp() = ls_src.packet_timestamp();

    // Copy fields
    for (const auto& ft : field_types) {
        if (ls_src.field_type(ft.first)) {
            ouster::impl::visit_field(ls_dest, ft.first,
                                      ouster::impl::copy_and_cast(), ls_src,
                                      ft.first);
        } else {
            ouster::impl::visit_field(ls_dest, ft.first, zero_field());
        }
    }

    return ls_dest;
}

// === LidarScanStream support functions ====

// After Flatbuffers >= 22.9.24 the alignment bug was introduced in the #7520
// https://github.com/google/flatbuffers/pull/7520
// That is manifested in the additional 2 bytes written to the vector buffer
// before adding the vector size that are not properly handled by the reading
// the buffer back since vector is the 4 bytes length with the data starting
// right after the size. And with those additional 2 zero bytes it's just
// ruining the vector data.
// It started happening because of alignment rules changed in #7520 and it's
// triggering for our code because he have a small structs of just 2 bytes
// which can result in not 4 bytes aligned memory that is min requirement
// for storing the subsequent vector length in uoffset_t type)
// FIX[pb]: We are changing the original CreateVectorOfStructs implementation
//      with CreateUninitializedVectorOfStructs impl that is different in
//      a way how it's using StartVector underneath.
template <typename T>
flatbuffers::Offset<flatbuffers::Vector<const T*>> CreateVectorOfStructs(
    flatbuffers::FlatBufferBuilder& _fbb, const T* v, size_t len) {
    T* buf_to_write;
    auto res_off = _fbb.CreateUninitializedVectorOfStructs(len, &buf_to_write);
    if (len > 0) {
        memcpy(buf_to_write, reinterpret_cast<const uint8_t*>(v),
               sizeof(T) * len);
    }
    return res_off;
}

flatbuffers::Offset<gen::LidarScanMsg> create_lidar_scan_msg(
    flatbuffers::FlatBufferBuilder& fbb, const LidarScan& lidar_scan,
    const ouster::sensor::sensor_info& info,
    const LidarScanFieldTypes meta_field_types) {
    auto ls = lidar_scan;
    if (!meta_field_types.empty()) {
        // Make a reduced field LidarScan (or extend if the field types is
        // different)
        // TODO[pb]: Consider to error instead of extending LidarScan, but be
        //           sure to check the consistence everywhere. That's why it's
        //           not done on the first pass here ...
        ls = slice_with_cast(lidar_scan, meta_field_types);
    }

    // Encode LidarScan to PNG buffers
    ScanData scan_data = scanEncode(ls, info.format.pixel_shift_by_row);

    // Prepare PNG encoded channels for LidarScanMsg.channels vector
    std::vector<flatbuffers::Offset<gen::ChannelData>> channels;
    for (const auto& channel_data : scan_data) {
        channels.emplace_back(gen::CreateChannelDataDirect(fbb, &channel_data));
    }

    // Prepare field_types for LidarScanMsg
    std::vector<ouster::osf::gen::ChannelField> field_types;
    for (const auto& f : ls) {
        field_types.emplace_back(to_osf_enum(f.first), to_osf_enum(f.second));
    }

    auto channels_off =
        fbb.CreateVector<::flatbuffers::Offset<gen::ChannelData>>(channels);
    auto field_types_off = osf::CreateVectorOfStructs<gen::ChannelField>(
        fbb, field_types.data(), field_types.size());
    auto timestamp_off = fbb.CreateVector<uint64_t>(ls.timestamp().data(),
                                                    ls.timestamp().size());
    auto measurement_id_off =
        fbb.CreateVector<uint16_t>(ls.measurement_id().data(), ls.w);
    auto status_off = fbb.CreateVector<uint32_t>(ls.status().data(), ls.w);

    flatbuffers::Offset<flatbuffers::Vector<double>> pose_off = 0;
    if (poses_present(ls)) {
        pose_off = fbb.CreateVector<double>(ls.pose().data()->data(),
                                            ls.pose().size() * 16);
    }

    auto packet_timestamp_id_off = fbb.CreateVector<uint64_t>(
        ls.packet_timestamp().data(), ls.packet_timestamp().size());
    return gen::CreateLidarScanMsg(
        fbb, channels_off, field_types_off, timestamp_off, measurement_id_off,
        status_off, ls.frame_id, pose_off, packet_timestamp_id_off);
}

std::unique_ptr<ouster::LidarScan> restore_lidar_scan(
    const std::vector<uint8_t> buf, const ouster::sensor::sensor_info& info) {
    auto ls_msg =
        flatbuffers::GetSizePrefixedRoot<ouster::osf::gen::LidarScanMsg>(
            buf.data());

    uint32_t width = info.format.columns_per_frame;
    uint32_t height = info.format.pixels_per_column;

    // read field_types
    LidarScanFieldTypes field_types;
    if (ls_msg->field_types() && ls_msg->field_types()->size()) {
        std::transform(
            ls_msg->field_types()->begin(), ls_msg->field_types()->end(),
            std::back_inserter(field_types), [](const gen::ChannelField* p) {
                return std::make_pair(from_osf_enum(p->chan_field()),
                                      from_osf_enum(p->chan_field_type()));
            });
    }

    // Init lidar scan with recovered fields
    auto ls = std::make_unique<LidarScan>(width, height, field_types.begin(),
                                          field_types.end());

    ls->frame_id = ls_msg->frame_id();

    // Set timestamps per column
    auto msg_ts_vec = ls_msg->header_timestamp();
    if (msg_ts_vec) {
        if (static_cast<uint32_t>(ls->w) == msg_ts_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                ls->timestamp()[i] = msg_ts_vec->Get(i);
            }
        } else if (msg_ts_vec->size() != 0) {
            std::cout << "ERROR: LidarScanMsg has header_timestamp of length: "
                      << msg_ts_vec->size() << ", expected: " << ls->w
                      << std::endl;
            return nullptr;
        }
    }

    // Set measurement_id per column
    auto msg_mid_vec = ls_msg->header_measurement_id();
    if (msg_mid_vec) {
        if (static_cast<uint32_t>(ls->w) == msg_mid_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                ls->measurement_id()[i] = msg_mid_vec->Get(i);
            }
        } else if (msg_mid_vec->size() != 0) {
            std::cout
                << "ERROR: LidarScanMsg has header_measurement_id of length: "
                << msg_mid_vec->size() << ", expected: " << ls->w << std::endl;
            return nullptr;
        }
    }

    // Set status per column
    auto msg_status_vec = ls_msg->header_status();
    if (msg_status_vec) {
        if (static_cast<uint32_t>(ls->w) == msg_status_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                ls->status()[i] = msg_status_vec->Get(i);
            }
        } else if (msg_status_vec->size() != 0) {
            std::cout << "ERROR: LidarScanMsg has header_status of length: "
                      << msg_status_vec->size() << ", expected: " << ls->w
                      << std::endl;
            return nullptr;
        }
    }

    // Set poses per column
    auto pose_vec = ls_msg->pose();
    if (pose_vec) {
        if (static_cast<uint32_t>(ls->pose().size() * 16) == pose_vec->size()) {
            for (uint32_t i = 0; i < static_cast<uint32_t>(ls->pose().size());
                 ++i) {
                for (uint32_t el = 0; el < 16; ++el) {
                    *(ls->pose()[i].data() + el) = pose_vec->Get(i * 16 + el);
                }
            }
        } else if (pose_vec->size() != 0) {
            std::cout << "ERROR: LidarScanMsg has pose of length: "
                      << pose_vec->size()
                      << ", expected: " << (ls->pose().size() * 16)
                      << std::endl;
            return nullptr;
        }
    }

    // Set packet timestamp per lidar packet
    auto packet_ts_vec = ls_msg->packet_timestamp();
    if (packet_ts_vec) {
        if (static_cast<uint32_t>(ls->packet_timestamp().size()) ==
            packet_ts_vec->size()) {
            for (uint32_t i = 0; i < packet_ts_vec->size(); ++i) {
                ls->packet_timestamp()[i] = packet_ts_vec->Get(i);
            }
        } else if (packet_ts_vec->size() != 0) {
            std::cout << "ERROR: LidarScanMsg has packet_timestamp of length: "
                      << packet_ts_vec->size()
                      << ", expected: " << ls->packet_timestamp().size()
                      << std::endl;
            return nullptr;
        }
    }

    // Fill Scan Data with scan channels
    auto msg_scan_vec = ls_msg->channels();
    if (!msg_scan_vec || !msg_scan_vec->size()) {
        std::cout
            << "ERROR: lidar_scan msg doesn't have scan field or it's empty.\n";
        return nullptr;
    }
    ScanData scan_data;
    for (uint32_t i = 0; i < msg_scan_vec->size(); ++i) {
        auto channel_buffer = msg_scan_vec->Get(i)->buffer();
        scan_data.emplace_back(channel_buffer->begin(), channel_buffer->end());
    }

    // Decode PNGs data to LidarScan
    if (scanDecode(*ls, scan_data, info.format.pixel_shift_by_row)) {
        return nullptr;
    }

    return ls;
}

std::vector<uint8_t> LidarScanStreamMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);

    // Make and store field_types with details for LidarScanStream
    std::vector<ouster::osf::gen::ChannelField> field_types;
    for (const auto& ft : field_types_) {
        field_types.emplace_back(to_osf_enum(ft.first), to_osf_enum(ft.second));
    }

    auto field_types_off = osf::CreateVectorOfStructs<gen::ChannelField>(
        fbb, field_types.data(), field_types.size());

    auto lss_offset = ouster::osf::gen::CreateLidarScanStream(
        fbb, sensor_meta_id_, field_types_off);

    fbb.FinishSizePrefixed(lss_offset);

    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
};

std::unique_ptr<MetadataEntry> LidarScanStreamMeta::from_buffer(
    const std::vector<uint8_t>& buf) {
    auto lidar_scan_stream = gen::GetSizePrefixedLidarScanStream(buf.data());
    auto sensor_meta_id = lidar_scan_stream->sensor_id();

    auto field_types_vec = lidar_scan_stream->field_types();
    LidarScanFieldTypes field_types;
    if (field_types_vec && field_types_vec->size()) {
        std::transform(
            field_types_vec->begin(), field_types_vec->end(),
            std::back_inserter(field_types), [](const gen::ChannelField* p) {
                return std::make_pair(from_osf_enum(p->chan_field()),
                                      from_osf_enum(p->chan_field_type()));
            });
    }

    // auto frame_mode = lidar_scan_stream->lidar_frame_mode();
    return std::make_unique<LidarScanStreamMeta>(sensor_meta_id, field_types);
};

std::string LidarScanStreamMeta::repr() const {
    std::stringstream ss;
    ss << "LidarScanStreamMeta: sensor_id = " << sensor_meta_id_
       << ", field_types = {";
    bool first = true;
    for (const auto& f : field_types_) {
        if (!first) ss << ", ";
        ss << sensor::to_string(f.first) << ":"
           << ouster::sensor::to_string(f.second);
        first = false;
    }
    ss << "}";
    return ss.str();
}

// ============== LidarScan Stream ops ===========================

LidarScanStream::LidarScanStream(Writer& writer, const uint32_t sensor_meta_id,
                                 const LidarScanFieldTypes& field_types)
    : writer_{writer},
      meta_(sensor_meta_id, field_types),
      sensor_meta_id_(sensor_meta_id) {
    // Check sensor and get sensor_info
    auto sensor_meta_entry = writer.getMetadata<LidarSensor>(sensor_meta_id_);
    if (sensor_meta_entry == nullptr) {
        std::cerr << "ERROR: can't find sensor_meta_id = " << sensor_meta_id
                  << std::endl;
        std::abort();
    }

    sensor_info_ = sensor_meta_entry->info();

    stream_meta_id_ = writer_.addMetadata(meta_);
}

// TODO[pb]: Every save func in Streams is uniform, need to nicely extract
// it and remove close dependence on Writer? ...
void LidarScanStream::save(const ouster::osf::ts_t ts,
                           const LidarScan& lidar_scan) {
    const auto& msg_buf = make_msg(lidar_scan);
    writer_.saveMessage(meta_.id(), ts, msg_buf);
}

std::vector<uint8_t> LidarScanStream::make_msg(const LidarScan& lidar_scan) {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    auto ls_msg_offset = create_lidar_scan_msg(fbb, lidar_scan, sensor_info_,
                                               meta_.field_types());
    fbb.FinishSizePrefixed(ls_msg_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
}

std::unique_ptr<LidarScanStream::obj_type> LidarScanStream::decode_msg(
    const std::vector<uint8_t>& buf, const LidarScanStream::meta_type& meta,
    const MetadataStore& meta_provider) {
    auto sensor = meta_provider.get<LidarSensor>(meta.sensor_meta_id());

    auto info = sensor->info();

    return restore_lidar_scan(buf, info);
}

}  // namespace osf
}  // namespace ouster
