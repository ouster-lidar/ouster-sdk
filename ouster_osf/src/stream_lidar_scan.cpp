/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/stream_lidar_scan.h"

#include <algorithm>
#include <deque>
#include <future>
#include <sstream>

#include "ouster/impl/logging.h"
#include "ouster/lidar_scan.h"
#include "ouster/osf/basics.h"
#include "ouster/strings.h"
#include "ouster/types.h"
#include "png_tools.h"

using namespace ouster::sensor;
using namespace ouster::strings;

namespace ouster {
namespace osf {

namespace {

gen::CHAN_FIELD_TYPE to_osf_enum(sensor::ChanFieldType f) {
    // TODO[pb]: When we start diverging add a better mapping.
    return static_cast<gen::CHAN_FIELD_TYPE>(f);
}

sensor::ChanFieldType from_osf_enum(gen::CHAN_FIELD_TYPE ft) {
    // TODO[pb]: When we start diverging add a better mapping.
    return static_cast<sensor::ChanFieldType>(ft);
}

gen::FIELD_CLASS to_osf_enum(ouster::FieldClass ff) {
    return static_cast<gen::FIELD_CLASS>(ff);
}

ouster::FieldClass from_osf_enum(gen::FIELD_CLASS ff) {
    return static_cast<ouster::FieldClass>(ff);
}

}  // namespace

bool poses_present(const LidarScan& ls) {
    auto&& pose = ls.pose();
    ouster::mat4d mat;
    for (size_t i = 0, end = pose.shape()[0]; i < end; ++i) {
        std::memcpy(mat.data(), pose.subview(i).get(), sizeof(double) * 16);
        if (!mat.isIdentity()) return true;
    }
    return false;
}

LidarScan slice_with_cast(const LidarScan& ls_src,
                          const ouster::LidarScanFieldTypes& field_types) {
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
    for (const auto& ft : field_types) {
        if (ls_src.has_field(ft.name)) {
            ouster::impl::visit_field(ls_dest, ft.name,
                                      ouster::impl::copy_and_cast(), ls_src,
                                      ft.name);
        } else {
            throw std::invalid_argument("Required field '" + ft.name +
                                        "' is missing from scan.");
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
// triggering for our code because we have a small structs of just 2 bytes
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

using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;

namespace impl {
template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;
}

template <typename K, typename V, size_t N>
static optional<V> lookup(const impl::Table<K, V, N> table, const K& k) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end, [&](const std::pair<K, V>& p) {
        return p.first == k;
    });

    return res == end ? nullopt : make_optional<V>(res->second);
}

template <typename K, size_t N>
static optional<K> rlookup(const impl::Table<K, const char*, N> table,
                           const char* v) {
    auto end = table.end();
    auto res = std::find_if(
        table.begin(), end, [&](const std::pair<K, const char*>& p) {
            return p.second && std::strcmp(p.second, v) == 0;
        });

    return res == end ? nullopt : make_optional<K>(res->first);
}

// mapping of channel name to osf ChanField
static impl::Table<ouster::osf::gen::CHAN_FIELD, const char*, 30>
    chanfield_strings{{
        {ouster::osf::gen::CHAN_FIELD::UNKNOWN, "UNKNOWN"},
        {ouster::osf::gen::CHAN_FIELD::RANGE, ChanField::RANGE},
        {ouster::osf::gen::CHAN_FIELD::RANGE2, ChanField::RANGE2},
        {ouster::osf::gen::CHAN_FIELD::SIGNAL, ChanField::SIGNAL},
        {ouster::osf::gen::CHAN_FIELD::SIGNAL2, ChanField::SIGNAL2},
        {ouster::osf::gen::CHAN_FIELD::REFLECTIVITY, ChanField::REFLECTIVITY},
        {ouster::osf::gen::CHAN_FIELD::REFLECTIVITY2, ChanField::REFLECTIVITY2},
        {ouster::osf::gen::CHAN_FIELD::NEAR_IR, ChanField::NEAR_IR},
        {ouster::osf::gen::CHAN_FIELD::FLAGS, ChanField::FLAGS},
        {ouster::osf::gen::CHAN_FIELD::FLAGS2, ChanField::FLAGS2},
        {ouster::osf::gen::CHAN_FIELD::RAW_HEADERS, ChanField::RAW_HEADERS},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM0, "CUSTOM0"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM1, "CUSTOM1"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM2, "CUSTOM2"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM3, "CUSTOM3"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM4, "CUSTOM4"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM5, "CUSTOM5"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM6, "CUSTOM6"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM7, "CUSTOM7"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM8, "CUSTOM8"},
        {ouster::osf::gen::CHAN_FIELD::CUSTOM9, "CUSTOM9"},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD1, ChanField::RAW32_WORD1},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD2, ChanField::RAW32_WORD2},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD3, ChanField::RAW32_WORD3},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD4, ChanField::RAW32_WORD4},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD5, ChanField::RAW32_WORD5},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD6, ChanField::RAW32_WORD6},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD7, ChanField::RAW32_WORD7},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD8, ChanField::RAW32_WORD8},
        {ouster::osf::gen::CHAN_FIELD::RAW32_WORD9, ChanField::RAW32_WORD9},
    }};

nonstd::optional<gen::CHAN_FIELD> to_osf_enum(const std::string& f) {
    // TODO[pb]: When we start diverging add a better mapping.
    return rlookup(chanfield_strings, f.c_str());
}

std::string from_osf_enum(gen::CHAN_FIELD f) {
    // TODO[pb]: When we start diverging add a better mapping.
    return lookup(chanfield_strings, f).value();
}

// ========== Encode Functions ===================================
ScanData LidarScanStream::scanEncode(
    const LidarScan& lidar_scan, const std::vector<int>& px_offset,
    const ouster::LidarScanFieldTypes& standard_field_types,
    const std::vector<std::pair<std::string, const Field*>> custom_fields,
    ScanData& custom_data) const {
    // Prepare scan data of size that fits all field_types we are about to
    // encode
    ScanData fields_data(standard_field_types.size());
    custom_data.resize(custom_fields.size());

    std::mutex queue_mutex;
    std::deque<int> queue;
    for (int i = 0; i < (int)standard_field_types.size(); i++) {
        queue.push_back(i);
    }
    for (int i = 0; i < (int)custom_fields.size(); i++) {
        queue.push_back(-i - 1);
    }
    auto thread_fn = [&]() {
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex);
            // if no items left, just exit
            if (queue.empty()) {
                return;
            }
            auto item_idx = queue.front();
            queue.pop_front();
            lock.unlock();

            // if idx is negative, its in custom fields
            if (item_idx < 0) {
                item_idx = -item_idx - 1;
                // encode!
                ScanChannelData data =
                    writer_.encoder().lidar_scan_encoder().encodeField(
                        *custom_fields[item_idx].second, {});
                custom_data[item_idx].swap(data);
            } else {
                auto& field =
                    lidar_scan.field(standard_field_types[item_idx].name);
                ScanChannelData data =
                    writer_.encoder().lidar_scan_encoder().encodeField(
                        field, px_offset);
                fields_data[item_idx].swap(data);
            }
        }
    };

#ifndef OUSTER_OSF_NO_THREADING
    unsigned int con_num = std::thread::hardware_concurrency();
    // looking for at least 4 cores if can't determine
    if (!con_num) con_num = 4;
    std::vector<std::future<void>> futures;
    size_t field_count = queue.size();
    size_t num_threads = std::min<size_t>(field_count, con_num);
    for (size_t i = 0; i < num_threads; i++) {
        futures.emplace_back(std::async(thread_fn));
    }

    for (auto& t : futures) {
        t.get();
    }
#else
    thread_fn();
#endif
    return fields_data;
}

// TWS 20240301 TODO: determine if we can deduplicate this code (see
// scanEncodeFields)
void scanDecode(LidarScan& lidar_scan, const ScanData& scan_data,
                const std::vector<int>& px_offset,
                const ouster::LidarScanFieldTypes& field_types,
                const std::vector<Field*>& custom_fields,
                const std::vector<std::vector<uint8_t>>& custom_fields_data,
                const error_handler_t& error_handler) {
    std::mutex queue_mutex;
    std::deque<int> queue;
    for (int i = 0; i < (int)field_types.size(); i++) {
        // only decode fields in the destination lidar scan
        if (lidar_scan.has_field(field_types[i].name)) {
            queue.push_back(i);
        }
    }
    for (int i = 0; i < (int)custom_fields.size(); i++) {
        queue.push_back(-i - 1);
    }

    std::mutex error_vector_mut;
    std::vector<std::pair<ouster::core::Severity, std::string>> errors;
    auto thread_fn = [&]() {
        while (true) {
            std::unique_lock<std::mutex> lock(queue_mutex);
            // if no items left, just exit
            if (queue.empty()) {
                return;
            }
            auto item_idx = queue.front();
            queue.pop_front();
            lock.unlock();

            try {
                if (item_idx < 0) {
                    item_idx = -item_idx - 1;
                    decodeField(*custom_fields[item_idx],
                                custom_fields_data[item_idx]);
                } else {
                    auto& field = lidar_scan.field(field_types[item_idx].name);
                    decodeField(field, scan_data[item_idx], px_offset);
                }
            } catch (const std::runtime_error& error) {
                std::unique_lock<std::mutex> lock(error_vector_mut);
                errors.push_back({Severity::OUSTER_WARNING, error.what()});
            }
        }
    };

#ifndef OUSTER_OSF_NO_THREADING
    unsigned int con_num = std::thread::hardware_concurrency();
    // looking for at least 4 cores if can't determine
    if (!con_num) con_num = 4;
    std::vector<std::future<void>> futures;
    size_t field_count = queue.size();
    size_t num_threads = std::min<size_t>(field_count, con_num);
    for (size_t i = 0; i < num_threads; i++) {
        futures.emplace_back(std::async(thread_fn));
    }

    for (auto& t : futures) {
        t.get();
    }
#else
    thread_fn();
#endif

    for (auto& err : errors) {
        error_handler(err.first, err.second);
    }
}

flatbuffers::Offset<gen::LidarScanMsg> LidarScanStream::create_lidar_scan_msg(
    flatbuffers::FlatBufferBuilder& fbb, const LidarScan& lidar_scan,
    const ouster::sensor::sensor_info& info,
    const ouster::LidarScanFieldTypes meta_field_types) const {
    const auto& ls = lidar_scan;

    // Prepare field_types for LidarScanMsg
    std::vector<std::pair<ouster::osf::gen::CHAN_FIELD, std::string>>
        standard_fields_to_sort;
    std::vector<std::pair<std::string, const Field*>> custom_field_names;
    for (const auto& f : ls.fields()) {
        // if we have meta_field_types ignore fields not in it
        if (meta_field_types.size()) {
            bool found = false;
            for (const auto& mft : meta_field_types) {
                if (mft.name == f.first) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                continue;
            }
        }
        // if the field is custom, add it as custom
        auto enum_value = to_osf_enum(f.first);  // optional
        if (!enum_value) {
            // todo maybe do single fields in this thread
            custom_field_names.emplace_back(f.first, &f.second);
        } else {
            // otherwise add it to field types and then scan encode those
            standard_fields_to_sort.emplace_back(enum_value.value(), f.first);
        }
    }

    // because older OSF readers are broken, we need to sort the standard fields
    // by value...
    std::sort(standard_fields_to_sort.begin(), standard_fields_to_sort.end());

    // now actually build our arrays from the sorted one
    std::vector<ouster::osf::gen::ChannelField> field_types;
    ouster::LidarScanFieldTypes standard_fields;
    for (const auto& f : standard_fields_to_sort) {
        const auto& field = ls.field(f.second);
        field_types.push_back(
            ouster::osf::gen::ChannelField(f.first, to_osf_enum(field.tag())));
        standard_fields.push_back({f.second, field.tag()});
    }

    // error if we are missing a required field
    for (const auto& mft : meta_field_types) {
        if (ls.fields().find(mft.name) == ls.fields().end()) {
            throw std::invalid_argument("Required field '" + mft.name +
                                        "' is missing from scan.");
        }
    }

    // Encode LidarScan to PNG buffers
    ScanData custom_data;
    ScanData scan_data =
        scanEncode(ls, info.format.pixel_shift_by_row, standard_fields,
                   custom_field_names, custom_data);
    // Prepare PNG encoded channels for LidarScanMsg.channels vector
    std::vector<flatbuffers::Offset<gen::ChannelData>> channels;
    for (const auto& channel_data : scan_data) {
        channels.emplace_back(gen::CreateChannelDataDirect(fbb, &channel_data));
    }

    // Copy in encoded custom fields
    std::vector<flatbuffers::Offset<gen::Field>> custom_fields;
    for (size_t i = 0; i < custom_data.size(); i++) {
        const auto& f = *custom_field_names[i].second;
        const auto& name = custom_field_names[i].first;
        std::vector<uint64_t> shape{f.shape().begin(), f.shape().end()};
        custom_fields.push_back(gen::CreateFieldDirect(
            fbb, name.c_str(), to_osf_enum(f.tag()), &shape,
            to_osf_enum(f.field_class()), &custom_data[i], f.bytes()));
    }
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<gen::Field>>>
        custom_fields_off = 0;
    if (custom_fields.size()) {
        custom_fields_off =
            fbb.CreateVector<flatbuffers::Offset<gen::Field>>(custom_fields);
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
        pose_off = fbb.CreateVector<double>(ls.pose().get<double>(),
                                            ls.pose().bytes() / sizeof(double));
    }
    auto packet_timestamp_id_off = fbb.CreateVector<uint64_t>(
        ls.packet_timestamp().data(), ls.packet_timestamp().size());

    auto alert_flags_off = fbb.CreateVector<uint8_t>(ls.alert_flags().data(),
                                                     ls.alert_flags().size());

    return gen::CreateLidarScanMsg(
        fbb, channels_off, field_types_off, timestamp_off, measurement_id_off,
        status_off, ls.frame_id, pose_off, packet_timestamp_id_off,
        custom_fields_off, ls.frame_status, ls.shutdown_countdown,
        ls.shot_limiting_countdown, alert_flags_off);
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
std::unique_ptr<ouster::LidarScan> restore_lidar_scan(
    const MessageRef& msg, const ouster::sensor::sensor_info& info,
    const nonstd::optional<std::vector<std::string>>& fields_to_decode) {
    const auto& buf = msg.buffer();
    auto ls_msg =
        flatbuffers::GetSizePrefixedRoot<ouster::osf::gen::LidarScanMsg>(
            buf.data());

    uint32_t width = info.format.columns_per_frame;
    uint32_t height = info.format.pixels_per_column;

    // read field_types
    ouster::LidarScanFieldTypes field_types;
    if (ls_msg->field_types() && ls_msg->field_types()->size()) {
        for (auto it = ls_msg->field_types()->begin();
             it != ls_msg->field_types()->end(); it++) {
            auto t = FieldType{from_osf_enum(it->chan_field()),
                               from_osf_enum(it->chan_field_type()),
                               {},
                               FieldClass::PIXEL_FIELD};
            field_types.push_back(t);
        }
    }

    // Init lidar scan with recovered fields
    ouster::LidarScanFieldTypes desired_field_types =
        !fields_to_decode.has_value() ? field_types
                                      : ouster::LidarScanFieldTypes();
    if (fields_to_decode) {
        for (const auto& f : fields_to_decode.value()) {
            for (const auto& ft : field_types) {
                if (ft.name == f) {
                    desired_field_types.push_back(ft);
                    break;
                }
            }
        }
    }
    auto ls = std::make_unique<LidarScan>(
        width, height, desired_field_types.begin(), desired_field_types.end(),
        info.format.columns_per_packet);

    // set frame status - unfortunately since this is a new field in the FB
    // schema and since LidarScan::frame_status is an integer we have no way to
    // differentiate between whether the value was zero when written or wasn't
    // provided by the writer.
    ls->frame_status = ls_msg->frame_status();
    ls->shutdown_countdown = ls_msg->shutdown_countdown();
    ls->shot_limiting_countdown = ls_msg->shot_limiting_countdown();

    ls->frame_id = ls_msg->frame_id();

    // Set timestamps per column
    auto msg_ts_vec = ls_msg->header_timestamp();
    if (msg_ts_vec) {
        if (static_cast<uint32_t>(ls->w) == msg_ts_vec->size()) {
            for (uint32_t i = 0; i < width; ++i) {
                ls->timestamp()[i] = msg_ts_vec->Get(i);
            }
        } else if (msg_ts_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has "
                "header_timestamp of length: "
                "{}, expected: {}",
                msg_ts_vec->size(), ls->w);
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
            logger().error(
                "ERROR: LidarScanMsg has "
                "header_measurement_id of length: "
                "{}, expected: {}",
                msg_mid_vec->size(), ls->w);
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
            logger().error(
                "ERROR: LidarScanMsg has "
                "header_status of length: "
                "{}, expected: {}",
                msg_status_vec->size(), ls->w);
            return nullptr;
        }
    }

    // Set poses per column
    auto pose_vec = ls_msg->pose();
    if (pose_vec) {
        auto& pose = ls->pose();
        if (static_cast<uint32_t>(pose.size()) == pose_vec->size()) {
            std::memcpy(pose.get<double>(), pose_vec->Data(), pose.bytes());
        } else if (pose_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has "
                "pose of length: "
                "{}, expected: {}",
                pose_vec->size(), pose.size());
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
            logger().error(
                "ERROR: LidarScanMsg has "
                "packet_timestamp of length: "
                "{}, expected: {}",
                packet_ts_vec->size(), ls->packet_timestamp().size());
            return nullptr;
        }
    }

    // Set alert flags per lidar packet
    auto alert_flags_vec = ls_msg->alert_flags();
    if (alert_flags_vec) {
        if (static_cast<size_t>(ls->alert_flags().size()) ==
            alert_flags_vec->size()) {
            for (size_t i = 0; i < alert_flags_vec->size(); ++i) {
                ls->alert_flags()[i] = alert_flags_vec->Get(i);
            }
        } else if (alert_flags_vec->size() != 0) {
            logger().error(
                "ERROR: LidarScanMsg has "
                "alert_flags of length: "
                "{}, expected: {}",
                alert_flags_vec->size(), ls->alert_flags().size());
            return nullptr;
        }
    }

    // Fill Scan Data with scan channels
    auto msg_scan_vec = ls_msg->channels();
    if (!msg_scan_vec) {
        logger().error(
            "ERROR: lidar_scan msg doesn't "
            "have scan fields.");
        return nullptr;
    }
    // todo, remove this extra copy of the encoded data into scan_data
    // we can just reference it directly instead
    ScanData scan_data;
    for (uint32_t i = 0; i < msg_scan_vec->size(); ++i) {
        auto channel_buffer = msg_scan_vec->Get(i)->buffer();
        scan_data.emplace_back(channel_buffer->begin(), channel_buffer->end());
    }

    // Decode PNGs data to LidarScan
    std::vector<Field*> custom_fields;
    std::vector<std::vector<uint8_t>> custom_fields_data;

    auto msg_custom_fields = ls_msg->custom_fields();
    if (msg_custom_fields && msg_custom_fields->size()) {
        for (uint32_t i = 0; i < msg_custom_fields->size(); ++i) {
            auto custom_field = msg_custom_fields->Get(i);

            std::string name{custom_field->name()->c_str()};
            if (fields_to_decode) {
                bool found = false;
                for (const auto& f : fields_to_decode.value()) {
                    if (f == name) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    continue;
                }
            }
            ChanFieldType tag = from_osf_enum(custom_field->tag());
            std::vector<size_t> shape{custom_field->shape()->begin(),
                                      custom_field->shape()->end()};
            auto desc = FieldDescriptor::array(tag, shape);
            ouster::FieldClass field_class =
                from_osf_enum(custom_field->field_class());
            auto& field = ls->add_field(name, desc, field_class);

            std::vector<uint8_t> encoded{custom_field->data()->begin(),
                                         custom_field->data()->end()};
            custom_fields_data.emplace_back(custom_field->data()->begin(),
                                            custom_field->data()->end());
            custom_fields.push_back(&field);
        }
    }

    scanDecode(*ls, scan_data, info.format.pixel_shift_by_row, field_types,
               custom_fields, custom_fields_data, msg.error_handler());

    // error if any of the requested fields did not end up in the lidar scan
    if (fields_to_decode) {
        for (const auto& field : fields_to_decode.value()) {
            if (!ls->has_field(field)) {
                throw std::runtime_error("Requested field '" + field +
                                         "' does not exist in OSF.");
            }
        }
    }

    return ls;
}

LidarScanStreamMeta::LidarScanStreamMeta(
    const uint32_t sensor_meta_id,
    const ouster::LidarScanFieldTypes field_types)
    : sensor_meta_id_{sensor_meta_id},
      field_types_{field_types.begin(), field_types.end()} {}

uint32_t LidarScanStreamMeta::sensor_meta_id() const { return sensor_meta_id_; }

std::vector<uint8_t> LidarScanStreamMeta::buffer() const {
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(512);

    // Make and store field_types with details for LidarScanStream
    std::vector<ouster::osf::gen::ChannelField> field_types;
    for (const auto& ft : field_types_) {
        auto field_enum = to_osf_enum(ft.name);
        if (field_enum) {
            field_types.emplace_back(field_enum.value(),
                                     to_osf_enum(ft.element_type));
        }
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
    ouster::LidarScanFieldTypes field_types;
    if (field_types_vec && field_types_vec->size()) {
        std::transform(
            field_types_vec->begin(), field_types_vec->end(),
            std::back_inserter(field_types), [](const gen::ChannelField* p) {
                return ouster::FieldType{from_osf_enum(p->chan_field()),
                                         from_osf_enum(p->chan_field_type()),
                                         {},
                                         FieldClass::PIXEL_FIELD};
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
        ss << f.name << ":" << ouster::sensor::to_string(f.element_type);
        first = false;
    }
    ss << "}";
    return ss.str();
}

// ============== LidarScan Stream ops ===========================

LidarScanStream::LidarScanStream(Token /*key*/, Writer& writer,
                                 const uint32_t sensor_meta_id,
                                 const ouster::LidarScanFieldTypes& field_types)
    : writer_{writer},
      meta_(sensor_meta_id, field_types),
      sensor_meta_id_(sensor_meta_id),
      field_types_(field_types) {
    // Note key is ignored and just used to gatekeep.
    // Check sensor and get sensor_info
    auto sensor_meta_entry = writer.get_metadata<LidarSensor>(sensor_meta_id_);
    if (sensor_meta_entry == nullptr) {
        std::stringstream ss;
        ss << "ERROR: can't find sensor_meta_id = " << sensor_meta_id;
        throw std::logic_error(ss.str());
    }

    sensor_info_ = sensor_meta_entry->info();

    stream_meta_id_ = writer_.add_metadata(meta_);
}

// TODO[pb]: Every save func in Streams is uniform, need to nicely extract
// it and remove close dependence on Writer? ...
void LidarScanStream::save(const ouster::osf::ts_t receive_ts,
                           const ouster::osf::ts_t sensor_ts,
                           const LidarScan& lidar_scan) {
    const auto& msg_buf = make_msg(lidar_scan);
    writer_.save_message(meta_.id(), receive_ts, sensor_ts, msg_buf);
}

std::vector<uint8_t> LidarScanStream::make_msg(const LidarScan& lidar_scan) {
    if (lidar_scan.w != sensor_info_.w() || lidar_scan.h != sensor_info_.h()) {
        std::stringstream exception_msg_stream;
        exception_msg_stream
            << "lidar scan size (" << lidar_scan.w << ", " << lidar_scan.h
            << ") does not match the sensor info resolution ("
            << sensor_info_.w() << ", " << sensor_info_.h() << ")";
        throw std::invalid_argument(exception_msg_stream.str());
    }
    flatbuffers::FlatBufferBuilder fbb = flatbuffers::FlatBufferBuilder(32768);
    auto ls_msg_offset =
        create_lidar_scan_msg(fbb, lidar_scan, sensor_info_, field_types_);
    fbb.FinishSizePrefixed(ls_msg_offset);
    const uint8_t* buf = fbb.GetBufferPointer();
    const size_t size = fbb.GetSize();
    return {buf, buf + size};
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
}  // namespace ouster
