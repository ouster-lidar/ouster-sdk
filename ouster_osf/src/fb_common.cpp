/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#include "fb_common.h"

#include <future>
#include <thread>

#include "ouster/osf/impl/png_tools.h"
#include "ouster/types.h"

using namespace ouster::sdk::core;
using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;

using ouster::sdk::core::Severity;

#ifndef OUSTER_OSF_NO_THREADING
#include "ouster/impl/threadpool.h"  //NOLINT
// Create the threadpool in a local static since Windows does not appear
// to allow creating threads in global static initializers (without locking up)
static std::mutex threadpool_mutex;
namespace {
Threadpool<void>& get_threadpool() {
    std::lock_guard<std::mutex> lock(threadpool_mutex);
    static Threadpool<void> threadpool;
    return threadpool;
}
}  // namespace
#endif

namespace ouster {
namespace sdk {
namespace osf {

impl::gen::CHAN_FIELD_TYPE to_osf_enum(ChanFieldType field_type) {
    return static_cast<impl::gen::CHAN_FIELD_TYPE>(field_type);
}

ChanFieldType from_osf_enum(impl::gen::CHAN_FIELD_TYPE field_type) {
    return static_cast<ChanFieldType>(field_type);
}

impl::gen::FIELD_CLASS to_osf_enum(ouster::sdk::core::FieldClass field_class) {
    return static_cast<impl::gen::FIELD_CLASS>(field_class);
}

ouster::sdk::core::FieldClass from_osf_enum(
    impl::gen::FIELD_CLASS field_class) {
    return static_cast<ouster::sdk::core::FieldClass>(field_class);
}

namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

}  // namespace impl

template <typename K, typename V, size_t N>
static optional<V> lookup(const impl::Table<K, V, N> table, const K& key) {
    auto end = table.end();
    auto res = std::find_if(
        table.begin(), end,
        [&](const std::pair<K, V>& pair) { return pair.first == key; });

    return res == end ? nullopt : make_optional<V>(res->second);
}

template <typename K, size_t N>
static optional<K> rlookup(const impl::Table<K, const char*, N> table,
                           const char* value) {
    auto end = table.end();
    auto res = std::find_if(
        table.begin(), end, [&](const std::pair<K, const char*>& pair) {
            return pair.second && std::strcmp(pair.second, value) == 0;
        });

    return res == end ? nullopt : make_optional<K>(res->first);
}

// mapping of channel name to osf ChanField
static impl::Table<ouster::sdk::osf::impl::gen::CHAN_FIELD, const char*, 30>
    chanfield_strings{{
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::UNKNOWN, "UNKNOWN"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RANGE, ChanField::RANGE},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RANGE2, ChanField::RANGE2},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::SIGNAL, ChanField::SIGNAL},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::SIGNAL2, ChanField::SIGNAL2},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::REFLECTIVITY,
         ChanField::REFLECTIVITY},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::REFLECTIVITY2,
         ChanField::REFLECTIVITY2},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::NEAR_IR, ChanField::NEAR_IR},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::FLAGS, ChanField::FLAGS},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::FLAGS2, ChanField::FLAGS2},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW_HEADERS,
         ChanField::RAW_HEADERS},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM0, "CUSTOM0"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM1, "CUSTOM1"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM2, "CUSTOM2"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM3, "CUSTOM3"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM4, "CUSTOM4"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM5, "CUSTOM5"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM6, "CUSTOM6"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM7, "CUSTOM7"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM8, "CUSTOM8"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::CUSTOM9, "CUSTOM9"},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD1,
         ChanField::RAW32_WORD1},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD2,
         ChanField::RAW32_WORD2},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD3,
         ChanField::RAW32_WORD3},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD4,
         ChanField::RAW32_WORD4},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD5,
         ChanField::RAW32_WORD5},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD6,
         ChanField::RAW32_WORD6},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD7,
         ChanField::RAW32_WORD7},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD8,
         ChanField::RAW32_WORD8},
        {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD9,
         ChanField::RAW32_WORD9},
    }};

nonstd::optional<impl::gen::CHAN_FIELD> to_osf_enum(
    const std::string& field_name) {
    return rlookup(chanfield_strings, field_name.c_str());
}

std::string from_osf_enum(impl::gen::CHAN_FIELD field) {
    return lookup(chanfield_strings, field).value();
}

template <typename FieldsContainer, typename GetField>
std::vector<ScanChannelData> encode_fields(const LidarScanEncoder& encoder,
                                           const FieldsContainer& fields,
                                           const GetField& get_field,
                                           const std::vector<int>& px_offset) {
    std::vector<ScanChannelData> fields_data(fields.size());

#ifndef OUSTER_OSF_NO_THREADING
    auto& threadpool = get_threadpool();
    std::vector<std::future<void>> futures;
    futures.reserve(fields_data.size());

    for (size_t i = 0; i < fields.size(); ++i) {
        const Field& field = get_field(fields[i]);

        futures.push_back(threadpool.enqueue([&, i]() {
            ScanChannelData data = encoder.encode_field(field, px_offset);
            fields_data[i].swap(data);
        }));
    }

    for (auto& future : futures) {
        future.get();
    }
#else
    for (size_t i = 0; i < fields.size(); ++i) {
        const Field& field = get_field(fields[i]);

        ScanChannelData data = encoder.encode_field(field, px_offset);
        fields_data[i].swap(data);
    }
#endif

    return fields_data;
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>>
fb_save_fields(
    flatbuffers::FlatBufferBuilder& fbb, const LidarScanEncoder& encoder,
    const std::vector<std::pair<std::string, const Field*>>& fields) {
    if (fields.empty()) {
        return 0;
    }

    /**
     * NOTE: we must specify that return type is `const Field&` otherwise
     *       compiler does silly things
     */
    auto get_field =
        [](const std::pair<std::string, const Field*>& pair) -> const Field& {
        return *pair.second;
    };

    std::vector<ScanChannelData> fields_data =
        encode_fields(encoder, fields, get_field, {});

    std::vector<flatbuffers::Offset<impl::gen::Field>> offs;
    for (size_t i = 0; i < fields.size(); ++i) {
        const std::string& name = fields[i].first;
        const Field& field = *fields[i].second;
        std::vector<uint64_t> shape{field.shape().begin(), field.shape().end()};

        offs.push_back(impl::gen::CreateFieldDirect(
            fbb, name.c_str(), to_osf_enum(field.tag()), &shape,
            to_osf_enum(field.field_class()), &fields_data[i], field.bytes()));
    }

    return fbb.CreateVector<flatbuffers::Offset<impl::gen::Field>>(offs);
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>>
fb_save_fields(flatbuffers::FlatBufferBuilder& fbb,
               const LidarScanEncoder& encoder,
               const std::unordered_map<std::string, Field>& fields) {
    std::vector<std::pair<std::string, const Field*>> fields_vec;
    for (const auto& field : fields) {
        fields_vec.emplace_back(field.first, &field.second);
    }
    return fb_save_fields(fbb, encoder, fields_vec);
}

flatbuffers::Offset<
    flatbuffers::Vector<flatbuffers::Offset<impl::gen::ChannelData>>>
fb_save_scan_channels(flatbuffers::FlatBufferBuilder& fbb,
                      const LidarScanEncoder& encoder, const LidarScan& scan,
                      const LidarScanFieldTypes& field_types,
                      const std::vector<int>& px_offset) {
    auto get_field = [&scan](const FieldType& field_type) -> const Field& {
        return scan.field(field_type.name);
    };

    std::vector<ScanChannelData> channels_data =
        encode_fields(encoder, field_types, get_field, px_offset);

    std::vector<flatbuffers::Offset<impl::gen::ChannelData>> channels;
    for (const auto& channel_data : channels_data) {
        channels.emplace_back(
            impl::gen::CreateChannelDataDirect(fbb, &channel_data));
    }

    return fbb.CreateVector<flatbuffers::Offset<impl::gen::ChannelData>>(
        channels);
}

void fb_restore_fields(
    const flatbuffers::Vector<flatbuffers::Offset<ouster::sdk::osf::v2::Field>>*
        fb_fields,
    const nonstd::optional<std::vector<std::string>>& fields_to_decode,
    AddFieldFn add_field, const core::error_handler_t& error_handler) {
    if ((fb_fields == nullptr) || fb_fields->size() == 0u) {
        return;
    }

    // TODO[tws] consider using std::future's exception propagation, deduplicate
    // the try/catch logic
    std::vector<std::pair<Severity, std::string>> errors;

#ifndef OUSTER_OSF_NO_THREADING
    std::mutex error_vector_mut;
    auto& threadpool = get_threadpool();
    std::vector<std::future<void>> futures;
#endif

    for (auto fb_field : *fb_fields) {
        std::string name{fb_field->name()->c_str()};

        // TODO: switch to set
        if (fields_to_decode) {
            bool found = false;
            for (const auto& field_filter : fields_to_decode.value()) {
                if (field_filter == name) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                continue;
            }
        }
        ChanFieldType tag = from_osf_enum(fb_field->tag());
        std::vector<size_t> shape{fb_field->shape()->begin(),
                                  fb_field->shape()->end()};
        auto desc = FieldDescriptor::array(tag, shape);
        ouster::sdk::core::FieldClass field_class =
            from_osf_enum(fb_field->field_class());

        auto& field = add_field(name, desc, field_class);

        impl::EncodedScanChannelData data;
        data.data_internal = fb_field->data()->data();
        data.size_internal = fb_field->data()->size();
#ifndef OUSTER_OSF_NO_THREADING
        futures.push_back(threadpool.enqueue([&, data]() {
            try {
                impl::decode_field(field, data);
            } catch (const std::runtime_error& error) {
                std::unique_lock<std::mutex> lock(error_vector_mut);
                errors.emplace_back(Severity::OUSTER_WARNING, error.what());
            }
        }));
#else
        try {
            impl::decode_field(field, data);
        } catch (const std::runtime_error& error) {
            errors.push_back({Severity::OUSTER_WARNING, error.what()});
        }
#endif
    }

#ifndef OUSTER_OSF_NO_THREADING
    for (auto& item : futures) {
        // TODO[tws] consider using std::future's exception propagation
        item.get();
    }
#endif

    for (auto& err : errors) {
        error_handler(err.first, err.second);
    }
}

void fb_restore_channels(
    const flatbuffers::Vector<flatbuffers::Offset<impl::gen::ChannelData>>*
        fb_channels,
    const LidarScanFieldTypes& field_types, const std::vector<int>& px_offset,
    LidarScan& scan, const core::error_handler_t& error_handler) {
    // TODO[tws] consider using std::future's exception propagation, deduplicate
    // the try/catch logic
    std::vector<std::pair<Severity, std::string>> errors;

#ifndef OUSTER_OSF_NO_THREADING
    std::mutex error_vector_mut;
    auto& threadpool = get_threadpool();
    std::vector<std::future<void>> futures;
#endif

    for (size_t i = 0; i < field_types.size(); i++) {
        // only decode fields in the destination lidar scan
        if (!scan.has_field(field_types[i].name)) {
            continue;
        }

        auto channel_buffer = fb_channels->Get(i)->buffer();

        impl::EncodedScanChannelData data;
        data.data_internal = channel_buffer->data();
        data.size_internal = channel_buffer->size();

        Field& field = scan.field(field_types[i].name);

#ifndef OUSTER_OSF_NO_THREADING
        futures.push_back(threadpool.enqueue([&, data]() {
            try {
                impl::decode_field(field, data, px_offset);
            } catch (const std::runtime_error& error) {
                std::unique_lock<std::mutex> lock(error_vector_mut);
                errors.emplace_back(Severity::OUSTER_WARNING, error.what());
            }
        }));
#else
        try {
            impl::decode_field(field, data, px_offset);
        } catch (const std::runtime_error& error) {
            errors.push_back({Severity::OUSTER_WARNING, error.what()});
        }
#endif
    }

#ifndef OUSTER_OSF_NO_THREADING
    for (auto& item : futures) {
        // TODO[tws] consider using std::future's exception propagation
        item.get();
    }
#endif

    for (auto& err : errors) {
        error_handler(err.first, err.second);
    }
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
