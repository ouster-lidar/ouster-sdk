/**
 * Copyright(c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#include "fb_common.h"

#include <future>
#include <stdexcept>
#include <thread>

#include "ouster/core/types.h"
#include "ouster/osf/impl/png_tools.h"

using namespace ouster::sdk::core;
using nonstd::make_optional;
using nonstd::nullopt;
using nonstd::optional;

using ouster::sdk::core::Severity;

#ifndef OUSTER_OSF_NO_THREADING
#include "ouster/core/impl/threadpool.h"  //NOLINT

// Create the threadpool in a local static since Windows does not appear
// to allow creating threads in global static initializers (without locking up)
namespace {
std::mutex threadpool_mutex;
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

impl::gen::FIELD_CLASS to_osf_enum(FieldClass field_class) {
    switch (field_class) {
        case FieldClass::PIXEL_FIELD:
            return impl::gen::FIELD_CLASS::PIXEL_FIELD;
        case FieldClass::COLUMN_FIELD:
            return impl::gen::FIELD_CLASS::COLUMN_FIELD;
        case FieldClass::PACKET_FIELD:
            return impl::gen::FIELD_CLASS::PACKET_FIELD;
        case FieldClass::FRAME_FIELD:
            return impl::gen::FIELD_CLASS::SCAN_FIELD;
        case FieldClass::COLLATION_FIELD:
            return impl::gen::FIELD_CLASS::COLLATION_FIELD;
        default:
            throw std::runtime_error("Unhandled or invalid FieldClass.");
    }
}

ouster::sdk::core::FieldClass from_osf_enum(impl::gen::FIELD_CLASS field_class) {
    switch (field_class) {
        case impl::gen::FIELD_CLASS::PIXEL_FIELD:
            return FieldClass::PIXEL_FIELD;
        case impl::gen::FIELD_CLASS::COLUMN_FIELD:
            return FieldClass::COLUMN_FIELD;
        case impl::gen::FIELD_CLASS::PACKET_FIELD:
            return FieldClass::PACKET_FIELD;
        case impl::gen::FIELD_CLASS::SCAN_FIELD:
            return FieldClass::FRAME_FIELD;
        case impl::gen::FIELD_CLASS::COLLATION_FIELD:
            return FieldClass::COLLATION_FIELD;
        default:
            throw std::runtime_error("Unhandled or invalid FieldClass in OSF.");
    }
}

namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

}  // namespace impl

namespace {

template <typename K, typename V, size_t N>
optional<V> lookup(const impl::Table<K, V, N> table, const K& key) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end,
                            [&](const std::pair<K, V>& pair) { return pair.first == key; });

    return res == end ? nullopt : make_optional<V>(res->second);
}

template <typename K, size_t N>
optional<K> rlookup(const impl::Table<K, const char*, N> table, const char* value) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end, [&](const std::pair<K, const char*>& pair) {
        return pair.second && std::strcmp(pair.second, value) == 0;
    });

    return res == end ? nullopt : make_optional<K>(res->first);
}

// mapping of channel name to osf ChanField
impl::Table<ouster::sdk::osf::impl::gen::CHAN_FIELD, const char*, 30> chanfield_strings{{
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::UNKNOWN, "UNKNOWN"},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RANGE, ChanField::RANGE},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RANGE2, ChanField::RANGE2},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::SIGNAL, ChanField::SIGNAL},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::SIGNAL2, ChanField::SIGNAL2},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::REFLECTIVITY, ChanField::REFLECTIVITY},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::REFLECTIVITY2, ChanField::REFLECTIVITY2},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::NEAR_IR, ChanField::NEAR_IR},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::FLAGS, ChanField::FLAGS},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::FLAGS2, ChanField::FLAGS2},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW_HEADERS, ChanField::RAW_HEADERS},
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
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD1, ChanField::RAW32_WORD1},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD2, ChanField::RAW32_WORD2},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD3, ChanField::RAW32_WORD3},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD4, ChanField::RAW32_WORD4},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD5, ChanField::RAW32_WORD5},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD6, ChanField::RAW32_WORD6},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD7, ChanField::RAW32_WORD7},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD8, ChanField::RAW32_WORD8},
    {ouster::sdk::osf::impl::gen::CHAN_FIELD::RAW32_WORD9, ChanField::RAW32_WORD9},
}};

}  // namespace

nonstd::optional<impl::gen::CHAN_FIELD> to_osf_enum(const std::string& field_name) {
    return rlookup(chanfield_strings, field_name.c_str());
}

std::string from_osf_enum(impl::gen::CHAN_FIELD field) {
    return lookup(chanfield_strings, field).value();
}

template <typename FieldsContainer, typename GetField>
std::vector<FrameChannelData> encode_fields(const LidarFrameEncoder& encoder,
                                            const FieldsContainer& fields,
                                            const GetField& get_field,
                                            const std::vector<int>& px_offset) {
    std::vector<FrameChannelData> fields_data(fields.size());

#ifndef OUSTER_OSF_NO_THREADING
    auto& threadpool = get_threadpool();
    std::vector<std::future<void>> futures;
    futures.reserve(fields_data.size());

    for (size_t i = 0; i < fields.size(); ++i) {
        const Field& field = get_field(fields[i]);

        futures.push_back(threadpool.enqueue([&, i]() {
            FrameChannelData data = encoder.encode_field(field, px_offset);
            fields_data[i].swap(data);
        }));
    }

    for (auto& future : futures) {
        future.get();
    }
#else
    for (size_t i = 0; i < fields.size(); ++i) {
        const Field& field = get_field(fields[i]);

        FrameChannelData data = encoder.encode_field(field, px_offset);
        fields_data[i].swap(data);
    }
#endif

    return fields_data;
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>> fb_save_fields(
    flatbuffers::FlatBufferBuilder& fbb, const LidarFrameEncoder& encoder,
    const std::vector<std::pair<std::string, const Field*>>& fields) {
    if (fields.empty()) {
        return 0;
    }

    /**
     * NOTE: we must specify that return type is `const Field&` otherwise
     *       compiler does silly things
     */
    auto get_field = [](const std::pair<std::string, const Field*>& pair) -> const Field& {
        return *pair.second;
    };

    std::vector<FrameChannelData> fields_data = encode_fields(encoder, fields, get_field, {});

    std::vector<flatbuffers::Offset<impl::gen::Field>> offs;
    for (size_t i = 0; i < fields.size(); ++i) {
        const std::string& name = fields[i].first;
        const Field& field = *fields[i].second;
        std::vector<uint64_t> shape{field.shape().begin(), field.shape().end()};

        offs.push_back(impl::gen::CreateFieldDirect(fbb, name.c_str(), to_osf_enum(field.tag()),
                                                    &shape, to_osf_enum(field.field_class()),
                                                    &fields_data[i], field.bytes()));
    }

    return fbb.CreateVector<flatbuffers::Offset<impl::gen::Field>>(offs);
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::Field>>> fb_save_fields(
    flatbuffers::FlatBufferBuilder& fbb, const LidarFrameEncoder& encoder,
    const std::unordered_map<std::string, Field>& fields) {
    std::vector<std::pair<std::string, const Field*>> fields_vec;
    for (const auto& field : fields) {
        fields_vec.emplace_back(field.first, &field.second);
    }
    return fb_save_fields(fbb, encoder, fields_vec);
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::ChannelData>>>
fb_save_frame_channels(flatbuffers::FlatBufferBuilder& fbb, const LidarFrameEncoder& encoder,
                       const LidarFrame& frame, const LidarFrameFieldTypes& field_types,
                       const std::vector<int>& px_offset) {
    auto get_field = [&frame](const FieldType& field_type) -> const Field& {
        return frame.field(field_type.name);
    };

    std::vector<FrameChannelData> channels_data =
        encode_fields(encoder, field_types, get_field, px_offset);

    std::vector<flatbuffers::Offset<impl::gen::ChannelData>> channels;
    for (const auto& channel_data : channels_data) {
        channels.emplace_back(impl::gen::CreateChannelDataDirect(fbb, &channel_data));
    }

    return fbb.CreateVector<flatbuffers::Offset<impl::gen::ChannelData>>(channels);
}

void fb_restore_fields(
    const flatbuffers::Vector<flatbuffers::Offset<ouster::sdk::osf::v2::Field>>* fb_fields,
    const nonstd::optional<std::vector<std::string>>& fields_to_decode, const AddFieldFn& add_field,
    const core::error_handler_t& error_handler) {
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

        std::vector<size_t> shape{fb_field->shape()->begin(), fb_field->shape()->end()};

        // Skip fields with unsupported types (e.g., from a newer SDK version)
        FieldDescriptor desc;
        try {
            desc = FieldDescriptor::array(tag, shape);
        } catch (const std::invalid_argument&) {
            error_handler(Severity::OUSTER_WARNING,
                          "Skipping field '" + name + "' with unsupported type (tag=" +
                              std::to_string(static_cast<int>(tag)) + ")");
            continue;
        }
        ouster::sdk::core::FieldClass field_class = from_osf_enum(fb_field->field_class());

        auto& field = add_field(name, desc, field_class);

        impl::EncodedFrameChannelData data;
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
    const flatbuffers::Vector<flatbuffers::Offset<impl::gen::ChannelData>>* fb_channels,
    const LidarFrameFieldTypes& field_types, const std::vector<int>& px_offset, LidarFrame& frame,
    const core::error_handler_t& error_handler) {
    // TODO[tws] consider using std::future's exception propagation, deduplicate
    // the try/catch logic
    std::vector<std::pair<Severity, std::string>> errors;

#ifndef OUSTER_OSF_NO_THREADING
    std::mutex error_vector_mut;
    auto& threadpool = get_threadpool();
    std::vector<std::future<void>> futures;
#endif

    for (size_t i = 0; i < field_types.size(); i++) {
        // only decode fields in the destination lidar frame
        if (!frame.has_field(field_types[i].name)) {
            continue;
        }

        auto channel_buffer = fb_channels->Get(i)->buffer();

        impl::EncodedFrameChannelData data;
        data.data_internal = channel_buffer->data();
        data.size_internal = channel_buffer->size();

        Field& field = frame.field(field_types[i].name);

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

flatbuffers::Offset<impl::gen::Pose> fb_save_pose(flatbuffers::FlatBufferBuilder& fbb,
                                                  const core::Pose& pose) {
    const Eigen::Vector3d& pos = pose.position();
    const Eigen::Quaterniond& rot = pose.rotation();
    std::vector<double> position{pos(0), pos(1), pos(2)};
    std::vector<double> rotation{rot.w(), rot.x(), rot.y(), rot.z()};
    return impl::gen::CreatePoseDirect(fbb, &position, &rotation);
}

bool fb_restore_pose(const impl::gen::Pose* fb_pose, core::Pose& pose,
                     std::vector<std::pair<Severity, std::string>>& errors,
                     const std::string& context) {
    if (fb_pose == nullptr) {
        errors.emplace_back(Severity::OUSTER_WARNING, "Deserializing " + context + " with no pose");
        return false;
    }

    const auto* position = fb_pose->position();
    if (position == nullptr || position->size() != 3) {
        errors.emplace_back(Severity::OUSTER_WARNING,
                            "Deserializing " + context + " with invalid pose position");
        return false;
    }

    const auto* rotation = fb_pose->rotation();
    if (rotation == nullptr || rotation->size() != 4) {
        errors.emplace_back(Severity::OUSTER_WARNING,
                            "Deserializing " + context + " with invalid pose rotation");
        return false;
    }

    pose.set_position({position->Get(0), position->Get(1), position->Get(2)});
    pose.set_rotation(
        Eigen::Quaterniond(rotation->Get(0), rotation->Get(1), rotation->Get(2), rotation->Get(3)));
    return true;
}

flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<impl::gen::ObjectList>>>
fb_save_object_lists(
    flatbuffers::FlatBufferBuilder& fbb,
    const std::unordered_map<std::string, std::vector<core::Object>>& object_lists) {
    if (object_lists.empty()) {
        return 0;
    }

    std::vector<flatbuffers::Offset<impl::gen::ObjectList>> offs;
    for (const auto& pair : object_lists) {
        const std::string& name = pair.first;
        const std::vector<core::Object>& objects = pair.second;

        std::vector<flatbuffers::Offset<impl::gen::Object>> obj_offsets;
        for (const auto& obj : objects) {
            const flatbuffers::Offset<impl::gen::Pose> object_to_body_offset =
                fb_save_pose(fbb, obj.object_to_body);
            const flatbuffers::Offset<impl::gen::Pose> body_to_world_offset =
                fb_save_pose(fbb, obj.body_to_world);
            std::vector<float> velocity{obj.velocity(0), obj.velocity(1), obj.velocity(2)};
            std::vector<float> dimensions{obj.dimensions(0), obj.dimensions(1), obj.dimensions(2)};

            auto index = 0;
            std::vector<flatbuffers::Offset<impl::gen::Property>> properties(obj.properties.size());
            for (auto it = obj.properties.begin(); it != obj.properties.end(); ++it, ++index) {
                properties[index] =
                    impl::gen::CreatePropertyDirect(fbb, it->first.c_str(), it->second.c_str());
            }

            obj_offsets.push_back(impl::gen::CreateObjectDirect(
                fbb, obj.id, obj.creation_ts, obj.class_id, obj.class_confidence,
                object_to_body_offset, body_to_world_offset, &velocity, &dimensions, &properties,
                obj.timestamp));
        }

        offs.push_back(impl::gen::CreateObjectListDirect(fbb, name.c_str(), &obj_offsets));
    }

    return fbb.CreateVector<flatbuffers::Offset<impl::gen::ObjectList>>(offs);
}

// BEGINNOLINT(readability-function-cognitive-complexity)
void fb_restore_object_lists(
    const flatbuffers::Vector<flatbuffers::Offset<impl::gen::ObjectList>>* fb_obj_lists,
    std::unordered_map<std::string, std::vector<core::Object>>& object_lists,
    const core::error_handler_t& error_handler) {
    if (fb_obj_lists == nullptr) {
        return;
    }

    std::vector<std::pair<Severity, std::string>> errors;

    for (auto&& obj_list_offset : *fb_obj_lists) {
        if (obj_list_offset->name() == nullptr) {
            errors.emplace_back(Severity::OUSTER_ERROR, "Deserializing object list with no name");
            continue;
        }

        std::string name = obj_list_offset->name()->str();

        if (object_lists.find(name) != object_lists.end()) {
            errors.emplace_back(Severity::OUSTER_ERROR,
                                "Name collision during object list deserialization"
                                "[name: " +
                                    name + "]");
            continue;
        }

        if (obj_list_offset->objects() == nullptr) {
            errors.emplace_back(Severity::OUSTER_ERROR,
                                "Deserializing object list with no objects"
                                "[name: " +
                                    name + "]");
            continue;
        }

        std::vector<core::Object> objects;
        objects.reserve(obj_list_offset->objects()->size());

        for (auto&& obj_offset : *(obj_list_offset->objects())) {
            core::Object obj;
            obj.id = obj_offset->id();
            obj.creation_ts = obj_offset->creation_ts();
            obj.timestamp = obj_offset->timestamp();
            obj.class_id = obj_offset->class_id();
            obj.class_confidence = obj_offset->class_confidence();

            const std::string object_context = "object [id: " + std::to_string(obj.id) + "]";
            fb_restore_pose(obj_offset->object_to_body(), obj.object_to_body, errors,
                            object_context + " object_to_body");
            fb_restore_pose(obj_offset->body_to_world(), obj.body_to_world, errors,
                            object_context + " body_to_world");

            if (obj_offset->velocity() != nullptr) {
                obj.velocity = {obj_offset->velocity()->Get(0), obj_offset->velocity()->Get(1),
                                obj_offset->velocity()->Get(2)};
            } else {
                errors.emplace_back(Severity::OUSTER_WARNING,
                                    "Deserializing object with no velocity"
                                    " [id: " +
                                        std::to_string(obj.id) + "]");
            }

            if (obj_offset->dimensions() != nullptr) {
                obj.dimensions = {obj_offset->dimensions()->Get(0),
                                  obj_offset->dimensions()->Get(1),
                                  obj_offset->dimensions()->Get(2)};
            } else {
                errors.emplace_back(Severity::OUSTER_WARNING,
                                    "Deserializing object with no dimensions"
                                    " [id: " +
                                        std::to_string(obj.id) + "]");
            }

            if (obj_offset->properties() != nullptr) {
                for (auto&& prop_offset : *(obj_offset->properties())) {
                    if (prop_offset->name() == nullptr || prop_offset->data() == nullptr) {
                        continue;
                    }

                    std::string name = prop_offset->name()->str();
                    std::string data = prop_offset->data()->str();

                    if (obj.properties.find(name) != obj.properties.end()) {
                        errors.emplace_back(Severity::OUSTER_WARNING,
                                            "Deserializing object with repeated property"
                                            " [id: " +
                                                std::to_string(obj.id) +
                                                "] "
                                                " [prop name: " +
                                                name + "]");
                        continue;
                    }

                    obj.properties[name] = data;
                }
            }

            objects.push_back(obj);
        }

        object_lists[name] = objects;
    }

    for (auto& err : errors) {
        error_handler(err.first, err.second);
    }
}
// ENDNOLINT(readability-function-cognitive-complexity)

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
