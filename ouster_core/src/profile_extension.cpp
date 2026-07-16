/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/profile_extension.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/types.h"

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

namespace ouster {
namespace sdk {
namespace core {

namespace impl {

// definition copied from lidar_frame.cpp
struct DefaultFieldsEntry {
    const std::pair<std::string, ChanFieldType>* fields;
    size_t n_fields;
};

// lidar_frame.cpp
extern Table<UDPProfileLidar, DefaultFieldsEntry, MAX_NUM_PROFILES> default_frame_fields;

namespace {

void extend_default_frame_fields(
    UDPProfileLidar profile,
    const std::vector<std::pair<std::string, ChanFieldType>>& frame_fields) {
    auto end = impl::default_frame_fields.end();
    auto it = std::find_if(impl::default_frame_fields.begin(), end, [](const auto& key_value) {
        return key_value.first == UDPProfileLidar::UNKNOWN;
    });

    if (it == end) {
        throw std::runtime_error("Limit of frame fields has been reached");
    }

    *it = {profile, {frame_fields.data(), frame_fields.size()}};
}

}  // namespace

struct ExtendedProfile {
    UDPProfileLidar profile;
    std::string name;
    std::vector<std::pair<std::string, ChanFieldType>> slots;
    std::vector<std::pair<std::string, FieldDecodeInfo>> fields;
    size_t chan_data_size;
};

/**
 * Storage container for dynamically added profiles
 *
 * used with std::list because we need pointers to elements inside to stay valid
 * when new elements are added
 */
std::list<ExtendedProfile> extended_profiles_data{};

// definition copied from parsing.cpp
struct ProfileEntry {
    const std::pair<std::string, FieldDecodeInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

// types.cpp
extern Table<UDPProfileLidar, const char*, MAX_NUM_PROFILES> udp_profile_lidar_strings;
// parsing.cpp
extern Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES> profiles;

namespace {

void extend_profile_entries(UDPProfileLidar profile,
                            const std::vector<std::pair<std::string, FieldDecodeInfo>>& fields,
                            size_t chan_data_size) {
    auto end = impl::profiles.end();
    auto it = std::find_if(impl::profiles.begin(), end, [](const auto& key_value) {
        return key_value.first == UDPProfileLidar::UNKNOWN;
    });

    if (it == end) {
        throw std::runtime_error("Limit of parsing profiles has been reached");
    }

    *it = {profile, {fields.data(), fields.size(), chan_data_size}};
}

}  // namespace

}  // namespace impl

void extend_udp_profile_lidar_strings(UDPProfileLidar profile, const char* name) {
    auto begin = impl::udp_profile_lidar_strings.begin();
    auto end = impl::udp_profile_lidar_strings.end();

    if (end !=
        std::find_if(begin, end, [profile](const auto& prof) { return prof.first == profile; })) {
        throw std::invalid_argument("Lidar profile of given number already exists");
    }
    if (end != std::find_if(begin, end, [name](const auto& prof) {
            return prof.second && std::strcmp(prof.second, name) == 0;
        })) {
        throw std::invalid_argument("Lidar profile of given name already exists");
    }

    // Make sure to only check the string for zero intialization.
    // The profile enum can be 0 normally (and is for UNKNOWN).
    auto it = std::find_if(begin, end, [](const auto& key_value) { return key_value.second == 0; });

    if (it == end) {
        throw std::runtime_error("Limit of lidar profiles has been reached");
    }

    *it = std::make_pair(profile, name);
}

void add_custom_profile(int profile_nr,  // int as UDPProfileLidar
                        const std::string& name,
                        const std::vector<std::pair<std::string, FieldDecodeInfo>>& fields,
                        size_t chan_data_size) {
    if (profile_nr == 0) {
        throw std::invalid_argument("profile_nr of 0 are not allowed");
    }

    auto udp_profile = static_cast<UDPProfileLidar>(profile_nr);

    {
        // fill in profile
        impl::ExtendedProfile profile{udp_profile, name, {}, {}, chan_data_size};
        for (auto&& pair : fields) {
            profile.slots.emplace_back(pair.first, pair.second.ty_tag);
            FieldDecodeInfo field = pair.second;
            if (field.mask == 0) {
                field.mask = field_type_mask(field.ty_tag);
            }
            profile.fields.emplace_back(pair.first, field);
        }

        impl::extended_profiles_data.push_back(std::move(profile));
    }

    // retake reference to stored data, profile.name.c_str() can get invalidated
    // after move
    auto&& profile = impl::extended_profiles_data.back();

    extend_udp_profile_lidar_strings(profile.profile, profile.name.c_str());
    impl::extend_profile_entries(profile.profile, profile.fields, profile.chan_data_size);
    impl::extend_default_frame_fields(profile.profile, profile.slots);
}

UDPProfileLidar add_custom_profile(
    const std::string& name, const std::vector<std::pair<std::string, FieldDecodeInfo>>& fields,
    size_t chan_data_size) {
    // find a new enum value for the new profile
    int max = 0;
    for (const auto& item : impl::udp_profile_lidar_strings) {
        // make sure there's a slot and find the max value
        auto int_value = static_cast<int>(item.first);
        max = int_value > max ? int_value : max;
    }

    OUSTER_DIAGNOSTIC_PUSH
    OUSTER_DIAGNOSTIC_IGNORE_DEPRECATED

    auto profile_number = max + 1;
    add_custom_profile(profile_number, name, fields, chan_data_size);

    OUSTER_DIAGNOSTIC_POP
    return static_cast<UDPProfileLidar>(profile_number);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
