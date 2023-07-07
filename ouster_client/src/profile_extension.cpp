/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/impl/profile_extension.h"

#include <algorithm>
#include <array>
#include <list>
#include <vector>

#include "ouster/types.h"

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;
using ouster::sensor::ChanField;
using ouster::sensor::ChanFieldType;
using ouster::sensor::UDPProfileLidar;
using ouster::sensor::impl::MAX_NUM_PROFILES;

namespace ouster {

namespace impl {

// definition copied from lidar_scan.cpp
struct DefaultFieldsEntry {
    const std::pair<ChanField, ChanFieldType>* fields;
    size_t n_fields;
};

// lidar_scan.cpp
extern Table<UDPProfileLidar, DefaultFieldsEntry, MAX_NUM_PROFILES>
    default_scan_fields;

static void extend_default_scan_fields(
    UDPProfileLidar profile,
    const std::vector<std::pair<ChanField, ChanFieldType>>& scan_fields) {
    auto end = impl::default_scan_fields.end();
    auto it = std::find_if(impl::default_scan_fields.begin(), end,
                           [](const auto& kv) { return kv.first == 0; });

    if (it == end)
        throw std::runtime_error("Limit of scan fields has been reached");

    *it = {profile, {scan_fields.data(), scan_fields.size()}};
}

}  // namespace impl

namespace sensor {
namespace impl {

struct ExtendedProfile {
    UDPProfileLidar profile;
    std::string name;
    std::vector<std::pair<ChanField, ChanFieldType>> slots;
    std::vector<std::pair<ChanField, FieldInfo>> fields;
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
    const std::pair<ChanField, FieldInfo>* fields;
    size_t n_fields;
    size_t chan_data_size;
};

// types.cpp
extern Table<UDPProfileLidar, const char*, MAX_NUM_PROFILES>
    udp_profile_lidar_strings;
// parsing.cpp
extern Table<UDPProfileLidar, ProfileEntry, MAX_NUM_PROFILES> profiles;

static void extend_profile_entries(
    UDPProfileLidar profile,
    const std::vector<std::pair<ChanField, FieldInfo>>& fields,
    size_t chan_data_size) {
    auto end = impl::profiles.end();
    auto it = std::find_if(impl::profiles.begin(), end,
                           [](const auto& kv) { return kv.first == 0; });

    if (it == end)
        throw std::runtime_error("Limit of parsing profiles has been reached");

    *it = {profile, {fields.data(), fields.size(), chan_data_size}};
}

}  // namespace impl

void extend_udp_profile_lidar_strings(UDPProfileLidar profile,
                                      const char* name) {
    auto begin = impl::udp_profile_lidar_strings.begin();
    auto end = impl::udp_profile_lidar_strings.end();

    if (end != std::find_if(begin, end, [profile](const auto& p) {
            return p.first == profile;
        }))
        throw std::invalid_argument(
            "Lidar profile of given number already exists");
    if (end != std::find_if(begin, end, [name](const auto& p) {
            return p.second && std::strcmp(p.second, name) == 0;
        }))
        throw std::invalid_argument(
            "Lidar profile of given name already exists");

    auto it =
        std::find_if(begin, end, [](const auto& kv) { return kv.first == 0; });

    if (it == end)
        throw std::runtime_error("Limit of lidar profiles has been reached");

    *it = std::make_pair(profile, name);
}

void add_custom_profile(int profile_nr,  // int as UDPProfileLidar
                        const std::string& name,
                        const std::vector<std::pair<int, impl::FieldInfo>>&
                            fields,  // int as ChanField
                        size_t chan_data_size) {
    if (profile_nr == 0)
        throw std::invalid_argument("profile_nr of 0 are not allowed");

    auto udp_profile = static_cast<UDPProfileLidar>(profile_nr);

    {
        // fill in profile
        impl::ExtendedProfile profile{
            udp_profile, name, {}, {}, chan_data_size};
        for (auto&& pair : fields) {
            ChanField chan = static_cast<ChanField>(pair.first);

            profile.slots.emplace_back(chan, pair.second.ty_tag);
            profile.fields.emplace_back(chan, pair.second);
        }

        impl::extended_profiles_data.push_back(std::move(profile));
    }

    // retake reference to stored data, profile.name.c_str() can get invalidated
    // after move
    auto&& profile = impl::extended_profiles_data.back();

    extend_udp_profile_lidar_strings(profile.profile, profile.name.c_str());
    impl::extend_profile_entries(profile.profile, profile.fields,
                                 profile.chan_data_size);
    ouster::impl::extend_default_scan_fields(profile.profile, profile.slots);
}

}  // namespace sensor
}  // namespace ouster
