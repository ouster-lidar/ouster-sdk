/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/core/profile_extension.h"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <list>
#include <string>
#include <utility>
#include <vector>

#include "ouster/core/types.h"

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
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern std::vector<std::pair<const char*, DefaultFieldsEntry>> default_frame_fields;

struct ExtendedProfile {
    std::string profile;
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
// parsing.cpp
// NOLINTNEXTLINE(cppcoreguidelines-avoid-non-const-global-variables)
extern std::vector<std::pair<const char*, ProfileEntry>> profiles;
}  // namespace impl

std::string add_custom_profile(const std::string& name,
                               const std::vector<std::pair<std::string, FieldDecodeInfo>>& fields,
                               size_t chan_data_size) {
    for (const auto& prof : impl::profiles) {
        if (name == prof.first) {
            throw std::invalid_argument("Lidar profile of given name already exists");
        }
    }
    {
        // fill in profile
        impl::ExtendedProfile profile{name, {}, {}, chan_data_size};
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

    auto& profile = impl::extended_profiles_data.back();

    const char* name_ptr = profile.profile.c_str();
    impl::profiles.push_back(
        {name_ptr, {profile.fields.data(), profile.fields.size(), profile.chan_data_size}});
    impl::default_frame_fields.push_back({name_ptr, {profile.slots.data(), profile.slots.size()}});
    return name;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
