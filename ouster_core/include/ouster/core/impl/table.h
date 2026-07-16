/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <algorithm>
#include <array>
#include <cstring>
#include <nonstd/optional.hpp>
#include <utility>

namespace ouster {
namespace impl {

template <typename K, typename V, size_t N>
using Table = std::array<std::pair<K, V>, N>;

template <typename K, typename V, size_t N>
nonstd::optional<V> lookup(const Table<K, V, N>& table, const K& key) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end,
                            [&](const std::pair<K, V>& pair) { return pair.first == key; });

    return res == end ? nonstd::nullopt : nonstd::make_optional<V>(res->second);
}

template <typename K, size_t N>
nonstd::optional<K> rlookup(const Table<K, const char*, N>& table, const char* value) {
    auto end = table.end();
    auto res = std::find_if(table.begin(), end, [&](const std::pair<K, const char*>& pair) {
        return pair.second && std::strcmp(pair.second, value) == 0;
    });

    return res == end ? nonstd::nullopt : nonstd::make_optional<K>(res->first);
}

}  // namespace impl
}  // namespace ouster
