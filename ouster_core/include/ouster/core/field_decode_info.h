/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#pragma once

#include <cstddef>
#include <utility>

#include "ouster/core/types.h"
#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {

/**
 * Helper struct to load/store bit sequences from packets
 *
 * NOTE: getters and setters require up to 64 bits of valid memory past the bit
 *       we are attempting to set/retrieve, so caution is advised.
 */
struct OUSTER_API_CLASS FieldDecodeInfo {
    ChanFieldType ty_tag;  ///< Type tag for the field, used to determine how to
                           ///< interpret the bits
    size_t offset;         ///< Offset in bytes from the start of the buffer
    uint64_t mask;         ///< Bitmask for the field
    int shift;             ///< Bit shift for the field
    int num_elements = 1;  ///< Number of elements with this type tag decoded

    /**
     * Retrieves the value from the buffer.
     * NOTE: the check that T is of at least the size of ChanFieldType used
     *       is deferred because this function is used in the hot loop
     *
     * @param[in] buffer buffer to retrieve the value from.
     *
     * @return value
     */
    template <typename T>
    T get(const uint8_t* buffer) const {
        uint64_t word = *reinterpret_cast<const uint64_t*>(buffer + offset);
        word &= mask;
        if (shift > 0) {
            word >>= shift;
        } else if (shift < 0) {
            word <<= std::abs(shift);
        }

        T out{};
        std::memcpy(&out, &word, sizeof(out));
        return out;
    }

    /**
     * Stores the value into the buffer.
     * NOTE: the check that T is of at least the size of ChanFieldType used
     *       is deferred because this function is used in the hot loop
     *
     * @param[in] buffer buffer to retrieve the value from.
     * @param[in] value value to store
     */
    template <typename T>
    void set(uint8_t* buffer, T value) const {
        uint64_t word = 0;
        std::memcpy(&word, &value, sizeof(value));
        if (shift > 0) {
            word <<= shift;
        }
        if (shift < 0) {
            word >>= std::abs(shift);
        }
        word &= mask;
        uint64_t* ptr = reinterpret_cast<uint64_t*>(buffer + offset);
        *ptr &= ~mask;
        *ptr |= word;
    }
};

}  // namespace core
}  // namespace sdk
}  // namespace ouster
