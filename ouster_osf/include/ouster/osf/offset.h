/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file file.h
 * @brief common OSF file resource for Reader and Writer operations
 *
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <ostream>
#include <vector>

#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * @brief Represents an offset and size in an OSF file.
 *
 * Includes helpers to check validity, compare offsets, and access individual
 * components.
 */
class OUSTER_API_CLASS OsfOffset {
   public:
    OUSTER_API_FUNCTION
    OsfOffset() = default;
    /**
     * Default copy constructor for OsfOffset
     * @param[in] other The OsfOffset to copy.
     */
    OUSTER_API_FUNCTION
    OsfOffset(const OsfOffset& other) = default;
    /**
     * Default move constructor for OsfOffset.
     * @param[in] other The OsfOffset to move from.
     */
    OUSTER_API_FUNCTION
    OsfOffset(OsfOffset&& other) = default;
    OUSTER_API_FUNCTION
    OsfOffset& operator=(const OsfOffset& other) = default;

    /**
     * Constructs an OsfOffset from a given offset and size.
     *
     * @param[in] offset_in Byte offset within the OSF file.
     * @param[in] size_in Size of the data span in bytes.
     */
    OUSTER_API_FUNCTION
    OsfOffset(const uint64_t offset_in, const uint64_t size_in);

    OUSTER_API_FUNCTION
    bool operator==(const OsfOffset& other) const;

    /**
     * Gets the byte offset in the OSF file.
     *
     * @return The offset in bytes.
     */
    OUSTER_API_FUNCTION
    uint64_t offset() const;
    /**
     * Gets the size of the chunk associated with this offset.
     *
     * @return The chunk size in bytes.
     */
    OUSTER_API_FUNCTION
    uint64_t size() const;
    /**
     * Indicates whether this OsfOffset is valid.
     *
     * @return True if valid; false otherwise.
     */
    OUSTER_API_FUNCTION
    bool valid() const;

   protected:
    uint64_t offset_{0};  ///< Offset in the file
    uint64_t size_{0};    ///< Size of the chunk
    bool valid_{false};
};

/**
 * @brief Output stream operator for OsfOffset.
 * @param[in,out] os Output stream.
 * @param[in] offset The OsfOffset to output.
 * @return Reference to the output stream.
 */
OUSTER_API_FUNCTION
std::ostream& operator<<(std::ostream& os, const OsfOffset& offset);

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
