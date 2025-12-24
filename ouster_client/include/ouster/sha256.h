/**
 * Copyright (c) 2025, Ouster, Inc.
 * All rights reserved.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <istream>
#include <string>
#include <utility>
#include <vector>

#include "ouster/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
/**The number of 32-bit words per 256 bit hash.*/
constexpr int WORDS_PER_SHA256_HASH = 8;

/**An array for a 256 byte hash.*/
using sha256_hash_t = std::array<uint32_t, WORDS_PER_SHA256_HASH>;

/**
 * A simple class to represent a SHA256 checksum.
 */
#pragma pack(push, 1)
class OUSTER_API_CLASS Sha256 {
   public:
    /**
     * Instantiate a Sha256 with an uninitialized hash value.
     */
    // TODO[tws] refactor uses of this class so that we can remove the default
    // constructor.
    OUSTER_API_FUNCTION
    Sha256() = default;

    /**
     * Initialize a Sha256 from an array.
     * @param[in] hash An array of words containing the hash value.
     */
    OUSTER_API_FUNCTION
    explicit Sha256(const sha256_hash_t& hash);

    /**
     * Initialize a Sha256 from a hex string.
     * @param[in] hex_str A hex string containing the hash value.
     */
    OUSTER_API_FUNCTION
    explicit Sha256(const std::string& hex_str);

    /**
     * Returns a hex string of the hash.
     * @return The hex string.
     */
    OUSTER_API_FUNCTION
    std::string str() const;

    /**
     * Compare two hash values for equality.
     * @return true if the hashes are equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator==(const Sha256& sha256_b) const;

    /**
     * Compare two hash values for inequality.
     * @return true if the hashes are not equal, false otherwise.
     */
    OUSTER_API_FUNCTION
    bool operator!=(const Sha256& sha256_b) const;

    /**
     * Returns a pointer to the hash data.
     * @return A pointer to the hash data.
     */
    OUSTER_API_FUNCTION
    const unsigned char* bytes() const {
        return reinterpret_cast<const unsigned char*>(hash_.data());
    }

    /**
     * Creates a Sha256 from an input stream.
     * @param[in] file A pair containing an istream and an offset to start
     * from.
     * @param[in] prefix An optional buffer to hash prior to the istream
     * contents.
     * @return The Sha256 of the stream and prefix.
     */
    OUSTER_API_FUNCTION
    static Sha256 hash_file(const std::pair<std::istream&, size_t>& file,
                            std::pair<const void*, size_t> prefix = {nullptr,
                                                                     0});
    /**
     * Creates a Sha256 from an input stream.
     * @param[in] files A vector of pair containing istreams and offsets to
     * start from.
     * @param[in] prefix An optional buffer to hash prior to the ifstream
     * contents.
     * @return The Sha256 of the stream and prefix.
     */
    OUSTER_API_FUNCTION
    static Sha256 hash_files(
        const std::vector<std::pair<std::istream&, size_t>>& files,
        std::pair<const void*, size_t> prefix = {nullptr, 0});

    /**
     * Creates a Sha256 from a vector of buffers.
     * @param[in] blobs A vector of pair containing pointers to buffers' data
     * and their sizes.
     * @return The Sha256 of the buffers.
     */
    OUSTER_API_FUNCTION
    static Sha256 hash_blobs(
        const std::vector<std::pair<const void*, size_t>>& blobs);

   private:
    sha256_hash_t hash_{};  ///< The hash data.
};
#pragma pack(pop)
}  // namespace core
}  // namespace sdk
}  // namespace ouster
