#include "ouster/sha256.h"

#include <openssl/evp.h>

#include <array>
#include <cassert>
#include <cstring>
#include <iomanip>
#include <ios>
#include <istream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace ouster {
namespace sdk {
namespace core {

namespace {
sha256_hash_t from_string(const std::string& hex_str) {
    sha256_hash_t hash{};
    if (hex_str.size() != sizeof(hash) * 2) {
        throw std::invalid_argument("invalid SHA256 hex string length");
    }
    for (size_t i = 0; i < sizeof(hash); i++) {
        std::string byte_str = hex_str.substr(i * 2, 2);
        // NOLINTNEXTLINE(cppcoreguidelines-pro-bounds-constant-array-index)
        hash[i / 4] |= (std::stoul(byte_str, nullptr, 16) << (i % 4) * 8);
    }
    return hash;
}
}  // namespace

Sha256::Sha256(const sha256_hash_t& hash) : hash_(hash) {}

Sha256::Sha256(const std::string& hex_str) : hash_{from_string(hex_str)} {}

std::string Sha256::str() const {
    std::stringstream stream;
    stream << std::hex << std::setfill('0');
    for (size_t i = 0; i < sizeof(hash_); i++) {
        stream << std::setw(2) << static_cast<int>(bytes()[i]);
    }
    return stream.str();
}

bool Sha256::operator==(const Sha256& sha256_b) const {
    return hash_ == sha256_b.hash_;
}

bool Sha256::operator!=(const Sha256& sha256_b) const {
    return !(*this == sha256_b);
}

Sha256 Sha256::hash_file(const std::pair<std::istream&, size_t>& file,
                         std::pair<const void*, size_t> prefix) {
    return Sha256::hash_files({file}, prefix);
}

Sha256 Sha256::hash_files(
    const std::vector<std::pair<std::istream&, size_t>>& files,
    std::pair<const void*, size_t> prefix) {
    std::array<unsigned char, 32> hash_bytes{};
    EVP_MD_CTX* mdctx = EVP_MD_CTX_new();
    unsigned int sha256_digest_len = EVP_MD_size(EVP_sha256());

    EVP_DigestInit_ex(mdctx, EVP_sha256(), nullptr);

    if (prefix.first != nullptr && prefix.second > 0u) {
        EVP_DigestUpdate(mdctx, prefix.first, prefix.second);
    }

    for (const auto& file_and_skip_bytes : files) {
        auto& file = file_and_skip_bytes.first;
        auto& skip_bytes = file_and_skip_bytes.second;
        file.seekg(static_cast<std::streamoff>(skip_bytes));

        std::streamsize bytes = 0;
        std::array<char, 1024> data{};
        file.read(data.data(), data.size());
        while ((bytes = file.gcount()) != 0) {
            if (file.bad()) {
                throw std::runtime_error("error reading file");
            }
            EVP_DigestUpdate(mdctx, data.data(), bytes);
            file.read(data.data(), data.size());
        }

        file.clear();
        file.seekg(0);
    }

    EVP_DigestFinal_ex(mdctx, hash_bytes.data(), &sha256_digest_len);
    EVP_MD_CTX_free(mdctx);
    sha256_hash_t hash_array;
    std::memcpy(static_cast<void*>(hash_array.data()), hash_bytes.data(),
                sizeof(hash_array));
    return Sha256(hash_array);
}

Sha256 Sha256::hash_blobs(
    const std::vector<std::pair<const void*, size_t>>& blobs) {
    std::array<unsigned char, 32> hash_bytes{};
    EVP_MD_CTX* mdctx = EVP_MD_CTX_new();
    unsigned int digest_len = EVP_MD_size(EVP_sha256());

    EVP_DigestInit_ex(mdctx, EVP_sha256(), nullptr);

    for (const auto& blob_and_blob_bytes : blobs) {
        auto& blob = blob_and_blob_bytes.first;
        auto& blob_bytes = blob_and_blob_bytes.second;
        if ((blob != nullptr) && (blob_bytes != 0u)) {
            EVP_DigestUpdate(mdctx, blob, blob_bytes);
        }
    }

    EVP_DigestFinal_ex(mdctx, hash_bytes.data(), &digest_len);
    EVP_MD_CTX_free(mdctx);
    sha256_hash_t hash_array;
    std::memcpy(static_cast<void*>(hash_array.data()), hash_bytes.data(),
                sizeof(hash_array));
    return Sha256(hash_array);
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
