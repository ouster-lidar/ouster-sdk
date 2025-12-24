#include "ouster/sha256.h"

#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

#include "ouster/vector_streambuf.h"
#include "test_utils.h"
#include "util.h"

using ouster::sdk::core::Sha256;
using ouster::sdk::core::VectorStreamBuf;

TEST(Sha256, hash_empty) {
    Sha256 empty_hash = Sha256::hash_blobs({});
    // Expected value derived by running "echo -n '' | sha256sum"
    EXPECT_EQ(
        empty_hash.str(),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256, hash_nonexistent_file) {
    std::ifstream file_stream("doesntexist", std::ios::binary);
    ASSERT_FALSE(file_stream.is_open());
    auto hash = Sha256::hash_file({file_stream, 0});
    // Expected value is the same as hashing empty input
    EXPECT_EQ(
        hash.str(),
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256, hash_file_and_blobs) {
    std::string data_dir = getenvs("DATA_DIR");
    std::string test_file = data_dir + "/0.stl";
    std::ifstream file_stream(test_file, std::ios::binary);
    ASSERT_TRUE(file_stream.is_open());
    auto hash = Sha256::hash_file({file_stream, 0});
    // Expected value derived by running "sha256sum test_data/0.stl"
    std::string expected_hash_str =
        "9cb392667efd9bb1dd2f02c138049243a6103b4a0ef86574681c0641a195c7fd";
    EXPECT_EQ(hash.str(), expected_hash_str);

    std::vector<uint8_t> bytes = get_file_as_bytes(test_file);
    auto hash2 = Sha256::hash_blobs({{bytes.data(), bytes.size()}});
    EXPECT_EQ(hash, hash2);

    Sha256 hash3(expected_hash_str);
    EXPECT_EQ(hash, hash3);
}

TEST(Sha256, from_string_invalid) {
    EXPECT_THROW(
        {
            try {
                Sha256 hash("not a valid sha256");
            } catch (const std::invalid_argument& e) {
                ASSERT_STREQ(e.what(), "invalid SHA256 hex string length");
                throw;
            }
        },
        std::invalid_argument);
}
