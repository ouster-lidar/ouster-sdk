#include "ouster/vector_streambuf.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <istream>
#include <ostream>
#include <string>
#include <vector>

using ouster::sdk::core::VectorStreamBuf;

TEST(VectorStreamBuf, read) {
    std::vector<uint8_t> buf({'h', 'e', 'l', 'l', 'o'});
    VectorStreamBuf sbuf(&buf);
    std::istream test_file(&sbuf);
    std::string val;
    test_file >> val;
    EXPECT_EQ(val, "hello");
}

TEST(VectorStreamBuf, write) {
    std::vector<uint8_t> buf;
    VectorStreamBuf sbuf(&buf);
    std::ostream test_file(&sbuf);
    test_file << "hello";
    EXPECT_FALSE(buf.empty());
    EXPECT_EQ(buf, std::vector<uint8_t>({'h', 'e', 'l', 'l', 'o'}));

    test_file << '!';
    EXPECT_EQ(buf, std::vector<uint8_t>({'h', 'e', 'l', 'l', 'o', '!'}));
}

TEST(VectorStreamBuf, null) {
    EXPECT_THROW(
        {
            std::vector<uint8_t>* buf = nullptr;
            VectorStreamBuf sbuf(buf);
        },
        std::invalid_argument);
}
