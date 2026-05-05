#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <vector>

// Adjust these includes to match your actual file structure
#include "ouster/osf/buffer.h"
#include "ouster/osf/offset.h"

namespace ouster {
namespace sdk {
namespace osf {

// ============================================================================
// OsfOffset Tests
// ============================================================================

class OsfOffsetTest : public ::testing::Test {};

TEST_F(OsfOffsetTest, ConstructorAndAccessors) {
    uint64_t expected_offset = 1024;
    uint64_t expected_size = 512;

    OsfOffset osf_offset(expected_offset, expected_size);

    EXPECT_EQ(osf_offset.offset(), expected_offset);
    EXPECT_EQ(osf_offset.size(), expected_size);
}

TEST_F(OsfOffsetTest, EqualityOperator) {
    OsfOffset a(100, 50);
    OsfOffset b(100, 50);
    OsfOffset c(100, 51);  // Diff size
    OsfOffset d(101, 50);  // Diff offset

    EXPECT_EQ(a, b);
    EXPECT_FALSE(a == c);
    EXPECT_FALSE(a == d);
}

TEST_F(OsfOffsetTest, StreamOperator) {
    OsfOffset offset(10, 20);
    std::stringstream ss;
    ss << offset;

    // Verify the string format matches the implementation
    EXPECT_EQ(ss.str(), "OsfOffset [offset = 10, size = 20]");
}

// ============================================================================
// OsfBuffer Tests
// ============================================================================

class OsfBufferTest : public ::testing::Test {};

TEST_F(OsfBufferTest, DefaultConstruction) {
    OsfBuffer buf;

    EXPECT_FALSE(buf.has_value());
    EXPECT_EQ(buf.size(), 0);
    EXPECT_EQ(buf.data(), nullptr);

    // Iterators should act like an empty container
    EXPECT_EQ(buf.cbegin(), nullptr);
    EXPECT_EQ(buf.cend(), nullptr);
}

TEST_F(OsfBufferTest, LoadRawPointerData) {
    uint8_t raw_data[] = {0x01, 0x02, 0x03};
    OsfBuffer buf;

    buf.load_data(raw_data, 3);

    EXPECT_TRUE(buf.has_value());
    EXPECT_EQ(buf.size(), 3);
    EXPECT_EQ(buf.data(), raw_data);  // Should point to exact address
    EXPECT_EQ(*buf.cbegin(), 0x01);
    EXPECT_EQ(*(buf.cend() - 1), 0x03);
}

TEST_F(OsfBufferTest, LoadRawPointerNullThrows) {
    OsfBuffer buf;
    EXPECT_THROW(buf.load_data(nullptr, 10), std::logic_error);
}

TEST_F(OsfBufferTest, LoadVectorCopy) {
    std::vector<uint8_t> vec = {10, 20, 30};
    OsfBuffer buf;

    // Pass by reference (copy semantics)
    buf.load_data(vec);

    EXPECT_TRUE(buf.has_value());
    EXPECT_EQ(buf.size(), 3);
    EXPECT_EQ(*buf.data(), 10);

    // Ensure deep copy or shared ownership logic logic:
    // The implementation creates a shared_ptr copy of the vector.
    // The original vector 'vec' should still be valid.
    EXPECT_EQ(vec.size(), 3);
}

TEST_F(OsfBufferTest, LoadVectorMove) {
    std::vector<uint8_t> vec = {10, 20, 30};
    OsfBuffer buf;

    // Pass by move
    buf.load_data(std::move(vec));

    EXPECT_TRUE(buf.has_value());
    EXPECT_EQ(buf.size(), 3);

    // Original vector is likely empty now (implementation dependent, but
    // standard move behavior)
    EXPECT_TRUE(vec.empty());
}

TEST_F(OsfBufferTest, LoadEmptyVectorThrows) {
    std::vector<uint8_t> vec;
    OsfBuffer buf;
    EXPECT_THROW(buf.load_data(vec), std::logic_error);
}

TEST_F(OsfBufferTest, CopyConstructorAndAssignment) {
    std::vector<uint8_t> vec = {1, 2, 3};
    OsfBuffer buf1;
    buf1.load_data(vec);

    // Test Copy Constructor
    OsfBuffer buf2(buf1);
    EXPECT_EQ(buf1, buf2);
    EXPECT_EQ(buf2.size(), 3);
    // They should point to the same underlying shared_ptr data
    EXPECT_EQ(buf1.data(), buf2.data());

    // Test Assignment Operator
    OsfBuffer buf3;
    buf3 = buf1;
    EXPECT_EQ(buf1, buf3);
    EXPECT_EQ(buf3.data(), buf1.data());
}

TEST_F(OsfBufferTest, Reset) {
    std::vector<uint8_t> vec = {1, 2, 3};
    OsfBuffer buf;
    buf.load_data(vec);

    EXPECT_TRUE(buf.has_value());
    buf.reset();
    EXPECT_FALSE(buf.has_value());
    EXPECT_EQ(buf.size(), 0);
    EXPECT_EQ(buf.data(), nullptr);
}

// ----------------------------------------------------------------------------
// Slicing Tests (Creating a buffer from another buffer)
// ----------------------------------------------------------------------------

TEST_F(OsfBufferTest, LoadFromBaseBuffer_VectorBacked) {
    // Setup base: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
    std::vector<uint8_t> vec(10);
    for (int i = 0; i < 10; ++i) vec[i] = i;

    OsfBuffer base;
    base.load_data(vec);

    // Create slice: offset 2, size 4 -> [2, 3, 4, 5]
    OsfBuffer slice;
    slice.load_data(base, 2, 4);

    EXPECT_TRUE(slice.has_value());
    EXPECT_EQ(slice.size(), 4);
    EXPECT_EQ(*slice.data(), 2);        // First element of slice
    EXPECT_EQ(*(slice.data() + 3), 5);  // Last element of slice

    // Verify pointer arithmetic
    EXPECT_EQ(slice.data(), base.data() + 2);
}

TEST_F(OsfBufferTest, LoadFromBaseBuffer_RecursiveSlicing) {
    // Setup base: [0 ... 9]
    std::vector<uint8_t> vec(10);
    for (int i = 0; i < 10; ++i) vec[i] = i;
    OsfBuffer base;
    base.load_data(vec);

    // --- Step 1: Create Slice 1 ---
    // Offset 2, Size 6 -> Expected Data: [2, 3, 4, 5, 6, 7]
    OsfBuffer slice1;
    slice1.load_data(base, 2, 6);

    // Verify Slice 1 immediately
    EXPECT_TRUE(slice1.has_value());
    EXPECT_EQ(slice1.size(), 6);
    EXPECT_EQ(*slice1.data(), 2);
    EXPECT_EQ(slice1.data(), base.data() + 2);

    // --- Step 2: Create Slice 2 from Slice 1 ---
    // Offset 1 (relative to slice1), Size 2 -> Expected Data: [3, 4]
    // Absolute offset should be: 2 (base) + 1 (slice) = 3
    OsfBuffer slice2;
    slice2.load_data(slice1, 1, 2);

    // Verify Slice 2
    EXPECT_EQ(slice2.size(), 2);
    EXPECT_EQ(*slice2.data(), 3);        // Value should be 3
    EXPECT_EQ(*(slice2.data() + 1), 4);  // Value should be 4

    // Verify address arithmetic: base + 2 + 1
    EXPECT_EQ(slice2.data(), base.data() + 3);
}

TEST_F(OsfBufferTest, LoadFromBaseBuffer_OutOfBounds) {
    std::vector<uint8_t> vec(10);
    OsfBuffer base;
    base.load_data(vec);

    OsfBuffer slice;

    // Offset + Size > Base Size (5 + 6 = 11 > 10)
    EXPECT_THROW(slice.load_data(base, 5, 6), std::logic_error);
}

TEST_F(OsfBufferTest, LoadFromBaseBuffer_InvalidBase) {
    OsfBuffer invalid_base;  // Has no value
    OsfBuffer slice;

    EXPECT_THROW(slice.load_data(invalid_base, 0, 0), std::logic_error);
}

TEST_F(OsfBufferTest, EqualityCheckSpecifics) {
    std::vector<uint8_t> vec1 = {1, 2, 3};
    std::vector<uint8_t> vec2 = {1, 2,
                                 3};  // Identical content, different memory

    OsfBuffer b1, b2, b3;
    b1.load_data(vec1);
    b2.load_data(vec1);  // Copies from vec1
    b3.load_data(vec2);  // Copies from vec2

    // b1 and b2 are NOT equal because load_data(vector&) creates a NEW
    // shared_ptr wrapping a copy of the data. The operator== checks shared_ptr
    // equality, not content equality.
    EXPECT_FALSE(b1 == b2);

    // However, if we copy construct, they share the underlying pointer
    OsfBuffer b4(b1);
    EXPECT_EQ(b1, b4);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
