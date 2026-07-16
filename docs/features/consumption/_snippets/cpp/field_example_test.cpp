#include "field_example.cpp"

#include <gtest/gtest.h>

using namespace ouster::sdk::core;

TEST(FieldExampleSnippet, FieldConstructArray) {
    EXPECT_NO_THROW(doc_field_construct_array());
}

TEST(FieldExampleSnippet, FieldAccessViaEigen) {
    EXPECT_NO_THROW(doc_field_access_via_eigen());
}

TEST(FieldExampleSnippet, FieldAccessViaArrayView) {
    EXPECT_NO_THROW(doc_field_access_via_arrayview());
}

TEST(FieldExampleSnippet, FieldAccessViaPointer) {
    EXPECT_NO_THROW(doc_field_access_via_pointer());
}

TEST(FieldExampleSnippet, FieldViewSubview) {
    EXPECT_NO_THROW(doc_field_subview());
}

TEST(FieldExampleSnippet, ArrayViewElementAccess) {
    EXPECT_NO_THROW(doc_arrayview_element_access());
}

TEST(FieldExampleSnippet, ArrayViewSubview) {
    EXPECT_NO_THROW(doc_arrayview_subview());
}

TEST(FieldExampleSnippet, ArrayViewSubviewKeepAxis) {
    EXPECT_NO_THROW(doc_arrayview_subview_keep());
}

TEST(FieldExampleSnippet, ArrayViewSubviewBand) {
    EXPECT_NO_THROW(doc_arrayview_subview_band());
}

TEST(FieldExampleSnippet, ArrayViewReshape) {
    EXPECT_NO_THROW(doc_arrayview_reshape());
}

TEST(FieldExampleSnippet, FieldViewReshape) {
    EXPECT_NO_THROW(doc_fieldview_reshape());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
