/**
 * Snippets for docs/features/consumption/field.rst
 */
#include <vector>

#include "ouster/core/array_view.h"
#include "ouster/core/field.h"

using namespace ouster::sdk::core;

static void doc_field_construct_array() {
    //! [doc-stag-field-construct-array]
    // templated version
    Field field(fd_array<uint32_t>(128, 1024));
    // ChanFieldType version
    Field field_cft(fd_array(ChanFieldType::UINT32, 128, 1024));
    //! [doc-etag-field-construct-array]
    uint32_t* raw_1 = field;
    uint32_t* raw_2 = field_cft;
    (void)raw_1;
    (void)raw_2;
}

static void doc_field_access_via_eigen() {
    //! [doc-stag-field-access-via-eigen]
    Field field(fd_array<uint32_t>(128, 1024));
    Eigen::Ref<img_t<uint32_t>> img = field;
    //! [doc-etag-field-access-via-eigen]
    (void)img;
}

static void doc_field_access_via_arrayview() {
    //! [doc-stag-field-access-via-arrayview]
    Field field(fd_array<uint32_t>(128, 1024));
    ArrayView<uint32_t, 2> array_view = field;
    //! [doc-etag-field-access-via-arrayview]
    (void)array_view;
}

static void doc_field_access_via_pointer() {
    //! [doc-stag-field-access-via-pointer]
    Field field(fd_array<uint32_t>(128, 1024));
    uint32_t* ptr = field;
    //! [doc-etag-field-access-via-pointer]
    (void)ptr;
}

static void doc_field_subview() {
    //! [doc-stag-field-subview]
    std::vector<int> data(100 * 100);
    FieldView view(data.data(), fd_array<int>(100, 100));
    FieldView column = view.subview(keep(), 42);
    //! [doc-etag-field-subview]
    (void)column;
}

static void doc_arrayview_element_access() {
    //! [doc-stag-arrayview-element-access]
    std::vector<float> data(100 * 100);
    ArrayView2<float> img(data.data(), {100, 100});
    const int row = 10;
    const int col = 20;
    float x = img(row, col);  // read
    img(row, col) = 3.14f;    // write
    //! [doc-etag-arrayview-element-access]
    (void)x;
}

static void doc_arrayview_subview() {
    //! [doc-stag-arrayview-subview]
    std::vector<int> data(100 * 100 * 100);
    ArrayView3<int> cube{data.data(), {100, 100, 100}};
    // Equivalent to NumPy arr[10]
    ArrayView2<int> square_slice = cube.subview(10);
    // Equivalent to NumPy arr[10, 20]
    ArrayView1<int> line_slice = cube.subview(10, 20);
    //! [doc-etag-arrayview-subview]
    (void)square_slice;
    (void)line_slice;
}

static void doc_arrayview_subview_keep() {
    //! [doc-stag-arrayview-subview-keep]
    std::vector<int> data(100 * 100 * 100);
    ArrayView3<int> cube{data.data(), {100, 100, 100}};
    const int fixed_row = 10;
    const int fixed_col = 20;
    // Equivalent to NumPy arr[:, fixed_row, fixed_col]
    ArrayView1<int> line = cube.subview(keep(), fixed_row, fixed_col);
    //! [doc-etag-arrayview-subview-keep]
    (void)line;
}

static void doc_arrayview_subview_band() {
    //! [doc-stag-arrayview-subview-band]
    std::vector<int> data(100 * 100);
    ArrayView2<int> matrix{data.data(), {100, 100}};
    // Equivalent to NumPy arr[:, 10:50]
    ArrayView2<int> band = matrix.subview(keep(), keep(10, 50));
    //! [doc-etag-arrayview-subview-band]
    (void)band;
}

static void doc_arrayview_reshape() {
    //! [doc-stag-arrayview-reshape]
    std::vector<int> data(12);
    ArrayView1<int> flat{data.data(), {12}};
    // Same storage, new rank: 12 -> 3 x 4 row-major, then 2 x 2 x 3
    ArrayView2<int> mat = flat.reshape(3, 4);
    ArrayView3<int> box = mat.reshape(2, 2, 3);
    //! [doc-etag-arrayview-reshape]
    (void)flat;
    (void)mat;
    (void)box;
}

static void doc_fieldview_reshape() {
    // clang-format off
    //! [doc-stag-fieldview-reshape]
    std::vector<int> data(12);
    FieldView view(data.data(), fd_array<int>(3, 4));
    // Same allocation and element count; descriptor gets a new shape + row-major strides
    FieldView box = view.reshape(2, 2, 3);
    //! [doc-etag-fieldview-reshape]
    // clang-format on
    (void)view;
    (void)box;
}
