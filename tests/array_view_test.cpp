/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/array_view.h"

#include <list>
#include <vector>

#include "gtest/gtest.h"
#include "ouster/impl/idx_range.h"

using namespace ouster;
using namespace ouster::impl;

TEST(ArrayView_tests, args_restride_test) {
    int32_t shape[4] = {1, 2, 3, 4};
    int32_t new_shape[2] = {0, 0};
    range_args_restride(shape, new_shape, 0, 0, idx_range{}, idx_range{});
    EXPECT_EQ(new_shape[0], shape[2]);
    EXPECT_EQ(new_shape[1], shape[3]);

    range_args_restride(shape, new_shape, 0, idx_range{}, idx_range{}, 0);
    EXPECT_EQ(new_shape[0], shape[1]);
    EXPECT_EQ(new_shape[1], shape[2]);

    range_args_restride(shape, new_shape, 0, idx_range{}, 0, idx_range{});
    EXPECT_EQ(new_shape[0], shape[1]);
    EXPECT_EQ(new_shape[1], shape[3]);

    range_args_restride(shape, new_shape, idx_range{}, 0, 0, idx_range{});
    EXPECT_EQ(new_shape[0], shape[0]);
    EXPECT_EQ(new_shape[1], shape[3]);

    range_args_restride(shape, new_shape, 0, 0);
    EXPECT_EQ(new_shape[0], shape[2]);
    EXPECT_EQ(new_shape[1], shape[3]);

    range_args_restride(shape, new_shape, idx_range{}, 0, 0);
    EXPECT_EQ(new_shape[0], shape[0]);
    EXPECT_EQ(new_shape[1], shape[3]);

    int32_t n_shape[3] = {0, 0, 0};
    range_args_restride(shape, n_shape, idx_range{}, idx_range{}, idx_range{},
                        15);
    EXPECT_EQ(n_shape[0], shape[0]);
    EXPECT_EQ(n_shape[1], shape[1]);
    EXPECT_EQ(n_shape[2], shape[2]);
}

TEST(ArrayView_tests, args_restride_vector_test) {
    std::vector<int32_t> shape = {1, 2, 3, 4};
    std::vector<int32_t> new_shape;
    new_shape = range_args_restride(shape, 0, 0, idx_range{}, idx_range{});
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], shape[2]);
    EXPECT_EQ(new_shape[1], shape[3]);

    new_shape = range_args_restride(shape, 0, idx_range{}, idx_range{}, 0);
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], shape[1]);
    EXPECT_EQ(new_shape[1], shape[2]);

    new_shape = range_args_restride(shape, 0, idx_range{}, 0, idx_range{});
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], shape[1]);
    EXPECT_EQ(new_shape[1], shape[3]);

    new_shape = range_args_restride(shape, idx_range{}, 0, 0, idx_range{});
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], shape[0]);
    EXPECT_EQ(new_shape[1], shape[3]);

    new_shape = range_args_restride(shape, 0, 0);
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], shape[2]);
    EXPECT_EQ(new_shape[1], shape[3]);

    new_shape = range_args_restride(shape, idx_range{}, 0, 0);
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], shape[0]);
    EXPECT_EQ(new_shape[1], shape[3]);

    new_shape =
        range_args_restride(shape, idx_range{}, idx_range{}, idx_range{}, 15);
    EXPECT_EQ(new_shape.size(), 3);
    EXPECT_EQ(new_shape[0], shape[0]);
    EXPECT_EQ(new_shape[1], shape[1]);
    EXPECT_EQ(new_shape[2], shape[2]);
}

TEST(ArrayView_tests, args_reshape_test) {
    int32_t shape[4] = {10, 12, 14, 16};
    int32_t new_shape[2] = {0, 0};
    EXPECT_NO_THROW(range_args_reshape(shape, new_shape, idx_range{}, 0, 0));
    EXPECT_EQ(new_shape[0], shape[0]);
    EXPECT_EQ(new_shape[1], shape[3]);
    EXPECT_NO_THROW(
        range_args_reshape(shape, new_shape, idx_range{2, 3}, 0, 0));
    EXPECT_EQ(new_shape[0], 1);
    EXPECT_EQ(new_shape[1], shape[3]);
    EXPECT_NO_THROW(
        range_args_reshape(shape, new_shape, idx_range{2, 8}, 0, 0));
    EXPECT_EQ(new_shape[0], 6);
    EXPECT_EQ(new_shape[1], shape[3]);
    EXPECT_THROW(range_args_reshape(shape, new_shape, idx_range{2, 2}, 0, 0),
                 std::runtime_error);
    EXPECT_THROW(range_args_reshape(shape, new_shape, idx_range{2, 1}, 0, 0),
                 std::runtime_error);
    EXPECT_THROW(range_args_reshape(shape, new_shape, idx_range{2, 11}, 0, 0),
                 std::runtime_error);
}

TEST(ArrayView_tests, args_reshape_vector_test) {
    std::vector<int32_t> shape = {10, 12, 14, 16};
    std::vector<int32_t> new_shape;
    EXPECT_NO_THROW(new_shape = range_args_reshape(shape, idx_range{}, 0, 0));
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], shape[0]);
    EXPECT_EQ(new_shape[1], shape[3]);
    EXPECT_NO_THROW(new_shape =
                        range_args_reshape(shape, idx_range{2, 3}, 0, 0));
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], 1);
    EXPECT_EQ(new_shape[1], shape[3]);
    EXPECT_NO_THROW(new_shape =
                        range_args_reshape(shape, idx_range{2, 8}, 0, 0));
    EXPECT_EQ(new_shape.size(), 2);
    EXPECT_EQ(new_shape[0], 6);
    EXPECT_EQ(new_shape[1], shape[3]);
    EXPECT_THROW(new_shape = range_args_reshape(shape, idx_range{2, 2}, 0, 0),
                 std::runtime_error);
    EXPECT_THROW(new_shape = range_args_reshape(shape, idx_range{2, 1}, 0, 0),
                 std::runtime_error);
    EXPECT_THROW(new_shape = range_args_reshape(shape, idx_range{2, 11}, 0, 0),
                 std::runtime_error);
}

TEST(ArrayView_tests, constructor_test) {
    int v = 10;
    {
        auto a = ArrayView4<int>(&v, {2, 3, 4, 5});

        EXPECT_EQ(a.shape[0], 2);
        EXPECT_EQ(a.shape[1], 3);
        EXPECT_EQ(a.shape[2], 4);
        EXPECT_EQ(a.shape[3], 5);

        EXPECT_EQ(a.strides[0], 3 * 4 * 5);
        EXPECT_EQ(a.strides[1], 4 * 5);
        EXPECT_EQ(a.strides[2], 5);
    }
    {
        std::vector<float> src = {2.f, 3.f, 4.f, 5.f};

        auto a = ArrayView4<int>(&v, src);

        EXPECT_EQ(a.shape[0], 2);
        EXPECT_EQ(a.shape[1], 3);
        EXPECT_EQ(a.shape[2], 4);
        EXPECT_EQ(a.shape[3], 5);

        EXPECT_EQ(a.strides[0], 3 * 4 * 5);
        EXPECT_EQ(a.strides[1], 4 * 5);
        EXPECT_EQ(a.strides[2], 5);
    }
    {
        std::list<size_t> src = {2, 3, 4, 5};

        auto a = ArrayView4<int>(&v, src);

        EXPECT_EQ(a.shape[0], 2);
        EXPECT_EQ(a.shape[1], 3);
        EXPECT_EQ(a.shape[2], 4);
        EXPECT_EQ(a.shape[3], 5);

        EXPECT_EQ(a.strides[0], 3 * 4 * 5);
        EXPECT_EQ(a.strides[1], 4 * 5);
        EXPECT_EQ(a.strides[2], 5);
    }
    {
        std::vector<size_t> shape = {2, 3, 4, 5};
        std::vector<size_t> strides = {3 * 4 * 5, 4 * 5, 5, 1};

        auto a = ArrayView4<int>(&v, shape, strides);

        EXPECT_EQ(a.shape[0], 2);
        EXPECT_EQ(a.shape[1], 3);
        EXPECT_EQ(a.shape[2], 4);
        EXPECT_EQ(a.shape[3], 5);

        EXPECT_EQ(a.strides[0], 3 * 4 * 5);
        EXPECT_EQ(a.strides[1], 4 * 5);
        EXPECT_EQ(a.strides[2], 5);
        EXPECT_EQ(a.strides[3], 1);
    }
}

TEST(ArrayView_tests, indexing_test) {
    std::vector<int> shape = {5, 6, 7};

    std::vector<int> src(5 * 6 * 7, 0);
    for (size_t i = 0; i < src.size(); ++i) src[i] = i;

    auto a = ArrayView3<int>(src.data(), shape);

    int i = 0;
    for (int x = 0; x < a.shape[0]; ++x)
        for (int y = 0; y < a.shape[1]; ++y)
            for (int z = 0; z < a.shape[2]; ++z) EXPECT_EQ(a(x, y, z), i++);

    auto b = ArrayView1<int>(src.data(), {5 * 6 * 7});
    for (size_t i = 0; i < src.size(); ++i) EXPECT_EQ(b(i), i);
}

TEST(ArrayView_tests, assignment_test) {
    std::vector<int> shape = {5, 6, 7};

    std::vector<int> src(5 * 6 * 7, 10);

    auto a = ArrayView3<int>(src.data(), shape);

    int i = 0;
    for (int x = 0; x < a.shape[0]; ++x)
        for (int y = 0; y < a.shape[1]; ++y)
            for (int z = 0; z < a.shape[2]; ++z) a(x, y, z) = i++;

    for (size_t j = 0; j < src.size(); ++j) EXPECT_EQ(src[j], j);
}

TEST(ArrayView_tests, subview_test) {
    // single subview test
    std::vector<int> src(100 * 100 * 2, 0);

    ArrayView3<int> a(src.data(), {2, 100, 100});
    for (size_t i = 0; i < src.size() / 2; ++i) {
        src[i] = 1;
    }
    for (size_t i = src.size() / 2; i < src.size(); ++i) {
        src[i] = 2;
    }

    auto sub0 = a.subview(0);
    auto sub1 = a.subview(1);
    EXPECT_THROW(a.subview(2), std::invalid_argument);
    EXPECT_THROW(a.subview(1, 145), std::invalid_argument);
    EXPECT_THROW(a.subview(3, 200), std::invalid_argument);
    EXPECT_THROW(a.subview(3, 80), std::invalid_argument);

    EXPECT_EQ(sub0.shape[0], a.shape[1]);
    EXPECT_EQ(sub0.shape[1], a.shape[2]);
    EXPECT_EQ(sub0.strides[0], a.strides[1]);

    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 100; ++j) {
            EXPECT_EQ(sub0(i, j), 1);
            EXPECT_EQ(sub1(i, j), 2);
        }
    }

    ArrayView3<int> b(src.data(), {100, 20, 10});
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 20; ++j) {
            auto c = b.subview(i, j);
            for (int k = 0; k < 10; ++k) {
                c(k) = i * 20 * 10 + j * 10 + k;
            }
        }
    }

    for (size_t i = 0; i < src.size(); ++i) EXPECT_EQ(src[i], i);
}

TEST(ArrayView_tests, window_subview_test) {
    std::vector<int> src(100 * 100);

    int i = 0;
    for (auto& v : src) v = i++;

    ArrayView2<int> view{src.data(), {100, 100}};
    ArrayView2<int> subview = view.subview(keep(10, 20), keep(30, 45));

    EXPECT_EQ(subview.shape[0], 10);
    EXPECT_EQ(subview.shape[1], 15);
    EXPECT_EQ(subview.strides[0], view.strides[0]);
    EXPECT_EQ(subview.strides[1], view.strides[1]);

    for (int y = 10; y < 20; ++y)
        for (int x = 30; x < 45; ++x)
            EXPECT_EQ(view(y, x), subview(y - 10, x - 30));
}

TEST(ArrayView_tests, sparse_subview_test) {
    std::vector<int> v(20 * 320 * 456 * 18);

    for (size_t i = 0; i < v.size(); ++i) {
        v[i] = i % 18;
    }

    ArrayView4<int> view(v.data(), {20, 320, 456, 18});

    ArrayView3<int> bin16 = view.subview(keep(), keep(), keep(), 15);
    EXPECT_EQ(bin16.shape[0], view.shape[0]);
    EXPECT_EQ(bin16.shape[1], view.shape[1]);
    EXPECT_EQ(bin16.shape[2], view.shape[2]);
    EXPECT_EQ(bin16.strides[0], view.strides[0]);
    EXPECT_EQ(bin16.strides[1], view.strides[1]);
    EXPECT_EQ(bin16.strides[2], view.strides[2]);

    for (int ss = 0; ss < 20; ++ss) {
        for (int xx = 0; xx < 320; ++xx) {
            for (int yy = 0; yy < 456; ++yy) {
                EXPECT_EQ(bin16(ss, xx, yy), 15);
            }
        }
    }

    for (auto&& val : v) {
        val = 0;
    }

    auto w310 = view.subview(keep(), keep(), 310, keep());
    EXPECT_EQ(w310.shape[0], view.shape[0]);
    EXPECT_EQ(w310.shape[1], view.shape[1]);
    EXPECT_EQ(w310.shape[2], view.shape[3]);
    EXPECT_EQ(w310.strides[0], view.strides[0]);
    EXPECT_EQ(w310.strides[1], view.strides[1]);
    EXPECT_EQ(w310.strides[2], view.strides[3]);

    for (int ss = 0; ss < 20; ++ss) {
        for (int xx = 0; xx < 320; ++xx) {
            for (int bb = 0; bb < 18; ++bb) {
                w310(ss, xx, bb) = -10;
                EXPECT_EQ(w310(ss, xx, bb), view(ss, xx, 310, bb));
            }
        }
    }
}

TEST(ArrayView_tests, sparse_test) {
    ConstArrayView4<float> view{nullptr, {20, 320, 456, 18}};

    EXPECT_EQ(view.sparse(), false);
    EXPECT_EQ(view.subview(0).sparse(), false);
    EXPECT_EQ(view.subview(1).sparse(), false);
    EXPECT_EQ(view.subview(3).sparse(), false);
    EXPECT_EQ(view.subview(3, 5).sparse(), false);
    EXPECT_EQ(view.subview(3, 5, 10).sparse(), false);

    EXPECT_EQ(view.subview(keep(), keep(), keep(), 10).sparse(), true);
    EXPECT_EQ(view.subview(keep(), 200, 134, 10).sparse(), true);
    EXPECT_EQ(view.subview(keep(), 2, keep(), 10).sparse(), true);
    EXPECT_EQ(view.subview(1, keep(), keep(), 10).sparse(), true);
    EXPECT_EQ(view.subview(1, 2, keep(), 10).sparse(), true);
    EXPECT_EQ(view.subview(keep(), keep(), 10).sparse(), true);
    EXPECT_EQ(view.subview(1, keep(), 10).sparse(), true);
    EXPECT_EQ(view.subview(keep(), 10).sparse(), true);
    EXPECT_EQ(view.subview(keep(), 10, 5).sparse(), true);
    EXPECT_EQ(view.subview(keep(), 10, 5, 3).sparse(), true);
}

TEST(ArrayView_tests, reshape_test) {
    std::vector<int> vec(100 * 200 * 300);
    ConstArrayView3<int> view{vec.data(), {100, 200, 300}};

    EXPECT_THROW(view.reshape(3, 9, 12, 10), std::invalid_argument);
    EXPECT_THROW(view.reshape(3, 10), std::invalid_argument);
    EXPECT_THROW(view.subview(keep(), 2).reshape(50, 2, 300),
                 std::invalid_argument);
    EXPECT_NO_THROW(view.subview(2).reshape(100, 2, 300));

    {
        int i = 0;
        for (auto& v : vec) v = i++;
    }
    {
        ConstArrayView2<int> reshaped = view.reshape(200 * 100, 300);

        int i = 0;
        for (int a = 0; a < reshaped.shape[0]; ++a)
            for (int b = 0; b < reshaped.shape[1]; ++b)
                EXPECT_EQ(reshaped(a, b), i++);
    }
    {
        ConstArrayView2<int> reshaped = view.reshape(2, 100 * 100 * 300);

        int i = 0;
        for (int a = 0; a < reshaped.shape[0]; ++a)
            for (int b = 0; b < reshaped.shape[1]; ++b)
                EXPECT_EQ(reshaped(a, b), i++);
    }
    {
        ConstArrayView2<int> reshaped = view.reshape(100 * 50, 4 * 300);

        int i = 0;
        for (int a = 0; a < reshaped.shape[0]; ++a)
            for (int b = 0; b < reshaped.shape[1]; ++b)
                EXPECT_EQ(reshaped(a, b), i++);
    }
    {
        ConstArrayView3<int> reshaped = view.reshape(200 * 10, 5, 2 * 300);

        int i = 0;
        for (int a = 0; a < reshaped.shape[0]; ++a)
            for (int b = 0; b < reshaped.shape[1]; ++b)
                for (int c = 0; c < reshaped.shape[2]; ++c)
                    EXPECT_EQ(reshaped(a, b, c), i++);
    }
    {
        ConstArrayView4<int> reshaped = view.reshape(2, 50, 200, 300);

        int i = 0;
        for (int a = 0; a < reshaped.shape[0]; ++a)
            for (int b = 0; b < reshaped.shape[1]; ++b)
                for (int c = 0; c < reshaped.shape[2]; ++c)
                    for (int d = 0; d < reshaped.shape[3]; ++d)
                        EXPECT_EQ(reshaped(a, b, c, d), i++);
    }
    {
        ConstArrayView4<int> reshaped = view.reshape(100, 50, 4, 300);

        int i = 0;
        for (int a = 0; a < reshaped.shape[0]; ++a)
            for (int b = 0; b < reshaped.shape[1]; ++b)
                for (int c = 0; c < reshaped.shape[2]; ++c)
                    for (int d = 0; d < reshaped.shape[3]; ++d)
                        EXPECT_EQ(reshaped(a, b, c, d), i++);
    }
    {
        ConstArrayView4<int> reshaped = view.reshape(100, 200, 100, 3);

        int i = 0;
        for (int a = 0; a < reshaped.shape[0]; ++a)
            for (int b = 0; b < reshaped.shape[1]; ++b)
                for (int c = 0; c < reshaped.shape[2]; ++c)
                    for (int d = 0; d < reshaped.shape[3]; ++d)
                        EXPECT_EQ(reshaped(a, b, c, d), i++);
    }
}

TEST(ArrayView_tests, iterator_test) {
    std::vector<int> vec(20 * 320 * 456 * 18);

    ArrayView4<int> view{vec.data(), {20, 320, 456, 18}};
    ConstArrayView4<int> cview{vec.data(), {20, 320, 456, 18}};

    EXPECT_THROW(view.subview(1, keep(), 10).begin(), std::logic_error);
    EXPECT_THROW(view.subview(1, keep(), 10).end(), std::logic_error);
    EXPECT_THROW(cview.subview(1, keep(), 10).begin(), std::logic_error);
    EXPECT_THROW(cview.subview(1, keep(), 10).end(), std::logic_error);

    int i = 0;
    for (auto& v : view) v = i++;

    i = 0;
    for (auto& v : vec) EXPECT_EQ(v, i++);
}
