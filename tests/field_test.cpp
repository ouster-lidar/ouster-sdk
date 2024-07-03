/**
 * Copyright (c) 2024, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/field.h"

#include <algorithm>
#include <limits>
#include <list>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

using namespace ouster;

TEST(Field_tests, comparison_test) {
    Field a(fd_array<int>(100));
    int* a_p = a;
    std::iota(a_p, a_p + 100, int{0});

    Field b(fd_array<int>(100));
    int* b_p = b;
    std::iota(b_p, b_p + 100, int{0});

    EXPECT_TRUE(a == b);

    *b_p = -1;
    EXPECT_FALSE(a == b);

    Field c(fd_array<float>(100));
    ASSERT_EQ(c.bytes(), a.bytes());
    float* c_p = c;
    std::memcpy(c_p, a_p, a.bytes());
    EXPECT_FALSE(a == c);
}

TEST(Field_tests, type_safety_test) {
    // clang-format off
    Field type_checking_ptr = Field{fd_array<uint8_t>(128)};

    EXPECT_NO_THROW({ uint8_t*    u8ptr = type_checking_ptr; });
    EXPECT_NO_THROW({ void*       vptr  = type_checking_ptr; });
    EXPECT_NO_THROW({ type_checking_ptr.get<uint8_t>(); });
    EXPECT_NO_THROW({ type_checking_ptr.get(); });

    EXPECT_THROW({ uint16_t*  u16ptr  = type_checking_ptr; }, std::invalid_argument);
    EXPECT_THROW({ uint32_t*  u32ptr  = type_checking_ptr; }, std::invalid_argument);
    EXPECT_THROW({ int8_t*    i8ptr   = type_checking_ptr; }, std::invalid_argument);
    EXPECT_THROW({ int16_t*   i16ptr  = type_checking_ptr; }, std::invalid_argument);
    EXPECT_THROW({ int32_t*   i32ptr  = type_checking_ptr; }, std::invalid_argument);
    EXPECT_THROW({ int64_t*   i64ptr  = type_checking_ptr; }, std::invalid_argument);
    EXPECT_THROW({ float*     fptr    = type_checking_ptr; }, std::invalid_argument);
    EXPECT_THROW({ double*    dptr    = type_checking_ptr; }, std::invalid_argument);

    EXPECT_THROW({ type_checking_ptr.get<uint16_t>(); }, std::invalid_argument);
    EXPECT_THROW({ type_checking_ptr.get<uint32_t>(); }, std::invalid_argument);
    EXPECT_THROW({ type_checking_ptr.get<int8_t>();   }, std::invalid_argument);
    EXPECT_THROW({ type_checking_ptr.get<int16_t>();  }, std::invalid_argument);
    EXPECT_THROW({ type_checking_ptr.get<int32_t>();  }, std::invalid_argument);
    EXPECT_THROW({ type_checking_ptr.get<int64_t>();  }, std::invalid_argument);
    EXPECT_THROW({ type_checking_ptr.get<float>();    }, std::invalid_argument);
    EXPECT_THROW({ type_checking_ptr.get<double>();   }, std::invalid_argument);

    Field non_type_checking_ptr = Field{FieldDescriptor::memory(128)};

    EXPECT_NO_THROW({ void*      vptr    = non_type_checking_ptr; });
    EXPECT_NO_THROW({ uint8_t*   u8ptr   = non_type_checking_ptr; });
    EXPECT_NO_THROW({ uint16_t*  u16ptr  = non_type_checking_ptr; });
    EXPECT_NO_THROW({ uint32_t*  u32ptr  = non_type_checking_ptr; });
    EXPECT_NO_THROW({ int8_t*    i8ptr   = non_type_checking_ptr; });
    EXPECT_NO_THROW({ int16_t*   i16ptr  = non_type_checking_ptr; });
    EXPECT_NO_THROW({ int32_t*   i32ptr  = non_type_checking_ptr; });
    EXPECT_NO_THROW({ int64_t*   i64ptr  = non_type_checking_ptr; });
    EXPECT_NO_THROW({ float*     fptr    = non_type_checking_ptr; });
    EXPECT_NO_THROW({ double*    dptr    = non_type_checking_ptr; });

    auto convert_uint8  = [](uint8_t*   ptr) { return ptr; };
    auto convert_uint16 = [](uint16_t*  ptr) { return ptr; };
    auto convert_uint32 = [](uint32_t*  ptr) { return ptr; };
    auto convert_uint64 = [](uint64_t*  ptr) { return ptr; };
    auto convert_int8   = [](int8_t*    ptr) { return ptr; };
    auto convert_int16  = [](int16_t*   ptr) { return ptr; };
    auto convert_int32  = [](int32_t*   ptr) { return ptr; };
    auto convert_int64  = [](int64_t*   ptr) { return ptr; };
    auto convert_float  = [](float*     ptr) { return ptr; };
    auto convert_double = [](double*    ptr) { return ptr; };
    auto convert_void   = [](void*      ptr) { return ptr; };

    EXPECT_NO_THROW(convert_uint8(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_uint16(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_uint32(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_uint64(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_int8(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_int16(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_int32(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_int64(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_float(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_double(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_void(non_type_checking_ptr));

    EXPECT_NO_THROW(convert_void(type_checking_ptr));
    EXPECT_NO_THROW(convert_uint8(type_checking_ptr));
    EXPECT_THROW(convert_uint16(type_checking_ptr),  std::invalid_argument);
    EXPECT_THROW(convert_uint32(type_checking_ptr),  std::invalid_argument);
    EXPECT_THROW(convert_uint64(type_checking_ptr),  std::invalid_argument);
    EXPECT_THROW(convert_int8(type_checking_ptr),    std::invalid_argument);
    EXPECT_THROW(convert_int16(type_checking_ptr),   std::invalid_argument);
    EXPECT_THROW(convert_int32(type_checking_ptr),   std::invalid_argument);
    EXPECT_THROW(convert_int64(type_checking_ptr),   std::invalid_argument);
    EXPECT_THROW(convert_float(type_checking_ptr),   std::invalid_argument);
    EXPECT_THROW(convert_double(type_checking_ptr),  std::invalid_argument);

    auto convert_const_uint8  = [](const uint8_t*   ptr) { return ptr; };
    auto convert_const_uint16 = [](const uint16_t*  ptr) { return ptr; };
    auto convert_const_uint32 = [](const uint32_t*  ptr) { return ptr; };
    auto convert_const_uint64 = [](const uint64_t*  ptr) { return ptr; };
    auto convert_const_int8   = [](const int8_t*    ptr) { return ptr; };
    auto convert_const_int16  = [](const int16_t*   ptr) { return ptr; };
    auto convert_const_int32  = [](const int32_t*   ptr) { return ptr; };
    auto convert_const_int64  = [](const int64_t*   ptr) { return ptr; };
    auto convert_const_float  = [](const float*     ptr) { return ptr; };
    auto convert_const_double = [](const double*    ptr) { return ptr; };
    auto convert_const_void   = [](const void*      ptr) { return ptr; };

    EXPECT_NO_THROW(convert_const_uint8(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_uint16(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_uint32(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_uint64(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_int8(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_int16(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_int32(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_int64(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_float(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_double(non_type_checking_ptr));
    EXPECT_NO_THROW(convert_const_void(non_type_checking_ptr));

    EXPECT_NO_THROW(convert_const_void(type_checking_ptr));
    EXPECT_NO_THROW(convert_const_uint8(type_checking_ptr));
    EXPECT_THROW(convert_const_uint16(type_checking_ptr),  std::invalid_argument);
    EXPECT_THROW(convert_const_uint32(type_checking_ptr),  std::invalid_argument);
    EXPECT_THROW(convert_const_uint64(type_checking_ptr),  std::invalid_argument);
    EXPECT_THROW(convert_const_int8(type_checking_ptr),    std::invalid_argument);
    EXPECT_THROW(convert_const_int16(type_checking_ptr),   std::invalid_argument);
    EXPECT_THROW(convert_const_int32(type_checking_ptr),   std::invalid_argument);
    EXPECT_THROW(convert_const_int64(type_checking_ptr),   std::invalid_argument);
    EXPECT_THROW(convert_const_float(type_checking_ptr),   std::invalid_argument);
    EXPECT_THROW(convert_const_double(type_checking_ptr),  std::invalid_argument);
    // clang-format on
}

TEST(Field_tests, array_view_test) {
    Field p4d{fd_array<float>(128, 128, 64, 3)};
    EXPECT_NO_THROW({ ArrayView4<float> a = p4d; });
    EXPECT_THROW({ ArrayView3<float> a = p4d; }, std::invalid_argument);
    EXPECT_THROW({ ArrayView2<float> a = p4d; }, std::invalid_argument);
    EXPECT_THROW({ ArrayView1<float> a = p4d; }, std::invalid_argument);

    EXPECT_THROW({ ArrayView4<int> a = p4d; }, std::invalid_argument);

    Field p3d{fd_array<int>(128, 64, 3)};
    EXPECT_THROW({ ArrayView4<int> a = p3d; }, std::invalid_argument);
    EXPECT_NO_THROW({ ArrayView3<int> a = p3d; });
    EXPECT_THROW({ ArrayView2<int> a = p3d; }, std::invalid_argument);
    EXPECT_THROW({ ArrayView1<int> a = p3d; }, std::invalid_argument);

    Field p2d{fd_array<size_t>(64, 3)};
    EXPECT_THROW({ ArrayView4<size_t> a = p2d; }, std::invalid_argument);
    EXPECT_THROW({ ArrayView4<size_t> a = p2d; }, std::invalid_argument);
    EXPECT_NO_THROW({ ArrayView2<size_t> a = p2d; });
    EXPECT_THROW({ ArrayView1<size_t> a = p2d; }, std::invalid_argument);

    Field p1d{fd_array<uint8_t>(64)};
    EXPECT_THROW({ ArrayView4<uint8_t> a = p1d; }, std::invalid_argument);
    EXPECT_THROW({ ArrayView3<uint8_t> a = p1d; }, std::invalid_argument);
    EXPECT_THROW({ ArrayView2<uint8_t> a = p1d; }, std::invalid_argument);
    EXPECT_NO_THROW({ ArrayView1<uint8_t> a = p1d; });
}

TEST(Field_tests, proxy_subview_test) {
    Field ptr{fd_array<float>(2, 100, 100)};

    Field& p = ptr;
    const Field& cp = ptr;

    EXPECT_NO_THROW({ ArrayView2<float> a = p.subview(1); });
    EXPECT_NO_THROW({ ArrayView1<float> a = p.subview(1, 50); });
    EXPECT_THROW({ ArrayView2<float> a = p.subview(3); },
                 std::invalid_argument);
    EXPECT_THROW({ ArrayView1<float> a = p.subview(3, 50); },
                 std::invalid_argument);
    EXPECT_THROW({ ArrayView1<float> a = p.subview(1, 110); },
                 std::invalid_argument);
    EXPECT_THROW({ ArrayView1<float> a = p.subview(3, 110); },
                 std::invalid_argument);

    EXPECT_NO_THROW({ ConstArrayView2<float> a = cp.subview(1); });
    EXPECT_NO_THROW({ ConstArrayView1<float> a = cp.subview(1, 50); });
    EXPECT_THROW({ ConstArrayView2<float> a = cp.subview(3); },
                 std::invalid_argument);
    EXPECT_THROW({ ConstArrayView1<float> a = cp.subview(3, 50); },
                 std::invalid_argument);
    EXPECT_THROW({ ConstArrayView1<float> a = cp.subview(1, 110); },
                 std::invalid_argument);
    EXPECT_THROW({ ConstArrayView1<float> a = cp.subview(3, 110); },
                 std::invalid_argument);

    EXPECT_NO_THROW({ ConstArrayView2<float> a = p.subview(1); });
    EXPECT_NO_THROW({ ConstArrayView1<float> a = p.subview(1, 50); });
    EXPECT_THROW({ ConstArrayView2<float> a = p.subview(3); },
                 std::invalid_argument);
    EXPECT_THROW({ ConstArrayView1<float> a = p.subview(3, 50); },
                 std::invalid_argument);
    EXPECT_THROW({ ConstArrayView1<float> a = p.subview(1, 110); },
                 std::invalid_argument);
    EXPECT_THROW({ ConstArrayView1<float> a = p.subview(3, 110); },
                 std::invalid_argument);

    ArrayView3<float> view = ptr;

    int i = 0;
    for (int ss = 0; ss < view.shape[0]; ++ss) {
        for (int xx = 0; xx < view.shape[1]; ++xx) {
            for (int yy = 0; yy < view.shape[2]; ++yy) {
                view(ss, xx, yy) = i++;
            }
        }
    }

    ArrayView1<float> subview1 = ptr.subview(1, 90);
    ArrayView1<float> subview2 = view.subview(1, 90);
    for (int j = 0; j < subview1.shape[0]; ++j) {
        EXPECT_EQ(subview1(j), subview2(j));
    }
}

TEST(Field_tests, bool_test) {
    Field empty{};
    Field assigned = Field{FieldDescriptor::memory(128)};

    bool should_be_false = false;
    bool should_be_true = false;

    if (empty) should_be_false = true;
    if (assigned) should_be_true = true;

    EXPECT_FALSE(should_be_false);
    EXPECT_TRUE(should_be_true);
}

TEST(Field_tests, bytes_test) {
    Field v128 = Field{FieldDescriptor::memory(128)};
    Field v32 = Field{FieldDescriptor::memory(32)};
    Field f128 = Field{fd_array<float>(128)};
    Field f32 = Field{fd_array<float>(32)};
    Field u29 = Field{fd_array<uint8_t>(29)};
    Field u32 = Field{fd_array<uint8_t>(32)};

    Field f32_16_64_20 = Field{fd_array<float>(32, 16, 64, 20)};

    EXPECT_EQ(v128.bytes(), 128);
    EXPECT_EQ(v32.bytes(), 32);
    EXPECT_EQ(f128.bytes(), 128 * sizeof(float));
    EXPECT_EQ(f32.bytes(), 32 * sizeof(float));
    EXPECT_EQ(u29.bytes(), 29 * sizeof(uint8_t));
    EXPECT_EQ(u32.bytes(), 32 * sizeof(uint8_t));
    EXPECT_EQ(f32_16_64_20.bytes(), 32 * 16 * 64 * 20 * sizeof(float));
}

TEST(Field_tests, sparse_memory_check_test) {
    Field contiguous_ptr = Field{fd_array<uint8_t>(128, 64, 32, 16)};
    FieldView sparse_view = contiguous_ptr.subview(keep(), keep(), keep(), 12);

    std::vector<uint8_t> v(128 * 64 * 32 * 16);

    EXPECT_TRUE(sparse_view.sparse());

    {
        Field ptr{FieldDescriptor::memory<void>(128)};
        EXPECT_FALSE(ptr.sparse());
    }
    {
        Field ptr{fd_array<float>(128)};
        EXPECT_FALSE(ptr.sparse());
    }
}

TEST(Field_tests, container_test) {
    // test vector
    {
        std::vector<Field> v;
        EXPECT_NO_THROW({
            v.push_back(Field{FieldDescriptor::memory(128)});
            v.push_back(Field{FieldDescriptor::memory(96)});
            v.push_back(Field{FieldDescriptor::memory(64)});
            v.push_back(Field{FieldDescriptor::memory(32)});
        });

        void* ptrs[4];
        int i = 0;
        for (auto&& p : v) {
            ptrs[i] = p;
            ++i;
        }

        EXPECT_NO_THROW({
            std::sort(v.begin(), v.end(),
                      [](auto&& a, auto&& b) { return a.bytes() < b.bytes(); });
        });

        for (auto&& p : v) {
            --i;
            EXPECT_EQ(ptrs[i], p.get());
        }
    }

    // test list
    {
        std::list<Field> l;
        EXPECT_NO_THROW({
            l.push_back(Field{FieldDescriptor::memory(128)});
            l.push_back(Field{FieldDescriptor::memory(96)});
            l.push_back(Field{FieldDescriptor::memory(64)});
            l.push_back(Field{FieldDescriptor::memory(32)});
        });

        void* ptrs[4];
        int i = 0;
        for (auto&& p : l) {
            ptrs[i] = p;
            ++i;
        }

        EXPECT_NO_THROW(
            l.sort([](auto&& a, auto&& b) { return a.bytes() < b.bytes(); }));

        for (auto&& p : l) {
            --i;
            EXPECT_EQ(ptrs[i], p.get());
        }
    }

    // test unordered_map
    {
        std::unordered_map<std::string, Field> m;
        m.emplace(std::make_pair("0", Field{FieldDescriptor::memory(128)}));
        m.emplace(std::make_pair("1", Field{FieldDescriptor::memory(96)}));
        m.emplace(std::make_pair("2", Field{FieldDescriptor::memory(64)}));
        m.emplace(std::make_pair("3", Field{FieldDescriptor::memory(32)}));

        EXPECT_NO_THROW(m["0"]);
        EXPECT_NO_THROW(m["1"]);
        EXPECT_NO_THROW(m["2"]);
        EXPECT_NO_THROW(m["3"]);
    }
}

TEST(Field_tests, matches_test) {
    Field ptr{fd_array<float>(32)};

    EXPECT_TRUE(ptr.matches(fd_array<float>(32)));

    EXPECT_FALSE(ptr.matches(fd_array<float>(8, 4)));
    EXPECT_FALSE(
        ptr.matches(FieldDescriptor::memory<float>(32 * sizeof(float))));
    EXPECT_FALSE(ptr.matches(fd_array<float>(64)));
    EXPECT_FALSE(ptr.matches(fd_array<int>(32)));
    EXPECT_FALSE(ptr.matches(FieldDescriptor::memory<int>(32 * sizeof(int))));
}

TEST(Field_tests, descriptor_strides_test) {
    auto a = fd_array<uint8_t>(2, 3, 4, 5);

    EXPECT_EQ(a.shape.size(), 4);
    EXPECT_EQ(a.shape[0], 2);
    EXPECT_EQ(a.shape[1], 3);
    EXPECT_EQ(a.shape[2], 4);
    EXPECT_EQ(a.shape[3], 5);

    EXPECT_EQ(a.strides.size(), 4);
    EXPECT_EQ(a.strides[0], 3 * 4 * 5);
    EXPECT_EQ(a.strides[1], 4 * 5);
    EXPECT_EQ(a.strides[2], 5);
    EXPECT_EQ(a.strides[3], 1);
}

TEST(Field_tests, view_sparse_test) {
    Field ptr{fd_array<float>(20, 320, 456, 18)};
    EXPECT_EQ(ptr.sparse(), false);
    EXPECT_EQ(ptr.subview(0).sparse(), false);
    EXPECT_EQ(ptr.subview(1).sparse(), false);
    EXPECT_EQ(ptr.subview(3).sparse(), false);
    EXPECT_EQ(ptr.subview(3, 5).sparse(), false);
    EXPECT_EQ(ptr.subview(3, 5, 10).sparse(), false);

    EXPECT_EQ(ptr.subview(keep(), keep(), keep(), 10).sparse(), true);
    EXPECT_EQ(ptr.subview(keep(), 200, 134, 10).sparse(), true);
    EXPECT_EQ(ptr.subview(keep(), 2, keep(), 10).sparse(), true);
    EXPECT_EQ(ptr.subview(1, keep(), keep(), 10).sparse(), true);
    EXPECT_EQ(ptr.subview(1, 2, keep(), 10).sparse(), true);
    EXPECT_EQ(ptr.subview(keep(), keep(), 10).sparse(), true);
    EXPECT_EQ(ptr.subview(1, keep(), 10).sparse(), true);
    EXPECT_EQ(ptr.subview(keep(), 10).sparse(), true);
    EXPECT_EQ(ptr.subview(keep(), 10, 5).sparse(), true);
    EXPECT_EQ(ptr.subview(keep(), 10, 5, 3).sparse(), true);
}

// TODO: would be good to move LidarScan::Header to types.h
template <typename T>
using Header = Eigen::Array<T, Eigen::Dynamic, 1>;

TEST(Field_tests, eigen_conversion_test) {
    Field ptr2d{fd_array<int>(100, 200)};

    ArrayView2<int> view2d = ptr2d;
    for (int y = 0; y < view2d.shape[0]; ++y)
        for (int x = 0; x < view2d.shape[1]; ++x) {
            view2d(y, x) = y * 1000 + x;
        }

    {
        Eigen::Ref<img_t<int>> ref = ptr2d;
        EXPECT_EQ(ref.data(), ptr2d.get<int>());
        EXPECT_EQ(ref.rows(), 100);
        EXPECT_EQ(ref.cols(), 200);

        for (int y = 0; y < ref.rows(); ++y)
            for (int x = 0; x < ref.cols(); ++x)
                EXPECT_EQ(ref(y, x), y * 1000 + x);
    }

    {
        Eigen::Ref<const img_t<int>> ref = ptr2d;
        EXPECT_EQ(ref.data(), ptr2d.get<int>());
        EXPECT_EQ(ref.rows(), 100);
        EXPECT_EQ(ref.cols(), 200);

        for (int y = 0; y < ref.rows(); ++y)
            for (int x = 0; x < ref.cols(); ++x)
                EXPECT_EQ(ref(y, x), y * 1000 + x);
    }

    Field ptr1d{fd_array<int>(100)};
    ArrayView1<int> view1d = ptr1d;

    for (int x = 0; x < view1d.shape[0]; ++x) {
        view1d(x) = x * 15;
    }

    {
        Eigen::Ref<Header<int>> ref = ptr1d;
        EXPECT_EQ(ref.data(), ptr1d.get<int>());
        EXPECT_EQ(ref.rows(), 100);

        for (int x = 0; x < ref.cols(); ++x) EXPECT_EQ(ref(x), x * 15);
    }

    {
        Eigen::Ref<const Header<int>> ref = ptr1d;
        EXPECT_EQ(ref.data(), ptr1d.get<int>());
        EXPECT_EQ(ref.rows(), 100);

        for (int x = 0; x < ref.cols(); ++x) EXPECT_EQ(ref(x), x * 15);
    }

    // clang-format off
    EXPECT_THROW({ Eigen::Ref<img_t<float>> r = ptr2d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<img_t<double>> r = ptr2d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<img_t<uint64_t>> r = ptr2d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<const img_t<float>> r = ptr2d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<const img_t<double>> r = ptr2d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<const img_t<uint64_t>> r = ptr2d; }, std::invalid_argument);

    EXPECT_THROW({ Eigen::Ref<Header<float>> r = ptr1d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<Header<double>> r = ptr1d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<Header<uint64_t>> r = ptr1d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<const Header<float>> r = ptr1d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<const Header<double>> r = ptr1d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<const Header<uint64_t>> r = ptr1d; }, std::invalid_argument);

    EXPECT_THROW({ Eigen::Ref<img_t<int>> r = ptr1d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<Header<int>> r = ptr2d; }, std::invalid_argument);

    FieldView sparse1d = ptr2d.subview(keep(), 10);
    FieldView sparse2d = ptr2d.subview(keep(), keep(2, 5));
    ASSERT_TRUE(sparse1d.sparse());
    ASSERT_TRUE(sparse2d.sparse());
    EXPECT_THROW({ Eigen::Ref<img_t<int>> r = sparse2d; }, std::invalid_argument);
    EXPECT_THROW({ Eigen::Ref<Header<int>> r = sparse1d; }, std::invalid_argument);
    // clang-format on
}

TEST(Field_tests, reshape_test) {
    std::vector<int> vec(100 * 200 * 300);
    auto view = FieldView{vec.data(), fd_array<int>(100, 200, 300)};
    EXPECT_THROW(view.reshape(3, 9, 12, 10), std::invalid_argument);
    EXPECT_THROW(view.reshape(3, 10), std::invalid_argument);
    EXPECT_THROW(view.subview(keep(), 2).reshape(50, 2, 300),
                 std::invalid_argument);
    EXPECT_NO_THROW(view.subview(2).reshape(100, 2, 300));

    /**
     * Below are copied directly from ArrayView tests and may be secondary,
     * but we are effectively testing matching runtime logic
     */
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

TEST(Field_tests, tag_tests) {
    EXPECT_EQ(FieldDescriptor::memory<void>(128).tag(),
              sensor::ChanFieldType::VOID);
    EXPECT_EQ(fd_array<uint8_t>(10).tag(), sensor::ChanFieldType::UINT8);
    EXPECT_EQ(fd_array<uint16_t>(10).tag(), sensor::ChanFieldType::UINT16);
    EXPECT_EQ(fd_array<uint32_t>(10).tag(), sensor::ChanFieldType::UINT32);
    EXPECT_EQ(fd_array<uint64_t>(10).tag(), sensor::ChanFieldType::UINT64);
    EXPECT_EQ(fd_array<int8_t>(10).tag(), sensor::ChanFieldType::INT8);
    EXPECT_EQ(fd_array<int16_t>(10).tag(), sensor::ChanFieldType::INT16);
    EXPECT_EQ(fd_array<int32_t>(10).tag(), sensor::ChanFieldType::INT32);
    EXPECT_EQ(fd_array<int64_t>(10).tag(), sensor::ChanFieldType::INT64);
    EXPECT_EQ(fd_array<float>(10).tag(), sensor::ChanFieldType::FLOAT32);
    EXPECT_EQ(fd_array<double>(10).tag(), sensor::ChanFieldType::FLOAT64);

    EXPECT_EQ(fd_array(sensor::ChanFieldType::UINT8, 10).tag(),
              sensor::ChanFieldType::UINT8);
    EXPECT_EQ(fd_array(sensor::ChanFieldType::UINT16, 10).tag(),
              sensor::ChanFieldType::UINT16);
    EXPECT_EQ(fd_array(sensor::ChanFieldType::UINT32, 10).tag(),
              sensor::ChanFieldType::UINT32);
    EXPECT_EQ(fd_array(sensor::ChanFieldType::UINT64, 10).tag(),
              sensor::ChanFieldType::UINT64);
    EXPECT_EQ(fd_array(sensor::ChanFieldType::INT8, 10).tag(),
              sensor::ChanFieldType::INT8);
    EXPECT_EQ(fd_array(sensor::ChanFieldType::INT16, 10).tag(),
              sensor::ChanFieldType::INT16);
    EXPECT_EQ(fd_array(sensor::ChanFieldType::INT32, 10).tag(),
              sensor::ChanFieldType::INT32);
    EXPECT_EQ(fd_array(sensor::ChanFieldType::INT64, 10).tag(),
              sensor::ChanFieldType::INT64);
}

template <typename T>
void uint_view_test() {
    std::vector<T> vec(100 * 200 * 300);
    FieldView view{vec.data(), fd_array<T>(100, 200, 300)};

    FieldView uint_v = uint_view(view);
    EXPECT_EQ(view.shape(), uint_v.shape());
    switch (sizeof(T)) {
        case 1:
            EXPECT_EQ(uint_v.tag(), sensor::ChanFieldType::UINT8);
            break;
        case 2:
            EXPECT_EQ(uint_v.tag(), sensor::ChanFieldType::UINT16);
            break;
        case 4:
            EXPECT_EQ(uint_v.tag(), sensor::ChanFieldType::UINT32);
            break;
        case 8:
            EXPECT_EQ(uint_v.tag(), sensor::ChanFieldType::UINT64);
            break;
    }
}

TEST(Field_tests, uint_view_tests) {
    EXPECT_NO_THROW(uint_view_test<short>());
    EXPECT_NO_THROW(uint_view_test<float>());
    EXPECT_NO_THROW(uint_view_test<double>());

    EXPECT_NO_THROW(uint_view_test<uint8_t>());
    EXPECT_NO_THROW(uint_view_test<uint16_t>());
    EXPECT_NO_THROW(uint_view_test<uint32_t>());
    EXPECT_NO_THROW(uint_view_test<uint64_t>());

    struct my_struct {
        double x, y;
    };
    std::vector<my_struct> vec(100);
    FieldView view{vec.data(), fd_array<my_struct>(100)};
    EXPECT_THROW(uint_view(view), std::invalid_argument);
}
