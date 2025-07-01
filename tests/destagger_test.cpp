/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <chrono>
#include <iomanip>
#include <numeric>
#include <random>

#include "ouster/lidar_scan.h"

class Timer {
   public:
    void start() { t_start = std::chrono::system_clock::now(); }

    void stop() { t_end = std::chrono::system_clock::now(); }

    int64_t elapsed_microseconds() {
        using namespace std::chrono;
        return duration_cast<microseconds>(t_end - t_start).count();
    }

   private:
    std::chrono::time_point<std::chrono::system_clock> t_start;
    std::chrono::time_point<std::chrono::system_clock> t_end;
};

template <typename T, typename Total, size_t N>
class MovingAverage {
   public:
    MovingAverage& operator()(T sample) {
        total_sum += sample;
        if (samples_count < N)
            samples[samples_count++] = sample;
        else {
            T& oldest = samples[samples_count++ % N];
            total_sum -= oldest;
            oldest = sample;
        }
        return *this;
    }

    operator double() const {
        return static_cast<double>(total_sum) /
               static_cast<double>(std::min(samples_count, N));
    }

   private:
    T samples[N];
    size_t samples_count{0};
    Total total_sum{0};
};

using namespace ouster;

// the original destagger method
template <typename T>
inline img_t<T> destagger_eigen(const Eigen::Ref<const img_t<T>>& img,
                                const std::vector<int>& pixel_shift_by_row,
                                bool inverse) {
    const size_t h = img.rows();
    const size_t w = img.cols();
    if (pixel_shift_by_row.size() != h)
        throw std::invalid_argument{"image height does not match shifts size"};

    img_t<T> destaggered{h, w};
    for (size_t u = 0; u < h; u++) {
        const std::ptrdiff_t offset =
            ((inverse ? -1 : 1) * pixel_shift_by_row[u] + w) % w;

        destaggered.row(u).segment(offset, w - offset) =
            img.row(u).segment(0, w - offset);
        destaggered.row(u).segment(0, offset) =
            img.row(u).segment(w - offset, offset);
    }

    return destaggered;
}

template <typename T>
inline img_t<T> destagger_memcpy(const Eigen::Ref<const img_t<T>>& img,
                                 const std::vector<int>& pixel_shift_by_row,
                                 bool inverse) {
    const size_t h = img.rows();
    const size_t w = img.cols();
    if (pixel_shift_by_row.size() != h)
        throw std::invalid_argument{"image height does not match shifts size"};

    int sign = inverse ? -1 : +1;

    img_t<T> destaggered{h, w};
    const auto* const g = img.data();
    const auto d = destaggered.data();

    for (size_t u = 0; u < h; ++u) {
        const auto g_row = g + u * w;
        const auto d_row = d + u * w;
        const int offset = (w + sign * pixel_shift_by_row[u] % w) % w;
        memcpy(d_row, g_row + (w - offset), offset * sizeof(T));
        memcpy(d_row + offset, g_row, (w - offset) * sizeof(T));
    }

    return destaggered;
}

template <typename T>
inline img_t<T> destagger_memcpy2(const Eigen::Ref<const img_t<T>>& img,
                                  const std::vector<int>& pixel_shift_by_row,
                                  bool inverse) {
    const size_t h = img.rows();
    const size_t w = img.cols();
    if (pixel_shift_by_row.size() != h)
        throw std::invalid_argument{"image height does not match shifts size"};

    int sign = inverse ? -1 : +1;

    std::vector<int> offsets(h);
    for (size_t u = 0; u < h; ++u) {
        offsets[u] = (w + sign * pixel_shift_by_row[u] % w) % w;
    }

    img_t<T> destaggered{h, w};
    const auto* const g = img.data();
    const auto d = destaggered.data();

    for (size_t u = 0; u < h; ++u) {
        const int offset = offsets[u];
        const auto g_row = g + u * w;
        const auto d_row = d + u * w;
        memcpy(d_row, g_row + (w - offset), offset * sizeof(T));
        memcpy(d_row + offset, g_row, (w - offset) * sizeof(T));
    }

    return destaggered;
}

template <typename T>
inline img_t<T> destagger_memcpy3(const Eigen::Ref<const img_t<T>>& img,
                                  const std::vector<int>& offsets) {
    const size_t h = img.rows();
    const size_t w = img.cols();
    if (offsets.size() != h)
        throw std::invalid_argument{"image height does not match shifts size"};

    img_t<T> destaggered{h, w};
    auto g_row = img.data();
    auto d_row = destaggered.data();

    for (size_t u = 0; u < h; ++u) {
        const int offset = offsets[u];
        memcpy(d_row, g_row + (w - offset), offset * sizeof(T));
        memcpy(d_row + offset, g_row, (w - offset) * sizeof(T));
        g_row += w;
        d_row += w;
    }

    return destaggered;
}

class DestaggerParameterizedTestFixture
    : public ::testing::TestWithParam<std::pair<int, int>> {
   protected:
    int scan_width;
    int scan_height;
};

std::vector<int> init_vector_rand(size_t size) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<int> rvector(size);
    std::uniform_int_distribution<> dis(-30, 30);
    for (int& v : rvector) {
        v = dis(gen);
    }

    return rvector;
}

TEST(DestaggerParameterizedTestFixture,
     OusterDestaggerRestaggerFunctionsMatch) {
    const auto WIDTH = 512;
    const auto HEIGHT = 64;

    img_t<uint32_t> range = img_t<uint32_t>::Random(HEIGHT, WIDTH);
    std::vector<int> shifts = init_vector_rand(HEIGHT);

    std::cout << "range.size(): " << range.rows() << std::endl;
    std::cout << "shift.size(): " << shifts.size() << std::endl;

    auto destagger_image = ouster::destagger<uint32_t>(range, shifts, true);
    auto staggered_image = destagger<uint32_t>(destagger_image, shifts, false);
    EXPECT_TRUE(staggered_image.isApprox(range));
}

TEST(DestaggerParameterizedTestFixture, DestaggerRestaggerFunctionsMatch) {
    const auto WIDTH = 512;
    const auto HEIGHT = 64;

    img_t<uint32_t> range = img_t<uint32_t>::Random(HEIGHT, WIDTH);
    std::vector<int> shifts = init_vector_rand(HEIGHT);

    std::cout << "range.size(): " << range.rows() << std::endl;
    std::cout << "shift.size(): " << shifts.size() << std::endl;

    auto destagger_image = destagger_eigen<uint32_t>(range, shifts, true);
    auto staggered_image =
        destagger_eigen<uint32_t>(destagger_image, shifts, false);
    EXPECT_TRUE(staggered_image.isApprox(range));
}

TEST(DestaggerParameterizedTestFixture, DestaggerVsDestaggerCpyFunctionsMatch) {
    const auto WIDTH = 512;
    const auto HEIGHT = 64;

    img_t<uint32_t> range = img_t<uint32_t>::Random(HEIGHT, WIDTH);
    std::vector<int> shifts = init_vector_rand(HEIGHT);

    std::cout << "range.size(): " << range.rows() << std::endl;
    std::cout << "shift.size(): " << shifts.size() << std::endl;

    auto destagger_image = destagger_eigen<uint32_t>(range, shifts, true);
    auto destagger_image2 = destagger_memcpy<uint32_t>(range, shifts, true);
    EXPECT_TRUE(destagger_image2.isApprox(destagger_image));

    auto idestagger_image = destagger_eigen<uint32_t>(range, shifts, false);
    auto idestagger_image2 = destagger_memcpy<uint32_t>(range, shifts, false);
    EXPECT_TRUE(idestagger_image2.isApprox(idestagger_image));
}

INSTANTIATE_TEST_CASE_P(DestaggerParamterizedTests,
                        DestaggerParameterizedTestFixture,
                        ::testing::Values(std::pair<size_t, size_t>{512, 128},
                                          std::pair<size_t, size_t>{1024, 128},
                                          std::pair<size_t, size_t>{2048, 128},
                                          std::pair<size_t, size_t>{4096,
                                                                    128}));

TEST_P(DestaggerParameterizedTestFixture, SpeedCheck) {
    ::testing::Test::RecordProperty("gtest_timeout_ms", 3000);

    std::map<std::string, std::string> styles = {
        {"red", "\033[0;31m"},     {"green", "\033[0;32m"},
        {"yellow", "\033[0;33m"},  {"blue", "\033[0;34m"},
        {"magenta", "\033[0;35m"}, {"cyan", "\033[0;36m"},
        {"bold", "\033[1m"},       {"reset", "\033[0m"}};

    const auto test_params = GetParam();
    const size_t WIDTH = test_params.first;
    const size_t HEIGHT = test_params.second;
    std::cout << styles["yellow"] << styles["bold"]
              << "CHECKING PERFORMANCE FOR LIDAR MODE: [" << WIDTH << "x"
              << HEIGHT << "]" << styles["reset"] << std::endl;

    img_t<uint32_t> range = img_t<uint32_t>(HEIGHT, WIDTH);
    std::vector<int> shifts = init_vector_rand(HEIGHT);

    img_t<uint32_t> output = img_t<uint32_t>(HEIGHT, WIDTH);

    constexpr int N_SCANS = 1000;
    constexpr int MOVING_AVG_WINDOW = 100;
    using MovingAverage64 = MovingAverage<int64_t, int64_t, MOVING_AVG_WINDOW>;
    static std::map<std::string, MovingAverage64> mv;

    Timer t;
    std::stringstream ss;
    int output_ctr = 0;

    using CartesianMethod = std::function<void(const img_t<uint32_t>& range)>;
    std::vector<std::pair<std::string, CartesianMethod>> all_cartesians;

    all_cartesians.emplace_back("c0", [&](const img_t<uint32_t>& range) {
        output = destagger_eigen<uint32_t>(range, shifts, false);
    });

    all_cartesians.emplace_back("mcpy", [&](const img_t<uint32_t>& range) {
        output = destagger_memcpy<uint32_t>(range, shifts, false);
    });

    all_cartesians.emplace_back("mcpy2", [&](const img_t<uint32_t>& range) {
        output = destagger_memcpy2<uint32_t>(range, shifts, false);
    });

    std::vector<int> offsets(HEIGHT);
    for (size_t u = 0; u < HEIGHT; ++u) {
        offsets[u] = (WIDTH - shifts[u] % WIDTH) % WIDTH;
    }

    all_cartesians.emplace_back("mcpy3", [&](const img_t<uint32_t>& range) {
        output = destagger_memcpy3<uint32_t>(range, offsets);
    });

    std::default_random_engine g;
    std::uniform_real_distribution<double> d(0.0, 1.0);
    std::vector<int> ids(all_cartesians.size());
    std::iota(std::begin(ids), std::end(ids), 0);

    for (auto i = 0; i < N_SCANS; ++i) {
        auto p = std::ceil(10 * float(i) / float(N_SCANS)) / 10;
        auto unary_expr = [&g, &d, p](auto) {
            return d(g) >= p ? 0U : static_cast<uint32_t>(d(g) * 10000);
        };

        auto valid_returns = (range != 0).count();
        auto percentage_valid =
            int(roundf(float(valid_returns) / float(range.size()) * 100));

        std::shuffle(std::begin(ids), std::end(ids), g);
        for (auto i : ids) {
            const auto& method = all_cartesians[i];
            range = range.unaryExpr(unary_expr);
            t.start();
            method.second(range);
            t.stop();
            mv[method.first](t.elapsed_microseconds());
        }

        if (++output_ctr % MOVING_AVG_WINDOW == 0) {
            ss.str("");
            ss << styles["bold"] << "returns: " << styles["reset"]
               << styles["magenta"] << std::setw(3) << percentage_valid << "%, "
               << styles["reset"];
            ss << styles["bold"] << "c0[time]: " << styles["reset"]
               << styles["cyan"] << std::setw(4) << mv["c0"] << "μs, "
               << styles["reset"];

            auto best_time = std::min_element(
                mv.begin(), mv.end(), [](const auto& a, const auto& b) -> bool {
                    return a.second < b.second;
                });

            for (const auto& x : mv) {
                auto speedup = lround(100.0f * mv["c0"] / x.second);
                auto color_modifier =
                    x.first == best_time->first
                        ? styles["blue"]
                        : (speedup >= 100 ? styles["green"] : styles["red"]);
                ss << styles["bold"] << x.first << ": " << color_modifier
                   << std::setw(4) << speedup << "%, " << styles["reset"];
            }
            std::cout << ss.str() << std::endl;
        }
    }
}
