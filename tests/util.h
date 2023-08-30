/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <chrono>
#include <cstdlib>
#include <map>
#include <random>
#include <string>

#include "ouster/impl/packet_writer.h"

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

std::map<std::string, std::string> term_styles() {
    // clang-format off
    return {
        {"red", "\033[0;31m"},     {"green", "\033[0;32m"},
        {"yellow", "\033[0;33m"},  {"blue", "\033[0;34m"},
        {"magenta", "\033[0;35m"}, {"cyan", "\033[0;36m"},
        {"bold", "\033[1m"},       {"reset", "\033[0m"}};
    // clang-format on
}

/**
 * randomize field with values conforming to value_mask
 *
 * seeded version for consistent replication
 */
template <typename T>
void randomize_field(Eigen::Ref<ouster::img_t<T>> field, uint64_t value_mask,
                     size_t seed) {
    auto g = std::mt19937(seed);
    auto d = std::uniform_int_distribution<uint64_t>(0, value_mask);

    T* data = field.data();
    for (int i = 0; i < field.size(); ++i) {
        uint64_t word = d(g);
        word &= value_mask;
        *(data + i) = static_cast<T>(word);
    }
}

/**
 * randomize field with values conforming to value_mask
 */
template <typename T>
void randomize_field(Eigen::Ref<ouster::img_t<T>> field, uint64_t value_mask) {
    std::random_device rd;
    randomize_field(field, value_mask, rd());
}

inline std::string getenvs(const std::string& var) {
    char* res = std::getenv(var.c_str());
    return res ? std::string{res} : std::string{};
}
