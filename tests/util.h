/**
 * Copyright (c) 2023, Ouster, Inc.
 * All rights reserved.
 */

#include <chrono>
#include <map>
#include <string>

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
