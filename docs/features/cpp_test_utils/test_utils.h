#ifndef DOCS_FEATURES_CPP_TEST_UTILS_H
#define DOCS_FEATURES_CPP_TEST_UTILS_H

#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace docs_test_utils {

inline std::string env_or_empty(const char* key) {
    const char* value = std::getenv(key);
    return value && *value ? std::string(value) : std::string{};
}

inline bool file_exists(const std::string& path) {
    std::ifstream f(path.c_str(), std::ios::binary);
    return f.good();
}

inline std::string join_path(const std::string& base, const std::string& leaf) {
    if (base.empty()) {
        return leaf;
    }
    if (base.back() == '/') {
        return base + leaf;
    }
    return base + '/' + leaf;
}

inline std::string data_root() {
#ifdef OUSTER_SDK_SOURCE_DIR
    return join_path(OUSTER_SDK_SOURCE_DIR, "tests");
#else
    throw std::runtime_error("OUSTER_SDK_SOURCE_DIR not defined");
#endif
}

inline std::string capture_stdout(const std::function<void()>& action) {
    std::ostringstream buffer;
    auto* original = std::cout.rdbuf(buffer.rdbuf());
    try {
        action();
    } catch (...) {
        std::cout.rdbuf(original);
        throw;
    }
    std::cout.rdbuf(original);
    return buffer.str();
}

}  // namespace docs_test_utils

#endif  // DOCS_FEATURES_CPP_TEST_UTILS_H
